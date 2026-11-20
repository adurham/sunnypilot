"""
Fork: low-speed minimum-gap brake floor (adurham/sunnypilot). Inert unless the ``MinGapBrakeFloor`` param is set.

Why this exists
---------------
Route 000001ac, 2026-11-19, I-80/94 stop-and-go at 12-14 mph, night, a flat-rear white RV trailer directly ahead.
From 59:37 to 59:45.7 the car crept from ~7.5 m to 1.4-2.7 m behind that trailer (THW 0.3-0.5 s) while the published ask
sat at exactly 0.00 and the car accelerated 4.08 -> 6.21 m/s; the driver braked and disengaged. Vision read the trailer as
pulling away (vRel +13.7/+15.0 m/s spikes while the gap shrank), so the MPC and the e2e candidates both stayed positive,
``NoCloseOnLead`` released twice, and nothing ever asked for a brake. The gap itself was never wrong - the 0.5 s median
dRel tracked the trailer at 1.4-2.7 m.

That is the hole this fills: every existing rule keys on the lead's SPEED (vRel / closing / vLead), and every one of them
is blind when the speed is wrong. The gap is the robust signal at these ranges, so the last resort keys on the gap alone.
It is NOT an AEB: a 1 m/s^2 brake that arms at a 3 m gap cannot prevent contact with a lead that stops hard.

What it does (v2.1 = v2 + the 2026-11-20 senior review's required changes)
-------------------------------------------------------------------------
A last-stage floor on the published accel request, applied after ``NoCloseCap`` and before the standstill debounce (so no
candidate and no later stage can raise it back). It only ever DEEPENS an ask (``min``). Four states:

    d = median of the last N_MED dRel samples, ONE sample per NEW radarState message (0.5 s at 20 Hz)

    OFF -> BRAKE   engaged, no driver gas, lead present + trusted + fresh, V_MIN < v <= V_MAX, d < max(D_MIN, K_ENTER*v),
                   >= MIN_SAMPLES samples, launch grace and re-arm cooldown expired, no duration-cap latch, budget left
    BRAKE          out = min(accel, A_FLOOR)                      # a real FCA11 brake request (continuous law: ~5-6 LSB)
    BRAKE -> HOLD  d >= max(D_EXIT, K_EXIT*v) CONTINUOUSLY for EXIT_DWELL_S, or v <= V_RELEASE, or v > V_MAX,
                   or BRAKE_MAX_S of brake in this approach (the DURATION CAP: latches, see below)
    HOLD           out = min(accel, hold_ask(v))                  # zero pedal, NO friction brake (see "the hold")
    HOLD -> OFF    d >= max(D_RECOVER, K_RECOVER*v) CONTINUOUSLY for RECOVER_DWELL_S, or v < V_STANDSTILL,
                   or (after a duration cap only) CAP_HOLD_MAX_S of hold
    BRAKE/HOLD -> BRIDGE   the lead goes absent, untrusted or STALE: brake dropped, pedal held at zero, median kept, for at
                   most LEAD_BRIDGE_S; a lead back inside it resumes BRAKE (only if the bridge came from BRAKE)
    BRIDGE -> OFF  the dropout outlasts the bridge (buffer cleared): a fail-safe stop

The hold (and why it is no longer -0.45)
----------------------------------------
A release must not hand the ask straight back to a planner that may still think the lead is pulling away (the pedal's
hold feedforward then pushes the car forward at ask 0). So a release enters HOLD: zero pedal and no brake until the gap
has recovered. v2 used A_HOLD = -0.45 on the premise that the FCA11 brake only opens at -0.5. That is the OLD (stepped)
law's onset (``FCA11_BRAKE_ONSET_ACCEL``). The car runs the 0037 CONTINUOUS law (``Fca11LongBrake.cont_law``: FCA11
braking + Shaping/AffineGain + DynamicBrakes), which has NO onset: it brakes for any demand beyond the coast,
``fca11_cont_brake_need(need, v) = blend(need - coast(v))``. At -0.45 that is 0.12-0.31 m/s^2 of friction-brake demand at
1.4-10 m/s (1-2 LSB held, brake lamps lit) and, below 5 km/h, the stop band's 7 LSB creep floor - so v2's "hold" and its
lead-loss "bridge" were a light brake on the real law.

The only ask that is both zero pedal and zero brake on that law is the law's own coast point: the continuous gas branch
commands ``hold(v)*max(0, 1 - need/coast(v))`` and the brake ``blend(need - coast(v))``; they meet at need == coast(v).
``hold_ask(v) = -(coast(v) - HOLD_COAST_MARGIN)`` sits HOLD_COAST_MARGIN (0.008 m/s^2) on the pedal side of it: the brake
demand is exactly 0 (no level, no trim, no stop-band floor) for any car-layer speed within ~0.1 m/s of the planner's, and
the pedal target is below one wire count, which the law's coast-zero latch turns into an exact 0 (ENABLE=0 pass-through).
``coast`` is IMPORTED from the car layer (``fca11_long.fca11_cont_coast``), never copied. Below ~1 m/s that hold does not
stop the car (the DCT still creeps): the stop stays the planner's and the driver's.

Dependency gate (review item 5)
-------------------------------
Those properties exist only while the car layer runs the continuous law. With it off, ``hold_ask`` is a live pedal
(``scale(v)*ask + hold(v) > 0`` above ~2 m/s). So the floor refuses to arm unless the car layer's own state is the
continuous law, computed from the SAME CarParamsSP fields the car layer reads, with the car layer's own helpers
(``law_status``; pinned by a test against the real ``Fca11LongBrake`` for every flag combination). The CarParamsSP the car
layer published is re-read once a second; a changed value is re-evaluated, an unreadable one disables the floor.
``dependency`` exposes the verdict and every change is logged.

Duration cap (review item 4: the most dangerous remaining failure is a SUSTAINED false brake)
------------------------------------------------------------------------------------------------
A persistently short dRel (a phantom) would otherwise hold -1.0 from 10 m/s down to 1 m/s. BRAKE therefore has a budget
of BRAKE_MAX_S per approach (the two real brakes were 2.55 s and 3.75 s). The budget is CUMULATIVE across lead dropouts,
so a phantom that blinks out for a second cannot buy a fresh 5 s; it is forgotten only when the approach really ends (the
gap recovers, the car stops, the driver disengages) or after BUDGET_FORGET_S with the floor off. When it runs out the floor
steps into HOLD for at most CAP_HOLD_MAX_S and then OFF, and LATCHES (``cap_latched``): no re-arm until the gap has been seen
above the recovery band for RECOVER_DWELL_S, the car stops, the driver disengages, or no usable lead has been seen for
LATCH_FORGET_ABSENT_S. A lead dropout never clears the latch.

Staleness (review item 4)
-------------------------
A radarState older than STALE_RADAR_S at this planner frame (by its own logMonoTime, or by the modelV2 frame it was
computed from, ``mdMonoTime``) is a LOST lead, not a stationary one. Median samples are counted per NEW radarState message,
so a frozen message cannot keep padding the median with the same range.

Re-arm, cooldown and the flicker bound (review item 7)
-------------------------------------------------------
A re-arm after a release (or after a driver gas override) needs REARM_COOLDOWN_S. A dropout EXPIRY is a fail-safe stop,
not a release, so its re-arm is not cooldown-gated (5 s of blindness next to a close lead is worse) - but once
FLICKER_REARMS such re-arms have happened inside FLICKER_WINDOW_S, the next expiry starts the cooldown too.

Containment (review items 2/3; the v1 bug)
------------------------------------------
v1 evaluated ``sm['radarState'].modelProb`` on every frame, param OFF included; RadarState has no such field, so pycapnp
raises ``AttributeError: struct has no such member`` and the planner would have died on frame 1. Here the planner calls
``apply()``, which with the param OFF (or the dependency unmet) reads NO message field and returns the ask object itself,
and with it ON runs inside one try: any exception passes the ask through UNMODIFIED, latches the floor off for the rest of
the drive (``faulted``) and logs an error with the traceback. Fields are read directly - a missing ``leadOne.modelProb`` is
such an exception, never a trusted 1.0 (review item 2). Note: for a vision lead radard sets ``present`` only when its
filtered prob is > 0.5 and publishes that prob as ``modelProb``, so the MIN_LEAD_PROB gate cannot reject a present vision
lead; it is kept as a defensive input, not counted as a filter.

The brake is delivered by the existing FCA11 path - the floor only asks; the car layer's continuous law and the panda gate
remain the only things that decide what the ESC sees.
"""
from collections import deque
import math

import numpy as np

from openpilot.common.realtime import DT_MDL

try:
  from openpilot.common.swaglog import cloudlog
except Exception:  # a harness that stubs the openpilot package: the latch still works, only the log line is lost
  cloudlog = None

PARAM = "MinGapBrakeFloor"
CP_SP_PARAM = "CarParamsSP"   # what the car layer (card.py) published at its start; the floor re-checks it once a second

# --- geometry / speed fences -----------------------------------------------------------------------------------------
V_MIN = 1.5           # m/s: no entry at/below this (the launch from rest is the standstill/launch logic's)
V_MAX = 10.0          # m/s: no entry above this (36 km/h / 22 mph) - a low-speed creep rule, not a highway rule
V_RELEASE = 1.0       # m/s: an active brake enters HOLD below this (never brake a car to a standstill; never free the ask)
V_STANDSTILL = 0.5    # m/s: a held car ends the HOLD here - the launch logic owns the pull-away, covered by the grace
D_MIN = 3.0           # m: entry gap floor
K_ENTER = 0.5         # s: entry gap = max(D_MIN, K_ENTER * v_ego)  -> 0.5 s headway
D_EXIT = 5.0          # m: exit gap floor
K_EXIT = 0.8          # s: exit gap = max(D_EXIT, K_EXIT * v_ego)  -> 0.8 s headway
D_RECOVER = 6.0       # m: post-release recovery gap floor
K_RECOVER = 1.0       # s: recovery gap = max(D_RECOVER, K_RECOVER * v_ego) -> 1.0 s headway

# --- asks ------------------------------------------------------------------------------------------------------------
A_FLOOR = -1.0        # m/s^2: the brake ask while BRAKING (continuous law: ~5-6 LSB; ~1.0 m/s^2 total, measured)
HOLD_COAST_MARGIN = 0.008  # m/s^2 the hold ask sits on the PEDAL side of the car layer's coast point (see hold_ask)

# --- timing ----------------------------------------------------------------------------------------------------------
N_MED = 10            # samples of the 0.5 s median (one per radarState message, 20 Hz)
MIN_SAMPLES = 5       # a lead must supply at least 0.25 s of its own samples before the floor may ADD a brake
EXIT_DWELL_S = 1.0    # s the MEDIAN must stay above the exit band before a brake releases
RECOVER_DWELL_S = 1.0  # s the MEDIAN must stay above the recovery band before the hold ends (or a cap latch clears)
LEAD_BRIDGE_S = 1.0   # s a dropout may last with the pedal held at zero and the buffer kept
LAUNCH_GRACE_S = 2.0  # s after v drops under 1 m/s during which the floor may not arm
REARM_COOLDOWN_S = 5.0  # s after a release / a driver override before the floor may arm again
MIN_LEAD_PROB = 0.5   # modelProb below this (or NaN) = untrusted -> no brake (bridge instead). Vacuous for vision leads.
BRAKE_MAX_S = 5.0     # s of BRAKE per approach, cumulative across dropouts (review item 4)
CAP_HOLD_MAX_S = 5.0  # s the zero-pedal hold may last after a duration cap before the floor lets go (latched)
BUDGET_FORGET_S = 10.0  # s of floor-OFF after which an approach's brake budget is forgotten
LATCH_FORGET_ABSENT_S = 5.0  # s with no usable lead after which a duration-cap latch is forgotten
STALE_RADAR_S = 0.25  # s: a radarState older than this at the planner frame is a lost lead (review item 4)
FLICKER_WINDOW_S = 10.0  # s: inside this window ...
FLICKER_REARMS = 2       # ... this many dropout-expiry re-arms make the NEXT expiry cooldown-gated (review item 7)
POLL_S = 1.0          # s between param / CarParamsSP re-reads

OFF, BRAKE, HOLD, BRIDGE = "off", "brake", "hold", "bridge"

# dependency verdicts (diagnostics + cloudlog; ``enabled`` requires DEP_OK)
DEP_OK = "ok"
DEP_PARAM_OFF = "param-off"
DEP_NO_CAR_LAYER = "car-layer-unavailable"   # the opendbc FCA11 module could not be imported / used
DEP_NO_CP_SP = "no-carparams-sp"
DEP_CP_SP_UNREADABLE = "carparams-sp-unreadable"
DEP_LAW_OFF = "law-off"                      # the car layer does not run the continuous FCA11 law
DEP_PARAMS_UNAVAILABLE = "params-unavailable"
DEP_FAULTED = "faulted"


def enter_gap(v_ego: float) -> float:
  """Gap (m) at/below which the floor may turn on."""
  return max(D_MIN, K_ENTER * float(v_ego))


def exit_gap(v_ego: float) -> float:
  """Gap (m) at/above which an active brake may begin the release dwell."""
  return max(D_EXIT, K_EXIT * float(v_ego))


def recover_gap(v_ego: float) -> float:
  """Gap (m) at/above which the post-release hold may end (higher than the exit band)."""
  return max(D_RECOVER, K_RECOVER * float(v_ego))


def load_car_layer():
  """The deployed car layer's FCA11 module (its coast curve, law switches and helpers), or None. Never raises."""
  try:
    from opendbc.sunnypilot.car.hyundai import fca11_long
    return fca11_long
  except Exception:
    return None


def hold_ask(v_ego: float, coast_fn) -> float:
  """The zero-pedal, zero-brake ask on the continuous law: HOLD_COAST_MARGIN above -coast(v); never positive."""
  return min(0.0, -(float(coast_fn(max(0.0, float(v_ego)))) - HOLD_COAST_MARGIN))


def floor_ask(accel: float, state: str, v_ego: float, coast_fn) -> float:
  """The floor itself: only ever deepens an ask. Never raises, never touches a brake that is already deeper."""
  if state == BRAKE:
    return min(float(accel), A_FLOOR)
  if state in (HOLD, BRIDGE):
    return min(float(accel), hold_ask(v_ego, coast_fn))
  return float(accel)


def law_status(cp_sp, fl) -> str:
  """DEP_OK if and only if the car layer built from ``cp_sp`` runs the continuous FCA11 law on its actuating instance, i.e.
  ``Fca11LongBrake(CP, cp_sp).enabled and .cont_law``. Reads the CarParamsSP fields the car layer reads DIRECTLY (a field
  this schema lacks is DEP_CP_SP_UNREADABLE, never a default) and combines them with the car layer's OWN helpers and code
  switches, so the two cannot drift silently; ``test_min_gap_floor`` pins it against the real class."""
  if fl is None:
    return DEP_NO_CAR_LAYER
  if cp_sp is None:
    return DEP_NO_CP_SP
  try:
    brake = bool(cp_sp.fca11Brake)
    pedal = bool(cp_sp.enableGasInterceptor)
    bool(cp_sp.fca11AffineGain)
    dynamic = bool(cp_sp.fca11DynamicBrakes)
    bool(cp_sp.fca11Shadow)
  except Exception:
    return DEP_CP_SP_UNREADABLE
  try:
    enabled = brake and pedal                                          # Fca11LongBrake.enabled
    shadow = bool(fl.shadow_enabled(cp_sp))                            # .shadow (actuating instance: core=False)
    affine = bool(fl.affine_gain_enabled(cp_sp)) and not shadow        # .affine_gain
    shape_track = bool(fl.FCA11_TRACK_ENABLED) and affine and dynamic  # .shape_track
    cont = bool(fl.FCA11_CONT_ENABLED) and shape_track                 # .cont_law
  except Exception:
    return DEP_NO_CAR_LAYER
  return DEP_OK if (enabled and cont) else DEP_LAW_OFF


def _parse_cp_sp(raw):
  """CarParamsSP bytes -> a capnp reader, exactly as plannerd builds its own (raises on junk)."""
  from openpilot.cereal import custom
  import openpilot.cereal.messaging as messaging
  return messaging.log_from_bytes(raw, custom.CarParamsSP)


def _log(level: str, msg: str, *args, exc_info: bool = False) -> None:
  try:
    if cloudlog is not None:
      getattr(cloudlog, level)(msg, *args, exc_info=exc_info)
  except Exception:
    pass


class MinGapFloor:
  """Owned by LongitudinalPlanner. ``apply`` is the contained planner hook; ``update`` is the rule with explicit inputs.

  Diagnostics (read by tests / the shadow replay, never by the planner): ``state``, ``gap_med`` (the median the decision
  used), ``launch_hold`` / ``rearm_t`` (clocks, s remaining), ``brake_t`` (s of brake in this approach), ``hold_t``,
  ``cap_latched``, ``dependency`` (why the floor may or may not arm), ``enabled``, ``faulted`` (the drive-long latch).
  """

  def __init__(self, params=None, dt: float = DT_MDL, cp_sp=None, car_layer="auto", cp_sp_parser=None):
    # The constructor must never raise: a planner that cannot build this object would take longitudinal control down.
    self.dt = float(dt)
    self._params = params
    self._cp_sp = cp_sp
    self._cp_sp_raw = None        # the last CarParamsSP bytes read back from Params (None = never seen)
    self._cp_sp_live = None
    self._cp_sp_parser = cp_sp_parser if cp_sp_parser is not None else _parse_cp_sp
    self._fl = None
    self._coast = None
    self._frame = 0
    self._poll_n = max(1, int(round(POLL_S / self.dt)))
    self.t = 0.0                  # rule clock (s), advanced once per rule step
    self.enabled = False
    self.param_on = False
    self.faulted = False
    self.dependency = DEP_PARAM_OFF
    self._dep_logged = None
    self.state = OFF
    self.gap_med = float("nan")
    self.launch_hold = 0.0
    self.rearm_t = 0.0
    self.exit_t = 0.0
    self.recover_t = 0.0
    self.latch_recover_t = 0.0
    self.bridge_t = 0.0
    self.brake_t = 0.0
    self.hold_t = 0.0
    self.off_t = 0.0              # s continuously OFF (forgets the brake budget)
    self.absent_t = 0.0           # s with no usable lead (forgets a cap latch)
    self.cap_latched = False
    self._bridge_from = OFF
    self._last_radar_t = None
    self._last_expiry_t = None
    self._expiry_rearms: deque[float] = deque()
    self._buf: deque[float] = deque(maxlen=max(1, int(round(N_MED))))
    try:
      fl = load_car_layer() if isinstance(car_layer, str) and car_layer == "auto" else car_layer
      coast = getattr(fl, "fca11_cont_coast", None) if fl is not None else None
      if fl is not None and callable(coast):
        self._fl, self._coast = fl, coast
      self._read()
    except Exception:
      self.enabled = False
      self.dependency = DEP_NO_CAR_LAYER

  # -- plumbing ------------------------------------------------------------------------------------------------------
  def _dependency(self) -> str:
    if self._fl is None:
      return DEP_NO_CAR_LAYER
    try:
      raw = self._params.get(CP_SP_PARAM)
    except Exception:
      raw = None                  # cannot read it back: judge the CarParamsSP this planner was built with
    if raw is not None and raw != self._cp_sp_raw:
      self._cp_sp_raw = raw
      try:
        self._cp_sp_live = self._cp_sp_parser(raw)
      except Exception:
        self._cp_sp_live = None
    if self._cp_sp_raw is not None:
      if self._cp_sp_live is None:
        return DEP_CP_SP_UNREADABLE
      return law_status(self._cp_sp_live, self._fl)
    return law_status(self._cp_sp, self._fl)

  def _read(self) -> None:
    """Param + dependency re-read (constructor, then every POLL_S). Never reads a message."""
    if self._params is None:
      try:
        from openpilot.common.params import Params
        self._params = Params()
      except Exception:
        self._set_enabled(False, DEP_PARAMS_UNAVAILABLE)
        return
    try:
      param_on = bool(self._params.get_bool(PARAM))
    except Exception:             # UnknownKeyName on a libparams that predates the key: the feature stays off
      param_on = False
    self.param_on = param_on
    if self.faulted:
      self._set_enabled(False, DEP_FAULTED)
    elif not param_on:
      self._set_enabled(False, DEP_PARAM_OFF)
    else:
      dep = self._dependency()
      self._set_enabled(dep == DEP_OK, dep)

  def _set_enabled(self, enabled: bool, dep: str) -> None:
    if self.enabled and not enabled:
      self._full_reset(clear_buffer=True)   # a later re-enable starts cold, never from a stale BRAKE
    self.enabled = bool(enabled)
    self.dependency = dep
    if dep != self._dep_logged:
      self._dep_logged = dep
      if dep == DEP_OK:
        _log("info", "min_gap_floor: armed (the car layer runs the continuous FCA11 law)")
      elif dep != DEP_PARAM_OFF:
        _log("warning", "min_gap_floor: param on but the floor is DISABLED: %s", dep)

  def _poll(self) -> None:
    if self._frame % self._poll_n == 0:
      self._read()
    self._frame += 1

  def _fault(self) -> None:
    self.faulted = True
    self._set_enabled(False, DEP_FAULTED)
    # loud: this is the one message that says a safety feature disabled itself on the car
    _log("error", "min_gap_floor: DISABLED for this drive after an exception in the planner hook; " +
                  "the ask now passes through unmodified", exc_info=True)

  def _to_off(self, clear_buffer: bool = False) -> None:
    """OFF, keeping the approach memory (brake budget, cap latch, cooldown, flicker history)."""
    self.state = OFF
    self.exit_t = 0.0
    self.recover_t = 0.0
    self.bridge_t = 0.0
    self.hold_t = 0.0
    self.gap_med = float("nan")
    if clear_buffer:
      self._buf.clear()

  def _end_approach(self) -> None:
    """The approach is over: forget the brake budget, the duration-cap latch and the flicker history."""
    self.brake_t = 0.0
    self.cap_latched = False
    self.latch_recover_t = 0.0
    self._last_expiry_t = None
    self._expiry_rearms.clear()

  def _full_reset(self, clear_buffer: bool = False) -> None:
    """OFF, and the approach is over (the gap recovered, the car stopped, the planner reset)."""
    self._to_off(clear_buffer)
    self._end_approach()

  # -- the hook ------------------------------------------------------------------------------------------------------
  def apply(self, sm, accel, v_ego, active):
    """Planner hook. With the param OFF / the dependency unmet it reads NO message field and returns ``accel`` itself
    (byte-identical). With it ON any exception passes ``accel`` through unmodified and latches the floor off."""
    try:
      self._poll()
      if not self.enabled:
        return accel
      car_state = sm['carState']
      radar = sm['radarState']
      lead = radar.leadOne
      present = bool(lead.present)
      d_rel = float(lead.dRel)
      lead_prob = float(lead.modelProb)     # a LeadData field: read directly; a missing field raises (fail closed)
      gas = bool(car_state.gasPressed)
      standstill = bool(car_state.standstill)
      radar_t = int(sm.logMonoTime['radarState']) * 1e-9
      now_t = int(sm.logMonoTime['modelV2']) * 1e-9
      radar_md_t = int(radar.mdMonoTime) * 1e-9
      return self._rule(float(accel), float(v_ego), bool(active), gas, present, d_rel, lead_prob, standstill,
                        radar_t, radar_md_t, now_t)
    except Exception:
      try:
        self._fault()
      except Exception:
        self.faulted = True
        self.enabled = False
      return accel

  # -- the rule ------------------------------------------------------------------------------------------------------
  def update(self, accel: float, v_ego: float, active: bool, gas_pressed: bool = False, lead_present: bool = False,
             d_rel: float | None = None, lead_prob: float = float("nan"), standstill: bool = False,
             radar_t: float | None = None, radar_md_t: float | None = None, now_t: float | None = None) -> float:
    """The rule with explicit inputs (tests / offline replay). ``lead_prob`` defaults to UNTRUSTED. ``radar_t`` /
    ``now_t`` (s) are the radarState and planner-frame times; ``None`` (a caller without message timing) counts every call
    as a new sample and never stale - the planner hook always passes the real times."""
    self._poll()
    if not self.enabled:
      return float(accel)
    return self._rule(float(accel), float(v_ego), bool(active), bool(gas_pressed), bool(lead_present), d_rel,
                      lead_prob, bool(standstill), radar_t, radar_md_t, now_t)

  def _rule(self, accel, v, active, gas_pressed, lead_present, d_rel, lead_prob, standstill, radar_t, radar_md_t, now_t):
    dt = self.dt
    self.t += dt

    # clocks. The launch grace (re)starts whenever the car is under 1 m/s, not only at a true standstill, so a crawl at
    # 0.5-0.9 m/s that never stops still gets the grace when it launches. It never releases an active floor.
    if standstill or v < V_RELEASE:
      self.launch_hold = LAUNCH_GRACE_S
    elif self.launch_hold > 0.0:
      self.launch_hold = max(0.0, self.launch_hold - dt)
    if self.rearm_t > 0.0:
      self.rearm_t = max(0.0, self.rearm_t - dt)

    # sample: ONE per new radarState message; a message older than STALE_RADAR_S is a lost lead
    new_msg = radar_t is None or radar_t != self._last_radar_t
    if radar_t is not None and new_msg:
      self._last_radar_t = radar_t
    stale = False
    if now_t is not None and radar_t is not None:
      stale = radar_t <= 0.0 or (now_t - radar_t) > STALE_RADAR_S
      if radar_md_t is not None:
        stale = stale or radar_md_t <= 0.0 or (now_t - radar_md_t) > STALE_RADAR_S
    prob = float(lead_prob) if lead_prob is not None else float("nan")
    prob_ok = math.isfinite(prob) and prob >= MIN_LEAD_PROB
    d_ok = d_rel is not None and math.isfinite(float(d_rel))
    usable = bool(lead_present) and d_ok and prob_ok and not stale
    if usable and new_msg:
      self._buf.append(float(d_rel))
    self.absent_t = 0.0 if usable else self.absent_t + dt
    if self.cap_latched and self.absent_t >= LATCH_FORGET_ABSENT_S - 1e-9:
      self._end_approach()                     # no usable lead for a while: whatever produced the phantom is gone

    # hard fail-safes: the planner / the driver own the frame
    if not active:
      self._full_reset(clear_buffer=not usable)
      return float(accel)
    if gas_pressed:
      if self.state != OFF:
        self.rearm_t = max(self.rearm_t, REARM_COOLDOWN_S)   # a driver override is a release
      self._to_off(clear_buffer=not usable)
      self._tick_off()
      return float(accel)

    if not usable:
      # LEAD DROPOUT / UNTRUSTED / STALE. Drop the brake at once, but keep the buffer and hold the pedal at zero for up
      # to LEAD_BRIDGE_S so a momentary loss does not hand the pedal back at a 2 m gap. From OFF: nothing to protect.
      if self.state in (BRAKE, HOLD, BRIDGE) and len(self._buf):
        if self.state != BRIDGE:
          self._bridge_from = self.state
          self.bridge_t = 0.0
        self.state = BRIDGE
        self.bridge_t += dt
        if self.bridge_t >= LEAD_BRIDGE_S - 1e-9:
          self._expire()
          return float(accel)
        return floor_ask(accel, BRIDGE, v, self._coast)
      self._to_off(clear_buffer=True)
      self._tick_off()
      return float(accel)

    self.bridge_t = 0.0
    if not len(self._buf):
      if self.state == OFF:
        self._tick_off()
      return float(accel)
    d = float(np.median(self._buf))
    self.gap_med = d

    if self.state == BRIDGE:
      # A lead back inside the bridge resumes BRAKE on the buffer it never lost - only if the bridge came from BRAKE. A
      # bridge from HOLD goes back to the hold (the release already happened; its cooldown keeps running).
      if (self._bridge_from == BRAKE and d < exit_gap(v) and V_RELEASE < v <= V_MAX and not self.cap_latched
          and self.brake_t < BRAKE_MAX_S - 1e-9):
        self.state = BRAKE
      elif self._bridge_from == BRAKE:
        self._enter_hold()                   # it came back beyond the exit band: a release
      else:
        self.state = HOLD
      self.exit_t, self.recover_t = 0.0, 0.0

    if self.state == BRAKE:
      if self.brake_t >= BRAKE_MAX_S - 1e-9:
        # THE DURATION CAP: a sustained false brake must not run the car down from 10 m/s. Zero-pedal hold (no brake)
        # for at most CAP_HOLD_MAX_S, then OFF; latched until the gap is seen to recover.
        self._enter_hold()
        self.cap_latched = True
        self.latch_recover_t = 0.0
      elif v > V_MAX or v <= V_RELEASE:
        self._enter_hold()                   # leaving the band never frees the ask, and starts the cooldown
      elif d >= exit_gap(v):
        self.exit_t += dt
        if self.exit_t >= EXIT_DWELL_S - 1e-9:
          self._enter_hold()
      else:
        self.exit_t = 0.0

    elif self.state == HOLD:
      self.hold_t += dt
      if v < V_STANDSTILL:
        self._full_reset()                   # a stopped car is the standstill/launch logic's (the grace covers it)
      elif d >= recover_gap(v):
        self.recover_t += dt
        if self.recover_t >= RECOVER_DWELL_S - 1e-9:
          self._full_reset()                 # the gap really recovered: the approach is over
      else:
        self.recover_t = 0.0
      if self.state == HOLD and self.cap_latched and self.hold_t >= CAP_HOLD_MAX_S - 1e-9:
        self._to_off()                       # the capped approach is handed back; the latch stays
        self.rearm_t = max(self.rearm_t, REARM_COOLDOWN_S)
      if self.state == HOLD and self._may_arm(v, d):
        self._enter_brake()                  # the gap collapsed again after the cooldown: a second genuine approach

    if self.state == OFF:
      self.exit_t = self.recover_t = 0.0
      if self.cap_latched:
        if v < V_STANDSTILL:
          self._end_approach()               # the car stopped: the capped approach is over
        elif d >= recover_gap(v):
          self.latch_recover_t += dt
          if self.latch_recover_t >= RECOVER_DWELL_S - 1e-9:
            self._end_approach()             # the gap was really seen to recover
        else:
          self.latch_recover_t = 0.0
      if self._may_arm(v, d):
        self._enter_brake()
      else:
        self._tick_off()

    if self.state == BRAKE:
      self.brake_t += dt                     # every frame that publishes the brake ask spends the budget
    return floor_ask(accel, self.state, v, self._coast)

  def _may_arm(self, v: float, d: float) -> bool:
    return (not self.cap_latched and self.launch_hold <= 0.0 and self.rearm_t <= 0.0
            and len(self._buf) >= MIN_SAMPLES and V_MIN < v <= V_MAX and d < enter_gap(v)
            and self.brake_t < BRAKE_MAX_S - 1e-9)

  def _tick_off(self) -> None:
    self.off_t += self.dt
    if self.off_t >= BUDGET_FORGET_S - 1e-9 and not self.cap_latched:
      self.brake_t = 0.0

  def _enter_brake(self) -> None:
    if self._last_expiry_t is not None and (self.t - self._last_expiry_t) <= FLICKER_WINDOW_S + 1e-9:
      self._expiry_rearms.append(self.t)     # a re-arm after a dropout expiry: not cooldown-gated, but counted
    self.state = BRAKE
    self.exit_t = self.recover_t = 0.0
    self.off_t = 0.0

  def _enter_hold(self) -> None:
    self.state = HOLD
    self.rearm_t = max(self.rearm_t, REARM_COOLDOWN_S)
    self.exit_t, self.recover_t, self.hold_t = 0.0, 0.0, 0.0

  def _expire(self) -> None:
    """A dropout outlasted the bridge: a fail-safe stop (OFF, buffer cleared). Not cooldown-gated - unless the lead has
    been flickering: FLICKER_REARMS expiry re-arms inside FLICKER_WINDOW_S make this expiry start the cooldown."""
    while self._expiry_rearms and (self.t - self._expiry_rearms[0]) > FLICKER_WINDOW_S + 1e-9:
      self._expiry_rearms.popleft()
    if len(self._expiry_rearms) >= FLICKER_REARMS:
      self.rearm_t = max(self.rearm_t, REARM_COOLDOWN_S)
    self._last_expiry_t = self.t
    self._to_off(clear_buffer=True)
    self.off_t = 0.0
