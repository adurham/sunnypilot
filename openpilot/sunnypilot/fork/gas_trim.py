"""
Fork: gas-side integral trim for the steady-state acceleration error (adurham/sunnypilot).

Why this exists
---------------
`carParams.longitudinalTuning` on this car (2022 Elantra N, `HYUNDAI_ELANTRA_2022_NON_SCC`, openpilot longitudinal via
a pedal interceptor) arrives as **kiV = [0] and kf = 0**: no integrator and no feedforward, so the loop is
proportional-only and a standing offset between the ask and what the car delivers is never integrated out. Measured on
the owner's own 3.49 h of engaged driving (ask flat for 1.5 s, delivered measured 0.5-1.5 s later, driver's feet off):

    10-15 m/s   ask +0.514 -> 0.358 m/s^2 delivered   (0.70)
    15-20 m/s   ask +0.503 -> 0.377                   (0.75)
    25-30 m/s   ask +0.280 -> 0.114                   (0.41)
    30-40 m/s   ask +0.175 -> 0.146                   (0.84)

Braking tracks near 1.0 because the ESC closes its own decel loop; the gas side does not. The pedal is not the
limiter: openpilot's own pedal peaks at 53 % where the driver's reaches 97 %, and at matched speed and matched pedal
the driver and openpilot achieve the same acceleration.

What it does
------------
    err    = a_target - a_ego
    trim  += KI * err * dt                      clamped to [0, TRIM_MAX]
    out    = a_target + ramp_in(trim)           gas side only

It stands down wherever the answer would be dishonest or unsafe:

  - **never below V_MIN (10 m/s)**: the baseline over-delivers below 5 m/s and is only 0.86 at 5-10, so there is
    nothing to trim there - and a stop is exactly where an integrator would wind up on a stationary car.
  - **never with the driver on the pedal**: he owns the throttle.
  - **never while `no_close` is capping the ask** (`cap_factor < 1`): that cap is the owner's standing rule that the
    car may not accelerate toward a lead, and the trim must not become a bypass for it. Freezing there also stops the
    integrator winding up against an ask the cap is about to zero anyway.
  - **never at the platform ceiling** (ask already >= A_CEIL).
  - a zero or negative ask passes through bit-for-bit: a brake frame is identical to stock.

`ramp_in` admits the trim at RATE m/s^2 per second and bleeds it out at the same rate, so neither a brake release nor
an arm change presents a step to the interceptor. Any non-active frame resets it: the state belongs to one engagement.

A/B (`GasTrimAB`, so a normal drive validates it without a scripted route): the arm alternates every AB_BLOCK_FRAMES
with a random start bit per planner start, so one drive carries both arms over the same roads. Read the arm per frame
from the logged `arm_effective`, never from the segment number (initData params are a once-per-route snapshot).

Inert unless `GasTrim` is set. A libparams that predates the key raises UnknownKeyName and the feature stays OFF, which
is the deployed behaviour - and `key_known` says which of the two happened.
"""
from openpilot.common.realtime import DT_MDL

import numpy as np

PARAM = "GasTrim"
PARAM_AB = "GasTrimAB"

KI = 0.3          # 1/s -- ~5 s to close a step against a 0.5 proportional loop
TRIM_MAX = 0.5    # m/s^2 -- gas side only, small enough that a wrong sign cannot become a surge
V_MIN = 10.0      # m/s -- nothing to trim below this, and it keeps the integrator away from stops
A_CEIL = 1.9      # m/s^2 -- do not integrate while the ask is already at the platform ceiling
RATE = 0.5        # m/s^2 per second -- admit and bleed rate at every boundary
AB_BLOCK_FRAMES = 1200   # 60 s at 20 Hz: >= 7 tau in the worst band


class GasTrim:
  """Owned by LongitudinalPlanner. ``update`` returns the (possibly trimmed) final ask."""

  def __init__(self, params=None, dt: float = DT_MDL, ab_start_bit: int | None = None):
    self.dt = dt
    self._params = params
    self.enabled = False
    self.ab_enabled = False
    self.key_known = False
    self.trim = 0.0          # m/s^2, the integrator state
    self.applied = 0.0       # m/s^2, the ramped-in part (diagnostics and tests)
    self.arm_wanted = 1
    self.arm_effective = 0
    self.frame = 0
    self.gas_frames = 0
    self._start_bit = int(ab_start_bit) if ab_start_bit is not None else int(np.random.randint(0, 2))

  def update_params(self) -> bool:
    """Read both flags. A libparams that predates the key raises UnknownKeyName: the feature stays OFF, which is
    exactly the deployed behaviour. Same rule as fork/no_close.py; `key_known` reports which happened."""
    try:
      if self._params is None:
        from openpilot.common.params import Params
        self._params = Params()
      new = bool(self._params.get_bool(PARAM))
      try:
        new_ab = bool(self._params.get_bool(PARAM_AB))
      except Exception:
        new_ab = False
      self.key_known = True
    except Exception:
      new, new_ab, self.key_known = False, False, False
    changed = (new != self.enabled) or (new_ab != self.ab_enabled)
    self.enabled, self.ab_enabled = new, new_ab
    if not new:
      self.reset()
    return changed

  def reset(self) -> None:
    self.trim = 0.0
    self.applied = 0.0
    self.arm_effective = 0

  def update(self, a_target: float, a_ego: float, v_ego: float, driver_pedal: bool, active: bool,
             cap_factor: float = 1.0) -> float:
    self.frame += 1

    if not self.enabled or not active:
      self.reset()
      return a_target

    if self.ab_enabled:
      # arm 1 = ON. Block 0 is ON when the start bit is 0; the bit flips which arm a drive starts on.
      self.arm_wanted = 1 - int((((self.frame - 1) // AB_BLOCK_FRAMES) + self._start_bit) & 1)
    else:
      self.arm_wanted = 1

    gas_side = (self.arm_wanted == 1
                and a_target > 0.0
                and not driver_pedal
                and v_ego >= V_MIN
                and a_target < A_CEIL
                and cap_factor >= 1.0)

    if gas_side:
      self.gas_frames += 1
      self.trim += KI * (a_target - a_ego) * self.dt
      self.trim = float(min(max(self.trim, 0.0), TRIM_MAX))
      self.applied = min(self.trim, self.applied + RATE * self.dt)
      self.arm_effective = 1
      return a_target + self.applied

    # Not gas side: the ask is untouched (bit-for-bit), and the applied part bleeds out at RATE so a brake frame,
    # a capped frame or an arm change is never a step.
    self.applied = max(0.0, self.applied - RATE * self.dt)
    self.arm_effective = 0
    return a_target
