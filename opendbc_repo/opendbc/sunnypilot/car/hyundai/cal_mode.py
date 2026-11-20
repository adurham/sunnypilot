"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Fork (adurham): FCA11-long CALIBRATION command path (patch 0029
``0029-hyundai-fca11-cal-command-mode``). NOT a road feature and NOT a second switch.

WHAT THIS IS
------------
The one calibration drive that pins the ESC brake constants (BITE / K in
``FCA11_ESC_BITE`` / ``FCA11_ESC_K``) needs the FCA11-long brake to be commanded at EXACT, scripted
levels, independent of whatever the planner happens to ask. Today FCA11-long only actuates when the
PLANNER asks for braking, so there is no way to hold a clean 10 LSB step for a BITE fit.

This module replaces the *source* of the FCA11 decel command (normally the planner ask) with an exact
level while every production gate stays on. It is a pure CAR-LAYER Python change: no safety model, no
firmware, no panda change (the panda polices every frame exactly as it does for the planner's asks).

THE SINGLE-SWITCH MODEL (owner directive)
-----------------------------------------
Cal mode is SUBORDINATE to the production FCA11 toggle. The whole cal path lives INSIDE the
``if not self.enabled: return`` guard of ``Fca11LongBrake.update`` (``self.enabled = CP_SP.fca11Brake
and CP_SP.enableGasInterceptor``). There is **no separate cal enable param and no bypass**: if the
owner's ``HyundaiFca11Brake`` UI toggle is OFF, ``self.enabled`` is False, ``update`` returns
byte-for-byte before the cal code is ever reached, and the state/plan file alone can NEVER actuate.
The owner turns the FCA11 toggle off to kill the whole subsystem (everyday FCA11 braking AND cal)
instantly, with no file cleanup. That is the intended single-switch model.

AUTONOMOUS SEQUENCER (owner directive: "I don't need YOU arming it every time")
------------------------------------------------------------------------------
There is NO per-session external arming and no SSH/runner in the actuation path. After a ONE-TIME
setup (deploy this patch, write ``/data/fca11-cal/plan.json`` once), acquisition is autonomous: a
small state machine ticked from the car layer at 100 Hz reads the plan and executes due reps
opportunistically, appending progress to ``/data/fca11-cal/``. The owner just drives; the FCA11
toggle is the stop switch; progress persists across reboots and is self-terminating when the plan
is complete. ``fca11_cal_runner.py`` (superproject) is only a ``--dry-run`` pacing validator and a
read-only progress monitor -- it is NOT required for actuation.

GATE + SAFETY
-------------
* Plan absent / malformed / expired / TTL stale / rev unknown  -> INERT (byte-identical behaviour).
* The cal level is clamped to the PANDA gate ``HYUNDAI_FCA11_LONG_MAX_DEC`` (30 LSB = 0.30 g), NOT
  the personality cap -- cal must reach 0.30 g regardless of the driver's feel dial. It is still
  <= the panda's personality-agnostic gate, so NO new authority.
* The +``HYUNDAI_FCA11_LONG_RATE_STEP`` (+4 LSB) per SENT frame limit is respected: a step to any
  level enters through the panda-rate ramp (the production hard-ask-style clamp: each SENT frame
  grows by at most +4 LSB over the last SENT frame). This yields the CLEAN STEP the BITE fit needs.
* Every production veto still applies on every frame (see ``Fca11LongBrake.update``): the cal
  episode is opened ONLY when ``fca11_ok()`` is True (engaged + heartbeat via the panda, gear D,
  speed floor, camera not owning, driver-cut latches clear, pedal not faulted) and it is dropped
  the instant any of them fires, exactly like a planner ask.
* HOLD BOUND (0035): the panda has NO actuation-duration budget any more, so the TOOL bounds its own
  holds where the bound belongs: a rep whose ``hold_s`` exceeds ``CAL_MAX_HOLD_S`` (10 s) is REFUSED
  (never armed), and ``Fca11LongBrake`` runs an independent WATCHDOG that drops any scripted ask held
  longer than ``CAL_WATCHDOG_FRAMES`` and forces the production release (passive close frame).
* Release goes through the PRODUCTION release path unchanged (the fade, the passive frame) so the
  release tests fit the shipped shaping.
* GAS NEUTRALIZATION: while a cal hold is active the planner sees the car slowing and asks for
  gas; a commanded gas press sets gas_pressed -> the panda's driver-cut latch fires and would cut
  the very episode we are measuring. The brake sets ``CS.fca11_cal_active`` and
  ``GasInterceptorCarController`` forces the commanded pedal to 0 while it is set -- replicating the
  production condition (when the planner asks brake it never gases). It can only REDUCE actuation
  (always safe) and it is inert when cal mode is off (the flag is False).
* NO NEW AUTHORITY: the command envelope (<= 30 LSB, Warn=3 actuating shape, 50 Hz window, the
  same camera-mirror bytes) is a strict subset of what the production planner can already command.

DRIVER CONSENT -- "TAP TO FIRE" (patch 0033)
--------------------------------------------
A rep is a real, 2.2 s scripted brake on a live road. The reference-model review ranked
DRIVER-CONSENT-PER-REP the #1 mitigation for the rear-approach risk: it converts an uninitiated
robot brake into a driver-INITIATED one. Consent is ADDITIVE and never weakens any existing veto
(the production gates, the 0035 hold cap, the 0032 blind-spot veto): it can only ADD a pending
state and a tap requirement, never remove one.

``conditions.consent_mode`` (or a top-level ``consent`` object) selects the policy:
  * ``off``  -- today's fully-autonomous behaviour. DEFAULT for backward compatibility. This is the
                ONLY mode that actuates without a tap.
  * ``all``  -- every rep needs a tap.
  * ``high`` -- only reps at/above ``consent.min_level_lsb`` (default 22 = 0.22 g) need a tap; the
                lower-g reps stay automatic.
A MISSING mode is ``off`` (compat); a PRESENT-but-unrecognised mode coerces to ``all`` (the strictest
non-off mode), so a typo can never silently re-enable autonomous actuation (fail-closed).

When a rep is DUE and every condition passes and consent is required for it, the sequencer does NOT
actuate: it PARKS the rep (``pending``) and writes ``/data/fca11-cal/pending.json`` for the UI to show
a prompt. The owner taps; the UI writes ``/data/fca11-cal/consent.json`` = {rep_id, ts, accept}. The
sequencer consumes that file and, ONLY if it is FRESH (<= 2 s), bound to THIS rep_id, and accepted,
RE-VERIFIES every condition (and the hold cap) at the instant of the tap and then fires. Anything else
-- absent, malformed, stale, mismatched, non-accepted -- NEVER actuates. A parked rep EXPIRES after
``consent.timeout_s`` (default 8 s) if untouched and is retried later; an explicit dismiss (accept =
false) is honoured for the rest of the session. The pending file is ADVISORY (it only drives the
prompt); the consent file is the sole authority and it is fail-closed.

FILES (all under ``/data/fca11-cal/``; the module never touches anything else)
------------------------------------------------------------------------------
* ``plan.json``      -- written ONCE (per revision) by the operator. The immutable matrix.
* ``progress.json``  -- written by THIS module. Completed reps (idempotent across reboots).
* ``reps.jsonl``     -- appended by THIS module. One JSON record per completed/interrupted rep.
"""
import hashlib
import json
import os
import time

# --- the exact constants the panda enforces for FCA11-long (values.py mirrors safety/modes/hyundai.h).
# Duplicated here (not imported) so this module has no import cycle with fca11_long.py; the unit tests
# pin them against the shipped values.
CAL_MAX_LSB = 30           # HYUNDAI_FCA11_LONG_MAX_DEC: the PANDA gate (personality-agnostic)
CAL_MIN_LSB = 2            # tool-side LOWER clamp on a scripted rep's command level, in LSB
#   (see Rep.__init__: level = max(CAL_MIN_LSB, min(CAL_MAX_LSB, level))). It bounds the CAL
#   TOOL's own plan, not the car layer and not the panda. NOTE: an earlier, never-shipped
#   brake-shaping patch had a similarly-named constant, FCA11_DEC_ACTIVE_FLOOR (also 2), and
#   this label used to borrow that name. That constant is a different thing (an active floor
#   inside the brake-shaping law) and does not exist in this tree. Do not conflate the two.
CAL_RATE_STEP_LSB = 4      # HYUNDAI_FCA11_LONG_RATE_STEP: +LSB per SENT frame ceiling

# --- 0035: TOOL-SIDE hold bound. 0031's arming predicate only existed to fit a rep inside the panda's
# 2.5 s budget; that budget is deleted (0035), so arming is budget-free and the bound lives HERE:
#   * CAL_MAX_HOLD_S: a rep whose hold exceeds it is REFUSED (never armed, never prompted). The shipped
#     matrix holds 2.2 s; 10 s leaves room for longer reps without letting a plan typo hold a brake.
#   * CAL_WATCHDOG_FRAMES: Fca11LongBrake drops a scripted ask that has been up longer than this
#     (independent of the sequencer's own hold arithmetic) and forces the production release (passive
#     close frame). 20 frames (0.2 s) of slack over the cap: it can never cut a legal hold.
CAL_MAX_HOLD_S = 10.0
CAL_WATCHDOG_MARGIN_FRAMES = 20
CAL_WATCHDOG_FRAMES = int(round(CAL_MAX_HOLD_S * 100.0)) + CAL_WATCHDOG_MARGIN_FRAMES

# --- 0032: the blind-spot veto hold-off. A rep is a 2.2 s scripted brake hold on a live road; the
# rear quarter radars (LCA11 CF_Lca_IndLeft/IndRight -> carState.leftBlindspot/rightBlindspot) can
# show an approaching vehicle AFTER the current frame looked clear. Holding the veto for a few
# seconds past the last set sample makes the decision depend on a recent window, not one frame.
#
# *** ONE-WAY SEMANTICS (do not weaken) *** : a SET bit means "something IS in that rear quarter"
# -> skip. A CLEAR bit NEVER means "the rear quarter is verified empty" -- it only means "no
# blind-spot sample in the retained window was set". The veto can only ADD a skip; it makes no
# claim about the rear being clear, and NO log/UI wording may imply otherwise.
CAL_BLINDSPOT_HOLDOFF_FRAMES = 500   # default 5.0 s @ 100 Hz -- the FCA11_LONG-style constant
                                    # (plan override ``conditions.blindspot_holdoff_s`` wins)

# --- defaults (all overridable in plan.json)

# --- 0033: driver consent ("tap to fire"). A rep is a real 2.2 s scripted brake on a live road; the
# review ranked driver-consent-per-rep the #1 mitigation for the rear-approach risk. Consent is a
# SEPARATE, ADDITIVE gate: it can only ADD a pending/skip, never remove a veto. FAIL-CLOSED by
# construction -- "off" (the compat default) is the ONLY mode that actuates without a tap, and a
# present-but-unrecognised mode coerces to the strictest non-off mode ("all").
CONSENT_OFF = "off"
CONSENT_ALL = "all"
CONSENT_HIGH = "high"
CONSENT_MODES = (CONSENT_OFF, CONSENT_ALL, CONSENT_HIGH)

# --- defaults (all overridable in plan.json)
DEFAULT_TTL_S = 900.0              # a plan is inert if not refreshed/valid within this many seconds of "now"
DEFAULT_EXPIRES_DAYS = 30.0        # optional plan expiry
DEFAULT_PER_DRIVE_CAP = 15         # soft cap on reps per drive (<= 0 = unlimited)
DEFAULT_MIN_LEAD_M = 60.0          # skip a rep when a lead is within this distance (closing or not)
DEFAULT_TURN_LAT_MAX = 0.5         # m/s^2: |lateral accel| above this = not near-straight, skip
DEFAULT_STRAIGHT_MIN_S = 2.0       # the straight condition must hold this long
DEFAULT_BAND_TOL_KPH = 5.0         # band window half-width
DEFAULT_BAND_STABLE_S = 5.0        # the band must be held this long before a rep may start
DEFAULT_HOLD_S = 2.2               # hold length (the shipped matrix; <= CAL_MAX_HOLD_S)
DEFAULT_RECOVERY_S = 3.5           # release -> next rep gap (band recovery; no panda cooldown since 0035)
# 0032 blind-spot veto: FAIL-CLOSED DEFAULT ON. An old plan that predates 0032 therefore still gets
# the veto (one-way information; absence of the key must not silently disable a safety addition).
DEFAULT_BLINDSPOT_VETO = True      # skip a rep while either rear-quarter blind-spot bit is set
DEFAULT_BLINDSPOT_HOLDOFF_S = 5.0  # hold the veto this long after the last SET sample
# 0033 consent defaults. mode "off" = the backward-compatible autonomous behaviour; min_level_lsb 22
# (0.22 g) = at/above this a rep is "high-g" and (in "high" mode) needs a tap; timeout_s 8 s bounds how
# long a prompt waits before it is dropped. A tap is honoured only within CONSENT_FRESHNESS_S (2 s) of
# its own timestamp, so a STALE file can never fire a later rep.
DEFAULT_CONSENT_MODE = CONSENT_OFF
DEFAULT_CONSENT_MIN_LEVEL_LSB = 22
DEFAULT_CONSENT_TIMEOUT_S = 8.0
CONSENT_FRESHNESS_S = 2.0
PLAN_HINT_TTL_S = 300.0            # a plan older than this is "stale": refuse to start a NEW rep

CAL_DIR = "/data/fca11-cal"
CAL_PLAN = os.path.join(CAL_DIR, "plan.json")
CAL_PROGRESS = os.path.join(CAL_DIR, "progress.json")
CAL_REPS_LOG = os.path.join(CAL_DIR, "reps.jsonl")

PHASE_STEPS = "steps"
PHASE_STAIRCASE = "staircase"
PHASE_RELEASES = "releases"
PHASES = (PHASE_STEPS, PHASE_STAIRCASE, PHASE_RELEASES)

_RELOAD_EVERY = 20          # re-read plan/progress at 5 Hz (100 Hz / 20), not every frame
_STAT_EVERY = 5             # stat the plan file at 20 Hz


def _to_float(v, default):
  try:
    x = float(v)
  except (TypeError, ValueError):
    return default
  return x if x == x else default  # rejects NaN


def _to_int(v, default):
  try:
    return int(v)
  except (TypeError, ValueError):
    return default


def _to_bool(v, default):
  """Bool coercion for plan flags. Only an EXPLICIT false-like value disables; a missing key (None)
  falls back to ``default``. This keeps the 0032 blind-spot veto fail-closed for plans that predate
  the key. Accepts real bools, JSON ints 0/1, and the usual string spellings."""
  if v is None:
    return default
  if isinstance(v, bool):
    return v
  if isinstance(v, (int, float)):
    return bool(v)
  if isinstance(v, str):
    s = v.strip().lower()
    if s in ("0", "false", "no", "off", ""):
      return False
    if s in ("1", "true", "yes", "on"):
      return True
  return default


def _to_consent_mode(v, default=CONSENT_OFF):
  """0033 consent-mode coercion. A MISSING mode -> ``default`` ("off", the compat behaviour). A
  PRESENT but unrecognised mode -> ``all`` (the strictest non-off mode): a typo must never silently
  re-enable autonomous actuation. Fail-closed in one direction only."""
  if v is None:
    return default
  s = str(v).strip().lower()
  if s in CONSENT_MODES:
    return s
  return CONSENT_ALL


def _to_consent_min_level(v, default):
  """0033 min-level coercion. MISSING -> ``default`` (22). PRESENT but unparseable -> 0, i.e. in
  ``high`` mode EVERY rep needs a tap (the strictest reading): a broken value must not let sub-22
  reps slip through tap-free."""
  if v is None:
    return default
  try:
    return int(v)
  except (TypeError, ValueError):
    return 0


def consent_opts(cond: dict, obj: dict) -> dict:
  """0033 consent config. Reads the top-level ``consent`` object if present, else the flat keys in
  ``conditions`` (``consent_mode`` / ``consent_min_level_lsb`` / ``consent_timeout_s``)."""
  top = obj.get("consent") if isinstance(obj.get("consent"), dict) else {}
  return {
    "consent_mode": _to_consent_mode(top.get("mode", cond.get("consent_mode")), DEFAULT_CONSENT_MODE),
    "consent_min_level_lsb": _to_consent_min_level(top.get("min_level_lsb", cond.get("consent_min_level_lsb")),
                                                   DEFAULT_CONSENT_MIN_LEVEL_LSB),
    "consent_timeout_s": _to_float(top.get("timeout_s", cond.get("consent_timeout_s")),
                                   DEFAULT_CONSENT_TIMEOUT_S),
  }


def hold_ok(hold_s) -> tuple[bool, str]:
  """0035 TOOL-SIDE HOLD BOUND (replaces the 0031 budget predicate). A rep may be armed only if its
  hold is a positive, finite number of seconds no longer than ``CAL_MAX_HOLD_S``. Anything else is
  refused -- never armed, never prompted -- with the reason surfaced in ``status()``."""
  h = _to_float(hold_s, None)
  if h is None or not (0.0 < h <= CAL_MAX_HOLD_S):
    return False, f"hold_over_cap(hold_s={hold_s},cap={CAL_MAX_HOLD_S})"
  return True, ""


def plan_id(obj: dict) -> str:
  """Stable id over the rep matrix + revision. A NEW plan revision resets all progress, as required."""
  reps = obj.get("reps") or []
  key = json.dumps({"rev": obj.get("rev", 1), "reps": reps}, sort_keys=True, separators=(",", ":"))
  return hashlib.sha256(key.encode()).hexdigest()[:16]


class CalRep:
  __slots__ = ("id", "band_kph", "level_lsb", "hold_s", "phase", "level2_lsb", "switch_at_s")

  def __init__(self, rid: str, band_kph: float, level_lsb: int, hold_s: float, phase: str,
               level2_lsb=None, switch_at_s=None):
    self.id = rid
    self.band_kph = band_kph
    self.level_lsb = max(CAL_MIN_LSB, min(CAL_MAX_LSB, int(level_lsb)))
    self.hold_s = hold_s
    self.phase = phase
    # optional MID-HOLD STEP (the contract's partial-release test, e.g. 0.20 -> 0.10 g): after
    # switch_at_s into the hold the command drops to level2_lsb. A downward step emits immediately
    # (the panda rate limit only bounds UPWARD growth), so it is a clean step-down.
    self.level2_lsb = (max(CAL_MIN_LSB, min(CAL_MAX_LSB, int(level2_lsb))) if level2_lsb is not None else None)
    self.switch_at_s = (_to_float(switch_at_s, None) if level2_lsb is not None else None)


def parse_plan(obj) -> tuple[list[CalRep], dict] | None:
  """Validate a plan dict -> (reps, opts). Returns None for anything malformed (caller stays INERT).

  A plan with no usable reps is treated as malformed/complete and yields an EMPTY rep list, which is
  inert. Never raises.
  """
  if not isinstance(obj, dict):
    return None
  reps_raw = obj.get("reps")
  if reps_raw is None or not isinstance(reps_raw, list):
    return None
  reps: list[CalRep] = []
  for i, r in enumerate(reps_raw):
    if not isinstance(r, dict):
      return None
    band = _to_float(r.get("band_kph"), None)
    level = _to_int(r.get("level_lsb"), None)
    if band is None or level is None:
      return None
    phase = str(r.get("phase", PHASE_STEPS))
    if phase not in PHASES:
      phase = PHASE_STEPS
    reps.append(CalRep(str(r.get("id", f"rep{i}")), band, level,
                       _to_float(r.get("hold_s"), DEFAULT_HOLD_S), phase,
                       level2_lsb=r.get("level2_lsb"), switch_at_s=r.get("switch_at_s")))
  cond = obj.get("conditions") if isinstance(obj.get("conditions"), dict) else {}
  opts = {
    "ttl_s": _to_float(obj.get("ttl_s"), DEFAULT_TTL_S),
    "expires_at": _to_float(obj.get("expires_at"), None),
    "per_drive_cap": _to_int(obj.get("per_drive_cap"), DEFAULT_PER_DRIVE_CAP),
    "min_lead_m": _to_float(cond.get("min_lead_m"), DEFAULT_MIN_LEAD_M),
    "turn_lat_max": _to_float(cond.get("turn_lat_max"), DEFAULT_TURN_LAT_MAX),
    "straight_min_s": _to_float(cond.get("straight_min_s"), DEFAULT_STRAIGHT_MIN_S),
    "band_tol_kph": _to_float(cond.get("band_tol_kph"), DEFAULT_BAND_TOL_KPH),
    "band_stable_s": _to_float(cond.get("band_stable_s"), DEFAULT_BAND_STABLE_S),
    "recovery_s": _to_float(cond.get("recovery_s"), DEFAULT_RECOVERY_S),
    # 0032 blind-spot veto: default ON (fail-closed). ``blindspot_veto`` is a bool; anything that is
    # not an explicit false (incl. a missing key) leaves the veto ON. Hold-off seconds -> frames.
    "blindspot_veto": _to_bool(cond.get("blindspot_veto"), DEFAULT_BLINDSPOT_VETO),
    "blindspot_holdoff_s": _to_float(cond.get("blindspot_holdoff_s"), DEFAULT_BLINDSPOT_HOLDOFF_S),
    # 0033 consent (tap-to-fire). Fail-closed: missing mode -> "off"; unrecognised -> "all".
    **consent_opts(cond, obj),
  }
  return reps, opts


def _read_json(path: str) -> dict | None:
  try:
    with open(path) as f:
      return json.load(f)
  except (OSError, ValueError):
    return None


def _write_json_atomic(path: str, doc: dict) -> None:
  """Best-effort atomic write. A log/progress write must NEVER take card down."""
  tmp = path + ".tmp"
  try:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(tmp, "w") as f:
      json.dump(doc, f, sort_keys=True)
    os.replace(tmp, path)
  except OSError:
    pass


def write_consent(cal_dir: str, rep_id, plan_id, seq, accept: bool, now: float | None = None) -> None:
  """0034: write the SAME ``consent.json`` the mici UI writes on a tap, so the STEERING-WHEEL gesture
  and the TOUCH path share ONE channel, ONE schema and ONE set of binding/freshness rules. This is the
  byte-identical twin of the widget's ``CalConsentWidget._write_consent``; the owner may fire a rep by
  either route and the sequencer cannot tell the two apart (it only ever reads the file).

  Atomic (tmp + ``os.replace``). The sequencer is the sole consumer: it re-binds on ``rep_id`` +
  ``plan_id`` + prompt ``seq``, re-checks ``CONSENT_FRESHNESS_S`` and RE-VERIFIES every condition at the
  instant it reads this. A failed write simply leaves the rep pending -- it can never actuate."""
  doc = {"rep_id": rep_id, "plan_id": plan_id, "seq": seq,
         "ts": (time.time() if now is None else now), "accept": bool(accept)}
  _write_json_atomic(os.path.join(cal_dir, "consent.json"), doc)


class CalSequencer:
  """In-car cal state machine. Ticked once per 100 Hz controller frame from ``Fca11LongBrake.update``
  (inside the ``enabled`` branch, so it is structurally dead when the FCA11 toggle is off).

  Reads ``plan.json`` + ``progress.json``; decides which rep is due NOW (engaged + band stable + feet
  off + no lead + near-straight + no rear-quarter blind spot + not expired + drive cap not hit);
  commands the level (the brake's ``update`` consumes ``level_lsb``/``active_actuation``); on the hold
  elapsing it releases and, after the recovery gap + band recovery, records the rep and moves on. Progress
  keys on ``plan_id`` so a new plan revision resets completion; a completed rep is never re-run.
  """

  def __init__(self, dir_path: str = CAL_DIR):
    self.dir = dir_path
    self.plan_path = os.path.join(dir_path, "plan.json")
    self.progress_path = os.path.join(dir_path, "progress.json")
    self.log_path = os.path.join(dir_path, "reps.jsonl")

    # ---- public command surface consumed by Fca11LongBrake.update ----
    self.active_actuation = False   # True while a cal hold should be commanded
    self.level_lsb = 0              # the exact cal level (LSB)

    # ---- plan / progress ----
    self.plan: list[CalRep] = []
    self.opts: dict = {}
    self.pid = ""
    self.plan_mtime = -1.0
    self._plan_ok = False
    self._done: dict = {}
    self._loaded_at = -1e9
    self._stat_at = -1e9

    # ---- run state ----
    self._phase_enabled: tuple[str, ...] = PHASES
    self._frame = 0
    self._session = 0            # bumped per process start (the per-drive cap counts within a session)
    self._session_reps = 0
    self._active: CalRep | None = None
    self._hold_frames = 0
    self._on_start = -1          # frame the current rep's actuation opened on
    self._rep_braked = False     # 0036: the brake actually commanded an actuating frame during this rep's hold
    self._release_frame = -1     # frame braking last went False
    self._last_outcome = ""
    self._vetoed = False         # a production veto suppressed a frame of the current hold
    self._band_since: dict[float, int] = {}
    self._v_hist: list[tuple[int, float]] = []
    self._lat_hist: list[tuple[int, float]] = []
    self._yaw_hist: list[tuple[int, float]] = []
    self._block_reason = "no_plan"
    self._last_act_frame = -1
    # ---- 0035: tool-side hold bound diagnostics (the 0031 budget mirror is gone) ----
    self._last_hold_reason = ""      # why the last over-cap rep was refused
    self._watchdog_skip: set[str] = set()   # reps the watchdog aborted this session (never re-armed)
    # ---- 0032: blind-spot veto hold-off ------------------------------------------------
    # ``_blindspot_last_set`` is the frame of the most recent SET sample in EITHER rear quarter;
    # the veto is held for ``blindspot_holdoff_s`` after it. -1 means "no set sample this session".
    # Reset alongside the rest of the sequencer state (on a new plan/session), NOT carried across.
    self._blindspot_last_set = -1
    self._blindspot_vetoed = False   # True if the veto is what blocked arming on the last tick
    self._last_pid = ""              # 0032: last plan_id seen, to reset the hold-off on a new revision
    # ---- 0033: driver consent ("tap to fire") ------------------------------------------
    # ``_pending`` is the rep parked awaiting a tap (or None). ``_consent_skip`` is the set of rep ids
    # the owner DISMISSED this session (never re-prompted until the next session). ``_consent_rerprompt_
    # after`` gates re-prompting an EXPIRED rep so it is not asked on every tick.
    self._pending: CalRep | None = None
    self._pending_since = -1
    self._pending_seq = -1            # the prompt's sequence number (bind a tap to THIS prompt)
    self._consent_plan_id = ""        # the plan_id the parked prompt belongs to (bind a tap to it)
    self._consent_state = "idle"      # idle | pending | firing | expired | dismissed | refused | ...
    self._consent_reason = ""
    self._consent_seq = 0             # bumped when a NEW prompt is parked (the UI's change key)
    self._consent_skip: set[str] = set()
    self._consent_rerprompt_after: dict[str, int] = {}

  # ------------------------------------------------------------------ config
  def set_phases(self, phases) -> None:
    """Operator/test phase selector (``steps``/``staircase``/``releases``). Defaults to all."""
    if not phases:
      self._phase_enabled = PHASES
      return
    sel = tuple(p for p in PHASES if p in set(phases))
    self._phase_enabled = sel if sel else PHASES

  @property
  def phases(self) -> tuple[str, ...]:
    return self._phase_enabled

  def mark_session(self, session: int) -> None:
    """Called once per controller construction with a fresh process id, so ``per_drive_cap`` is
    counted per drive (the superproject restarts card per drive)."""
    self._session = int(session)
    self._session_reps = 0
    # 0032: a new session clears the blind-spot hold-off window (like the rest of the run state).
    self._blindspot_last_set = -1
    self._blindspot_vetoed = False
    # 0033: a new drive clears the consent run state (prompts, dismissals, re-prompt gates).
    self._pending = None
    self._pending_since = -1
    self._pending_seq = -1
    self._consent_plan_id = ""
    self._consent_state = "idle"
    self._consent_reason = ""
    self._consent_skip = set()
    self._consent_rerprompt_after = {}
    self._watchdog_skip = set()

  # ------------------------------------------------------------------ 0035 watchdog
  def watchdog_abort(self, frame: int) -> None:
    """0035: called by ``Fca11LongBrake`` when its independent watchdog finds a scripted ask held past
    ``CAL_WATCHDOG_FRAMES``. Drop the ask NOW (the brake's production release path emits the passive
    close frame), record the rep as ``watchdog`` (NOT ok, so it is not counted done), and never re-arm
    it this session. A trip means the sequencer's own hold arithmetic failed -- it must be loud."""
    rep = self._active
    self._active = None
    self.active_actuation = False
    self.level_lsb = 0
    self._block_reason = "watchdog"
    if rep is not None:
      self._watchdog_skip.add(rep.id)
      self._complete(rep, "watchdog", frame - self._on_start)

  # ------------------------------------------------------------------ gate
  def _reload_plan(self, now_wall: float, frame: int) -> None:
    if frame - self._stat_at < _STAT_EVERY and self._loaded_at > 0:
      return
    self._stat_at = frame
    try:
      mt = os.stat(self.plan_path).st_mtime
    except OSError:
      mt = -1.0
    if mt < 0:
      self.plan, self.opts, self.pid, self._plan_ok = [], {}, "", False
      self._block_reason = "no_plan"
      return
    self.plan_mtime = mt
    if frame - self._loaded_at < _RELOAD_EVERY and self._plan_ok:
      return
    self._loaded_at = frame
    obj = _read_json(self.plan_path)
    parsed = parse_plan(obj) if obj is not None else None
    if parsed is None:
      self.plan, self.opts, self.pid, self._plan_ok = [], {}, "", False
      self._block_reason = "malformed_plan"
      return
    self.plan, self.opts = parsed
    self.pid = plan_id(obj)
    # 0032: a NEW plan revision resets the blind-spot hold-off window (the same way a new plan resets
    # the other per-plan sequencer state); a plan whose rev has not changed keeps its window.
    if self.pid != self._last_pid:
      self._last_pid = self.pid
      self._blindspot_last_set = -1
      self._blindspot_vetoed = False
      # 0033: a new plan revision resets the consent run state too (fresh prompts, no stale skips).
      self._pending = None
      self._pending_since = -1
      self._pending_seq = -1
      self._consent_plan_id = ""
      self._consent_state = "idle"
      self._consent_reason = ""
      self._consent_skip = set()
      self._consent_rerprompt_after = {}
    # expiry (absolute) + issued_at staleness (relative)
    expires_at = self.opts.get("expires_at")
    if expires_at is not None and now_wall > expires_at:
      self._plan_ok = False
      self._block_reason = "plan_expired"
      return
    issued_at = _to_float(obj.get("issued_at"), now_wall)
    ttl = self.opts.get("ttl_s", DEFAULT_TTL_S)
    self._plan_ok = ttl > 0.0 and (now_wall - issued_at) <= ttl
    if not self._plan_ok:
      self._block_reason = "plan_stale_ttl"
      return
    self._block_reason = "plan_ok"
    # Completion is keyed by REP ID (persisted globally in progress.json), NOT by plan revision: a
    # reboot, or a new plan revision that re-lists an already-done rep, must NOT re-run it. A new
    # revision's NEW rep ids run; to force a deliberate re-run, give the rep a new id (or clear
    # progress.json). This is the idempotent gap-fill semantics (progress survives in /data/fca11-cal/).
    # A MALFORMED (present-but-unparseable) progress file is treated as INERT, never as "no progress":
    # silently ignoring it would re-run completed reps (a real hazard).
    progress_present = os.path.exists(self.progress_path)
    pr = _read_json(self.progress_path)
    if pr is None and progress_present:
      self._plan_ok = False
      self._block_reason = "progress_malformed"
      return
    self._done = (pr or {}).get("reps") or {}

  @property
  def plan_valid(self) -> bool:
    return self._plan_ok and len(self.plan) > 0

  @property
  def is_done(self) -> bool:
    return self.plan_valid and all(self._is_complete(r) for r in self.plan if r.phase in self._phase_enabled)

  # ------------------------------------------------------------------ helpers
  def _is_complete(self, rep: CalRep) -> bool:
    rec = self._done.get(rep.id)
    return bool(rec) and rec.get("outcome") == "ok"

  def _enabled_reps(self) -> list[CalRep]:
    return [r for r in self.plan if r.phase in self._phase_enabled]

  def _band_ready(self, band_kph: float) -> bool:
    tol = self.opts.get("band_tol_kph", DEFAULT_BAND_TOL_KPH)
    stable_s = self.opts.get("band_stable_s", DEFAULT_BAND_STABLE_S)
    since = self._band_since.get(float(band_kph))
    return since is not None and (self._frame - since) >= int(round(stable_s * 100.0))

  def _update_band_window(self, v_kph: float) -> None:
    tol = self.opts.get("band_tol_kph", DEFAULT_BAND_TOL_KPH)
    bands = {float(r.band_kph) for r in self._enabled_reps() if not self._is_complete(r)}
    for b in bands:
      if abs(v_kph - b) <= tol:
        self._band_since.setdefault(b, self._frame)
      else:
        self._band_since.pop(b, None)

  def _straight_ok(self) -> bool:
    """|lateral accel| under the limit, sustained. Lateral accel = v * yawRate (no grade/lateral model
    needed); falls back to the steering-rate signal if yawRate is absent."""
    lat_max = self.opts.get("turn_lat_max", DEFAULT_TURN_LAT_MAX)
    min_s = self.opts.get("straight_min_s", DEFAULT_STRAIGHT_MIN_S)
    need = int(round(min_s * 100.0))
    if need <= 0:
      return True
    hist = [(f, a) for f, a in self._lat_hist if self._frame - f <= need]
    if len(hist) < min(need, 50):
      return False
    return all(abs(a) <= lat_max for _, a in hist)

  def _direction(self) -> str:
    """Mean yaw-rate sign over the last ~2 s: sign convention (turning left = positive yaw) is not
    pinned here; it is a per-driver label for out-and-back pairing and the ANALYSIS flips it if the
    car's convention differs."""
    hist = [y for f, y in self._yaw_hist if self._frame - f <= 200]
    if not hist:
      return "straight"
    m = sum(hist) / len(hist)
    if abs(m) < 1e-3:
      return "straight"
    return "left" if m > 0 else "right"

  def _lead_ok(self, CS) -> bool:
    d = getattr(CS, "fca11_lead_drel_m", None)
    if d is None:
      return True
    return float(d) >= self.opts.get("min_lead_m", DEFAULT_MIN_LEAD_M)

  def _blindspot_ok(self, CS) -> bool:
    """0032 BLIND-SPOT VETO (one-way information). Read the ALREADY-DECODED rear-quarter radar bits
    ``CarState.leftBlindspot`` / ``CarState.rightBlindspot`` (hyundai carstate.py:184-185, from LCA11
    ``CF_Lca_IndLeft``/``CF_Lca_IndRight``, gated on ``enableBsm`` which is ON for this car; the raw
    CAN bits can be 1, 2 or 3), via the same ``CS.out.<field>`` access pattern the rest of the car
    layer uses for CarState fields. ANY non-zero sample in EITHER field is treated as SET.

    ONE-WAY SEMANTICS (do NOT weaken): a SET bit -> do not arm. A CLEAR bit NEVER certifies the rear
    quarter is empty -- it only means no sample in the retained window was set. This veto is the
    ONLY thing that looks behind the car; it can only ADD a skip and makes no claim of a clear rear.
    The veto is held ``blindspot_holdoff_s`` (default 5 s) past the most recent SET sample, so one
    clear frame cannot be mistaken for a clear quarter.
    """
    if not self.opts.get("blindspot_veto", DEFAULT_BLINDSPOT_VETO):
      self._blindspot_vetoed = False
      return True                                   # inert flag -> the pre-0032 path, byte-identical
    try:
      left = bool(getattr(CS.out, "leftBlindspot", False))
      right = bool(getattr(CS.out, "rightBlindspot", False))
    except (AttributeError, TypeError):
      left = right = False
    if left or right:                               # a SET sample refreshes the hold-off window
      self._blindspot_last_set = self._frame
      self._blindspot_vetoed = True
      return False
    hold = int(round(self.opts.get("blindspot_holdoff_s", DEFAULT_BLINDSPOT_HOLDOFF_S) * 100.0))
    if hold > 0 and self._blindspot_last_set >= 0 and (self._frame - self._blindspot_last_set) < hold:
      self._blindspot_vetoed = True                 # still inside the hold-off after the last set
      return False
    self._blindspot_vetoed = False
    return True

  # ------------------------------------------------------------------ 0033 consent
  def consent_required(self, rep: CalRep) -> bool:
    """0033: does THIS rep need a driver tap before it may actuate? ``off`` never does (the autonomous,
    backward-compatible path -- the ONLY tap-free mode). ``all`` always does. ``high`` only for reps
    at/above ``consent.min_level_lsb``. Anything that reaches here that is not exactly ``off`` can only
    ADD a requirement, never remove one (fail-closed)."""
    mode = self.opts.get("consent_mode", DEFAULT_CONSENT_MODE)
    if mode == CONSENT_OFF:
      return False
    if mode == CONSENT_ALL:
      return True
    return int(rep.level_lsb) >= int(self.opts.get("consent_min_level_lsb", DEFAULT_CONSENT_MIN_LEVEL_LSB))

  @property
  def pending(self) -> bool:
    """0033: True while a rep is parked awaiting a driver tap."""
    return self._pending is not None

  @property
  def pending_rep_id(self):
    return self._pending.id if self._pending is not None else None

  @property
  def pending_level_lsb(self) -> int:
    return int(self._pending.level_lsb) if self._pending is not None else 0

  def _pending_path(self) -> str:
    return os.path.join(self.dir, "pending.json")

  def _tap_path(self) -> str:
    return os.path.join(self.dir, "consent.json")

  def _write_pending(self, active: bool) -> None:
    """Publish the (ADVISORY) prompt state for the UI. Not in the actuation path -- the UI may read it
    late or not at all; the consent FILE below is the only thing that can fire a rep. ``seq`` uniquely
    identifies THIS prompt (bumped on every park) and must be echoed by the tap."""
    rep = self._pending
    live = bool(active and rep is not None)
    doc = {"pending": live,
           "rep_id": (rep.id if live else None),
           "level_lsb": (int(rep.level_lsb) if live else 0),
           "level_g": (round(int(rep.level_lsb) * 0.01, 2) if live else 0.0),
           "band_kph": (float(rep.band_kph) if live else 0.0),
           "seq": self._pending_seq if live else -1,
           "plan_id": (self.pid if live else ""),
           "ts": round(time.time(), 3)}
    _write_json_atomic(self._pending_path(), doc)

  def _clear_tap_file(self) -> None:
    try:
      os.remove(self._tap_path())
    except OSError:
      pass

  def _read_tap(self) -> tuple[bool, str]:
    """(accepted, reason) from the UI's consent file. FAIL-CLOSED: absent / malformed / mismatched
    (rep OR plan OR prompt-seq) / non-accepted / stale / untimestamped -> (False, reason). Never
    raises, never actuates on doubt. ``accept`` must be a REAL bool True (not 1/"true"/truthy)."""
    pending = self._pending
    if pending is None:
      return False, "no_pending"
    obj = _read_json(self._tap_path())
    if not isinstance(obj, dict):
      return False, "absent_or_malformed"
    if obj.get("rep_id") != pending.id:
      return False, "rep_mismatch"
    if obj.get("plan_id") != self.pid:
      return False, "plan_mismatch"
    if obj.get("seq") != self._pending_seq:
      return False, "seq_mismatch"
    accept = obj.get("accept")
    if accept is False:
      return False, "not_accepted"              # an EXPLICIT dismiss (accept:false)
    if not (type(accept) is bool and accept is True):
      return False, "bad_accept"                # present but not a real bool True -> keep waiting
    ts = _to_float(obj.get("ts"), None)
    if ts is None:
      return False, "no_ts"
    if abs(time.time() - ts) > CONSENT_FRESHNESS_S:
      return False, "stale"
    return True, "accepted"

  def wheel_gesture(self, gesture: str, frame: int) -> str:
    """0034: act on a STEERING-WHEEL consent gesture recognised by the CarState-phase detector. ``gesture``
    is ``"fire"`` (double-press UP) or ``"dismiss"`` (double-press DOWN). It writes the EXACT consent.json
    the mici UI writes -- bound to the rep parked RIGHT NOW (rep_id + plan + prompt seq), with ``accept``
    True for fire / False for dismiss -- so the STEERING WHEEL and the TOUCH path share ONE channel, ONE
    schema and ONE set of rules. This is a SECOND WRITER of the one 0033 channel, never a second channel:
    the sequencer consumes the file on its next tick and applies the SAME freshness window, the SAME
    rep/plan/seq binding and the SAME re-verification (0032 blind-spot veto + 0035 hold cap) as for a tap.

    Returns the gesture name when it resolved a parked rep, else ``"consumed"`` (nothing was parked, so
    the gesture is dropped and NOTHING is written -- a double-tap with no rep is completely inert)."""
    pending = self._pending
    if pending is None:
      return "consumed"
    write_consent(self.dir, pending.id, self.pid, self._pending_seq, gesture == "fire")
    self._append_log({"event": "consent_wheel", "gesture": gesture, "rep": pending.id, "plan_id": self.pid,
                      "seq": self._pending_seq, "session": self._session, "ts": round(time.time(), 3)})
    return gesture

  def _start_rep(self, rep: CalRep, frame: int, now_wall: float, consent: bool = False) -> None:
    """Arm ``rep``: command its level NOW (the brake enters on the panda-rate ramp, a clean step).
    Shared by the autonomous pick path and the 0033 accepted-tap path, so the two never drift."""
    self._active = rep
    self._rep_braked = False
    self.level_lsb = rep.level_lsb
    self.active_actuation = True
    self._on_start = frame
    self._release_frame = -1
    self._block_reason = "active"
    rec = {"event": "rep_start", "rep": rep.id, "plan_id": self.pid, "session": self._session,
           "band_kph": rep.band_kph, "level_lsb": rep.level_lsb, "phase": rep.phase,
           "direction": self._direction(), "ts": round(now_wall, 3)}
    if consent:
      rec["consent"] = True
    self._append_log(rec)

  def _park_pending(self, rep: CalRep, frame: int, now_wall: float) -> None:
    self._pending = rep
    self._pending_since = frame
    self._consent_seq += 1
    self._pending_seq = self._consent_seq      # THIS prompt's identity (the UI change key + tap bind)
    self._consent_plan_id = self.pid
    self._consent_state = "pending"
    self._consent_reason = "awaiting_tap"
    # delete any leftover tap BEFORE publishing the prompt, so an old tap can never satisfy this one
    # (and the UI can never respond to the new prompt with a write this deletion would then eat).
    self._clear_tap_file()
    self._write_pending(True)
    self._append_log({"event": "consent_pending", "rep": rep.id, "plan_id": self.pid,
                      "level_lsb": rep.level_lsb, "band_kph": rep.band_kph,
                      "session": self._session, "ts": round(now_wall, 3)})

  def _drop_pending(self, reason: str) -> None:
    rep = self._pending
    self._pending = None
    self._pending_since = -1
    self._consent_state = reason
    self._consent_reason = reason
    self._write_pending(False)
    if rep is not None:
      self._append_log({"event": "consent_" + reason, "rep": rep.id, "plan_id": self.pid,
                        "session": self._session, "ts": round(time.time(), 3)})

  def _tick_pending(self, CC, CS, frame: int) -> None:
    """0033: resolve a parked pending rep. FIRE only on a fresh, rep+plan+seq-bound, accepted tap
    whose conditions (+ the hold cap) STILL hold at the instant of the tap; otherwise DROP (expire / dismiss /
    lapse / refuse). Called only while ``self._pending is not None``, and NEVER lets a malformed file
    take the 100 Hz car loop down (fail-closed)."""
    pending = self._pending
    if pending is None:
      return
    try:
      timeout_f = int(round(self.opts.get("consent_timeout_s", DEFAULT_CONSENT_TIMEOUT_S) * 100.0))
      if timeout_f > 0 and (frame - self._pending_since) >= timeout_f:
        self._drop_pending("expired")           # untouched -> skip, retried later (after a hold-off)
        self._consent_rerprompt_after[pending.id] = frame + max(timeout_f, 1)
        return
      # RE-VERIFY at the instant of the tap: a condition that lapsed while the prompt was up (a lead
      # closed, entered a turn, disengaged, or -- 0032 -- a rear blind-spot bit appeared) DROPS the
      # prompt. Never fire on a stale world.
      cond_ok, cond_why = self._conditions_ok(CC, CS)
      if not cond_ok:
        self._drop_pending("lapsed")
        self._consent_reason = "lapsed:" + cond_why
        return
      accepted, why = self._read_tap()
      if not accepted:
        if why == "not_accepted":               # an explicit DISMISS -> skip this rep this session
          self._consent_skip.add(pending.id)
          self._drop_pending("dismissed")
          return
        self._consent_reason = why              # absent / stale / mismatched -> keep waiting
        return
      can, hwhy = hold_ok(pending.hold_s)
      if not can:
        self._drop_pending("hold_over_cap")     # never fire a rep whose hold exceeds the tool cap
        self._consent_reason = hwhy
        return
      # FIRE: arm exactly like the autonomous START path (shared helper -> no drift).
      rep = pending
      self._pending = None
      self._pending_since = -1
      self._consent_state = "firing"
      self._consent_reason = "accepted"
      self._clear_tap_file()
      self._write_pending(False)
      self._start_rep(rep, frame, time.time(), consent=True)
    except Exception:  # noqa: BLE001 - a malformed file must never crash the car loop; fail closed
      self._drop_pending("error")

  def _conditions_ok(self, CC, CS) -> tuple[bool, str]:
    if not (bool(CC.enabled) and bool(CC.longActive)):
      return False, "not_engaged"
    if not self._lead_ok(CS):
      return False, "lead_close"
    if not self._straight_ok():
      return False, "not_straight"
    if not self._blindspot_ok(CS):
      return False, "blindspot"
    return True, ""

  # ------------------------------------------------------------------ record
  def _append_log(self, rec: dict) -> None:
    try:
      os.makedirs(self.dir, exist_ok=True)
      with open(self.log_path, "a") as f:
        f.write(json.dumps(rec, sort_keys=True) + "\n")
    except OSError:
      pass

  def _complete(self, rep: CalRep, outcome: str, hold_frames: int) -> None:
    rec = {"outcome": outcome, "ts": round(time.time(), 3), "frame": self._frame,
           "band_kph": rep.band_kph, "level_lsb": rep.level_lsb, "phase": rep.phase,
           "direction": self._direction(), "hold_frames": int(hold_frames)}
    self._done[rep.id] = rec
    if outcome == "ok":
      self._session_reps += 1
    _write_json_atomic(self.progress_path, {"plan_id": self.pid, "reps": self._done,
                                            "session": self._session, "updated": round(time.time(), 3)})
    self._append_log({"event": "rep", "rep": rep.id, "plan_id": self.pid, "session": self._session, **rec})
    if self.is_done:
      _write_json_atomic(self.progress_path, {"plan_id": self.pid, "reps": self._done, "done": True,
                                              "session": self._session, "updated": round(time.time(), 3)})
      self._append_log({"event": "plan_done", "plan_id": self.pid, "ts": round(time.time(), 3)})

  # ------------------------------------------------------------------ step
  def tick(self, CC, CS, frame: int) -> None:
    """One 100 Hz decision tick. Only ever called from the ``enabled`` branch of Fca11LongBrake.update."""
    self._frame = frame
    now_wall = time.time()

    # sample the condition signals every frame (cheap; vEgo already on CS)
    try:
      v_kph = float(CS.out.vEgo) * 3.6
    except (AttributeError, TypeError, ValueError):
      v_kph = 0.0
    self._v_hist.append((frame, v_kph))
    while self._v_hist and frame - self._v_hist[0][0] > 300:
      self._v_hist.pop(0)
    try:
      yaw = float(getattr(CS.out, "yawRate", 0.0))
    except (TypeError, ValueError):
      yaw = 0.0
    lat = abs(v_kph * yaw / 3.6)   # |v[m/s] * yaw[rad/s]| = |a_lat|
    self._lat_hist.append((frame, lat))
    while self._lat_hist and frame - self._lat_hist[0][0] > 300:
      self._lat_hist.pop(0)
    self._yaw_hist.append((frame, yaw))
    while self._yaw_hist and frame - self._yaw_hist[0][0] > 300:
      self._yaw_hist.pop(0)

    self._reload_plan(now_wall, frame)
    self._update_band_window(v_kph)

    # ---- if no rep is active: pick the next due one ------------------------------------------------
    if self._active is None:
      self.active_actuation = False
      # 0033: a parked consent rep whose plan/file lapsed is dropped (fail-closed) -- never actuate a
      # rep whose authorising plan is gone, tap or no tap.
      if self._pending is not None and not self.plan_valid:
        self._drop_pending("plan_lapsed")
      if not self.plan_valid:
        return
      # 0033: an outstanding PENDING rep is resolved FIRST and EXCLUSIVELY. While one is parked the
      # sequencer actuates NOTHING (a brake firing while the owner reads a prompt is exactly the
      # surprise consent exists to remove). Tap / dismiss / expiry all resolve here.
      if self._pending is not None:
        self._tick_pending(CC, CS, frame)
        return
      if self.is_done:
        self._block_reason = "done"
        return
      cap = self.opts.get("per_drive_cap", DEFAULT_PER_DRIVE_CAP)
      if cap and cap > 0 and self._session_reps >= cap:
        self._block_reason = "drive_cap"
        return
      if not self._conditions_ok(CC, CS)[0]:
        self._block_reason = self._conditions_ok(CC, CS)[1]
        return
      # 0035: arming is BUDGET-FREE (the panda has no duration budget). The only duration gate is the
      # tool-side hold cap: a rep whose hold exceeds CAL_MAX_HOLD_S is refused, never armed.
      pick = None
      blocked_by_hold = False
      for rep in self._enabled_reps():
        if self._is_complete(rep):
          continue
        if rep.id in self._consent_skip:          # 0033: dismissed this session -> never re-prompt
          continue
        if rep.id in self._watchdog_skip:         # 0035: the watchdog aborted it this session
          continue
        can, why = hold_ok(rep.hold_s)
        if not can:
          blocked_by_hold = True
          self._last_hold_reason = why
          continue
        if not self._band_ready(rep.band_kph):
          continue
        # 0033: a rep that requires consent is PARKED (published for the UI), never armed. A rep whose
        # prompt just expired waits out its re-prompt hold-off so it is not asked on every tick.
        if self.consent_required(rep):
          if frame < self._consent_rerprompt_after.get(rep.id, -1):
            self._block_reason = "consent_holdoff"
            continue
          self._park_pending(rep, frame, now_wall)
          self._block_reason = "pending_consent"
          return
        pick = rep
        break
      if pick is None:
        self._block_reason = "hold_over_cap" if blocked_by_hold else "band_not_ready"
        return
      # START the rep: command the level NOW. The brake enters on the panda-rate ramp (a clean step).
      self._start_rep(pick, frame, now_wall, consent=False)
      return

    # ---- a rep is active: hold, then release + recovery --------------------------------------------
    if not self.plan_valid:
      # the plan/file lapsed MID-HOLD (deleted, expired, TTL fired, or a malformed progress write).
      # Explicit veto: drop the cal; the brake's production release/fade path closes the episode on
      # this same frame. This is the "plan unreadable mid-hold" path Fable flagged -- it is a VETO,
      # not a silent continue.
      self._active = None
      self.active_actuation = False
      self.level_lsb = 0
      self._block_reason = "plan_lapsed_mid_rep"
      return
    cond_ok, cond_why = self._conditions_ok(CC, CS)
    if not cond_ok:
      # a condition lapsed mid-rep (a lead closed, entered a turn, disengaged, or -- 0032 -- a rear
      # quarter blind-spot bit went set). Abandon the rep; the brake's own fca11_ok/fade path releases
      # the episode. No record: the rep is simply retried when conditions return (reps are short, so
      # the cost is negligible and the log stays clean). 0032 surfaces the veto reason so the operator
      # can see WHY reps are being skipped.
      self._active = None
      self.active_actuation = False
      self.level_lsb = 0
      self._block_reason = "blindspot" if cond_why == "blindspot" else "conditions_lapsed"
      return
    # 0035: the 0031 mid-hold budget abort is gone with the budget (there is no cooldown to arm into).
    # 0036: remember whether the brake actually COMMANDED an actuating frame for this rep. Fca11LongBrake.update
    # publishes CS._fca11_braking_now (its `braking` flag) before every tick; a hold that a production veto
    # suppressed end to end (e.g. a driver-cut latch from a gas press) never sets it.
    if bool(getattr(CS, "_fca11_braking_now", False)):
      self._rep_braked = True
    if (frame - self._on_start) < int(round(self._active.hold_s * 100.0)):
      # an optional MID-HOLD STEP-DOWN (the partial-release test): drop the command to level2 once
      # switch_at_s of the hold has elapsed. A downward step is emitted immediately.
      if self._active.level2_lsb is not None and self._active.switch_at_s is not None \
         and (frame - self._on_start) >= int(round(self._active.switch_at_s * 100.0)):
        self.level_lsb = self._active.level2_lsb
      self.active_actuation = True
      return
    # hold elapsed: drop the cal ask (the brake's production fade/passive path releases the episode)
    self.active_actuation = False
    if self._release_frame < 0:
      self._release_frame = frame
    recov = int(round(self.opts.get("recovery_s", DEFAULT_RECOVERY_S) * 100.0))
    if (frame - self._release_frame) >= recov and self._band_ready(self._active.band_kph):
      # 0036: 'ok' ONLY if the hold actually commanded braking. A rep whose every frame was vetoed delivered
      # nothing to fit: record it as 'no_brake' (not complete -> it is retried), never as a done rep.
      self._finish("ok" if self._rep_braked else "no_brake", frame)

  def _finish(self, outcome: str, frame: int) -> None:
    rep = self._active
    self._active = None
    self.active_actuation = False
    self.level_lsb = 0
    if rep is not None:
      self._complete(rep, outcome, frame - self._on_start)

  def status(self) -> dict:
    """Pre-flight status (Fable item 2: the historic failure class is SILENT config state). The operator
    reads this before a drive: toggle, plan validity/freshness, progress, and the current inert reason.
    Read-only; never raises."""
    done = self._done or {}
    reps = self._enabled_reps()
    return {
      "plan_present": self.plan_mtime >= 0,
      "plan_valid": self.plan_valid,
      "plan_id": self.pid,
      "inert_reason": self._block_reason,
      "n_reps_total": len(reps),
      "n_reps_done": sum(1 for r in reps if self._is_complete(r)),
      "done": self.is_done,
      "session_reps": self._session_reps,
      "active_rep": self._active.id if self._active is not None else None,
      "phases": list(self._phase_enabled),
      "max_hold_s": CAL_MAX_HOLD_S,
      "hold_reason": self._last_hold_reason,
      # 0032: blind-spot veto diagnostics. ``blindspot_veto`` is the configured flag; ``is_vetoing``
      # reports whether the veto is HELD right now (a set sample inside the hold-off window).
      "blindspot_veto": bool(self.opts.get("blindspot_veto", DEFAULT_BLINDSPOT_VETO)),
      "blindspot_holdoff_s": self.opts.get("blindspot_holdoff_s", DEFAULT_BLINDSPOT_HOLDOFF_S),
      "blindspot_is_vetoing": self._blindspot_vetoed,
      # 0033 consent diagnostics. ``consent_mode`` is the configured policy; ``consent_pending``
      # reports whether a rep is parked awaiting a tap right now.
      "consent_mode": self.opts.get("consent_mode", DEFAULT_CONSENT_MODE),
      "consent_min_level_lsb": self.opts.get("consent_min_level_lsb", DEFAULT_CONSENT_MIN_LEVEL_LSB),
      "consent_timeout_s": self.opts.get("consent_timeout_s", DEFAULT_CONSENT_TIMEOUT_S),
      "consent_pending": self._pending is not None,
      "consent_state": self._consent_state,
      "consent_reason": self._consent_reason,
    }
