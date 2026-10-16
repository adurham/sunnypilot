"""
Fork (adurham): drive-mode-follows-personality.

The car's drive mode (decoded from CLU13 CF_Clu_DriveMode on the opendbc side and published on
``carStateSP.driveMode``) drives openpilot when the ``DriveModePersonality`` param is ON:

  * NORMAL -> personality standard, ECO -> relaxed, SPORT -> aggressive. Applied on every mode CHANGE:
    the ``LongitudinalPersonality`` param is written and selfdrived's live ``self.personality`` is
    updated (the same rail the gap button uses; every downstream consumer follows live).
  * N / N-CUSTOM (either slot) -> block openpilot LONGITUDINAL (no new engage; disengage if engaged).
    Lateral is untouched: the log event rides the same MADS-stripped rail as the factory-cruise
    lockout, so a MADS lateral-only engagement survives.
  * Unknown mode / signal absent -> do nothing (never force a personality, never block).

The param defaults OFF: with it OFF the whole feature is inert and today's behaviour is byte-for-byte
unchanged. Manual gap-button cycling between mode changes stays allowed; the mode re-asserts on the
next change.

All of the decision logic here is pure (it reads the opendbc ``drive_mode`` mapping), so it is unit
tested without a car.
"""

from openpilot.cereal import log
# The pure mapping + enum live in the opendbc patch (0021), on the drive-mode decode side, so the
# superproject and the car layer share one source of truth. This is a coupled pair like 0019's
# CarControlSP.personality mirror: both MUST ship in the same series.
from opendbc.sunnypilot.car.hyundai.drive_mode import DriveModeResult, map_drive_mode

PARAM = "DriveModePersonality"

# The three personality outcomes map straight onto log.LongitudinalPersonality (aggressive 0 /
# standard 1 / relaxed 2). A non-negative DriveModeResult IS the personality int.
BLOCK = DriveModeResult.BLOCK
UNKNOWN = DriveModeResult.UNKNOWN


def read_enabled(params) -> bool:
  """Read the DriveModePersonality toggle. Missing key (stale libparams) -> OFF, never raises."""
  try:
    return bool(params.get_bool(PARAM))
  except Exception:
    return False


def result_for(drive_mode_raw) -> DriveModeResult:
  """CarStateSP.driveMode raw -> the action (personality | BLOCK | UNKNOWN). Pure passthrough."""
  return map_drive_mode(drive_mode_raw)


def desired_personality(result: DriveModeResult) -> int | None:
  """The personality (a log.LongitudinalPersonality int) to force, or None to leave it untouched."""
  return int(result) if result >= 0 else None


def blocks_longitudinal(result: DriveModeResult) -> bool:
  """True only for an explicit N / N-Custom block. UNKNOWN never blocks (see the module docstring)."""
  return result == BLOCK


def apply_live(ss, result: DriveModeResult) -> bool:
  """
  Apply the personality to a selfdrived-like object (``ss.personality`` live + the persisted param).

  Returns True when the value changed (so the caller can raise personalityChanged). A BLOCK / UNKNOWN
  result leaves the personality untouched. This follows the gap-button rail exactly: set
  ``self.personality`` AND ``params.put('LongitudinalPersonality', ...)`` so every downstream consumer
  (planner / MPC / setspeed-ease / adaptive-follow / CC_SP.personality) follows live.
  """
  personality = desired_personality(result)
  if personality is None or ss.personality == personality:
    return False
  ss.personality = personality
  ss.params.put("LongitudinalPersonality", personality)
  return True


def step(ss, events, drive_mode_raw) -> DriveModeResult:
  """
  One selfdrived tick of the feature (only called when the param is ON).

  On a mode CHANGE: NORMAL/ECO/SPORT (re)write the live personality + param and raise
  ``personalityChanged``; N / N-Custom add ``driveModePersonalityBlock`` (an ET.NO_ENTRY / ET.USER_DISABLE
  long event). UNKNOWN / absent -> nothing. Manual gap-button cycling between changes is untouched; the
  mode re-asserts on the next change. The block rides the MADS-stripped rail (see mads.py) so lateral is
  unaffected. Returns the mapped result for the caller's bookkeeping.
  """
  result = result_for(drive_mode_raw)
  if result != ss.drive_mode_result:
    ss.drive_mode_result = result
    if apply_live(ss, result):
      events.add(log.OnroadEvent.EventName.personalityChanged)
  if blocks_longitudinal(result):
    events.add(log.OnroadEvent.EventName.driveModePersonalityBlock)
  return result


def personality_name(personality: int) -> str:
  return log.LongitudinalPersonality.schema.enumerants.get(personality, "standard")
