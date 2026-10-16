"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Fork (adurham): FCA11-long CAL driver consent over the STEERING WHEEL (opendbc patch 0034).

WHY
---
0033 gave the owner a touch prompt ("tap to fire") for a cal rep. The owner asked for a wheel gesture
instead -- but "a steering wheel button that isn't in use". This car's CLU11 emits exactly THREE cruise
button codes (measured across routes 14a/14b/14d/14e/14f): 1 = RES_ACCEL (up), 2 = SET_DECEL (down),
4 = CANCEL (pause/resume). There is NO gap button and NO unused button. All three are already consumed:
CANCEL is the ONLY engage/disengage (pause_resume.py); up = set-speed up (+ a long-press big-step via
cruise_ext.update_v_cruise_delta); down = set-speed down (+ long-press-down = set-speed-to-current, in
cruise.py::_update_v_cruise_pedal). So the ONLY free encoding is a DOUBLE-TAP of an existing arrow -- an
event PATTERN nothing in the fork consumes (verified: no double/release_toggle gesture exists anywhere;
the raw bitmask is published as selfdriveStateSP.buttonsPressed/.buttonsReleaseToggle but nothing treats
a double-press as a gesture).

WHAT
----
* double-press UP   -> FIRE the parked rep    (the wheel twin of the UI's "TAP TO FIRE")
* double-press DOWN -> DISMISS the parked rep (the wheel twin of the UI's "X")

A "double press" is TWO complete taps of the SAME arrow: the FIRST tap's RELEASE opens the window, and
the SECOND tap's PRESS (falling within DOUBLE_PRESS_WINDOW_S of that release) is the trigger. This needs
no new button, emits no new CAN frame, and adds no cereal/capnp field.

HOW IT COMPOSES WITH THE EXISTING GESTURES (why it is separable -- verified against the source)
--------------------------------------------------------------------------------------------------
* The long-press gestures are HOLDs, not taps. cruise.py::_update_v_cruise_pedal acts on a long press only
  once a button timer reaches CRUISE_LONG_PRESS = 50 controller frames (0.5 s at 100 Hz) and only while the
  button is HELD. A tap on this car is 1-4 CLU11 samples at 50 Hz (20-80 ms); a double-TAP keeps the button
  down only for those short taps, so it never approaches 50 held frames. A double-tap therefore can never
  be mistaken for a long press, and vice versa:
    - long-press DOWN (set speed = current) fires at the 50th held frame and RETURNS; its release applies
      nothing extra. A double-tap never reaches 50 frames -> set-to-current is untouched.
    - long-press UP (the x5 big-step) has the same 0.5 s hold prerequisite -> untouched.
* The caller runs the detector ONLY while a rep is parked. With no rep parked it is never called, so a
  single press, a double press, or an up/down pair all behave byte-for-byte as they do today.
* This module CONSUMES the edges it acted on (the triggering tap's press and its release) so the two taps
  of a gesture are not ALSO re-read as set-speed short presses. The FIRST tap of a pair is deliberately
  left alone: it is a normal single press and behaves exactly as one (this is why a wheel fire is
  accompanied by the same +1 set-speed that a lone tap gives -- it is the price of not inventing a button).

THE WINDOW (DOUBLE_PRESS_WINDOW_S = 0.8 s)
-----------------------------------------
Measured from the RELEASE of the first tap to the PRESS of the second. 0.8 s is chosen from the measured
CLU11 timing on this car plus human double-tap behaviour:
  * a CLU11 press/release here is 1-4 samples at 50 Hz (20-80 ms), so a deliberate double-tap is dominated
    by the open-loop gap BETWEEN the two taps, not by the taps themselves;
  * human double-tap inter-tap intervals cluster around 200-500 ms; 0.8 s sits comfortably above that (a
    deliberate double-tap is essentially never missed) yet well below the ~1.5-2 s at which a second press
    starts to read as a fresh, unrelated single press;
  * 0.8 s is ~40x the machine-side cost of a tap (the sequencer consumes a consent on its next 50 Hz send
    slot, ~20 ms), so the timing budget is dominated by the human, not by the code.
Rejected: 0.4 s (too tight -- a slow deliberate double-tap is missed and the owner re-aims); 1.5 s (a
driver who taps up, pauses, then taps up again out of habit would fire a rep they did not intend).

FAIL-CLOSED
-----------
This module only DETECTS; the authority stays entirely in cal_mode.CalSequencer. The written consent file
is bound to the rep_id + plan_id + prompt seq of the parked rep, must be <= CONSENT_FRESHNESS_S (2 s)
fresh, and the sequencer RE-VERIFIES every condition (incl. the 0032 blind-spot veto) and the 0035 hold cap
at the instant it consumes it. A detector bug can at worst fail to prompt, or fire a rep the sequencer
would have parked anyway -- it can never actuate without the sequencer's own re-verification. The
detector itself writes nothing and holds no actuation state.
"""
import time
from enum import IntEnum

from opendbc.car import structs

ButtonType = structs.CarState.ButtonEvent.Type

_UP = ButtonType.accelCruise
_DOWN = ButtonType.decelCruise

# Measured: a deliberate human double-tap. See the module docstring for the derivation + rejected values.
DOUBLE_PRESS_WINDOW_S = 0.8
# A genuine TAP is short. A press held at/above CRUISE_LONG_PRESS (cruise.py / cruise_helpers.py = 50
# controller frames = 0.5 s at 100 Hz) is a HOLD -- the long-press UP big-step or long-press DOWN
# set-to-current. A hold may never SEED a double-press, so those gestures can never be mistaken for the
# first half of a wheel consent gesture, and a long-press-down can never arm a later misfire.
TAP_MAX_HOLD_S = 0.5


class WheelGesture(IntEnum):
  NONE = 0
  FIRE = 1
  DISMISS = 2


class WheelConsentDetector:
  """Recognises the FCA11-CAL consent gestures on the cruise arrows.

  ``update(events, pending_rep_id, now)`` is called once per CarState frame with that frame's
  ``CS.buttonEvents`` and the currently parked rep id (or None). It returns ``(gesture, consumed)``:

    * ``gesture``  -- ``FIRE`` (double-press UP), ``DISMISS`` (double-press DOWN) or ``NONE``;
    * ``consumed`` -- a set of ``(ButtonType, pressed)`` edges the caller must DROP from the button
      stream, so the triggering tap is not also read as a set-speed short press.

  It is a pure function of the event stream: it holds no actuation state, writes nothing, and every
  timing decision uses ``time.monotonic``. When ``pending_rep_id`` is None it fires nothing AND consumes
  nothing (completely inert), so the caller's no-rep path is byte-identical to today.
  """

  def __init__(self):
    self._armed_btn = None                  # arrow whose first tap's release opened the window (or None)
    self._armed_at = 0.0                    # monotonic ts of that release
    self._suppress_release = None           # after firing, drop that tap's release so it cannot re-arm
    self._press_btn = None                  # arrow currently held (to time a hold vs a tap)
    self._press_at = 0.0                    # monotonic ts of that press

  @property
  def active(self) -> bool:
    """True while the detector holds state (an open first-tap window, or a release still owed a
    suppression). The caller uses this to keep stepping the detector across the few frames it takes for a
    tap's release to arrive, even if the rep has already left the pending state in the meantime."""
    return self._armed_btn is not None or self._suppress_release is not None

  def _expire(self, now: float) -> None:
    if self._armed_btn is not None and (now - self._armed_at) > DOUBLE_PRESS_WINDOW_S:
      self._armed_btn = None

  def update(self, events, pending_rep_id, now: float | None = None):
    if now is None:
      now = time.monotonic()
    self._expire(now)

    gesture = WheelGesture.NONE
    consumed: set = set()
    fire = pending_rep_id is not None

    for b in events:
      bt = b.type
      if bt not in (_UP, _DOWN):
        continue
      pressed = bool(b.pressed)
      if pressed:
        if self._suppress_release == bt:
          # a PRESS before the expected release (cannot happen on a real stream): clear the stale
          # suppression and fall through to treat this as an ordinary press.
          self._suppress_release = None
        if self._armed_btn == bt and (now - self._armed_at) <= DOUBLE_PRESS_WINDOW_S:
          # SECOND tap of a pair on the SAME arrow -> this IS the gesture.
          self._armed_btn = None
          # Only a FIRE/DISMISS gesture suppresses its trigger tap; a double-tap with no rep parked is
          # inert and its two taps flow on as ordinary presses (so it cannot swallow a set-speed step).
          self._suppress_release = bt if fire else None
          if fire:
            consumed.add((bt, True))           # drop the trigger press now; its release is dropped below
            gesture = WheelGesture.FIRE if bt == _UP else WheelGesture.DISMISS
          # no rep parked -> inert: consume nothing so the double-tap is two ordinary presses
        elif self._armed_btn is not None and self._armed_btn != bt:
          # a press on the OTHER arrow is not a continuation -> abort the open window
          self._armed_btn = None
        # otherwise this press may be the FIRST tap of a pair; the window opens on its release (below)
        self._press_btn = bt
        self._press_at = now
      else:
        if self._suppress_release == bt:
          self._suppress_release = None
          consumed.add((bt, False))            # the release that ended a fired pair does not re-arm (nor step)
          self._press_btn = None
          continue
        # a release ends a tap -> open the window for a matching second tap, IFF it was a genuine short
        # TAP. A hold at/above TAP_MAX_HOLD_S is the long-press gesture (big-step / set-to-current) and
        # must NOT seed a double-press.
        held = (now - self._press_at) if self._press_btn == bt else 0.0
        if held < TAP_MAX_HOLD_S:
          self._armed_btn = bt
          self._armed_at = now
        self._press_btn = None

    return gesture, consumed
