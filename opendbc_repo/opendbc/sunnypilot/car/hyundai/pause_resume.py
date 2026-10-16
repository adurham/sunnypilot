"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Steering-wheel pause/resume button for Hyundai gas-interceptor (pedal) longitudinal.

On this car the button the driver calls "pause/resume" sends CLU11 CF_Clu_CruiseSwState = 4 (named CANCEL in the DBC).
Route 11c: four presses of it with the factory cruise off all made the factory cruise resume ~40 ms later, so for the
car it is a toggle and not a plain cancel. openpilot used to treat it only as cancel, which is why it could not
re-engage pedal-long after a brake press.

Mirror of hyundai_gas_interceptor_pause_button_check() in opendbc/safety/modes/hyundai_common.h. The panda grant is a
PERMISSION and must be a superset of openpilot's engage: every resumeCruise emitted here coincides with a panda grant on
the same CLU11 sample (TestPandaLockstep), so openpilot can never be engaged while panda refuses the pedal.

  * Press while engaged: emits `cancel`, which disengages (unchanged). Panda clears controls while it is held; it may
    re-grant on the release, but openpilot stays disengaged (sends zero gas) and the heartbeat clears the unused grant.
  * Press while disengaged: emits `resumeCruise` (pressed + released in the same frame) only if ALL of these hold:
      - the press started with the brake released and the factory cruise MAIN off;
      - the brake was not pressed at any sample during the press or its release debounce, and MAIN is still off;
      - the button then stayed released for PAUSE_RELEASE_SAMPLES consecutive samples.
    Gas may be held (engages into override). Speed is NOT checked anywhere: pedal mode has no engage floor
    (minEnableSpeed = -1), so this press engages at any speed including a standstill.
  * This is the ONLY engage path in pedal mode (buttons-v3): the up/down arrows only change the set speed.
    openpilot's buttonEnable (CarState.update_button_enable) is the only consumer, so the engage source is always a
    deliberate driver press. Nothing else produces resumeCruise: no timer, no brake release, no speed recovery.
  * Bounce: a release shorter than PAUSE_RELEASE_SAMPLES does not start a new press. So a press that began while
    engaged (a disengage) can never be re-interpreted as a resume.

Route 00000128 @139.2 s: the old panda rule (arm only if !controls_allowed) diverged from this tracker after a SET that
openpilot refused below 25 mph left a stale panda grant: openpilot judged the press a resume, panda a disengage.
"""
from opendbc.car import structs

ButtonType = structs.CarState.ButtonEvent.Type

PAUSE_BUTTON = 4  # CF_Clu_CruiseSwState (Buttons.CANCEL)
NO_BUTTON = 0
# MUST equal HYUNDAI_PAUSE_RELEASE_SAMPLES in opendbc/safety/modes/hyundai_common.h. CLU11 is 50 Hz: 3 samples = 60 ms.
PAUSE_RELEASE_SAMPLES = 3


class PauseResumeTracker:
  def __init__(self):
    self.armed = False                      # panda mirror: the current/last press grants on its debounced release
    self.resume_press = False               # the current/last press started while disengaged
    self.cancel_press = False               # the current/last press is a disengage (cancel) press
    self.released_cnt = PAUSE_RELEASE_SAMPLES

  def update(self, samples, engaged: bool, brake: bool, gas: bool, factory_main: bool) -> list[structs.CarState.ButtonEvent]:
    """samples: every CF_Clu_CruiseSwState value received since the last call (CLU11 is 50 Hz, CarState 100 Hz).
    engaged: openpilot longitudinal engaged (CarControl.enabled from the previous frame, i.e. before this press).
    gas: accepted for interface stability; a held gas pedal does not block a resume (engages into override).
    Returns the button events for this frame: cancel and/or resumeCruise."""
    del gas
    events: list[structs.CarState.ButtonEvent] = []
    for b in samples:
      if b == PAUSE_BUTTON:
        if self.released_cnt >= PAUSE_RELEASE_SAMPLES:
          # a new press: panda arms its grant on brake/MAIN only; openpilot resumes only if it was disengaged
          self.armed = not brake and not factory_main
          self.cancel_press = engaged
          self.resume_press = not engaged
          if self.cancel_press:
            events.append(structs.CarState.ButtonEvent(pressed=True, type=ButtonType.cancel))
        self.released_cnt = 0
      elif b != NO_BUTTON:
        # rolled onto another cruise button: abandon the pause press (that button follows its own rules)
        self._finish_cancel(events)
        self.armed = False
        self.resume_press = False
        self.released_cnt = PAUSE_RELEASE_SAMPLES
      elif self.released_cnt < PAUSE_RELEASE_SAMPLES:
        self.released_cnt += 1
        if self.released_cnt == PAUSE_RELEASE_SAMPLES:
          self._finish_cancel(events)
          if self.armed and self.resume_press and not brake and not factory_main:
            events.append(structs.CarState.ButtonEvent(pressed=True, type=ButtonType.resumeCruise))
            events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.resumeCruise))
          self.armed = False
          self.resume_press = False
      if brake:
        self.armed = False  # brake at any time during the press / release debounce cancels the resume
    if brake:
      self.armed = False  # also on frames with no new CLU11 sample (CarState runs at 100 Hz, CLU11 at 50 Hz)
    return events

  def _finish_cancel(self, events: list) -> None:
    if self.cancel_press:
      events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.cancel))
      self.cancel_press = False
