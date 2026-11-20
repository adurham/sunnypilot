"""
Fork: a held standstill is released only by a SUSTAINED launch decision (adurham/sunnypilot, route 194 bookmark 8).

Why this exists
---------------
Route 194 at t_route 820.1 s: the car had been held at a no-lead stop for 9.7 s (longcontrol ``stopping``, FCA11 hold at
20 LSB, plan.shouldStop True). Then the vision model's ``action.shouldStop`` FLICKERED - planner go-runs of 4, 1, 2 and 3
frames (0.05 s each) inside 1.5 s, with ``desiredAcceleration`` +0.02 ... +0.29 - and every go frame was published
straight through: plan.shouldStop False takes ``long_control_state_trans`` from ``stopping`` to ``pid`` on the same frame,
the PID's output (+0.02 ... +0.29) is above the brake law's -0.30 release threshold, so the 20 LSB hold dropped to 0 LSB
within 0.1 s. The flicker then went back to ``stopping``, whose ramp starts from the last (positive) output and descends at
only 1 m/s^2/s, so no new brake episode could open (onset needs <= -0.5) for ~0.8 s - the DCT's creep torque (+22..26 Nm at
our holds, see lane-C STATE) rolled the car to 0.71 m/s with no pedal and the driver braked. No lead, no driver input, no
launch: one model flag that never stayed False for more than 4 frames.

Archive census (2,381 segments, every held stop that the planner released while long-active, release_all.py): 81
releases; every real launch has a go-run of >= 10 frames inside its first 4 s (74/74 launches), while both releases that
ended in a driver brake never did on their own before it (194#8: 4, 1, 2, 3; 159 s51: 2, 2, 1 then 36 frames later).

What it does
------------
While the car is at a standstill that the plan was holding (the previous published shouldStop was True and
carState.standstill), a frame whose candidates say "go" is published as a HOLD (shouldStop True, accel min(ask, 0)) until
the go decision has been seen on LAUNCH_CONFIRM_FRAMES consecutive frames. The first frame that reaches the count is passed
through unchanged, and so is every frame after it until the car is held stopped again. It never adds a go, never raises an
ask, never touches a moving car, and resets (pass-through) whenever the planner itself resets (long inactive / override).
Cost: a genuine launch from a held stop starts 0.5 s later.
"""

LAUNCH_CONFIRM_FRAMES = 10   # 0.5 s at 20 Hz: consecutive go frames before a held standstill may release


class StandstillLaunchDebounce:
  """Owned by LongitudinalPlanner. ``update`` returns the (possibly held) (should_stop, a_target) pair."""

  def __init__(self):
    self.held = False      # the previous PUBLISHED frame was a hold at standstill
    self.go_frames = 0     # consecutive frames whose raw candidates said go, while held
    self.holding = False   # this frame was converted from go to hold (diagnostics / tests)

  def reset(self) -> None:
    self.held = False
    self.go_frames = 0
    self.holding = False

  def update(self, should_stop: bool, a_target: float, standstill: bool, active: bool) -> tuple[bool, float]:
    self.holding = False
    if not active:
      self.reset()
      return should_stop, a_target
    if self.held and standstill and not should_stop:
      self.go_frames += 1
      if self.go_frames < LAUNCH_CONFIRM_FRAMES:
        self.holding = True
        return True, min(a_target, 0.)
    elif should_stop:
      self.go_frames = 0
    # published this frame: held iff the car is stopped and the output is a stop
    self.held = bool(standstill and should_stop)
    if not standstill:
      self.go_frames = 0
    return should_stop, a_target
