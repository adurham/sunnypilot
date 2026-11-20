"""
Lane C (routes 193/194, 2026-11-04): two planner defects that moved the car when it should have stayed stopped / slowed.

1. 194 #8 (t_route 820.1 s): a 9.7 s no-lead standstill hold was released by the vision model's shouldStop FLICKERING
   (planner go-runs of 4, 1, 2, 3 frames, desiredAcceleration +0.02..+0.29). plan.shouldStop False took LongControl from
   `stopping` to `pid` on the same frame, the brake law's hold (20 LSB) released at the -0.30 threshold, and the DCT's
   creep rolled the car to 0.71 m/s with no pedal. Fix: fork/standstill_launch.py - a held standstill releases only after
   LAUNCH_CONFIRM_FRAMES consecutive go frames.
2. 193 #3 (229 s) / 194 #6 (681 s): the far-lead e2e gate raised a BRAKING model wish (-1.04 / -1.41, its own plan falling
   to 0.82 / 1.23 m/s) to exactly E2E_GATE_MAX_RAISE = +0.40 because the lead 28-30 m ahead was driving AWAY. The car
   accelerated into the stop until the driver braked. Fix: model_plans_stop(velocity.x) joins the gate's should_stop input.

Every planner test drives the REAL LongitudinalPlanner.update. The frame sequences are the logged ones (replay_inputs.json
in the lane-C workspace, 20 Hz plan frames): on the deployed planner they fail, on the fixed planner they pass.
"""
from unittest import mock

import numpy as np

from openpilot.cereal import log
import openpilot.cereal.messaging as messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.modeld.constants import ModelConstants
import openpilot.selfdrive.controls.lib.longitudinal_planner as lp

T = np.array(ModelConstants.T_IDXS)
N = len(T)

# 194 #8: the LOGGED plan frames t = -1.00 .. +0.55 s around the bookmark (route 00000194--4d0c08c749, 20 Hz, all at
# carState.standstill with long active): (published shouldStop == model shouldStop, model desiredAcceleration).
# Go-runs 4, 1, 2 - never sustained. The car was held at 20 LSB before the first go frame.
FLICKER_194_8 = [(1, -0.0)] * 10 + [(1, 0.0), (0, 0.02), (0, 0.04), (0, 0.05), (0, 0.07), (1, 0.06), (1, 0.06), (1, 0.06),
                                    (0, 0.08), (1, 0.07), (1, 0.07), (0, 0.17), (0, 0.29), (1, 0.25), (1, 0.22), (1, 0.19),
                                    (1, 0.17), (1, 0.15), (1, 0.13), (1, 0.12), (1, 0.12)]

# 193 #3: the LOGGED frames -4.87 .. -3.97 s (route 00000193--48061bbc44, long active, lead present, source e2e) on which the
# deployed gate published +0.40-0.45: (vEgo, dRel, vRel, model desiredAcceleration, min(model velocity.x)).
GATE_193_3 = [(3.59, 30.0, 4.35, -1.04, 0.82), (3.54, 29.9, 5.42, -1.04, 0.82), (3.48, 28.2, 4.65, -1.06, 0.95),
              (3.43, 28.8, 5.82, -1.08, 1.04), (3.39, 29.3, 6.58, -1.08, 1.19), (3.37, 30.9, 6.38, -1.05, 1.4),
              (3.35, 28.3, 5.83, -1.03, 1.5), (3.34, 27.6, 5.4, -0.99, 1.65), (3.34, 25.7, 4.78, -0.99, 1.5),
              (3.34, 34.2, 5.19, -0.98, 1.5), (3.34, 32.8, 5.12, -0.97, 1.55), (3.34, 30.5, 5.22, -0.98, 1.37),
              (3.35, 32.1, 5.37, -1.0, 1.19), (3.35, 32.9, 5.8, -1.03, 1.01), (3.35, 31.3, 5.78, -1.04, 1.12),
              (3.35, 29.1, 5.78, -1.04, 1.23), (3.36, 26.9, 4.89, -1.04, 1.29), (3.37, 30.0, 5.04, -1.03, 1.44),
              (3.4, 31.7, 4.63, -1.02, 1.65)]


def make_planner():
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  CP.openpilotLongitudinalControl = True
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
  return lp.LongitudinalPlanner(CP, CP_SP, init_v=0.)


def planner_sm(v_ego, desired_accel, should_stop=False, plan_v=None, standstill=False, lead_present=False, d_rel=60.,
               v_lead=None, long_off=False):
  msgs = {k: messaging.new_message(k) for k in ('modelV2', 'carState', 'carControl', 'controlsState', 'selfdriveState',
                                                'radarState', 'liveMapDataSP', 'vehicleParameters', 'carStateSP',
                                                'gpsLocation', 'gpsLocationExternal', 'selfdriveStateSP')}
  md = msgs['modelV2'].modelV2
  pv = np.full(N, float(v_ego)) if plan_v is None else np.asarray(plan_v, dtype=float)
  md.velocity.x = [float(v) for v in pv]
  md.position.x = [float(x) for x in np.concatenate([[0.], np.cumsum(np.diff(T) * (pv[1:] + pv[:-1]) / 2.)])]
  md.position.t = [float(t) for t in T]
  md.orientationRate.z = [0.] * N
  md.action.desiredAcceleration = float(desired_accel)
  md.action.shouldStop = bool(should_stop)
  md.meta.disengagePredictions.gasPressProbs = [1.] * 6
  cs = msgs['carState'].carState
  cs.vEgo = float(v_ego)
  cs.standstill = bool(standstill)
  cs.vCruise = 100.
  cs.vCruiseCluster = 100.
  cc = msgs['carControl'].carControl
  cc.enabled = True
  cc.orientationNED = [0., 0., 0.]
  msgs['controlsState'].controlsState.longControlState = LongCtrlState.off if long_off else LongCtrlState.pid
  sd = msgs['selfdriveState'].selfdriveState
  sd.enabled = True
  sd.personality = log.LongitudinalPersonality.standard
  l1 = msgs['radarState'].radarState.leadOne
  l1.present = lead_present
  if lead_present:
    vl = v_ego if v_lead is None else v_lead
    l1.dRel = float(d_rel)
    l1.vLead = float(vl)
    l1.vRel = float(vl - v_ego)
    l1.vLeadK = float(vl)
    l1.aLeadTau = 1.5
    l1.modelProb = 1.0
  return {k: getattr(m, k) for k, m in msgs.items()}


def stop_shaped(v0, v_min):
  """A model plan that slows from v0 to v_min inside 4 s and stays there (the 193 #3 / 194 #6 shape)."""
  return np.maximum(v_min, v0 - (v0 - v_min) * np.clip(T / 4., 0., 1.))


# 194 #8: the MPC candidate the car's planner computed on every frame of that hold, rebuilt from the logged plan trajectory
# with the planner's own get_accel_from_plan (no lead, cruise far above): +2.56 m/s^2, i.e. the MPC never asked to stop. The
# harness MPC cannot solve at v = 0 (solution_status 4 -> reset -> a stop-shaped candidate), which would hold the car by
# itself and hide the defect, so the standstill tests pin the candidate to the logged value.
MPC_194_8 = 2.559


# 193 #3: the MPC candidate the car computed on the 19 gated frames, rebuilt the same way: +1.59 .. +2.14 m/s^2 (the lead was
# driving away, so the MPC wanted to close the gap). The harness MPC solves a lead-braking candidate there instead and would
# keep the gate shut on its own, hiding the defect - so the gate test pins it to the logged median.
MPC_193_3 = 2.06


def mpc_as_logged(value=MPC_194_8):
  return mock.patch.object(lp, 'get_accel_from_plan', lambda *a, **k: value)


def held_stop(p, frames=40):
  for _ in range(frames):
    p.update(planner_sm(0., 0., should_stop=True, standstill=True))
  assert p.output_should_stop


class TestStandstillFlicker(OpenpilotTestCase):
  def test_194_8_flicker_never_releases_the_hold(self):
    """FAIL-BEFORE: the deployed planner publishes shouldStop False on the 7 go frames (the 20 LSB hold drops)."""
    p = make_planner()
    outs = []
    with mpc_as_logged():
      held_stop(p)
      for stop, des in FLICKER_194_8:
        p.update(planner_sm(0., des, should_stop=bool(stop), standstill=True))
        outs.append((bool(p.output_should_stop), float(p.output_a_target), bool(p.standstill_launch.holding)))
    self.assertTrue(all(s for s, _, _ in outs), f'hold released on {sum(not s for s, _, _ in outs)} flicker frames')
    # a frame the debounce CONVERTED never carries a positive ask (frames the model itself flagged stop keep their small
    # positive ask, exactly as on the car: LongControl ignores a_target in `stopping`)
    self.assertTrue(all(a <= 0. for _, a, conv in outs if conv))
    self.assertEqual(sum(conv for _, _, conv in outs), 7)              # the 7 logged go frames

  def test_194_8_longcontrol_keeps_the_brake_hold(self):
    """END TO END to the brake law's input: the real LongControl, fed the planner's published (shouldStop, aTarget) at
    100 Hz, must keep actuators.accel at or below the FCA11 release threshold (-0.30) for the whole flicker. FAIL-BEFORE:
    the deployed planner's go frames take LongControl to `pid` and its output to +0.02..+0.29 - the 20 LSB hold drops."""
    from types import SimpleNamespace
    from openpilot.selfdrive.controls.lib.longcontrol import LongControl
    p = make_planner()
    CP = p.CP
    CP.stopAccel = -2.0                                                # the Hyundai value (car's longcontrol ramp floor)
    loc = LongControl(CP, SimpleNamespace(enableGasInterceptor=True))
    cs = SimpleNamespace(aEgo=0., vEgo=0., brakePressed=False, cruiseState=SimpleNamespace(standstill=False))
    acc = []
    with mpc_as_logged():
      for i in range(40 + len(FLICKER_194_8)):
        if i < 40:
          p.update(planner_sm(0., 0., should_stop=True, standstill=True))
        else:
          stop, des = FLICKER_194_8[i - 40]
          p.update(planner_sm(0., des, should_stop=bool(stop), standstill=True))
        for _ in range(5):                                             # controlsd runs 5 x per plan frame
          acc.append(loc.update(True, cs, float(p.output_a_target), bool(p.output_should_stop), (-3.5, 2.0)))
    flick = acc[40 * 5:]
    self.assertLessEqual(max(flick), -0.30, f'brake-law input rose to {max(flick):+.2f}: the hold would release')

  def test_sustained_go_releases_on_the_confirm_frame(self):
    from openpilot.sunnypilot.fork.standstill_launch import LAUNCH_CONFIRM_FRAMES
    p = make_planner()
    stops = []
    with mpc_as_logged():
      held_stop(p)
      for _ in range(LAUNCH_CONFIRM_FRAMES + 5):
        p.update(planner_sm(0., 0.6, should_stop=False, standstill=True))
        stops.append(bool(p.output_should_stop))
    self.assertEqual(stops.index(False), LAUNCH_CONFIRM_FRAMES - 1)   # released on the 10th consecutive go frame
    self.assertFalse(any(stops[LAUNCH_CONFIRM_FRAMES - 1:]))           # ...and stays released
    self.assertGreater(float(p.output_a_target), 0.)                   # the launch ask itself is untouched
    self.assertEqual(LAUNCH_CONFIRM_FRAMES, 10)                        # 0.5 s at 20 Hz (archive: 74/74 launches pass)

  def test_moving_car_is_byte_identical(self):
    """A car that is not at a held standstill (rolling, or stopped without a held plan) is never touched."""
    from openpilot.sunnypilot.fork import standstill_launch as sl
    seq = [planner_sm(v, d, should_stop=s) for v, d, s in
           [(5., -0.5, False)] * 20 + [(0.2, -0.2, True)] * 5 + [(0.2, 0.3, False)] * 10 + [(1.0, 0.5, False)] * 10]
    a, b = make_planner(), make_planner()
    b.standstill_launch = sl.StandstillLaunchDebounce()
    b.standstill_launch.update = lambda s, acc, ss, act: (s, acc)    # the stock path
    with mpc_as_logged():
      for sm in seq:
        a.update(sm)
        b.update(sm)
        self.assertEqual(bool(a.output_should_stop), bool(b.output_should_stop))
        self.assertEqual(float(a.output_a_target), float(b.output_a_target))

  def test_reset_passes_through(self):
    from openpilot.sunnypilot.fork.standstill_launch import StandstillLaunchDebounce
    d = StandstillLaunchDebounce()
    for _ in range(5):
      d.update(True, 0., True, True)
    self.assertEqual(d.update(False, 0.4, True, True), (True, 0.))   # held: converted
    self.assertEqual(d.update(False, 0.4, True, False), (False, 0.4))  # long inactive / override: pass through, reset
    self.assertEqual(d.update(False, 0.4, True, True), (False, 0.4))   # no held stop since the reset: nothing to hold

  def test_a_genuine_stop_still_stops(self):
    """The debounce only ever ADDS a hold: a stop request is never delayed or removed."""
    from openpilot.sunnypilot.fork.standstill_launch import StandstillLaunchDebounce
    d = StandstillLaunchDebounce()
    rng = np.random.default_rng(194)
    for _ in range(5000):
      s, a, ss, act = bool(rng.random() < 0.5), float(rng.uniform(-2, 2)), bool(rng.random() < 0.7), bool(rng.random() < 0.95)
      s2, a2 = d.update(s, a, ss, act)
      if s:
        self.assertEqual((s2, a2), (s, a))
      self.assertLessEqual(a2, a)


class TestGateStopPlanVeto(OpenpilotTestCase):
  def test_model_plans_stop(self):
    self.assertTrue(lp.model_plans_stop(stop_shaped(4., 0.8)))
    self.assertTrue(lp.model_plans_stop(stop_shaped(4., lp.E2E_GATE_STOP_PLAN_V)))
    self.assertFalse(lp.model_plans_stop(stop_shaped(20., 5.25)))   # the slowest NON-stop raise in the archive census
    self.assertFalse(lp.model_plans_stop([]))
    self.assertFalse(lp.model_plans_stop([float('nan')] * N))

  def test_193_3_gate_does_not_raise_a_stop(self):
    """FAIL-BEFORE: the deployed planner publishes exactly +0.40 on these frames (lead driving away, model stopping)."""
    p = make_planner()
    v0 = GATE_193_3[0][0]
    published = []
    with mpc_as_logged(MPC_193_3):
      # 1 s of the same approach first so the gate's 10-frame entry count is satisfied, as on the car
      for _ in range(20):
        p.update(planner_sm(v0, -1.0, plan_v=stop_shaped(v0, 0.9), lead_present=True, d_rel=30., v_lead=v0 + 5.))
      for v, d_rel, v_rel, des, vmin in GATE_193_3:
        p.update(planner_sm(v, des, plan_v=stop_shaped(v, vmin), lead_present=True, d_rel=d_rel, v_lead=v + v_rel))
        published.append(float(p.output_a_target))
    self.assertFalse(p.e2e_gate)
    self.assertTrue(all(a < 0. for a in published), f'positive ask while the model stops: {max(published):.2f}')

  def test_far_follow_gate_still_raises(self):
    """The case the gate was built for (far lead, model braking early, model plan NOT stopping) is unchanged."""
    v = 15.
    p = make_planner()
    with mpc_as_logged(0.3):
      for _ in range(40):
        p.update(planner_sm(v, -0.8, plan_v=np.full(N, v - 1.), lead_present=True,
                            d_rel=2.0 * (lp.STOP_DISTANCE + 1.45 * v), v_lead=v))
    self.assertTrue(p.e2e_gate)
    self.assertGreater(float(p.output_a_target), -0.8)
