"""
Tests for the LOG-ONLY planner telemetry added 2026-11-19 (card t_391c5e8b): the fields appended to
``LongitudinalPlanSP`` in ``openpilot/cereal/custom.capnp`` so a review can read the pre-cap candidate, the NoCloseCap
factor and the closing estimate out of an rlog instead of rebuilding them offline.

Why the fields exist: every 2026-11-19 review had to reconstruct the pre-cap ask from ``longitudinalPlan.speeds/accels``
(sim/noclose reproduced 99.2 % of the ``ask == 0.00`` frames) and the creep review had to rebuild the closing filter's
internals, because the logged ask (``carControl.actuators.accel`` / ``longitudinalPlan.aTarget``) is POST-cap.

What this file pins:
  * the schema is APPEND-ONLY: ``noClose @8`` is a new ordinal, every pre-existing member keeps its name and ordinal and
    no ordinal is used twice (a renumber would silently break old logs);
  * a message round-trips through the real log reader with every field intact (so the rlog decodes on a laptop);
  * the REAL ``LongitudinalPlanner`` fills the chain it acted on: ``postCapAccel == preCapAccel * factor`` when the cap
    touches a positive ask, ``postCapAccel == preCapAccel`` otherwise, ``preCapSource`` = the candidate that won the
    ``min()``, and ``closingSpeed == max(filteredClosing, rawMeanClosing)`` while the filter has a track;
  * filling the fields cannot change the ask: the same drive with the fill disabled is bit-identical, frame for frame.
"""
import re
import struct
from pathlib import Path

import numpy as np

from openpilot.cereal import log, messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.fork.tests.test_lead_closing import DT, make_planner, planner_sm

SRC = log.LongitudinalPlan.LongitudinalPlanSource
# the members that existed before the 2026-11-19 append, in order (ordinal -> name); the deprecated group owns @0.
EXISTING = [(1, 'longitudinalPlanSource'), (2, 'smartCruiseControl'), (3, 'speedLimit'), (4, 'vTarget'), (5, 'aTarget'),
            (6, 'events'), (7, 'e2eAlerts')]


class FakeSM(dict):
  """The planner's SubMaster surface: item access for the services, ``all_checks`` and ``logMonoTime`` for publish()."""

  def __init__(self, msgs):
    super().__init__(msgs)
    self.logMonoTime = {'modelV2': 1_000_000_000}

  def all_checks(self, service_list=None):
    return True


class FakePM:
  def __init__(self):
    self.sent = {}

  def send(self, name, msg):
    self.sent[name] = msg.to_bytes()


def _capnp_text() -> str:
  import openpilot.cereal
  return (Path(openpilot.cereal.__file__).parent / 'custom.capnp').read_text()


def _struct_block(text: str, name: str) -> str:
  m = re.search(rf"(?ms)^struct {re.escape(name)} @0x[0-9a-fA-F]+ \{{\n(.*?)^\}}", text)
  assert m, f'{name} not found in custom.capnp'
  return m.group(1)


class TestSchemaAppendOnly(OpenpilotTestCase):
  def test_no_existing_member_moved_and_noClose_is_appended(self):
    body = _struct_block(_capnp_text(), 'LongitudinalPlanSP')
    members = [(int(o), n) for n, o in re.findall(r'(?m)^  ([A-Za-z_][A-Za-z0-9_]*)\s+@(\d+)\s', body)]
    for ordinal, name in EXISTING:
      self.assertIn((ordinal, name), members, f'{name} @{ordinal} moved or was renamed')
    self.assertIn((8, 'noClose'), members, 'the appended field must keep ordinal 8')
    self.assertEqual(members, EXISTING + [(8, 'noClose')], 'the struct must stay append-only, in order')

  def test_every_ordinal_is_used_once(self):
    body = _struct_block(_capnp_text(), 'LongitudinalPlanSP')
    ordinals = [int(o) for o in re.findall(r'(?m)^  (?:[A-Za-z_][A-Za-z0-9_]*)\s+@(\d+)\s', body)]
    self.assertEqual(len(ordinals), len(set(ordinals)), f'duplicate capnp ordinal in LongitudinalPlanSP: {ordinals}')

  def test_the_candidate_mirror_matches_the_log_enum(self):
    """`noClose.preCapSource` is a local enum (custom.capnp cannot import log.capnp -- log.capnp imports it), so the
    mirror must be pinned to log.LongitudinalPlan.LongitudinalPlanSource or a consumer would decode the wrong winner."""
    import openpilot.cereal
    cereal = Path(openpilot.cereal.__file__).parent
    m_log = re.search(r'(?ms)enum LongitudinalPlanSource \{(.*?)\}', (cereal / 'log.capnp').read_text())
    m_sp = re.search(r'(?ms)enum Candidate \{(.*?)\}', _struct_block(_capnp_text(), 'LongitudinalPlanSP'))
    if m_log is None or m_sp is None:
      self.fail('the Candidate mirror or LongitudinalPlanSource was not found (see the parse above)')
    parse = lambda body: [(n, int(o)) for n, o in re.findall(r'(\w+)\s+@(\d+)', body)]
    self.assertEqual(parse(m_sp.group(1)), parse(m_log.group(1)))
    self.assertEqual(parse(m_sp.group(1)), [(n, int(o)) for o, n in
                                            [(0, 'cruise'), (1, 'lead0'), (2, 'lead1'), (3, 'lead2'), (4, 'e2e')]])

  def test_the_log_message_round_trips_every_field(self):
    msg = messaging.new_message('longitudinalPlanSP')
    nc = msg.longitudinalPlanSP.noClose
    nc.enabled, nc.leadPresent, nc.leadUsable, nc.tracked, nc.applies = True, True, True, True, True
    nc.preCapAccel, nc.preCapSource, nc.postCapAccel = 0.37, int(SRC.lead0), 0.0
    nc.factor, nc.rawFactor, nc.dwellFrames = 0.0, 0.25, 10
    nc.closingSpeed, nc.filteredClosing, nc.rawMeanClosing, nc.vRelBias = 0.125, 0.4, -0.1, 0.75
    nc.filterValid, nc.filterCoasting, nc.filterRejects = True, False, 3
    nc.gateReject, nc.reinit = True, False
    nc.e2eGate, nc.e2eGated, nc.e2eFinal = True, 0.4, 0.65
    raw = msg.to_bytes()
    got = messaging.log_from_bytes(raw).longitudinalPlanSP.noClose
    self.assertTrue(got.enabled and got.leadPresent and got.leadUsable and got.tracked and got.applies)
    self.assertAlmostEqual(got.preCapAccel, 0.37, places=6)
    self.assertEqual(got.preCapSource, int(SRC.lead0))
    self.assertEqual(got.postCapAccel, 0.0)
    self.assertEqual(got.factor, 0.0)
    self.assertAlmostEqual(got.rawFactor, 0.25, places=6)
    self.assertEqual(got.dwellFrames, 10)
    self.assertAlmostEqual(got.closingSpeed, 0.125, places=6)
    self.assertAlmostEqual(got.filteredClosing, 0.4, places=6)
    self.assertAlmostEqual(got.rawMeanClosing, -0.1, places=6)
    self.assertAlmostEqual(got.vRelBias, 0.75, places=6)
    self.assertTrue(got.filterValid and got.gateReject and got.e2eGate)
    self.assertFalse(got.filterCoasting or got.reinit)
    self.assertEqual(got.filterRejects, 3)
    self.assertAlmostEqual(got.e2eGated, 0.4, places=6)
    self.assertAlmostEqual(got.e2eFinal, 0.65, places=6)


class TestFilterLogFlags(OpenpilotTestCase):
  def test_gate_reject_then_reinit_is_visible(self):
    from openpilot.sunnypilot.fork.lead_closing import LeadClosingFilter, REINIT_FRAMES
    f = LeadClosingFilter()
    f.update(True, 60., -1.0)                      # a fresh track: init, no reject, no reinit
    self.assertTrue(f.valid)
    self.assertFalse(f.gate_reject or f.reinit)
    f.update(True, 90., 0.)                        # 30 m away from the prediction: a different object
    self.assertTrue(f.gate_reject, 'a gated frame must say so on the frame it happens')
    self.assertFalse(f.reinit)
    for _ in range(REINIT_FRAMES - 1):
      f.update(True, 90., 0.)
    self.assertTrue(f.reinit, 'the re-initialising frame must say so')
    self.assertTrue(f.gate_reject, 'the re-initialising frame IS a rejected frame as well as a reinit')
    f.update(True, 90. - 0.5 * DT, -0.5)           # a normal frame: the flags clear
    self.assertFalse(f.gate_reject or f.reinit)

  def test_bias_state_is_the_filters_third_state(self):
    from openpilot.sunnypilot.fork.lead_closing import LeadClosingFilter
    f = LeadClosingFilter()
    self.assertEqual(f.bias, 0.)                   # stale filter: no state to report
    for i in range(200):
      f.update(True, 60. - 1.2 * i * DT, 0.3)      # the gap shrinks 1.2 m/s while vRel claims +0.3 opening
    self.assertGreater(f.bias, 0.5)                # vRel = rate + bias, so the bias must carry the lie
    self.assertAlmostEqual(f.closing, 1.2, delta=0.4)


class TestPlannerFillsTheChain(OpenpilotTestCase):
  def _run(self, n, v_ego, d0, gap_rate, vrel, desired=0.3):
    p = make_planner()
    pm = FakePM()
    rows = []
    for i in range(n):
      sm = FakeSM(planner_sm(v_ego, desired, d0 + gap_rate * i * DT, vrel(i)))
      p.update(sm)
      p.publish(sm, pm)
      nc = messaging.log_from_bytes(pm.sent['longitudinalPlanSP']).longitudinalPlanSP.noClose
      rows.append((float(p.output_a_target), nc))
    return p, rows

  def test_the_cap_chain_reproduces_on_a_closing_lead(self):
    p, rows = self._run(120, 17., 80., -1.0, lambda i: 0.3 if i % 2 else 0.0)
    engaged = [nc for _, nc in rows if nc.applies]
    self.assertTrue(engaged, 'the cap never applied on a 1 m/s closing lead')
    self.assertTrue(any(nc.factor < 0.5 for nc in engaged), 'the factor never dropped on a 1 m/s closing lead')
    for ask, nc in rows:
      self.assertTrue(0. <= nc.factor <= 1.)
      self.assertEqual(nc.leadPresent, True)
      if nc.preCapAccel > 0. and nc.applies:
        self.assertAlmostEqual(nc.postCapAccel, nc.preCapAccel * nc.factor, places=6,
                               msg=f'postCapAccel must be the capped pre-cap ask (frame factor {nc.factor:.3f})')
      else:
        self.assertEqual(nc.postCapAccel, nc.preCapAccel, 'a zero/negative ask passes through bit-for-bit')
      self.assertAlmostEqual(nc.postCapAccel, ask, places=6, msg='the logged post-cap ask must be what was published')
      if nc.filterValid:
        self.assertAlmostEqual(nc.closingSpeed, max(nc.filteredClosing, nc.rawMeanClosing), places=6)

  def test_preCapSource_is_the_candidate_that_won_the_min(self):
    p, rows = self._run(60, 17., 70., 0., lambda i: 0.)
    self.assertTrue(all(nc.preCapSource in (int(SRC.cruise), int(SRC.lead0), int(SRC.lead1), int(SRC.lead2),
                                            int(SRC.e2e)) for _, nc in rows))
    self.assertEqual(rows[-1][1].preCapSource, int(p.mpc.source), 'the logged winner must be the source min() chose')

  def test_dwell_state_is_the_window_the_factor_came_from(self):
    from openpilot.sunnypilot.fork import no_close as nc_mod
    p, rows = self._run(40, 17., 80., -1.0, lambda i: 0.)
    window = max(1, int(round(nc_mod.RELEASE_DWELL_S / p.no_close.dt)))
    for _, nc in rows:
      self.assertLessEqual(nc.dwellFrames, window)
    # every frame after the first few carries the full window: the cap held for the whole run
    self.assertEqual(rows[-1][1].dwellFrames, window)

  def test_no_lead_frame_says_so_and_leaves_the_ask_alone(self):
    p = make_planner()
    pm = FakePM()
    sm = FakeSM(planner_sm(17., 0.3, 0., 0., lead_present=False))
    p.update(sm)
    p.publish(sm, pm)
    nc = messaging.log_from_bytes(pm.sent['longitudinalPlanSP']).longitudinalPlanSP.noClose
    self.assertFalse(nc.leadPresent)
    self.assertFalse(nc.tracked)
    self.assertFalse(nc.applies)
    self.assertEqual(nc.factor, 1.)
    self.assertEqual(nc.postCapAccel, nc.preCapAccel)


class TestZeroBehaviourChange(OpenpilotTestCase):
  """The bar: the logging must not move the published ask by one bit. Same inputs, same planner, fill on vs fill off."""

  def _drive(self, disable_fill, n=200, v_ego=17., d0=60., gap_rate=+1.0, seed=1):
    rng = np.random.default_rng(seed)
    noise = rng.normal(0., 0.7, n)
    p = make_planner()
    if disable_fill:
      p.publish_fork_plan_log = lambda plan_sp: None    # exactly the pre-2026-11-19 publish path
    pm = FakePM()
    asks = []
    for i in range(n):
      vrel = 1.0 + noise[i]
      sm = FakeSM(planner_sm(v_ego, 0.5, d0 + gap_rate * i * DT, vrel))
      p.update(sm)
      p.publish(sm, pm)
      asks.append(struct.pack('<d', float(p.output_a_target)))
    return b''.join(asks)

  def test_filling_the_fields_cannot_move_the_ask(self):
    for gap_rate, seed in ((-1.0, 3), (0.0, 4), (+1.0, 1)):
      with self.subTest(gap_rate=gap_rate):
        on = self._drive(False, gap_rate=gap_rate, seed=seed)
        off = self._drive(True, gap_rate=gap_rate, seed=seed)
        self.assertEqual(len(on), len(off))
        self.assertEqual(on, off, 'the log-only fill changed the published ask stream')
