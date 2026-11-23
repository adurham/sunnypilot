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
  * ``postCapAccel`` is the CAP's output, not the published request: every stage after the cap (any optional later
    stage, the standstill-launch hold, the ACCEL_MIN/MAX clip) runs AFTER it; on the 2026-11-19 drive the hold and the
    clip made it differ from ``longitudinalPlan.aTarget`` on 231 frames (224 held at 0 with shouldStop, 7 clipped) --
    the schema says so and ``TestPostCapIsTheCapsOutput`` pins both cases;
  * filling the fields cannot change the ask: the same drive with the fill disabled is bit-identical, frame for frame;
  * a failure ANYWHERE in the fill cannot change the ask either, and the struct is FULLY filled or ALL defaults, never
    half-filled: every value is converted and range/enum-checked before the first capnp write, a write that still fails
    resets the struct to its defaults, and the error log itself cannot raise (``TestFillFailureCannotTouchThePlan``).
"""
import json
import logging
import os
import random
import re
import struct
import tempfile
from pathlib import Path
from unittest import mock

import capnp
import numpy as np

from opendbc.car.interfaces import ACCEL_MIN
from openpilot.cereal import custom, log, messaging
from openpilot.common.logging_extra import SwagFormatter
from openpilot.common.test import OpenpilotTestCase
import openpilot.selfdrive.controls.lib.longitudinal_planner as lp
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

  def test_the_defaults_read_as_no_cap_on_a_message_that_never_set_them(self):
    """A message written before the fields existed (or any message that leaves them alone) must read factor/rawFactor = 1
    -- "no cap" -- and not 0, which would read as "fully capped" to a review of an old log."""
    nc = messaging.new_message('longitudinalPlanSP').longitudinalPlanSP.noClose
    self.assertEqual(nc.factor, 1.0)
    self.assertEqual(nc.rawFactor, 1.0)
    self.assertFalse(nc.enabled)
    self.assertFalse(nc.applies)

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
      # `ask` is the PUBLISHED aTarget (after the standstill-launch hold and the ACCEL_MIN/MAX clip); postCapAccel is the
      # cap's output, before both. This drive never reaches either stage, so here the two must agree -- the stages that
      # make them differ are pinned by TestPostCapIsTheCapsOutput, not assumed away by this line.
      self.assertAlmostEqual(nc.postCapAccel, ask, places=6,
                             msg='no hold and no clip in this drive: the cap output is the published ask here')
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


class _RaisingBias:
  """A transparent stand-in for the closing filter EXCEPT that reading ``bias`` raises: a bad value in the MIDDLE of the
  fill (after 14 conversions). Every other attribute, including ``update()``, is forwarded to the real filter, so the
  cap computes exactly what it computed without the injection."""

  def __init__(self, inner):
    self._inner = inner

  def __getattr__(self, name):
    if name == 'bias':
      raise RuntimeError('injected: vRelBias is unreadable this frame')
    return getattr(self._inner, name)


NC = custom.LongitudinalPlanSP.NoClose
NC_FIELDS = list(NC.schema.fieldnames)      # the 23 fields in ordinal order = the order the fill writes them


def _nc_read(raw: bytes) -> dict:
  """Every noClose field of a published longitudinalPlanSP, read the way a review reads the rlog. Decoding the whole
  event (``to_dict``) is part of the read: a preCapSource that is not an enumerant writes fine but makes it raise."""
  ev = messaging.log_from_bytes(raw)
  ev.to_dict(verbose=True)
  nc = ev.longitudinalPlanSP.noClose
  return {f: (int(nc.preCapSource.raw) if f == 'preCapSource' else getattr(nc, f)) for f in NC_FIELDS}


def _nc_defaults() -> dict:
  return _nc_read(messaging.new_message('longitudinalPlanSP').to_bytes())


def _set(getter, attr, make):
  """A fault for `_frames`: ``getter(p).attr = make(old)``; the returned undo restores ``old``."""
  def inject(p):
    obj = getter(p)
    old = getattr(obj, attr)
    setattr(obj, attr, make(old))
    return lambda: setattr(obj, attr, old)
  return inject


class _WriteFailingStruct:
  """The REAL noClose builder, except that its k-th field write raises: a capnp write failing part-way through the
  fill. Reads and the k-1 writes before it reach the real builder, so without a rollback those fields stay written."""

  def __init__(self, real, k):
    object.__setattr__(self, '_real', real)
    object.__setattr__(self, '_k', k)
    object.__setattr__(self, '_n', 0)

  def __getattr__(self, name):
    return getattr(object.__getattribute__(self, '_real'), name)

  def __setattr__(self, name, value):
    n = object.__getattribute__(self, '_n') + 1
    object.__setattr__(self, '_n', n)
    if n == object.__getattribute__(self, '_k'):
      raise RuntimeError(f'injected: capnp write #{n} ({name}) fails')
    setattr(object.__getattribute__(self, '_real'), name, value)


class _WriteFailingPlanSP:
  """The real longitudinalPlanSP builder with `noClose` seen through ``_WriteFailingStruct``; everything else, init()
  included, is the real builder, so a reset really resets the struct that gets published."""

  def __init__(self, real, k):
    self._real, self._k = real, k

  def __getattr__(self, name):
    value = getattr(self._real, name)
    return _WriteFailingStruct(value, self._k) if name == 'noClose' else value


def _write_fails_at(k):
  def inject(p):
    fill = type(p).publish_fork_plan_log.__get__(p)
    p.publish_fork_plan_log = lambda plan_sp: fill(_WriteFailingPlanSP(plan_sp, k))
    return lambda: delattr(p, 'publish_fork_plan_log')
  return inject


class _UnprintableError(Exception):
  """An exception whose text cannot be built: ``str()`` of it raises."""

  def __str__(self):
    raise RuntimeError('injected: str() of this exception raises')


class _Unfloatable:
  def __float__(self):
    raise _UnprintableError()


class _InterruptingBool:
  def __bool__(self):
    raise KeyboardInterrupt('injected')


_CAPNP_TYPES = {'bool': 'Bool', 'float32': 'Float32', 'uint8': 'UInt8', 'enum': 'Candidate'}


def _short_noclose_module(n_fields: int, tmpdir: str):
  """A REAL capnp schema whose NoClose has only the first ``n_fields`` fields of the real one (same names, ordinals,
  types, defaults): the fill's next write then names a member that does not exist and pycapnp raises -- a real write
  failure part-way through the fill, the kind a schema out of step with this planner produces."""
  lines = []
  for f in list(NC.schema.node.struct.fields)[:n_fields]:     # a pycapnp list does not slice
    kind = f.slot.type.which()
    default = f' = {f.slot.defaultValue.float32:g}' if kind == 'float32' and f.slot.defaultValue.float32 != 0 else ''
    lines.append(f'    {f.name} @{f.ordinal.explicit} :{_CAPNP_TYPES[kind]}{default};')
  path = os.path.join(tmpdir, f'short_noclose_{n_fields}.capnp')
  with open(path, 'w') as fh:
    fh.write(f'@0x{random.getrandbits(63) | (1 << 63):016x};\nstruct Sp {{\n  noClose @0 :NoClose;\n  struct NoClose {{\n'
             '    enum Candidate { cruise @0; lead0 @1; lead1 @2; lead2 @3; e2e @4; }\n' + '\n'.join(lines) + '\n  }\n}\n')
  return capnp.load(path)


class _SwaglogCapture(logging.Handler):
  """Formats each record with the REAL swaglog formatter -- the JSON line logmessaged would write to the swaglog."""

  def __init__(self):
    super().__init__(logging.ERROR)
    self.setFormatter(SwagFormatter(lp.cloudlog))
    self.lines = []

  def emit(self, record):
    if str(record.msg).startswith('fork: planner log fields'):
      self.lines.append(self.format(record))


class TestFillFailureCannotTouchThePlan(OpenpilotTestCase):
  """The fields are diagnostics: a failure ANYWHERE in the fill must leave the published plan byte-identical to what the
  pre-2026-11-19 publish path publishes, keep the process alive, leave the struct FULLY filled or ALL at its schema
  defaults -- never half-filled -- and log once, with the traceback, without the log call itself being able to raise.

  FAIL-BEFORE (the 2026-11-19 land): the 23 writes ran outside the guard and ``plannerd`` has no try around publish(),
  so an exception here kills the planner process. The plan itself is already sent (publish() sends longitudinalPlan
  first), but the frame's longitudinalPlanSP is lost and every frame after it with it. FAIL-BEFORE (47c55b0ae6, the
  first hardening): preCapSource was not checked against its enum and a failing write was not rolled back, so the
  struct could go out half-filled or undecodable, and the error line formatted ``str(e)`` outside any guard."""

  N = 6

  def _drive(self, break_fill=None):
    p = make_planner()
    if break_fill is None:
      p.publish_fork_plan_log = lambda plan_sp: None     # exactly the pre-2026-11-19 publish path
    pm = FakePM()
    plans = []
    for i in range(self.N):
      sm = FakeSM(planner_sm(17., 0.3, 80. - 1.0 * i * DT, 0.0))
      p.update(sm)
      if break_fill is not None:
        break_fill(p)                                    # a "bad value" that only the fill reads
      p.publish(sm, pm)                                  # must not raise
      plans.append(pm.sent['longitudinalPlan'])
    return pm, plans

  @staticmethod
  def _plan_canon(raw: bytes) -> str:
    """Every published plan field, exact floats, minus the three CLOCK fields: the event's own logMonoTime, the
    processingDelay derived from it and acados' solverExecutionTime. Those are wall-clock measurements taken inside
    new_message()/the solver, not planner outputs (the A6 replay replaces the host clock with the log clock for the same
    reason); everything the planner computes -- aTarget, shouldStop, source, speeds, accels, jerks, boost, fcw -- stays."""
    d = messaging.log_from_bytes(raw).to_dict(verbose=True)['longitudinalPlan']
    for k in ('solverExecutionTime', 'processingDelay'):
      d.pop(k, None)
    return json.dumps(d, sort_keys=True, allow_nan=True, separators=(',', ':'))

  def _assert_schema_defaults(self, pm):
    nc = messaging.log_from_bytes(pm.sent['longitudinalPlanSP']).longitudinalPlanSP.noClose
    self.assertEqual(nc.factor, 1.0, 'factor must read the schema default 1 ("no cap") when the fill failed')
    self.assertEqual(nc.rawFactor, 1.0)
    for name in ('enabled', 'leadPresent', 'leadUsable', 'tracked', 'applies', 'filterValid', 'filterCoasting',
                 'gateReject', 'reinit', 'e2eGate'):
      self.assertFalse(getattr(nc, name), f'{name} must be at its schema default False when the fill failed')
    for name in ('preCapAccel', 'preCapSource', 'postCapAccel', 'dwellFrames', 'closingSpeed', 'filteredClosing',
                 'rawMeanClosing', 'vRelBias', 'filterRejects', 'e2eGated', 'e2eFinal'):
      self.assertEqual(getattr(nc, name), 0, f'{name} must be at its schema default 0 when the fill failed')

  def test_any_bad_value_in_the_fill_cannot_change_the_plan(self):
    control = [self._plan_canon(x) for x in self._drive()[1]]
    injections = {
      'a coercion raises (float(object()))': lambda p: setattr(p.no_close, 'factor', object()),
      'a filter property raises mid-fill': lambda p: setattr(p.no_close, 'lead_filter', _RaisingBias(p.no_close.lead_filter)),
      'dwellFrames above UInt8': lambda p: setattr(p.no_close, 'dwell_frames', 300),
      'dwellFrames negative': lambda p: setattr(p.no_close, 'dwell_frames', -1),
      'filterRejects above UInt8': lambda p: setattr(p.no_close.lead_filter, 'rejects', 999),
    }
    for name, inject in injections.items():
      with self.subTest(injection=name):
        with mock.patch.object(lp, 'cloudlog') as clog:
          pm, plans = self._drive(inject)
        self.assertEqual([self._plan_canon(x) for x in plans], control,
                         f'{name}: a fill failure must publish the fill-disabled plan, field for field')
        self._assert_schema_defaults(pm)
        self.assertEqual(clog.error.call_count, 1, f'{name}: the guard must log once per planner, not once per frame')

  def test_a_transient_failure_does_not_latch_the_fill_off(self):
    p = make_planner()
    pm = FakePM()
    sm = FakeSM(planner_sm(17., 0.3, 80., 0.0))
    p.update(sm)
    p.no_close.factor = object()
    p.publish(sm, pm)
    self._assert_schema_defaults(pm)
    pm2 = FakePM()
    sm2 = FakeSM(planner_sm(17., 0.3, 79., 0.0))
    p.update(sm2)
    p.publish(sm2, pm2)
    nc = messaging.log_from_bytes(pm2.sent['longitudinalPlanSP']).longitudinalPlanSP.noClose
    self.assertTrue(nc.leadPresent, 'the fill must recover on the next frame (the guard is not a latch)')

  def test_a_schema_without_the_appended_field_is_a_no_op(self):
    p = make_planner()
    p.update(FakeSM(planner_sm(17., 0.3, 80., 0.0)))
    p.publish_fork_plan_log(object())      # the pre-append schema: there is no `.noClose` member to write

  # -- R1/R2 of the 2026-11-20 Opus review: FULL or ALL defaults, never half-filled; the log call cannot escape --------

  def _frames(self, inject=None, every=1):
    """N frames through the real update()+publish(). ``inject(p)`` plants a fault that only the fill reads, after
    update() on frames ``i % every == 0``, and returns its undo, called after publish() so the planner enters the next
    frame exactly as the control does. publish() must not raise (on plannerd that ends the planner process). Returns
    ``[(plan canon, noClose dict)]``, one per frame, both decoded the way a review decodes the rlog."""
    p = make_planner()
    pm = FakePM()
    out = []
    for i in range(self.N):
      sm = FakeSM(planner_sm(17., 0.3, 80. - 1.0 * i * DT, 0.0))
      p.update(sm)
      undo = inject(p) if inject is not None and i % every == 0 else None
      try:
        p.publish(sm, pm)
      finally:
        if undo is not None:
          undo()
      out.append((self._plan_canon(pm.sent['longitudinalPlan']), _nc_read(pm.sent['longitudinalPlanSP'])))
    return out

  def _assert_full_or_defaults(self, name, got, control, faulted_frames):
    defaults = _nc_defaults()
    for i, ((plan, nc), (plan0, nc0)) in enumerate(zip(got, control, strict=True)):
      self.assertEqual(plan, plan0, f'{name}: frame {i}: a fill failure moved the published plan')
      if i in faulted_frames:
        self.assertEqual(nc, defaults, f'{name}: frame {i}: a failed fill must publish ALL schema defaults, never a '
                                       f'half-filled struct')
      else:
        self.assertEqual(nc, nc0, f'{name}: frame {i}: the next good frame must fill completely (the guard is no latch)')

  def test_an_unloggable_value_or_a_failing_write_leaves_all_defaults_never_half(self):
    """R1. RED on 47c55b0ae6: preCapSource -1/65536 raised on capnp write #7 and published 6 real fields next to 17
    defaults (factor = 1 "no cap", postCapAccel = 0); 5/7/65535 wrote fine and made every reader's to_dict() raise; a
    write failing at #7/#12/#23 published the first 6/11/22 fields. Faults on frames 0/2/4, clean frames 1/3/5."""
    control = self._frames()
    for i, (_, nc) in enumerate(control):
      # write #1 (enabled) is live on every control frame, so a fill that stopped after ANY write cannot pass for the
      # all-defaults struct this test demands on a faulted frame
      self.assertTrue(nc['enabled'], f'control frame {i}: field #1 must be live, or a half-fill would be invisible')
    pre_cap_src = lambda value: _set(lambda p: p, '_log_pre_cap_src', lambda old: value)
    faults = {
      'preCapSource -1 (pycapnp raises on write #7)': pre_cap_src(-1),
      'preCapSource 5 (the first non-enumerant: writes, cannot be read)': pre_cap_src(5),
      'preCapSource 7 (writes, cannot be read)': pre_cap_src(7),
      'preCapSource 65535 (the last uint16: writes, cannot be read)': pre_cap_src(65535),
      'preCapSource 65536 (pycapnp raises on write #7)': pre_cap_src(65536),
      'capnp write #1 fails': _write_fails_at(1),
      'capnp write #7 (preCapSource) fails': _write_fails_at(7),
      'capnp write #12 (closingSpeed) fails': _write_fails_at(12),
      'capnp write #23 (e2eFinal, the last) fails': _write_fails_at(23),
    }
    for name, inject in faults.items():
      with self.subTest(fault=name):
        with mock.patch.object(lp, 'cloudlog') as clog:
          got = self._frames(inject, every=2)
        self._assert_full_or_defaults(name, got, control, faulted_frames={0, 2, 4})
        self.assertEqual(clog.error.call_count, 1, f'{name}: one error line per planner, not one per failed frame')

  def test_a_real_capnp_write_failure_part_way_leaves_all_defaults(self):
    """R1 with a REAL pycapnp failure instead of a proxy: a NoClose schema that lacks every field from #k on (a schema
    out of step with this planner). Write #k raises AttributeError ("no such member") after k-1 real writes; the struct
    must still read all defaults. RED on 47c55b0ae6 (the k-1 fields stay written) and on the 2026-11-19 land (escapes)."""
    with tempfile.TemporaryDirectory() as tmp:
      for k in (7, 12, 23):
        with self.subTest(failing_write=k, field=NC_FIELDS[k - 1]):
          short = _short_noclose_module(k - 1, tmp)
          p = make_planner()
          p.update(FakeSM(planner_sm(17., 0.3, 80., 0.0)))
          full = messaging.new_message('longitudinalPlanSP')
          p.publish_fork_plan_log(full.longitudinalPlanSP)      # the same state into the real schema: the control
          live = _nc_read(full.to_bytes())
          self.assertNotEqual({f: live[f] for f in NC_FIELDS[:k - 1]}, {f: _nc_defaults()[f] for f in NC_FIELDS[:k - 1]},
                              'the first k-1 fields must be live, or a half-fill would be invisible here')
          msg = short.Sp.new_message()
          with mock.patch.object(lp, 'cloudlog') as clog:
            p.publish_fork_plan_log(msg)                          # must not raise
          nc = msg.noClose
          got = {f: (int(getattr(nc, f).raw) if f == 'preCapSource' else getattr(nc, f)) for f in NC_FIELDS[:k - 1]}
          self.assertEqual(got, {f: _nc_defaults()[f] for f in NC_FIELDS[:k - 1]},
                           f'write #{k} failed for real: the struct must be reset to ALL defaults, not left half-filled')
          self.assertEqual(clog.error.call_count, 1)
          self.assertEqual(clog.error.call_args.args[1], 'AttributeError', 'the line names the real failure')

  def test_an_exception_whose_text_cannot_be_built_cannot_escape(self):
    """R2. RED on 47c55b0ae6: its error line was an f-string over the exception, built outside any guard, so an
    exception whose __str__ raises escaped publish() -- on plannerd, a dead planner. Run through the REAL cloudlog and
    its real handlers; the swaglog line, formatted by the real SwagFormatter, still carries the traceback."""
    capture = _SwaglogCapture()
    lp.cloudlog.addHandler(capture)
    self.addCleanup(lp.cloudlog.removeHandler, capture)
    control = self._frames()
    got = self._frames(_set(lambda p: p.no_close, 'factor', lambda old: _Unfloatable()))
    self._assert_full_or_defaults('str() raises', got, control, faulted_frames=set(range(self.N)))
    self.assertEqual(len(capture.lines), 1, 'one error line per planner')
    line = json.loads(capture.lines[0])
    self.assertIn('_UnprintableError', line['msg'], 'the line names the exception type')
    self.assertIn('Traceback (most recent call last)', line.get('exc_info', ''), 'the line carries the traceback')
    self.assertIn('publish_fork_plan_log', line['exc_info'], 'the traceback points into the fill')

  def test_a_logger_that_raises_cannot_escape_and_is_tried_once(self):
    """R2. RED on 47c55b0ae6: the cloudlog.error call sat outside any guard, so a logger that raises (a handler whose
    format() fails -- swaglog's ipc handler formats outside its own try) escaped publish()."""
    control = self._frames()
    with mock.patch.object(lp, 'cloudlog') as clog:
      clog.error.side_effect = RuntimeError('injected: the logger itself fails')
      got = self._frames(_set(lambda p: p.no_close, 'factor', lambda old: object()))
    self._assert_full_or_defaults('logger raises', got, control, faulted_frames=set(range(self.N)))
    self.assertEqual(clog.error.call_count, 1, 'a failing logger is not retried at 20 Hz either')

  def test_a_keyboard_interrupt_is_not_swallowed(self):
    """``except Exception``, never ``BaseException``: an interrupt inside the fill must still end the process."""
    p = make_planner()
    p.update(FakeSM(planner_sm(17., 0.3, 80., 0.0)))
    setattr(p.no_close, 'enabled', _InterruptingBool())
    with self.assertRaises(KeyboardInterrupt):
      p.publish_fork_plan_log(messaging.new_message('longitudinalPlanSP').longitudinalPlanSP)


class TestPostCapIsTheCapsOutput(OpenpilotTestCase):
  """``postCapAccel`` is the NoClose cap's OUTPUT, and ``longitudinalPlan.aTarget`` is taken after every stage that runs
  after the cap (any optional later stage, the standstill-launch hold, the ACCEL_MIN/MAX clip), so the two are different
  quantities: on the 2026-11-19 drive they differ on 231 of 128,136 frames (224 held at 0 by the standstill-launch
  debounce, 7 clipped at ACCEL_MIN). The 2026-11-19 land's chain test asserted equality as a law, which only held
  because its synthetic drive never reaches either stage; these two cases pin what the field is."""

  def _published(self, p, sm):
    pm = FakePM()
    p.update(sm)
    p.publish(sm, pm)
    plan = messaging.log_from_bytes(pm.sent['longitudinalPlan']).longitudinalPlan
    nc = messaging.log_from_bytes(pm.sent['longitudinalPlanSP']).longitudinalPlanSP.noClose
    return float(plan.aTarget), bool(plan.shouldStop), nc

  def test_the_accel_min_clip_sits_between_the_field_and_the_published_ask(self):
    p = make_planner()      # init_v=17: no standstill, so the launch debounce is a pure pass-through
    with mock.patch.object(lp, 'get_accel_from_plan', lambda *a, **k: -4.0):    # an MPC candidate past ACCEL_MIN
      ask, _, nc = self._published(p, FakeSM(planner_sm(17., 0.3, 80., 0.0)))
    self.assertEqual(ask, ACCEL_MIN, 'the published ask is the clipped value')
    self.assertAlmostEqual(nc.postCapAccel, -4.0, places=6,
                           msg='postCapAccel carries the cap output BEFORE the clip, as custom.capnp says')

  def test_a_held_stop_publishes_0_while_the_cap_output_is_positive(self):
    from openpilot.sunnypilot.fork.tests.test_standstill_leadloss import (make_planner as stop_planner,
                                                                         planner_sm as stop_sm, mpc_as_logged, held_stop)
    p = stop_planner()      # init_v=0
    with mpc_as_logged():   # the MPC candidate the car computed on the 194 #8 hold (+2.559): the harness MPC cannot solve at 0
      held_stop(p)          # 40 stop frames: the debounce is holding
      ask, stop, nc = self._published(p, FakeSM(stop_sm(0., 0.3, should_stop=False, standstill=True)))
    self.assertTrue(stop, 'the standstill-launch hold rewrites this go frame to a hold')
    self.assertEqual(ask, 0.0, 'a held frame publishes 0 ...')
    self.assertGreater(nc.postCapAccel, 0., '... while the cap already saw the positive candidate')
    self.assertNotAlmostEqual(nc.postCapAccel, ask, places=6,
                              msg='postCapAccel != aTarget here, exactly as custom.capnp documents')
