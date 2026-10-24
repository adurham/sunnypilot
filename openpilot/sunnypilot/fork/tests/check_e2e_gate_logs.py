#!/usr/bin/env python3
"""
Logged-signal harness for the cycle-1 far-lead e2e gate (longitudinal_planner.LongitudinalPlanner.update_e2e_gate).

Replays ONLY the gate decision over recorded drives, open-loop: every input comes from the rlogs of the planner
frame being replayed:
  radarState.leadOne (present/dRel/vRel/vLead/aLeadK), carState (vEgo/aEgo/steeringAngleDeg/vCruise),
  vehicleParameters.angleOffsetDeg, modelV2.action (desiredAcceleration/shouldStop, matched by modelMonoTime),
  longitudinalPlan (source/aTarget/shouldStop/accelBoost/speeds/accels), liveMapDataSP.roadType (adaptive follow),
  selfdriveState (enabled/personality), carControl.longActive, controlsState, longitudinalPlanSP.vTarget.

Per planner frame:
  mpc        = get_accel_from_plan(longitudinalPlan.speeds/accels)  (== output_a_target_mpc; checked on mpc frames)
  t_follow   = AdaptiveFollow replayed with the logged param state (== what mpc.update() got)
  gate       = the REAL update_e2e_gate() on (leadOne, vEgo, raw e2e, mpc, model shouldStop, t_follow)
  e2e_old/new= AccelBoost(logged total) o turn-limit(G4) o {raw | gated raw}   (old checked against aTarget)
  plan_new   = plan_old when the logged source was not e2e (raising e2e cannot win a min it already lost),
               else clip(min(mpc, cruise, e2e_new)); cruise is not logged: it is reconstructed from its recursion and
               floored at the logged aTarget (it lost the old min, so it was >= it).
The closed loop (the car would then drive differently, and AccelBoost eligibility could change) is NOT simulated.

Usage: check_e2e_gate_logs.py [--routes 00000167--0bfe2c0c45 00000168--d69017f041] [--out DIR]
"""
import argparse
import json
import os
import re
import sys
from concurrent.futures import ProcessPoolExecutor
from types import SimpleNamespace

import numpy as np

ROUTES_DIR = os.path.expanduser('~/comma-routes')
WS = os.path.expanduser('~/comma-sim/wsSTOP/feel-167-168')
DEFAULT_ROUTES = ['00000167--0bfe2c0c45', '00000168--d69017f041']
SRC = {'cruise': 0, 'lead0': 1, 'lead1': 2, 'lead2': 3, 'e2e': 4}
COLS = ['t', 'a_old', 'src', 'ss_old', 'boost', 'mpc', 'md_mono', 'md_ss', 'md_found', 'l_present', 'l_drel', 'l_vrel',
        'l_vlead', 'v', 'a_ego', 'steer', 'vcruise_kph', 'offset', 'enabled', 'pers', 'long_active', 'lcs_off',
        'force_decel', 'vtarget_sp', 'road', 'seg', 'l_alead']
MD_A = len(COLS)  # the raw model accel is appended after COLS at route level
ROAD = {'urban': 0, 'highway': 1, 'interstate': 2, 'unknown': 3}
ROAD_INV = {v: k for k, v in ROAD.items()}


def extract_segment(args):
  """One segment -> per-longitudinalPlan-frame rows (state = latest message of each service before the plan)."""
  route, seg = args
  from openpilot.tools.lib.logreader import LogReader
  from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
  from openpilot.selfdrive.modeld.constants import ModelConstants
  from openpilot.common.realtime import DT_MDL
  t_idx = ModelConstants.T_IDXS[:CONTROL_N]
  st = {'l': (0, 0., 0., 0.), 'cs': (0., 0., 0., 255.), 'off': 0., 'en': 0, 'pers': 1, 'la': 0, 'lcs_off': 1, 'fd': 0, 'vt': 255.,
            'road': 3}
  st_l_alead = 0.
  rows, models, meta = [], [], {}
  for m in LogReader(f'{ROUTES_DIR}/{route}--{seg}/rlog.zst'):
    w = m.which()
    if w == 'carParams':
      cp = m.carParams
      meta.update(delay=cp.longitudinalActuatorDelay, steerRatio=cp.steerRatio, wheelbase=cp.wheelbase,
                  brand=cp.brand)
    elif w == 'carParamsSP':
      meta['gas_interceptor'] = bool(m.carParamsSP.enableGasInterceptor)
    elif w == 'initData':
      for e in m.initData.params.entries:
        if e.key == 'AdaptiveFollowDistance':
          meta['adaptive_follow'] = bytes(e.value) == b'1'
    elif w == 'modelV2':
      a = m.modelV2.action
      models.append((float(m.logMonoTime), a.desiredAcceleration, float(a.shouldStop)))
    elif w == 'radarState':
      ld = m.radarState.leadOne
      st['l'] = (int(ld.present), ld.dRel, ld.vRel, ld.vLead)
      st_l_alead = ld.aLeadK
    elif w == 'carState':
      c = m.carState
      st['cs'] = (c.vEgo, c.aEgo, c.steeringAngleDeg, c.vCruise)
    elif w == 'vehicleParameters':
      st['off'] = m.vehicleParameters.angleOffsetDeg
    elif w == 'selfdriveState':
      st['en'] = int(m.selfdriveState.enabled)
      st['pers'] = int(m.selfdriveState.personality.raw)
    elif w == 'carControl':
      st['la'] = int(m.carControl.longActive)
    elif w == 'controlsState':
      st['lcs_off'] = int(str(m.controlsState.longControlState) == 'off')
      st['fd'] = int(m.controlsState.forceDecel)
    elif w == 'longitudinalPlanSP':
      st['vt'] = m.longitudinalPlanSP.vTarget
    elif w == 'liveMapDataSP':
      st['road'] = ROAD.get(str(m.liveMapDataSP.roadType), 3)
    elif w == 'longitudinalPlan':
      p = m.longitudinalPlan
      if 'delay' not in meta:
        continue
      mpc = get_accel_from_plan(np.array(p.speeds), np.array(p.accels), t_idx, action_t=meta['delay'] + DT_MDL) \
        if len(p.speeds) == CONTROL_N else np.nan
      rows.append((m.logMonoTime * 1e-9, p.aTarget, SRC.get(str(p.longitudinalPlanSource), 9), int(p.shouldStop),
                   p.accelBoost, mpc, float(p.modelMonoTime), 0, 0,
                   *st['l'], *st['cs'][:3], st['cs'][3], st['off'], st['en'], st['pers'], st['la'], st['lcs_off'],
                   st['fd'], st['vt'], st['road'], seg, st_l_alead))
  return (np.array(rows, dtype=np.float64).reshape(-1, len(COLS)), np.array(models, dtype=np.float64).reshape(-1, 3),
          meta)


def extract_route(route, cache_dir):
  cache = os.path.join(cache_dir, f'{route[:8]}.npz')
  meta_f = cache[:-4] + '.meta.json'
  if os.path.exists(cache) and os.path.exists(meta_f):
    rows = np.load(cache)['rows']
    if rows.shape[1] == len(COLS) + 1:  # a cache from an older column set is re-extracted
      return rows, json.load(open(meta_f))
  segs = sorted(int(re.match(r'.*--(\d+)$', d).group(1)) for d in os.listdir(ROUTES_DIR)
                if d.startswith(route + '--') and os.path.exists(f'{ROUTES_DIR}/{d}/rlog.zst'))
  with ProcessPoolExecutor(max_workers=min(12, os.cpu_count() or 4)) as ex:
    res = list(ex.map(extract_segment, [(route, s) for s in segs]))
  meta = {}
  for _, _, mt in res:
    for k, v in mt.items():
      meta.setdefault(k, v)
  rows = np.concatenate([r for r, _, _ in res if len(r)])
  rows = rows[np.argsort(rows[:, 0], kind='stable')]
  # match each plan to the modelV2 it was computed from (plan.modelMonoTime == that modelV2's logMonoTime); done at
  # route level because the rlog is not strictly time-ordered and a plan can reference the previous segment's model
  models = np.concatenate([m for _, m, _ in res if len(m)])
  models = models[np.argsort(models[:, 0], kind='stable')]
  C = {k: i for i, k in enumerate(COLS)}
  k = np.clip(np.searchsorted(models[:, 0], rows[:, C['md_mono']]), 0, len(models) - 1)
  found = models[k, 0] == rows[:, C['md_mono']]
  rows = np.column_stack([rows, np.where(found, models[k, 1], np.nan)])  # last column: raw model accel
  rows[:, C['md_ss']] = np.where(found, models[k, 2], 0.)
  rows[:, C['md_found']] = found
  rows = rows[found]  # a plan without its model message cannot be replayed (counted in meta)
  meta['plans_without_model'] = int((~found).sum())
  meta['segments'] = segs
  os.makedirs(cache_dir, exist_ok=True)
  np.savez_compressed(cache, rows=rows)
  json.dump(meta, open(meta_f, 'w'))
  return rows, meta


class _GateState:
  def __init__(self):
    self.e2e_gate = False
    self.e2e_gate_cnt = 0
    self.e2e_gate_alead = []


class _Params:
  def __init__(self, on):
    self.on = on

  def get_bool(self, key):
    return self.on


def replay(rows, meta):
  """Sequential replay of the gate (and of AdaptiveFollow / the cruise candidate, which carry state)."""
  from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
  from openpilot.common.constants import CV
  from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
  from openpilot.selfdrive.controls.lib.accel_boost import ACCEL_BOOST_MIN_SPEED
  from openpilot.sunnypilot.fork.adaptive_follow import AdaptiveFollow
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW
  from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
  C = {k: i for i, k in enumerate(COLS)}
  CP = SimpleNamespace(steerRatio=meta['steerRatio'], wheelbase=meta['wheelbase'])
  throttle_only = bool(meta.get('gas_interceptor')) and meta.get('brand') == 'hyundai'
  af = AdaptiveFollow(throttle_only, params=_Params(bool(meta.get('adaptive_follow'))))
  g = _GateState()
  n = len(rows)
  out = {k: np.zeros(n) for k in ('gate', 'e2e_old', 'e2e_new', 'raw_new', 'tl', 'tf', 'cruise', 'a_new', 'src_new')}
  a_cruise = 0.
  for i in range(n):
    r = rows[i]
    v = r[C['v']]
    lead = SimpleNamespace(present=bool(r[C['l_present']]), dRel=r[C['l_drel']], vRel=r[C['l_vrel']],
                           aLeadK=r[C['l_alead']])
    tf = af.update(int(r[C['pers']]), v, ROAD_INV[int(r[C['road']])], lead.present, r[C['l_vlead']])
    if tf is None:
      tf = get_T_FOLLOW(int(r[C['pers']]))
    raw = r[MD_A]
    mpc = r[C['mpc']]
    raw_new = lp.LongitudinalPlanner.update_e2e_gate(g, lead, v, raw, mpc, bool(r[C['md_ss']]), tf)
    tl = max(lp.turn_accel_limit(v, r[C['steer']] - r[C['offset']], CP), 0.3)
    speed_scale = np.interp(v, [ACCEL_BOOST_MIN_SPEED, 2 * ACCEL_BOOST_MIN_SPEED], [0.0, 1.0])

    def boost(x, _B=r[C['boost']], _s=speed_scale):
      return x + _B * _s * np.interp(x, [-1.0, -0.5, 5.0], [0.0, 1.0, 1.0], right=0.0)
    e2e_old = boost(min(raw, tl))
    e2e_new = boost(min(raw_new, tl))
    # cruise candidate recursion (not logged): reconstructed, used only where the old source was e2e
    reset = bool(r[C['lcs_off']]) or r[C['vcruise_kph']] == V_CRUISE_UNSET
    if reset:
      a_cruise = float(np.clip(r[C['a_ego']], ACCEL_MIN, ACCEL_MAX))
    v_cruise = 0. if r[C['force_decel']] else min(r[C['vtarget_sp']], min(r[C['vcruise_kph']], V_CRUISE_MAX) * CV.KPH_TO_MS)
    a_cruise = lp.get_cruise_accel(v_cruise, v, a_cruise, lp.DT_MDL)
    a_old = r[C['a_old']]
    if raw_new == raw or int(r[C['src']]) != SRC['e2e']:
      # the gate did not change e2e (the real planner is then bit-identical), or e2e already lost the old min
      # (raising it cannot win): the logged plan stands
      a_new, src_new = a_old, r[C['src']]
    else:
      # e2e won the logged min, so logged aTarget IS the old e2e candidate (pre-clip). The gate's effect is applied as
      # a delta on that logged value so turn-limit / boost reconstruction noise (vehicleParameters / boost timing)
      # cannot leak into the result; the chain is monotone, so delta >= 0 whenever raw_new >= raw.
      e2e_new_logged = a_old + (e2e_new - e2e_old)
      cruise_eff = max(a_cruise, a_old)  # cruise lost the old min, so it was >= the logged aTarget
      cands = [(mpc, SRC['lead0']), (cruise_eff, SRC['cruise']), (e2e_new_logged, SRC['e2e'])]
      m, src_new = min(cands, key=lambda c: c[0])
      a_new = float(np.clip(m, ACCEL_MIN, ACCEL_MAX))
    for k, val in (('gate', g.e2e_gate), ('e2e_old', e2e_old), ('e2e_new', e2e_new), ('raw_new', raw_new), ('tl', tl),
                   ('tf', tf), ('cruise', a_cruise), ('a_new', a_new), ('src_new', src_new)):
      out[k][i] = val
  return out


def load_phantom_onsets(r8, t0):
  """The phantom-hunt planner episodes (phantoms-slow/res-<r8>.json), onset times on the log clock."""
  res = json.load(open(f'{WS}/phantoms-slow/res-{r8}.json'))
  return [dict(e, t_abs=e['t'] + t0) for e in res['episodes'] if e['cls'] == 'planner']


def main():
  from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE
  ap = argparse.ArgumentParser()
  ap.add_argument('--routes', nargs='+', default=DEFAULT_ROUTES)
  ap.add_argument('--out', default=os.path.join(WS, 'cycle1-gate'))
  a = ap.parse_args()
  os.makedirs(a.out, exist_ok=True)
  C = {k: i for i, k in enumerate(COLS)}
  stops_all = json.load(open(f'{WS}/stops/stops.json'))['stops']
  report = {}
  pooled = {'onsets_4c_model': [], 'onsets_lead_e2e': [], 'pull': [0, 0, 0, 0]}
  for route in a.routes:
    r8 = route[:8]
    rows, meta = extract_route(route, os.path.join(a.out, 'cache'))
    o = replay(rows, meta)
    t = rows[:, C['t']]
    v = rows[:, C['v']]
    src = rows[:, C['src']].astype(int)
    a_old, a_new = rows[:, C['a_old']], o['a_new']
    raw = rows[:, MD_A]
    mpc = rows[:, C['mpc']]
    eng = (rows[:, C['enabled']] > 0.5) & (rows[:, C['long_active']] > 0.5)
    lead = rows[:, C['l_present']] > 0.5
    gate = o['gate'] > 0.5
    R = {'frames': int(len(rows)), 'engaged_frames': int(eng.sum()), 'meta': {k: v_ for k, v_ in meta.items() if k != 'segments'},
             'n_segments': len(meta['segments']), 'model_match': float(rows[:, C['md_found']].mean())}
    # --- harness fidelity: reconstructions vs the logged plan
    m_mpc = src == SRC['lead0']
    m_e2e = src == SRC['e2e']
    m_cru = src == SRC['cruise']
    R['fidelity'] = {
      'mpc_vs_aTarget_on_lead0_frames_p99_abs': float(np.percentile(np.abs(mpc[m_mpc] - a_old[m_mpc]), 99)) if m_mpc.any() else None,
      'e2e_vs_aTarget_on_e2e_frames_p99_abs': float(np.percentile(np.abs(o['e2e_old'][m_e2e] - a_old[m_e2e]), 99)),
      'e2e_vs_aTarget_on_e2e_frames_max_abs': float(np.abs(o['e2e_old'][m_e2e] - a_old[m_e2e]).max()),
      'cruise_vs_aTarget_on_cruise_frames_p90_abs': float(np.percentile(np.abs(o['cruise'][m_cru] - a_old[m_cru]), 90)) if m_cru.any() else None,
      'cruise_reconstruction_below_logged_on_e2e_frames': int((o['cruise'][m_e2e] < a_old[m_e2e]).sum())}
    # --- 1. raise-only
    R['gate_frames'] = int(gate.sum())
    R['gate_frames_engaged'] = int((gate & eng).sum())
    R['e2e_changed_frames'] = int((o['raw_new'] != raw).sum())
    R['plan_changed_frames'] = int((a_new != a_old).sum())
    R['plan_changed_frames_engaged'] = int(((a_new != a_old) & eng).sum())
    R['e2e_new_lt_raw_violations'] = int((o['raw_new'] < raw).sum())
    R['e2e_post_new_lt_old_violations'] = int((o['e2e_new'] < o['e2e_old']).sum())
    R['plan_new_lt_old_frames'] = int((a_new < a_old).sum())
    R['plan_new_lt_old_frames_tol1e-6'] = int((a_new < a_old - 1e-6).sum())
    R['e2e_chain_delta_negative'] = int(((o['e2e_new'] - o['e2e_old']) < 0).sum())
    R['plan_new_ge_old_frames'] = int((a_new >= a_old).sum())
    R['gate_on_excluded_frames'] = {'no_lead': int((gate & ~lead).sum()), 'model_shouldStop': int((gate & (rows[:, C['md_ss']] > 0.5)).sum()),
                                        'v_le_3': int((gate & (v <= 3.)).sum()),
                                        'changed_non_e2e_frames': int(((a_new != a_old) & ~m_e2e).sum())}
    # --- 3. onset coverage (phantom-hunt planner episodes, onset = first ask <= -0.3 / DecCmd >= 4 sample)
    t0 = float(np.load(f'{WS}/phantoms-slow/ts-{r8}.npz')['cs'][0, 0])
    eps = load_phantom_onsets(r8, t0)

    def at_onset(e, t=t, gate=gate, a_old=a_old, a_new=a_new, mpc=mpc, v=v):
      i = max(int(np.searchsorted(t, e['t_abs'], side='right')) - 1, 0)
      j = int(np.searchsorted(t, e['t_abs'] + e['dur_s'], side='right'))
      return {'wall': e['wall'], 'why': e['why'][:2], 'lead': e['lead'], 'lp_src': e['lp_src'], 'binder': e['binder'],
                  'gate': bool(gate[i]), 'gate_any_in_episode': bool(gate[i:max(j, i + 1)].any()),
                  'ask_old': round(float(a_old[i]), 3), 'ask_new': round(float(a_new[i]), 3),
                  'min_old': round(float(a_old[i:max(j, i + 1)].min()), 3), 'min_new': round(float(a_new[i:max(j, i + 1)].min()), 3),
                  'gap_ratio': e['gap_ratio'], 'ttc': e['ttc'], 'vrel': e['vrel'], 'mpc': round(float(mpc[i]), 3), 'v': round(float(v[i]), 2),
                  'miss': None if gate[i] else miss_reason(i)}

    def miss_reason(i, rows=rows, o=o, v=v, mpc=mpc):
      # which ENTER condition(s) failed at the onset frame ('enter count' = all held but not yet for 10 frames)
      dd = STOP_DISTANCE + o['tf'][i] * v[i]
      dr, vr = rows[i, C['l_drel']], rows[i, C['l_vrel']]
      ttc = dr / max(-vr, 0.1)
      checks = ((rows[i, C['l_present']] < 0.5, 'no lead'), (v[i] <= 3., 'v<=3'),
                (rows[i, C['md_ss']] > 0.5, 'model shouldStop'), (dr <= 1.2 * dd, f'gap {dr / dd:.2f}x<=1.2'),
                (ttc <= 12., f'ttc {ttc:.1f}<=12'), (vr <= -3., f'vRel {vr:.1f}<=-3'),
                (mpc[i] < -0.1, f'mpc {mpc[i]:.2f}<-0.1'),
                (rows[i, C['l_alead']] <= lp.E2E_GATE_STAY_ALEAD,
                 f'aLeadK {rows[i, C["l_alead"]]:.2f}<={lp.E2E_GATE_STAY_ALEAD}'),
                (float(np.median(rows[max(i - lp.E2E_GATE_ENTER_ALEAD_MEDIAN_FRAMES + 1, 0):i + 1, C['l_alead']])) <=
                 lp.E2E_GATE_ENTER_ALEAD, f'median aLeadK <={lp.E2E_GATE_ENTER_ALEAD}'))
      why = [txt for bad, txt in checks if bad]
      return ', '.join(why) or 'enter count < 10 frames'
    o4c = [at_onset(e) for e in eps if e['why'].startswith('4c') and e['binder'] == 'model']
    olead = [at_onset(e) for e in eps if e['lead'] and e['lp_src'] == 'e2e']
    R['onsets_4c_model_bound'] = o4c
    R['onsets_lead_present_e2e_sourced'] = olead
    pooled['onsets_4c_model'] += o4c
    pooled['onsets_lead_e2e'] += olead
    # --- 4. pull-away follow frames
    pull = eng & lead & (rows[:, C['l_vrel']] > 0.) & (mpc > 0.3)
    pull_e2e = pull & m_e2e
    no_longer = pull_e2e & (o['src_new'] != SRC['e2e'])
    R['pullaway'] = {'frames': int(pull.sum()), 'e2e_min_old': int(pull_e2e.sum()), 'e2e_not_min_new': int(no_longer.sum()),
                         'frac': float(no_longer.sum() / max(pull_e2e.sum(), 1)),
                         'e2e_min_new': int((pull & (o['src_new'] == SRC['e2e'])).sum())}
    for k, val in enumerate((pull.sum(), pull_e2e.sum(), no_longer.sum(), (pull & (o['src_new'] == SRC['e2e'])).sum())):
      pooled['pull'][k] += int(val)
    # --- 5. aTarget < -0.5
    R['atarget_lt_m0p5'] = {'engaged_old': int((eng & (a_old < -0.5)).sum()), 'engaged_new': int((eng & (a_new < -0.5)).sum()),
                                'all_old': int((a_old < -0.5).sum()), 'all_new': int((a_new < -0.5).sum())}
    x = R['atarget_lt_m0p5']
    x['engaged_reduction'] = 1. - x['engaged_new'] / max(x['engaged_old'], 1)
    x['all_reduction'] = 1. - x['all_new'] / max(x['all_old'], 1)
    # why the remaining engaged aTarget < -0.5 frames were not gated (first matching reason, in this order)
    rem = eng & (a_new < -0.5)
    dd = STOP_DISTANCE + o['tf'] * v
    ttc = rows[:, C['l_drel']] / np.maximum(-rows[:, C['l_vrel']], 0.1)
    reasons = (('source not e2e (mpc/cruise)', ~m_e2e), ('no lead', ~lead), ('model shouldStop', rows[:, C['md_ss']] > 0.5),
               ('v<=3', v <= 3.), ('mpc < -0.1', mpc < -0.1), ('gap <= 1.2x', rows[:, C['l_drel']] <= 1.2 * dd),
               ('ttc <= 12 s', ttc <= 12.), ('vRel <= -3', rows[:, C['l_vrel']] <= -3.),
               ('gated, but cruise/mpc still < -0.5', gate), ('enter count < 10', np.ones_like(gate)))
    left = rem.copy()
    x['remaining_engaged_by_reason'] = {}
    for name, msk in reasons:
      k = left & msk
      x['remaining_engaged_by_reason'][name] = int(k.sum())
      left &= ~k
    # --- 2. stops: shouldStop unchanged; first ask <= -0.5 before standstill
    st = [s for s in stops_all if s['route'] == r8]
    still_t = t[v < 0.3]
    srows = []
    for s in st:
      ts = s['t_stop']
      prev = still_t[still_t < ts - 3.]
      w0 = max(ts - 30., prev.max() if len(prev) else -1e9)
      w = (t >= w0) & (t <= ts)
      fo = t[w & (a_old <= -0.5)]
      fn = t[w & (a_new <= -0.5)]
      sw = (t >= ts - 30.) & (t <= ts + 3.)
      srows.append({'cdt': s['cdt'], 'seg': s['seg'], 'engaged': s['engaged'], 'has_ask': s['has_ask'], 't_stop': ts,
                        'first_ask_old_s': round(float(ts - fo[0]), 2) if len(fo) else None,
                        'first_ask_new_s': round(float(ts - fn[0]), 2) if len(fn) else None,
                        'gate_frames': int((gate & sw).sum()), 'plan_changed': int(((a_new != a_old) & sw).sum()),
                        'model_shouldStop_frames': int((rows[:, C['md_ss']][sw] > 0.5).sum()),
                        'logged_shouldStop_frames': int((rows[:, C['ss_old']][sw] > 0.5).sum())})
    R['stops'] = srows
    eng_st = [x for x in srows if x['engaged'] and x['has_ask']]
    R['stops_summary'] = {
      'n_engaged_asked': len(eng_st),
      'first_ask_new_ge_5s': sum(1 for x in eng_st if x['first_ask_new_s'] is not None and x['first_ask_new_s'] >= 5.),
      'first_ask_old_ge_5s': sum(1 for x in eng_st if x['first_ask_old_s'] is not None and x['first_ask_old_s'] >= 5.),
      'first_ask_unchanged': sum(1 for x in eng_st if x['first_ask_new_s'] == x['first_ask_old_s']),
      'first_ask_later': sum(1 for x in eng_st if x['first_ask_old_s'] is not None and
                          (x['first_ask_new_s'] is None or x['first_ask_new_s'] < x['first_ask_old_s']))}
    # shouldStop: the gate cannot reach it (output_should_stop = any of the three candidate flags; none use e2e accel)
    R['shouldStop_changed_frames'] = 0
    R['shouldStop_note'] = 'structural: should_stop flags are model shouldStop + should_stop(v, mpc) + should_stop(v, cruise)'
    report[r8] = R
    np.savez_compressed(os.path.join(a.out, f'replay-{r8}.npz'), t=t, a_old=a_old, a_new=a_new, gate=gate, raw=raw,
                        raw_new=o['raw_new'], mpc=mpc, src=src, src_new=o['src_new'], eng=eng)

  def cov(xs):
    return {'n': len(xs), 'gate_at_onset': sum(x['gate'] for x in xs), 'gate_any': sum(x['gate_any_in_episode'] for x in xs),
                'lead_present': sum(x['lead'] for x in xs),
                'gate_at_onset_of_lead_present': sum(x['gate'] for x in xs if x['lead']),
                'ask_removed': sum(x['ask_new'] > -0.3 for x in xs),
                'episode_min_now_gt_m0p3': sum(x['min_new'] > -0.3 for x in xs)}
  p = pooled['pull']
  report['pooled'] = {'onsets_4c_model_bound': cov(pooled['onsets_4c_model']),
                          'onsets_lead_present_e2e_sourced': cov(pooled['onsets_lead_e2e']),
                          'onsets_lead_present_e2e_sourced_167': cov(list(report.get('00000167', {}).get(
                            'onsets_lead_present_e2e_sourced', []))),
                          'pullaway': {'frames': p[0], 'e2e_min_old': p[1], 'e2e_not_min_new': p[2], 'frac': p[2] / max(p[1], 1)}}
  json.dump(report, open(os.path.join(a.out, 'report.json'), 'w'), indent=1, default=float)
  for r8, R in report.items():
    if r8 == 'pooled':
      continue
    print(f'== {r8}: frames {R["frames"]} engaged {R["engaged_frames"]} segs {R["n_segments"]} model-match {R["model_match"]:.4f}')
    print('  fidelity', json.dumps(R['fidelity']))
    print('  gate frames', R['gate_frames'], 'engaged', R['gate_frames_engaged'], '| e2e changed', R['e2e_changed_frames'],
          '| plan changed', R['plan_changed_frames'], 'engaged', R['plan_changed_frames_engaged'])
    print('  violations: e2e_new<raw', R['e2e_new_lt_raw_violations'], 'e2e_post_new<old', R['e2e_post_new_lt_old_violations'],
          'plan_new<old', R['plan_new_lt_old_frames'], '(tol1e-6:', R['plan_new_lt_old_frames_tol1e-6'], ') new>=old',
          R['plan_new_ge_old_frames'], '/', R['frames'])
    print('  excluded-frame touches', R['gate_on_excluded_frames'])
    print('  pullaway', R['pullaway'])
    print('  aTarget<-0.5', {k: v_ for k, v_ in R['atarget_lt_m0p5'].items() if k != 'remaining_engaged_by_reason'})
    print('  remaining engaged aTarget<-0.5 by reason', R['atarget_lt_m0p5']['remaining_engaged_by_reason'])
    for s in R['stops']:
      print('  stop', s)
    for x in R['onsets_4c_model_bound']:
      print('  onset4c', x['wall'], 'gate' if x['gate'] else 'MISS', x['ask_old'], '->', x['ask_new'], x['miss'] or '')
  print('== pooled', json.dumps(report['pooled'], indent=1))


if __name__ == '__main__':
  sys.exit(main())
