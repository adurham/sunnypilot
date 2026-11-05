"""
Fork: a robust closing-speed estimate for the lead (adurham/sunnypilot).

Why this exists
---------------
NoCloseCap (fork/no_close.py) used to decide "are we closing on the lead?" from the RAW per-frame ``vEgo - vLead`` of
radarState.leadOne. On this car every lead is vision-only (radard.get_RadarState_from_vision): vRel is a one-frame model
regression, not a measured range rate. Measured on routes 189-194 (lane D, 2026-11-04, vrel_range.py / eval_filter.py):

  * it UNDER-READS closing at range: over 4 s windows in which the gap itself shrank faster than 0.5 m/s, vRel still
    said "not closing" (> -0.25 m/s) on 42/183 windows at 30-50 m, 71/243 at 50-70 m and 55/201 at 70-100 m;
  * it JITTERS: the frame-to-frame standard deviation of vEgo - vLead is 0.41-0.72 m/s at 20 Hz, i.e. larger than
    NoCloseCap's whole 0.35 m/s ramp. Route 194 t=429-431: raw closing -1.35 / +0.57 / -0.69 / +0.93 / -0.68 on
    consecutive frames at 80 m; the cap factor followed it 1 -> 0 -> 1 -> 0, the ask went 0.40 -> 0.04 -> 0.64 -> 0.00
    inside one second and the pedal 19 -> 11 -> 16 -> 10 % - the owner's "gas flutter" (complaint 4).

What it does
------------
A two-state Kalman filter on the lead's gap, state ``[d, r]`` (gap m, gap rate m/s; closing = -r), fed by BOTH
measurements every frame: dRel and vRel, each with a range-dependent noise. The gap's change over time is the physical
truth of "closing"; vRel only speeds the response up. On the six routes this cut the frame-to-frame jitter of the
closing estimate 4-8x (0.41-0.72 -> 0.06-0.16 m/s) and raised the hit rate on true closing > 0.5 m/s (gap measured over
the next 3 s) on 5 of 6 routes (e.g. 18b: 1475 -> 2077 frames detected, misses 2259 -> 1657) at an unchanged bias.

Track hygiene:
  * innovation gate on dRel: a range more than GATE_SIGMA sigmas (and GATE_MIN_M) from the prediction is a different
    object (vision alternates between two cars ~20 m apart at 70-100 m on 194) and is not fused; after REINIT_FRAMES
    consecutive rejections the track re-initialises on the new object;
  * a lead that drops out for up to HOLD_FRAMES is coasted, not reset (176-366 dropouts per engaged hour on these
    routes, 40-60 % of them 1-10 frames long); a longer absence resets;
  * a new track starts at (dRel, vRel): its first closing value is exactly the raw ``-vRel`` the old cap used.
"""
import math

import numpy as np

from openpilot.common.realtime import DT_MDL

# measurement noise (1 sigma), range dependent
D_SIGMA_BASE = 0.6       # m at 0 m
D_SIGMA_PER_M = 0.02     # m per m of range (~2.6 m at 100 m)
V_SIGMA_BASE = 0.8       # m/s at 0 m
V_SIGMA_PER_M = 0.012    # m/s per m of range (~2.0 m/s at 100 m). Swept 0.012/0.025/0.05: 0.012 lowest jitter
# process noise: relative acceleration (lead accel minus ours) as white noise, 1 sigma
A_REL_SIGMA = 1.0        # m/s^2 (swept 0.5/1.0/2.0; 1.0 = the middle, the misses/false-alarm trade is flat around it)
R0_SIGMA = 2.0           # m/s, extra prior rate uncertainty of a new track (the first frames follow vRel)
# vRel is modelled as  rate + bias  with a slowly wandering bias: a vision lead's vRel is BIASED for seconds at a time
# (194 t=329-335: +0.7..+1.3 while the gap shrank ~3.8 m/s), so the long-term rate must come from the gap itself
B0_SIGMA = 1.0           # m/s, prior bias uncertainty of a new track
B_WALK_SIGMA = 0.3       # m/s per sqrt(s), bias random walk
GATE_SIGMA = 4.0
GATE_MIN_M = 4.0
REINIT_FRAMES = 10       # 0.5 s of consistently rejected range at 20 Hz -> it is a new lead
HOLD_FRAMES = 10         # a dropout up to 0.5 s is coasted, not reset


def d_sigma(d: float) -> float:
  return D_SIGMA_BASE + D_SIGMA_PER_M * max(d, 0.)


def v_sigma(d: float) -> float:
  return V_SIGMA_BASE + V_SIGMA_PER_M * max(d, 0.)


class LeadClosingFilter:
  """``update(lead_present, d_rel, v_rel)`` once per planner frame (DT_MDL); then read ``valid``, ``closing`` (m/s,
  > 0 = the gap is shrinking), ``coasting`` (True while a short dropout is being bridged). Pure numpy, no I/O."""

  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    # state [gap d (m), gap rate r (m/s), vRel bias b (m/s)];  dRel = d,  vRel = r + b
    self._F = np.array([[1., dt, 0.], [0., 1., 0.], [0., 0., 1.]])
    G = np.array([[0.5 * dt * dt], [dt], [0.]])
    self._Q = (G @ G.T) * A_REL_SIGMA ** 2
    self._Q[2, 2] = B_WALK_SIGMA ** 2 * dt
    self.reset()

  def reset(self) -> None:
    self.x = np.zeros(3)
    self.P = np.eye(3)
    self.valid = False
    self.coasting = False
    self.rejects = 0
    self.missed = 0
    self.closing = 0.

  def _init(self, d: float, v: float) -> None:
    self.x = np.array([d, v, 0.])
    # the first rate is the raw vRel (bias 0): r = vRel - b - noise with b ~ N(0, B0^2), so var(r) = B0^2 + sv^2 (+ the
    # extra R0 prior), cov(r, b) = -B0^2. r + b is known to the vRel noise; the split is learnt from the gap's slope.
    sv2 = v_sigma(d) ** 2
    self.P = np.array([[d_sigma(d) ** 2, 0., 0.],
                       [0., B0_SIGMA ** 2 + sv2 + R0_SIGMA ** 2, -B0_SIGMA ** 2],
                       [0., -B0_SIGMA ** 2, B0_SIGMA ** 2]])
    self.valid = True
    self.rejects = 0
    self.missed = 0

  def _predict(self) -> None:
    self.x = self._F @ self.x
    self.P = self._F @ self.P @ self._F.T + self._Q

  def update(self, lead_present: bool, d_rel: float | None, v_rel: float | None) -> float:
    d_ok = d_rel is not None and math.isfinite(d_rel) and d_rel >= 0.
    v_ok = v_rel is not None and math.isfinite(v_rel)
    if not (lead_present and d_ok):
      if self.valid and self.missed < HOLD_FRAMES:
        self.missed += 1
        self.coasting = True
        self._predict()
        self.closing = float(-self.x[1])
      else:
        self.reset()
      return self.closing
    self.coasting = False
    d = float(d_rel)  # type: ignore[arg-type]  # d_ok checked above
    v = float(v_rel) if v_ok else 0.  # type: ignore[arg-type]
    if not self.valid:
      self._init(d, v)
      self.closing = float(-self.x[1])
      return self.closing
    self.missed = 0
    self._predict()
    S = self.P[0, 0] + d_sigma(d) ** 2
    innov = d - self.x[0]
    if abs(innov) > max(GATE_SIGMA * math.sqrt(S), GATE_MIN_M):
      self.rejects += 1
      if self.rejects >= REINIT_FRAMES:
        self._init(d, v)
        self.closing = float(-self.x[1])
        return self.closing
      v_ok = False  # a rejected frame's vRel belongs to the same (other) object: do not fuse it either
    else:
      self.rejects = 0
      K = self.P[:, 0] / S
      self.x = self.x + K * innov
      self.P = self.P - np.outer(K, self.P[0, :])
    if v_ok:
      H = np.array([0., 1., 1.])
      PH = self.P @ H
      S = float(H @ PH) + v_sigma(self.x[0]) ** 2
      K = PH / S
      self.x = self.x + K * (v - (self.x[1] + self.x[2]))
      self.P = self.P - np.outer(K, PH)
    self.closing = float(-self.x[1])
    return self.closing
