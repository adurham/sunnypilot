"""Elantra N (HYUNDAI_ELANTRA_2022_NON_SCC) faster steering-torque ramp: car-layer gating and C/Python consistency.

STEER_MAX stays 384 everywhere; only STEER_DELTA_UP changes (3 -> 4) and only for this platform, and the panda is told via
HyundaiSafetyFlagsSP.CN7_STEER_RAMP, which must be set for this platform only.
"""
import re
from pathlib import Path

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR, CarControllerParams
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP

CN7 = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
SAFETY = Path(__file__).resolve().parents[4] / "safety" / "modes"


def _params(car):
  CarInterface = interfaces[car]
  fp = gen_empty_fingerprint()
  CP = CarInterface.get_params(car, fp, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, car, fp, [], alpha_long=False, is_release_sp=False, docs=False)
  return CP, CP_SP


def test_cn7_controller_params():
  CP, _ = _params(CN7)
  p = CarControllerParams(CP)
  assert (p.STEER_MAX, p.STEER_DELTA_UP, p.STEER_DELTA_DOWN) == (384, 4, 7)
  assert (p.STEER_DRIVER_ALLOWANCE, p.STEER_DRIVER_MULTIPLIER, p.STEER_DRIVER_FACTOR) == (50, 2, 1)


def test_cn7_sets_panda_bit():
  _, CP_SP = _params(CN7)
  assert CP_SP.safetyParam & HyundaiSafetyFlagsSP.CN7_STEER_RAMP
  assert CP_SP.safetyParam & HyundaiSafetyFlagsSP.NON_SCC  # panda only honors the bit together with NON_SCC


@pytest.mark.parametrize("car", sorted(c for c in CAR if c != CN7 and c.startswith(("HYUNDAI", "KIA", "GENESIS"))))
def test_other_hkg_unchanged(car):
  CP, CP_SP = _params(car)
  p = CarControllerParams(CP)
  assert p.STEER_DELTA_UP in (2, 3)  # 2 for CAN-FD / ALT_LIMITS*, 3 otherwise: never 4
  assert p.STEER_MAX in (170, 255, 270, 384)
  assert not CP_SP.safetyParam & HyundaiSafetyFlagsSP.CN7_STEER_RAMP


def test_controller_and_panda_rate_agree():
  src = (SAFETY / "hyundai.h").read_text()
  m = re.search(r"HYUNDAI_STEERING_LIMITS_CN7_RAMP\s*=\s*HYUNDAI_LIMITS\((\d+),\s*(\d+),\s*(\d+)\)", src)
  assert m is not None
  CP, _ = _params(CN7)
  p = CarControllerParams(CP)
  assert tuple(int(x) for x in m.groups()) == (p.STEER_MAX, p.STEER_DELTA_UP, p.STEER_DELTA_DOWN)
  # the real-time window must still admit a continuous max-rate ramp: 25 frames per 250 ms at 100 Hz
  rt_m = re.search(r"\.max_rt_delta\s*=\s*(\d+)", src)
  assert rt_m is not None
  assert p.STEER_DELTA_UP * 25 < int(rt_m.group(1))


def test_cn7_ramp_time():
  CP, _ = _params(CN7)
  p = CarControllerParams(CP)
  last, frames = 0, 0
  while last < p.STEER_MAX:
    last = apply_driver_steer_torque_limits(p.STEER_MAX, last, 0.0, p)
    frames += 1
  assert frames == 96  # 0 -> 384 in 0.96 s (was 128 frames / 1.28 s at 3/frame)
