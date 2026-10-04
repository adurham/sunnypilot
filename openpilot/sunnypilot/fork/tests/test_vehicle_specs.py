"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import STD_CARGO_KG, structs

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.fork.vehicle_specs import FORK_VEHICLE_SPECS, LB_TO_KG, apply_fork_vehicle_specs

BASE_ELANTRA_MASS_LB = 2800
BASE_ELANTRA_STEER_RATIO = 12.9
ELANTRA_N_MASS_LB = 3296
ELANTRA_N_STEER_RATIO = 12.2


def _car(fingerprint: str) -> structs.CarParams:
  """A CarParams in the exact state card.py has it when the fork hook runs.

  By that point CarInterfaceBase.get_params() has already set the platform spec
  mass and added STD_CARGO_KG, so the seed must include it — comparing against a
  bare curb weight would let a missing cargo allowance pass unnoticed.
  """
  cp = structs.CarParams()
  cp.carFingerprint = fingerprint
  cp.mass = BASE_ELANTRA_MASS_LB * LB_TO_KG + STD_CARGO_KG
  cp.steerRatio = BASE_ELANTRA_STEER_RATIO
  return cp


class TestForkVehicleSpecs(OpenpilotTestCase):
  def test_elantra_n_specs_override_inherited_base_elantra_values(self):
    cp = _car("HYUNDAI_ELANTRA_2022_NON_SCC")

    self.assertTrue(apply_fork_vehicle_specs(cp))
    self.assertAlmostEqual(cp.mass, ELANTRA_N_MASS_LB * LB_TO_KG + STD_CARGO_KG, places=3)
    self.assertAlmostEqual(cp.steerRatio, ELANTRA_N_STEER_RATIO, places=3)
    # the point of the override: it must differ from the inherited spec
    self.assertNotAlmostEqual(cp.mass, BASE_ELANTRA_MASS_LB * LB_TO_KG + STD_CARGO_KG, places=1)
    self.assertNotAlmostEqual(cp.steerRatio, BASE_ELANTRA_STEER_RATIO, places=1)

  def test_cargo_allowance_is_preserved(self):
    """The override must keep upstream's assumed payload, not replace it.

    Regression guard: setting mass to a bare curb weight here would silently drop
    STD_CARGO_KG, because this hook runs *after* get_params() added it.
    """
    cp = _car("HYUNDAI_ELANTRA_2022_NON_SCC")
    before = cp.mass - BASE_ELANTRA_MASS_LB * LB_TO_KG
    self.assertAlmostEqual(before, STD_CARGO_KG, places=3)

    apply_fork_vehicle_specs(cp)
    after = cp.mass - ELANTRA_N_MASS_LB * LB_TO_KG
    self.assertAlmostEqual(after, STD_CARGO_KG, places=3)

  def test_other_platforms_are_left_alone(self):
    for fingerprint in ("HYUNDAI_ELANTRA_2021", "HYUNDAI_ELANTRA", "TOYOTA_COROLLA_TSS2"):
      with self.subTest(fingerprint=fingerprint):
        cp = _car(fingerprint)
        self.assertFalse(apply_fork_vehicle_specs(cp))
        self.assertAlmostEqual(cp.mass, BASE_ELANTRA_MASS_LB * LB_TO_KG + STD_CARGO_KG, places=3)
        self.assertAlmostEqual(cp.steerRatio, BASE_ELANTRA_STEER_RATIO, places=3)

  def test_unknown_fingerprint_is_a_noop(self):
    cp = _car("")
    self.assertFalse(apply_fork_vehicle_specs(cp))
    self.assertAlmostEqual(cp.mass, BASE_ELANTRA_MASS_LB * LB_TO_KG + STD_CARGO_KG, places=3)

  def test_spec_table_only_carries_expected_fields(self):
    for fingerprint, spec in FORK_VEHICLE_SPECS.items():
      with self.subTest(fingerprint=fingerprint):
        self.assertLessEqual(set(spec), {"mass", "steerRatio", "wheelSpeedFactor"})

  def test_elantra_n_wheel_speed_factor(self):
    """GPS-measured (routes 127/128): true speed = 1.0125 x wheel-speed vEgo. Other platforms keep upstream's 1.0."""
    cp = _car("HYUNDAI_ELANTRA_2022_NON_SCC")
    cp.wheelSpeedFactor = 1.0  # what CarInterfaceBase.get_std_params sets before the hook runs
    apply_fork_vehicle_specs(cp)
    self.assertAlmostEqual(cp.wheelSpeedFactor, 1.0125, places=6)

    other = _car("HYUNDAI_ELANTRA_2021")
    other.wheelSpeedFactor = 1.0
    apply_fork_vehicle_specs(other)
    self.assertAlmostEqual(other.wheelSpeedFactor, 1.0, places=6)

  def test_wheel_speed_factor_reaches_car_state(self):
    """The override must reach vEgoRaw through the real CarState path (card mutates CP after CarState is built)."""
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.car.hyundai.values import CAR
    CP = CarInterface.get_non_essential_params(CAR.HYUNDAI_ELANTRA_2022_NON_SCC)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HYUNDAI_ELANTRA_2022_NON_SCC)
    CI = CarInterface(CP, CP_SP)
    apply_fork_vehicle_specs(CI.CP)
    self.assertIs(CI.CS.CP, CI.CP)
    ret = structs.CarState()
    CI.CS.parse_wheel_speeds(ret, 100., 100., 100., 100.)
    self.assertAlmostEqual(ret.vEgoRaw, 100. / 3.6 * 1.0125, places=4)
