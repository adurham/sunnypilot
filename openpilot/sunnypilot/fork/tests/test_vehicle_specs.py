"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.fork.vehicle_specs import FORK_VEHICLE_SPECS, apply_fork_vehicle_specs

BASE_ELANTRA_STEER_RATIO = 12.9
ELANTRA_N_STEER_RATIO = 12.2


def _car(fingerprint: str) -> structs.CarParams:
  """A CarParams in the state card.py has it when the fork hook runs."""
  cp = structs.CarParams()
  cp.carFingerprint = fingerprint
  cp.steerRatio = BASE_ELANTRA_STEER_RATIO
  cp.wheelSpeedFactor = 1.0  # what CarInterfaceBase.get_std_params sets before the hook runs
  return cp


class TestForkVehicleSpecs(OpenpilotTestCase):
  def test_elantra_n_specs_override_inherited_base_elantra_values(self):
    cp = _car("HYUNDAI_ELANTRA_2022_NON_SCC")

    self.assertTrue(apply_fork_vehicle_specs(cp))
    self.assertAlmostEqual(cp.steerRatio, ELANTRA_N_STEER_RATIO, places=3)
    self.assertNotAlmostEqual(cp.steerRatio, BASE_ELANTRA_STEER_RATIO, places=1)

  def test_every_spec_key_is_a_live_carparams_field(self):
    """Regression: the table once carried "mass", which upstream removed from CarParams
    (it lives under CarParams.deprecated now). Assigning it raised "struct has no such member"
    inside card, which crash-looped at every ignition and the car never started. Every key in
    the table must be a real, assignable field of the CarParams schema the build ships."""
    fields = set(structs.CarParams.schema.fields)
    for fingerprint, spec in FORK_VEHICLE_SPECS.items():
      for key in spec:
        with self.subTest(fingerprint=fingerprint, key=key):
          self.assertIn(key, fields, f"CarParams has no field {key!r}; card would crash at startup")

  def test_apply_runs_on_a_real_carparams_without_raising(self):
    """The exact call card makes (set_car_specific_params -> apply_fork_vehicle_specs) on a
    CarParams built by the Elantra N interface, not a hand-seeded one."""
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.car.hyundai.values import CAR
    CP = CarInterface.get_non_essential_params(CAR.HYUNDAI_ELANTRA_2022_NON_SCC)
    self.assertTrue(apply_fork_vehicle_specs(CP))
    self.assertAlmostEqual(CP.steerRatio, ELANTRA_N_STEER_RATIO, places=3)

  def test_other_platforms_are_left_alone(self):
    for fingerprint in ("HYUNDAI_ELANTRA_2021", "HYUNDAI_ELANTRA", "TOYOTA_COROLLA_TSS2"):
      with self.subTest(fingerprint=fingerprint):
        cp = _car(fingerprint)
        self.assertFalse(apply_fork_vehicle_specs(cp))
        self.assertAlmostEqual(cp.steerRatio, BASE_ELANTRA_STEER_RATIO, places=3)

  def test_unknown_fingerprint_is_a_noop(self):
    cp = _car("")
    self.assertFalse(apply_fork_vehicle_specs(cp))
    self.assertAlmostEqual(cp.steerRatio, BASE_ELANTRA_STEER_RATIO, places=3)

  def test_elantra_n_wheel_speed_factor(self):
    """GPS-measured (routes 127/128): true speed = 1.0125 x wheel-speed vEgo. Other platforms keep upstream's 1.0."""
    cp = _car("HYUNDAI_ELANTRA_2022_NON_SCC")
    apply_fork_vehicle_specs(cp)
    self.assertAlmostEqual(cp.wheelSpeedFactor, 1.0125, places=6)

    other = _car("HYUNDAI_ELANTRA_2021")
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
