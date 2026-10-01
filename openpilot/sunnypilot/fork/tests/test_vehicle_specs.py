"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.fork.vehicle_specs import FORK_VEHICLE_SPECS, LB_TO_KG, apply_fork_vehicle_specs

BASE_ELANTRA_MASS_LB = 2800
BASE_ELANTRA_STEER_RATIO = 12.9


def _car(fingerprint: str) -> structs.CarParams:
  cp = structs.CarParams()
  cp.carFingerprint = fingerprint
  # seed with the values upstream would produce for this platform
  cp.mass = BASE_ELANTRA_MASS_LB * LB_TO_KG
  cp.steerRatio = BASE_ELANTRA_STEER_RATIO
  return cp


class TestForkVehicleSpecs(OpenpilotTestCase):
  def test_elantra_n_specs_override_inherited_base_elantra_values(self):
    cp = _car("HYUNDAI_ELANTRA_2022_NON_SCC")

    self.assertTrue(apply_fork_vehicle_specs(cp))
    self.assertAlmostEqual(cp.mass, 3296 * LB_TO_KG, places=3)
    self.assertAlmostEqual(cp.steerRatio, 12.2, places=3)
    # the point of the override: it must differ from the inherited spec
    self.assertNotAlmostEqual(cp.mass, BASE_ELANTRA_MASS_LB * LB_TO_KG, places=1)
    self.assertNotAlmostEqual(cp.steerRatio, BASE_ELANTRA_STEER_RATIO, places=1)

  def test_other_platforms_are_left_alone(self):
    for fingerprint in ("HYUNDAI_ELANTRA_2021", "HYUNDAI_ELANTRA", "TOYOTA_COROLLA_TSS2"):
      with self.subTest(fingerprint=fingerprint):
        cp = _car(fingerprint)
        self.assertFalse(apply_fork_vehicle_specs(cp))
        self.assertAlmostEqual(cp.mass, BASE_ELANTRA_MASS_LB * LB_TO_KG, places=3)
        self.assertAlmostEqual(cp.steerRatio, BASE_ELANTRA_STEER_RATIO, places=3)

  def test_unknown_fingerprint_is_a_noop(self):
    cp = _car("")
    self.assertFalse(apply_fork_vehicle_specs(cp))
    self.assertAlmostEqual(cp.mass, BASE_ELANTRA_MASS_LB * LB_TO_KG, places=3)

  def test_spec_table_only_carries_expected_fields(self):
    for fingerprint, spec in FORK_VEHICLE_SPECS.items():
      with self.subTest(fingerprint=fingerprint):
        self.assertLessEqual(set(spec), {"mass", "steerRatio"})
