"""
Fork-specific vehicle spec overrides (adurham/sunnypilot).

Why this exists
---------------
The owner's car — a 2022 Hyundai Elantra N (DCT) — fingerprints as
``HYUNDAI_ELANTRA_2022_NON_SCC``. Upstream sunnypilot/opendbc configures that
platform with *base Elantra* specs (see ``HYUNDAI_ELANTRA_2021``):

    mass=2800 lb, wheelbase=2.72, steerRatio=12.9, tireStiffnessFactor=0.65

The Elantra N is a different car: ~3296 lb curb weight (DCT) and a quicker
12.2:1 steering rack. Since this fork is the deploy target for that car, apply
the real numbers here instead of living with the inherited ones.

Where it runs
-------------
Called from ``set_car_specific_params()``, which ``card.py`` invokes *before*
CarParams is persisted (CarParams / CarParamsCache / CarParamsPersistent) and
published on the ``carParams`` message. Mutating the CarParams object at that
point means every consumer — controlsd's VehicleModel, paramsd, torqued, the
UI — sees the same values, with no desync between processes.

Do not move this to a downstream consumer (controlsd/torqued): those read the
already-persisted CarParams, and mutating only there would disagree with what
card computed.

Sources for the numbers
-----------------------
* Curb weight 3,296 lb (2022 Elantra N DCT): Hyundai 2022 Elantra product
  guide (N DCT curb 3,296 lb), Edmunds, MotorTrend first-test spec panels.
* Overall steering gear ratio 12.2:1: Hyundai Elantra N published steering
  specification (rack-and-pinion, 2.2 turns lock-to-lock).
* wheelbase 2.72 m and tireStiffnessFactor 0.65 are already correct for the N
  (same CN7 platform, 107.1 in wheelbase) — deliberately not overridden.
"""
from opendbc.car import STD_CARGO_KG, structs

LB_TO_KG = 0.453592

# Keyed by the fingerprint string the car actually resolves to at runtime.
#
# mass is CURB weight, matching how platform CarSpecs express it. This hook runs
# after CarInterfaceBase.get_params() has already added STD_CARGO_KG (136 kg,
# upstream's assumed driver/payload), so the same allowance is added here to
# land on the identical convention: curb + cargo.
FORK_VEHICLE_SPECS: dict[str, dict[str, float]] = {
  "HYUNDAI_ELANTRA_2022_NON_SCC": {
    "mass": 3296 * LB_TO_KG,  # [kg] N DCT curb weight (base Elantra spec: 2800 lb)
    "steerRatio": 12.2,       # [] N rack ratio (base Elantra spec: 12.9)
  },
}


def apply_fork_vehicle_specs(CP: structs.CarParams) -> bool:
  """Apply fork spec overrides for this car. Returns True if anything changed."""
  spec = FORK_VEHICLE_SPECS.get(str(CP.carFingerprint))
  if spec is None:
    return False

  CP.mass = spec["mass"] + (0. if CP.notCar else STD_CARGO_KG)
  CP.steerRatio = spec["steerRatio"]
  return True
