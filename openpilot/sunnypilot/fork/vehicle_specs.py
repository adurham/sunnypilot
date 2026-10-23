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
* (Curb weight 3,296 lb is no longer applied: upstream removed CarParams.mass.)
* Overall steering gear ratio 12.2:1: Hyundai Elantra N published steering
  specification (rack-and-pinion, 2.2 turns lock-to-lock).
* wheelbase 2.72 m and tireStiffnessFactor 0.65 are already correct for the N
  (same CN7 platform, 107.1 in wheelbase) — deliberately not overridden.
"""
from opendbc.car import structs

LB_TO_KG = 0.453592

# Keyed by the fingerprint string the car actually resolves to at runtime.
#
# NO "mass" key: upstream removed CarParams.mass (opendbc "Remove car mass", moved to
# CarParams.deprecated.mass in the 922513f312 sync). Vehicle dynamics are mass-normalised
# (rotationalInertiaPerMass / tireStiffness*PerMass), so no consumer reads mass any more, and
# assigning CP.mass raises "struct has no such member" -> card crash-loops at every ignition.
# Add a key here only after checking it exists in opendbc/car/car.capnp (test_vehicle_specs
# asserts every key is a live CarParams field).
FORK_VEHICLE_SPECS: dict[str, dict[str, float]] = {
  "HYUNDAI_ELANTRA_2022_NON_SCC": {
    "steerRatio": 12.2,       # [] N rack ratio (base Elantra spec: 12.9)
    # [] true speed / wheel-speed speed. Measured on routes 00000127+00000128 (46 km): GPS doppler/vEgo 1.0125,
    # GPS position distance/vEgo distance 1.0131 (127: 1.0118/1.0109), flat across 12-36 m/s. Upstream default 1.0
    # made the car run ~1.25 % (0.9 mph at 70) faster than the set speed. Tire-dependent: re-measure after a tire
    # change (fork drive report drive-128-planner-report.md has the method).
    "wheelSpeedFactor": 1.0125,
  },
}


def apply_fork_vehicle_specs(CP: structs.CarParams) -> bool:
  """Apply fork spec overrides for this car. Returns True if anything changed."""
  spec = FORK_VEHICLE_SPECS.get(str(CP.carFingerprint))
  if spec is None:
    return False

  CP.steerRatio = spec["steerRatio"]
  if "wheelSpeedFactor" in spec:
    # CarState reads CP.wheelSpeedFactor live (CarStateBase.parse_wheel_speeds) and holds this same CP object, so the
    # override applies from the first CarState update after card's set_car_specific_params
    CP.wheelSpeedFactor = spec["wheelSpeedFactor"]
  return True
