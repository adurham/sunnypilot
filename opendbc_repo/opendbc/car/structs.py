from dataclasses import dataclass as _dataclass, field, is_dataclass
from enum import Enum, StrEnum as _StrEnum, auto
from typing import dataclass_transform, get_origin

import os
import capnp
from opendbc.car.common.basedir import BASEDIR

capnp.remove_import_hook()
car = capnp.load(os.path.join(BASEDIR, "car.capnp"), imports=[BASEDIR])

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule

# sunnypilot structs

AUTO_OBJ = object()


def auto_field():
  return AUTO_OBJ


@dataclass_transform()
def auto_dataclass(cls=None, /, **kwargs):
  cls_annotations = cls.__dict__.get('__annotations__', {})
  for name, typ in cls_annotations.items():
    current_value = getattr(cls, name)
    if current_value is AUTO_OBJ:
      origin_typ = get_origin(typ) or typ
      if isinstance(origin_typ, str):
        raise TypeError(f"Forward references are not supported for auto_field: '{origin_typ}'. Use a default_factory with lambda instead.")
      elif origin_typ in (int, float, str, bytes, list, tuple, bool) or is_dataclass(origin_typ):
        setattr(cls, name, field(default_factory=origin_typ))
      elif issubclass(origin_typ, Enum):  # first enum is the default
        setattr(cls, name, field(default=next(iter(origin_typ))))
      else:
        raise TypeError(f"Unsupported type for auto_field: {origin_typ}")

  # TODO: use slots, this prevents accidentally setting attributes that don't exist
  return _dataclass(cls, **kwargs)


class StrEnum(_StrEnum):
  @staticmethod
  def _generate_next_value_(name, *args):
    # auto() defaults to name.lower()
    return name


@auto_dataclass
class CarParamsSP:
  flags: int = auto_field()        # flags for car specific quirks
  safetyParam: int = auto_field()  # flags for custom safety flags
  pcmCruiseSpeed: bool = auto_field()
  intelligentCruiseButtonManagementAvailable: bool = auto_field()
  enableGasInterceptor: bool = auto_field()
  fca11Brake: bool = auto_field()  # fork: production FCA11 longitudinal braking (HyundaiFca11Brake, default OFF)
  # 0041: OPT-IN affine command law (HyundaiFca11AffineGain, default OFF). When ON the car layer inverts the fitted
  # affine plant (extra = BITE + K*cmd_g, K~9.9 BITE~0.40) instead of the through-origin 0.67 gain, and the
  # release-band floor drops from 8 to 4 LSB. OFF = byte-identical to 0040. Read once in Fca11LongBrake.__init__.
  fca11AffineGain: bool = auto_field()

  neuralNetworkLateralControl: 'CarParamsSP.NeuralNetworkLateralControl' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl())

  @auto_dataclass
  class NeuralNetworkLateralControl:
    model: 'CarParamsSP.NeuralNetworkLateralControl.Model' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl.Model())
    fuzzyFingerprint: bool = auto_field()

    @auto_dataclass
    class Model:
      path: str = auto_field()
      name: str = auto_field()


@auto_dataclass
class ModularAssistiveDrivingSystem:
  state: 'ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState' = field(
    default_factory=lambda: ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState.disabled
  )
  enabled: bool = auto_field()
  active: bool = auto_field()
  available: bool = auto_field()

  class ModularAssistiveDrivingSystemState(StrEnum):
    disabled = auto()
    paused = auto()
    enabled = auto()
    softDisabling = auto()
    overriding = auto()


@auto_dataclass
class IntelligentCruiseButtonManagement:
  state: 'IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState' = field(
    default_factory=lambda: IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState.inactive
  )
  sendButton: 'IntelligentCruiseButtonManagement.SendButtonState' = field(
    default_factory=lambda: IntelligentCruiseButtonManagement.SendButtonState.none
  )
  vTarget: float = auto_field()

  class IntelligentCruiseButtonManagementState(StrEnum):
    inactive = auto()
    preActive = auto()
    increasing = auto()
    decreasing = auto()
    holding = auto()

  class SendButtonState(StrEnum):
    none = auto()
    increase = auto()
    decrease = auto()


@auto_dataclass
class LeadData:
  dRel: float = auto_field()
  yRel: float = auto_field()
  vRel: float = auto_field()
  aRel: float = auto_field()
  vLead: float = auto_field()
  dPath: float = auto_field()
  vLat: float = auto_field()
  vLeadK: float = auto_field()
  aLeadK: float = auto_field()
  fcw: bool = auto_field()
  status: bool = auto_field()
  aLeadTau: float = auto_field()
  modelProb: float = auto_field()
  radar: bool = auto_field()
  radarTrackId: int = auto_field()

  aLeadDEPRECATED: float = auto_field()


@auto_dataclass
class CarControlSP:
  mads: 'ModularAssistiveDrivingSystem' = field(default_factory=lambda: ModularAssistiveDrivingSystem())
  params: list['CarControlSP.Param'] = auto_field()
  leadOne: 'LeadData' = field(default_factory=lambda: LeadData())
  leadTwo: 'LeadData' = field(default_factory=lambda: LeadData())
  intelligentCruiseButtonManagement: 'IntelligentCruiseButtonManagement' = field(default_factory=lambda: IntelligentCruiseButtonManagement())
  # fork (adurham): raw LongitudinalPersonality (0 = aggressive, 1 = standard, 2 = relaxed) so the Hyundai pedal law can
  # scale its ceilings/gains by the driver's feel dial. Defaults to STANDARD so no-arg construction is never aggressive.
  personality: int = field(default=1)

  @auto_dataclass
  class Param:
    key: str = auto_field()
    value: bytes = auto_field()
    type: 'CarControlSP.ParamType' = field(
      default_factory=lambda: CarControlSP.ParamType.string
    )

  class ParamType(StrEnum):
    string = auto()
    bool = auto()
    int = auto()
    float = auto()
    time = auto()
    json = auto()
    bytes = auto()


@auto_dataclass
class CarStateSP:
  speedLimit: float = auto_field()
  # fork (adurham): raw CLU13 CF_Clu_DriveMode (44|4) — the current drive mode (1 normal / 2 eco /
  # 3 sport / 6 N-custom / 7 N), debounced one frame. 0 = not decoded / unknown. See drive_mode.py.
  driveMode: int = auto_field()
  # 0030 (D1-b): FCA11-long braking is unavailable for the REST of this ignition — the panda's fail-closed
  # hyundai_fca11_long_cut (camera owns FCA11 / pedal fault) is set and only hyundai_init can clear it. The
  # superproject raises a persistent driver alert from this when CP_SP.fca11Brake is on (the feature otherwise
  # fails silently while openpilot keeps planning stops it cannot execute). Coupled with custom.capnp CarStateSP.
  fca11Unavailable: bool = auto_field()
  # 0040: the three owner-approved FCA11 braking alerts. Set by GasInterceptorCarController.create_gas_command (the
  # same 1-frame pattern as fca11_unavailable) and read by the superproject (car_specific.py -> events.py). Coupled
  # with custom.capnp CarStateSP.
  #   superviseStop: steady YELLOW, braking below FCA11_SUPERVISE_KPH ("supervise stop").
  #   stopComplete:  steady YELLOW at/near a full stop while still holding ("stop complete - hold brake pedal").
  #   brakeNow:      persistent RED on ESC non-response or hold-lost; clears ONLY on driver brake ("BRAKE NOW").
  superviseStop: bool = auto_field()
  stopComplete: bool = auto_field()
  brakeNow: bool = auto_field()