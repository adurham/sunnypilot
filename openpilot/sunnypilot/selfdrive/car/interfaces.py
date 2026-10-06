"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from typing import Any

from opendbc.car import structs
from opendbc.car.interfaces import CarInterfaceBase
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.fork.cruise_prefs import remove_unless_preserved
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.helpers import get_nn_model_path
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import set_speed_limit_assist_availability

import openpilot.system.sentry as sentry

from openpilot.sunnypilot.sunnylink.statsd import STATSLOGSP


def log_fingerprint(CP: structs.CarParams) -> None:
  if CP.carFingerprint == "MOCK":
    sentry.capture_fingerprint_mock()
  else:
    sentry.capture_fingerprint(CP.carFingerprint, CP.brand)


def _enforce_torque_lateral_control(CP: structs.CarParams, params: Params | None = None, enabled: bool = False) -> bool:
  if params is None:
    params = Params()

  if CP.steerControlType != structs.CarParams.SteerControlType.angle:
    enabled = params.get_bool("EnforceTorqueControl")

  return enabled


def _initialize_neural_network_lateral_control(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                               params: Params | None = None, enabled: bool = False) -> bool:
  if params is None:
    params = Params()

  nnlc_model_path, nnlc_model_name, exact_match = get_nn_model_path(CP)

  if nnlc_model_name == "MOCK":
    cloudlog.error({"nnlc event": "car doesn't match any Neural Network model"})

  if nnlc_model_name != "MOCK" and CP.steerControlType != structs.CarParams.SteerControlType.angle:
    enabled = params.get_bool("NeuralNetworkLateralControl")

  CP_SP.neuralNetworkLateralControl.model.path = nnlc_model_path
  CP_SP.neuralNetworkLateralControl.model.name = nnlc_model_name
  CP_SP.neuralNetworkLateralControl.fuzzyFingerprint = not exact_match

  return enabled


def _initialize_intelligent_cruise_button_management(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params | None = None) -> None:
  if params is None:
    params = Params()

  icbm_enabled = params.get_bool("IntelligentCruiseButtonManagement")
  if icbm_enabled and CP_SP.intelligentCruiseButtonManagementAvailable and not CP.openpilotLongitudinalControl:
    CP_SP.pcmCruiseSpeed = False


def _initialize_torque_lateral_control(CI: CarInterfaceBase, CP: structs.CarParams, enforce_torque: bool, nnlc_enabled: bool) -> None:
  if nnlc_enabled or enforce_torque:
    CI.configure_torque_tune(CP.carFingerprint, CP.lateralTuning)


def _cleanup_unsupported_params(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params | None = None) -> None:
  if params is None:
    params = Params()

  if params.get_bool("LateralJerkTorqueController") and params.get_bool("NeuralNetworkLateralControl"):
    cloudlog.warning("LateralJerkTorqueController and NeuralNetworkLateralControl both enabled, disabling both")
    params.put_bool("LateralJerkTorqueController", False, block=True)
    params.put_bool("NeuralNetworkLateralControl", False, block=True)

  if CP.steerControlType == structs.CarParams.SteerControlType.angle:
    cloudlog.warning("SteerControlType is angle, cleaning up params")
    params.remove("NeuralNetworkLateralControl")
    params.remove("EnforceTorqueControl")
    params.remove("LateralJerkTorqueController")

  if not CP_SP.intelligentCruiseButtonManagementAvailable or CP.openpilotLongitudinalControl:
    cloudlog.warning("ICBM not available or openpilot Longitudinal Control enabled, cleaning up params")
    params.remove("IntelligentCruiseButtonManagement")

  if not CP.openpilotLongitudinalControl and CP_SP.pcmCruiseSpeed:
    # fork #30: these are the owner's stored cruise preferences. Keep them when longitudinal is
    # only transiently unavailable (pedal interceptor disarmed for an ignition); the consumers
    # are already gated on longitudinal being active, so nothing reads them while unavailable.
    cloudlog.warning("openpilot Longitudinal Control and ICBM not available, cleaning up params")
    for key in ("DynamicExperimentalControl", "CustomAccIncrementsEnabled",
                "SmartCruiseControlVision", "SmartCruiseControlMap"):
      remove_unless_preserved(params, key)

  set_speed_limit_assist_availability(CP, CP_SP, params)


def setup_interfaces(CI: CarInterfaceBase, params: Params | None = None) -> None:
  enforce_torque = _enforce_torque_lateral_control(CI.CP, params)
  nnlc_enabled = _initialize_neural_network_lateral_control(CI.CP, CI.CP_SP, params)
  _initialize_intelligent_cruise_button_management(CI.CP, CI.CP_SP, params)
  _initialize_torque_lateral_control(CI, CI.CP, enforce_torque, nnlc_enabled)
  _cleanup_unsupported_params(CI.CP, CI.CP_SP)

  try:
    STATSLOGSP.raw('sunnypilot.car_params', CI.CP.to_dict())
  except RuntimeError:
    pass  # to_dict fails on macOS due to library issues.
  # STATSLOGSP.raw('sunnypilot_params.car_params_sp', CP_SP.to_dict()) # https://github.com/sunnypilot/opendbc/pull/361


def initialize_params(params) -> list[dict[str, Any]]:
  keys: list = []

  # hyundai
  keys.extend([
    "HyundaiLongitudinalTuning",
  ])

  # subaru
  keys.extend([
    "SubaruStopAndGo",
    "SubaruStopAndGoManualParkingBrake",
  ])

  # tesla
  keys.extend([
    "TeslaCoopSteering",
    "TeslaMadsScreenButton",
  ])

  # toyota
  keys.extend([
    "ToyotaEnforceStockLongitudinal",
    "ToyotaStopAndGoHack",
  ])

  params_list = [{k: params.get(k, return_default=True)} for k in keys]

  # fork: comma pedal opt-in. The key lives in params_keys.h, i.e. inside the compiled libparams; an overlay deploy that
  # ships this Python without rebuilding libparams would raise UnknownKeyName here and take down card (and lateral) with
  # it. Read defensively so a stale libparams just leaves the feature off.
  try:
    params_list.append({"HyundaiGasInterceptor": params.get("HyundaiGasInterceptor", return_default=True)})
  except UnknownKeyName:
    cloudlog.warning("HyundaiGasInterceptor param unknown to libparams (not rebuilt?); gas interceptor stays disabled")

  # fork: optional pedal CAN ID dialect override (auto/standard/remapped). Same stale-libparams hazard; missing -> "auto"
  # (opendbc treats an absent key as auto, i.e. whichever pedal dialect the fingerprint saw, standard preferred).
  try:
    params_list.append({"HyundaiGasInterceptorIDSet": params.get("HyundaiGasInterceptorIDSet", return_default=True)})
  except UnknownKeyName:
    cloudlog.warning("HyundaiGasInterceptorIDSet param unknown to libparams (not rebuilt?); using auto pedal ID detection")

  return params_list
