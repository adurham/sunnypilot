"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp, toggle_item_sp
from opendbc.car.hyundai.values import CAR, UNSUPPORTED_LONGITUDINAL_CAR


class HyundaiSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.alpha_long_available = False

    tuning_texts = [tr("Off"), tr("Dynamic"), tr("Predictive")]
    self.longitudinal_tuning_item = multiple_button_item_sp(tr("Custom Longitudinal Tuning"), "", tuning_texts,
                                                            button_width=300, callback=self._on_tuning_selected,
                                                            param="HyundaiLongitudinalTuning", inline=False)
    self.fca11_brake_toggle = toggle_item_sp(tr("FCA11 Braking (Beta)"), "", param="HyundaiFca11Brake",
                                             callback=self._on_fca11_toggle_changed)
    self.items = [self.longitudinal_tuning_item, self.fca11_brake_toggle]

  @staticmethod
  def _on_tuning_selected(index):
    ui_state.params.put("HyundaiLongitudinalTuning", index)

  def _on_fca11_toggle_changed(self, _):
    self.update_settings()

  def update_settings(self):
    self.alpha_long_available = False
    bundle = ui_state.params.get("CarPlatformBundle")
    if bundle:
      platform = bundle.get("platform")
      self.alpha_long_available = CAR[platform] not in set().union(*UNSUPPORTED_LONGITUDINAL_CAR.values())
    elif ui_state.CP is not None:
      self.alpha_long_available = ui_state.CP.alphaLongitudinalAvailable

    tuning_param = int(ui_state.params.get("HyundaiLongitudinalTuning") or "0")
    long_enabled = ui_state.has_longitudinal_control

    long_tuning_descs = [
      tr("Your vehicle will use the Default longitudinal tuning."),
      tr("Your vehicle will use the Dynamic longitudinal tuning."),
      tr("Your vehicle will use the Predictive longitudinal tuning."),
    ]
    long_tuning_desc = long_tuning_descs[tuning_param] if tuning_param < len(long_tuning_descs) else long_tuning_descs[0]

    longitudinal_tuning_disabled = not ui_state.is_offroad() or not long_enabled
    if longitudinal_tuning_disabled:
      if not ui_state.is_offroad():
        long_tuning_desc = tr("This feature is unavailable while the car is onroad.")
      elif not long_enabled:
        long_tuning_desc = tr("This feature is unavailable because sunnypilot Longitudinal Control (Alpha) is not enabled.")

    self.longitudinal_tuning_item.action_item.set_enabled(not longitudinal_tuning_disabled)
    self.longitudinal_tuning_item.set_description(long_tuning_desc)
    self.longitudinal_tuning_item.show_description(True)
    self.longitudinal_tuning_item.action_item.set_selected_button(tuning_param)
    self.longitudinal_tuning_item.set_visible(self.alpha_long_available)

    # fork: FCA11 Braking (Beta) — production longitudinal braking on top of the comma pedal. Only meaningful when the
    # gas interceptor is on (it is the same car layer), and only settable offroad. Default OFF; the caveat is explicit.
    pedal_on = int(ui_state.params.get("HyundaiGasInterceptor") or "0") == 1
    brake_offroad = ui_state.is_offroad()
    self.fca11_brake_toggle.action_item.set_enabled(pedal_on and brake_offroad)
    fca11_desc = tr("Experimental: uses the car's own FCA11 (forward collision avoidance) message to add light " +
                    "braking through the ESC on top of the comma pedal, so sunnypilot can slow for a lead. " +
                    "The dash shows a collision-braking notification while it is active. Braking is limited to " +
                    "comfortable ACC strength and hands over to you below 12 km/h and at every stop; you remain the " +
                    "driver. Requires the comma pedal; keep OFF unless you have reviewed this feature.")
    if pedal_on and not brake_offroad:
      fca11_desc = tr("Enable \"Always Offroad\" in the Device panel, or turn the vehicle off to toggle.") + "<br><br>" + fca11_desc
    self.fca11_brake_toggle.set_description(fca11_desc)
    self.fca11_brake_toggle.show_description(True)
    self.fca11_brake_toggle.set_visible(pedal_on)
