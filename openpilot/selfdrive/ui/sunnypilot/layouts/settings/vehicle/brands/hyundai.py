"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp, toggle_item_sp
from openpilot.sunnypilot.fork.fca11_settings import fca11_brake_toggle_state, fca11_shaping_toggle_state, \
  fca11_shadow_toggle_state
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
    # fork (0044): the opt-in brake-SHAPING law (HyundaiFca11AffineGain - historical key name). Shown only under FCA11
    # Braking; visibility/enable come from the PURE fca11_settings functions (unit-tested headless).
    self.fca11_shaping_toggle = toggle_item_sp(tr("FCA11 Brake Shaping (Beta)"), "", param="HyundaiFca11AffineGain",
                                               callback=self._on_fca11_toggle_changed)
    # fork (0045): FCA11 SHADOW (log-only). Visible with the pedal on, settable offroad only (pure fca11_settings function).
    self.fca11_shadow_toggle = toggle_item_sp(tr("FCA11 Shadow (Log-Only)"), "", param="HyundaiFca11Shadow",
                                              callback=self._on_fca11_toggle_changed)
    self.drive_mode_personality_toggle = toggle_item_sp(
      tr("Drive Mode Personality"), tr("ECO relaxed / NORMAL standard / SPORT aggressive; N blocks openpilot longitudinal"),
      param="DriveModePersonality", callback=self._on_drive_mode_toggle_changed)
    self.items = [self.longitudinal_tuning_item, self.fca11_brake_toggle, self.fca11_shaping_toggle,
                  self.fca11_shadow_toggle, self.drive_mode_personality_toggle]

  @staticmethod
  def _on_tuning_selected(index):
    ui_state.params.put("HyundaiLongitudinalTuning", index)

  def _on_fca11_toggle_changed(self, _):
    self.update_settings()

  def _on_drive_mode_toggle_changed(self, _):
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
    pedal_raw = ui_state.params.get("HyundaiGasInterceptor")
    brake_raw = ui_state.params.get("HyundaiFca11Brake")
    offroad = ui_state.is_offroad()
    offroad_hint = tr("Enable \"Always Offroad\" in the Device panel, or turn the vehicle off to toggle.") + "<br><br>"
    brake_state = fca11_brake_toggle_state(pedal_raw, offroad)
    self.fca11_brake_toggle.action_item.set_enabled(brake_state.enabled)
    # 0044: the old text claimed braking hands over to the driver at a speed floor and at every stop - stale since patch 0040
    # deleted both speed floors (it brakes to a stop and holds it).
    fca11_desc = tr("Experimental: uses the car's own FCA11 (forward collision avoidance) message to add light " +
                    "braking through the ESC on top of the comma pedal, so sunnypilot can slow for a lead and bring " +
                    "the car to a stop. The dash shows a collision-braking notification while it is active. Braking " +
                    "is limited to comfortable ACC strength; you remain the driver and must brake whenever more is " +
                    "needed. Requires the comma pedal; keep OFF unless you have reviewed this feature.")
    if brake_state.visible and not offroad:
      fca11_desc = offroad_hint + fca11_desc
    self.fca11_brake_toggle.set_description(fca11_desc)
    self.fca11_brake_toggle.show_description(True)
    self.fca11_brake_toggle.set_visible(brake_state.visible)

    # fork (0044): FCA11 Brake Shaping (Beta) - visible only with the pedal AND FCA11 Braking on, settable offroad only.
    shaping_state = fca11_shaping_toggle_state(pedal_raw, brake_raw, offroad)
    self.fca11_shaping_toggle.action_item.set_enabled(shaping_state.enabled)
    shaping_desc = tr("Experimental: eases the FCA11 brake onset to match your own pedal feel (a gentle start, then " +
                      "a smooth build) and asks for the braking level the car actually delivers. Hard stops are NOT " +
                      "slowed: a firm brake request still ramps at full rate. Requires FCA11 Braking. Takes effect " +
                      "at the next ignition.")
    if shaping_state.visible and not offroad:
      shaping_desc = offroad_hint + shaping_desc
    self.fca11_shaping_toggle.set_description(shaping_desc)
    self.fca11_shaping_toggle.show_description(True)
    self.fca11_shaping_toggle.set_visible(shaping_state.visible)

    # fork (0045): FCA11 Shadow (Log-Only) - visible with the pedal on, settable offroad only; next ignition.
    shadow_state = fca11_shadow_toggle_state(pedal_raw, offroad)
    self.fca11_shadow_toggle.action_item.set_enabled(shadow_state.enabled)
    shadow_desc = tr("Data collection only: computes what FCA11 brake shaping WOULD command on every frame and writes " +
                     "it to a log on the device. It NEVER acts on the car and never sends a brake command. While ON, " +
                     "FCA11 Braking and Brake Shaping do NOT brake the car (it drives exactly as with FCA11 Braking " +
                     "off; you brake for every stop). Takes effect at the next ignition.")
    if shadow_state.visible and not offroad:
      shadow_desc = offroad_hint + shadow_desc
    self.fca11_shadow_toggle.set_description(shadow_desc)
    self.fca11_shadow_toggle.show_description(True)
    self.fca11_shadow_toggle.set_visible(shadow_state.visible)
