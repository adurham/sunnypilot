"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, TextAlignment
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp, toggle_item_sp
from openpilot.system.ui.widgets.label import Label
from openpilot.sunnypilot.fork.fca11_settings import (experimental_header_state, experimental_row_state,
                                                     mode_toggle_state, path_toggle_state,
                                                     show_experimental_toggle_state, tuning_toggle_state)
from openpilot.sunnypilot.fork.hyundai_settings import (MODE_KEY, MODE_LABELS, SHAPING_KEY,
                                                       migrate_hyundai_settings, resolve_state)
from opendbc.car.hyundai.values import CAR, UNSUPPORTED_LONGITUDINAL_CAR


def _section(title: str) -> Label:
  """A non-interactive group heading.  The old page was one flat list; the headings are the structure
  the owner asked for."""
  label = Label(title, font_size=40, font_weight=FontWeight.MEDIUM, text_alignment=TextAlignment.LEFT, text_padding=20)
  label.set_rect(rl.Rectangle(0, 0, 600, 64))
  return label


def _on(raw) -> bool:
  if isinstance(raw, bytes):
    raw = raw.decode(errors="ignore")
  return bool(raw is True or str(raw if raw is not None else "0").strip() == "1")


class HyundaiSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.alpha_long_available = False
    self.show_experimental = False

    # ---- Longitudinal -------------------------------------------------------------------------
    self.longitudinal_header = _section(tr("Longitudinal"))
    self.mode_item = multiple_button_item_sp(
      tr("Longitudinal Mode"),
      tr("Choose what controls speed: the car's own cruise, the comma pedal, or the comma pedal plus FCA11 braking."),
      [tr(label) for label in MODE_LABELS], button_width=200, callback=self._on_mode_selected, inline=False)
    self.drive_mode_personality_toggle = toggle_item_sp(
      tr("Drive Mode Personality"),
      tr("ECO relaxes acceleration, NORMAL keeps it standard, SPORT makes it sharper, and N turns openpilot acceleration off."),
      param="DriveModePersonality", callback=self._on_toggle_changed)
    self.longitudinal_tuning_item = multiple_button_item_sp(
      tr("Stock Acceleration Tuning"),
      tr("How the car's own cruise shapes acceleration; only used in Stock mode."),
      [tr("Default"), tr("Dynamic"), tr("Predictive")], button_width=200,
      callback=self._on_tuning_selected, param="HyundaiLongitudinalTuning", inline=False)

    # Following and stopping are longitudinal behaviours, not Hyundai-specific switches.
    self.follow_header = _section(tr("Following and stopping"))
    self._elastic_desc = tr("Keeps one following gap at every personality, shrinking a little at highway speed.")
    self._no_close_desc = tr("Never speeds up to close on a car it can already see; it holds or coasts instead.")
    self._stop_desc = tr("On an empty-road stop, commits to the stop point the car's own plan already chose instead of creeping past it.")
    self.elastic_follow_toggle = toggle_item_sp(tr("Single Follow Gap"), self._elastic_desc,
                                                 param="AdaptiveFollowDistance", callback=self._on_toggle_changed)
    self.no_close_toggle = toggle_item_sp(tr("Don't Speed Up To A Lead"), self._no_close_desc,
                                          param="NoCloseOnLead", callback=self._on_toggle_changed)
    self.stop_approach_toggle = toggle_item_sp(tr("Commit To The Stop"), self._stop_desc,
                                               param="StopApproachCommit", callback=self._on_toggle_changed)

    # ---- FCA11 braking ------------------------------------------------------------------------
    self.fca11_header = _section(tr("FCA11 braking"))
    self._shaping_desc = tr("Builds and releases the FCA11 brake request smoothly instead of stepping it.")
    self._dynamic_desc = tr("Adds a gentle start, a rate limit and a soft release to FCA11 braking.")
    self.fca11_shaping_toggle = toggle_item_sp(tr("Smooth FCA11 Braking"), self._shaping_desc,
                                               callback=self._on_shaping_changed)
    self.fca11_dynamic_release_toggle = toggle_item_sp(tr("Smooth Brake Release"), self._dynamic_desc,
                                                       param="HyundaiFca11DynamicBrakes", callback=self._on_toggle_changed)

    # ---- Pedal --------------------------------------------------------------------------------
    self.pedal_header = _section(tr("Pedal"))
    self._delivery_desc = tr("Uses the pedal's measured response for light braking, so small slowdowns actually happen.")
    self._rise_desc = tr("Smooths how quickly a gas request is applied; lifting off is never delayed.")
    self.pedal_delivery_toggle = toggle_item_sp(tr("Light Braking Response"), self._delivery_desc,
                                                param="HyundaiPedalDeliveryLaw", callback=self._on_toggle_changed)
    self.gas_rise_lag_toggle = toggle_item_sp(tr("Smooth Gas Application"), self._rise_desc,
                                              param="HyundaiGasRiseLag", callback=self._on_toggle_changed)

    # ---- Experimental -------------------------------------------------------------------------
    self.experimental_header = _section(tr("Experimental"))
    self._hold_desc = tr("Experimental: asks the car's ESC to hold the brakes at a stop.")
    self._shadow_desc = tr("Records what the newer brake shaping would have done, without sending it to the car.")
    self.show_experimental_toggle = toggle_item_sp(
      tr("Show experimental"), tr("Show the experimental and diagnostic rows below."),
      initial_state=False, callback=self._on_show_experimental_changed)
    self.fca11_stop_hold_toggle = toggle_item_sp(tr("ESC Hold At A Stop"), self._hold_desc,
                                                 param="HyundaiFca11StopHoldClutch", callback=self._on_toggle_changed)
    self.fca11_shadow_toggle = toggle_item_sp(tr("Log What Shaping Would Do"), self._shadow_desc,
                                              param="HyundaiFca11Shadow", callback=self._on_toggle_changed)

    self.items = [
      self.longitudinal_header, self.mode_item, self.drive_mode_personality_toggle, self.longitudinal_tuning_item,
      self.follow_header, self.elastic_follow_toggle, self.no_close_toggle, self.stop_approach_toggle,
      self.fca11_header, self.fca11_shaping_toggle, self.fca11_dynamic_release_toggle,
      self.pedal_header, self.pedal_delivery_toggle, self.gas_rise_lag_toggle,
      self.experimental_header, self.show_experimental_toggle, self.fca11_stop_hold_toggle, self.fca11_shadow_toggle,
    ]

  @staticmethod
  def _on_tuning_selected(index):
    ui_state.params.put("HyundaiLongitudinalTuning", index)

  @staticmethod
  def _on_mode_selected(index):
    # No param= on the row: the write is guarded so a stale libparams (new key not registered in
    # the compiled registry) cannot take the settings page down.  The row re-reads the resolved mode
    # every frame, so a failed write simply shows the old value.
    try:
      ui_state.params.put(MODE_KEY, int(index))
    except Exception:
      pass

  def _on_shaping_changed(self, state):
    try:
      ui_state.params.put(SHAPING_KEY, bool(state))
    except Exception:
      pass
    self.update_settings()

  def _on_toggle_changed(self, _state):
    self.update_settings()

  def _on_show_experimental_changed(self, state):
    self.show_experimental = bool(state)
    self.update_settings()

  @staticmethod
  def _offroad_hint(text: str, offroad: bool) -> str:
    if offroad:
      return text
    return tr("Turn the car off (or enable \"Always Offroad\") to change this.") + "<br><br>" + text

  def update_settings(self):
    self.alpha_long_available = False
    bundle = ui_state.params.get("CarPlatformBundle")
    if bundle:
      platform = bundle.get("platform")
      self.alpha_long_available = CAR[platform] not in set().union(*UNSUPPORTED_LONGITUDINAL_CAR.values())
    elif ui_state.CP is not None:
      self.alpha_long_available = ui_state.CP.alphaLongitudinalAvailable

    offroad = ui_state.is_offroad()
    # One-time migration: writes only when the new keys are absent, only offroad.  initialize_params()
    # has already done the same at ignition, so the page never depends on this write for behaviour.
    if offroad:
      migrate_hyundai_settings(ui_state.params)

    state = resolve_state(ui_state.params)
    pedal = state.pedal
    brake = state.brake

    # ---- Longitudinal -----------------------------------------------------------------------
    self.mode_item.action_item.set_enabled(mode_toggle_state(offroad).enabled)
    self.mode_item.action_item.set_selected_button(state.mode)
    self.mode_item.set_description(self._offroad_hint(
      tr("Choose what controls speed: the car's own cruise, the comma pedal, or the comma pedal plus FCA11 "
         "braking; the change takes effect at the next ignition."), offroad))
    self.mode_item.show_description(True)
    self.mode_item.set_visible(True)

    self.drive_mode_personality_toggle.action_item.set_enabled(True)
    self.drive_mode_personality_toggle.show_description(True)
    self.drive_mode_personality_toggle.set_visible(True)

    tuning_param = int(ui_state.params.get("HyundaiLongitudinalTuning") or "0")
    tuning_state = tuning_toggle_state(state.mode, self.alpha_long_available, ui_state.has_longitudinal_control, offroad)
    tuning_desc = tr("How the car's own cruise shapes acceleration; only used in Stock mode.")
    if not ui_state.has_longitudinal_control:
      tuning_desc = tr("This needs sunnypilot Longitudinal Control (Alpha) turned on.")
    self.longitudinal_tuning_item.action_item.set_enabled(tuning_state.enabled)
    self.longitudinal_tuning_item.set_description(self._offroad_hint(tuning_desc, offroad))
    self.longitudinal_tuning_item.show_description(True)
    self.longitudinal_tuning_item.action_item.set_selected_button(tuning_param)
    self.longitudinal_tuning_item.set_visible(tuning_state.visible)

    for item, desc in ((self.elastic_follow_toggle, self._elastic_desc),
                       (self.no_close_toggle, self._no_close_desc),
                       (self.stop_approach_toggle, self._stop_desc)):
      item.action_item.set_enabled(path_toggle_state(True, offroad).enabled)
      item.set_description(self._offroad_hint(desc, offroad))
      item.show_description(True)
      item.set_visible(True)

    # ---- FCA11 braking ----------------------------------------------------------------------
    self.fca11_header.set_visible(True)
    fca11_hint = "" if brake else tr("Needs Pedal + FCA11. ") + "<br><br>"
    for item, desc in ((self.fca11_shaping_toggle, self._shaping_desc),
                       (self.fca11_dynamic_release_toggle, self._dynamic_desc)):
      item.action_item.set_enabled(path_toggle_state(brake, offroad).enabled)
      item.set_description(fca11_hint + self._offroad_hint(desc, offroad))
      item.show_description(True)
      item.set_visible(True)
    self.fca11_shaping_toggle.action_item.set_state(state.shaping)

    # ---- Pedal ------------------------------------------------------------------------------
    self.pedal_header.set_visible(True)
    pedal_hint = "" if pedal else tr("Needs Comma pedal or Pedal + FCA11. ") + "<br><br>"
    for item, desc in ((self.pedal_delivery_toggle, self._delivery_desc),
                       (self.gas_rise_lag_toggle, self._rise_desc)):
      item.action_item.set_enabled(path_toggle_state(pedal, offroad).enabled)
      item.set_description(pedal_hint + self._offroad_hint(desc, offroad))
      item.show_description(True)
      item.set_visible(True)

    # ---- Experimental -----------------------------------------------------------------------
    hold_on = _on(ui_state.params.get("HyundaiFca11StopHoldClutch"))
    shadow_on = _on(ui_state.params.get("HyundaiFca11Shadow"))
    header_state = experimental_header_state(pedal)
    reveal_state = show_experimental_toggle_state(pedal)
    hold_state = experimental_row_state(pedal, brake, self.show_experimental, hold_on, offroad)
    shadow_state = experimental_row_state(pedal, True, self.show_experimental, shadow_on, offroad)
    self.experimental_header.set_visible(header_state.visible)
    self.show_experimental_toggle.set_visible(reveal_state.visible)
    self.show_experimental_toggle.action_item.set_enabled(reveal_state.enabled)
    self.show_experimental_toggle.show_description(True)

    self.fca11_stop_hold_toggle.action_item.set_enabled(hold_state.enabled)
    self.fca11_stop_hold_toggle.set_description(self._offroad_hint(self._hold_desc, offroad))
    self.fca11_stop_hold_toggle.show_description(True)
    self.fca11_stop_hold_toggle.set_visible(hold_state.visible)

    self.fca11_shadow_toggle.action_item.set_enabled(shadow_state.enabled)
    self.fca11_shadow_toggle.set_description(self._offroad_hint(self._shadow_desc, offroad))
    self.fca11_shadow_toggle.show_description(True)
    self.fca11_shadow_toggle.set_visible(shadow_state.visible)
