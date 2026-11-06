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
  fca11_shadow_toggle_state, fca11_dynamic_release_toggle_state, pedal_delivery_toggle_state, stop_approach_toggle_state, \
  fca11_stop_hold_toggle_state, gas_rise_lag_toggle_state, longitudinal_tuning_toggle_state, fca11_brake_state, \
  fca11_brake_state_sentence
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
    # fork (0045): FCA11 SHADOW - wire = OLD-LAW (today's law), V3 logged. Visible with the pedal on, settable offroad only.
    self.fca11_shadow_toggle = toggle_item_sp(tr("FCA11 Shadow (Log-Only)"), "", param="HyundaiFca11Shadow",
                                              callback=self._on_fca11_toggle_changed)
    # fork (17c/18x): the three lane switches that used to be param/SSH only. Same rules as the toggles above:
    # visible only where they can do anything, settable offroad only (CarParams/CarParamsSP are built at ignition).
    self.pedal_delivery_toggle = toggle_item_sp(tr("Pedal Delivery Law"), "", param="HyundaiPedalDeliveryLaw",
                                                callback=self._on_fca11_toggle_changed)
    # fork (0042-Z): the rise-only gas lag (HyundaiGasRiseLag). Pedal on, offroad only, same rules as the toggle above.
    self.gas_rise_lag_toggle = toggle_item_sp(tr("Gas Rise Lag"), "", param="HyundaiGasRiseLag",
                                              callback=self._on_fca11_toggle_changed)
    self.fca11_dynamic_release_toggle = toggle_item_sp(tr("Dynamic Brakes"), "", param="HyundaiFca11DynamicBrakes",
                                                       callback=self._on_fca11_toggle_changed)
    self.stop_approach_toggle = toggle_item_sp(tr("Stop-Approach Commitment"), "", param="StopApproachCommit",
                                               callback=self._on_fca11_toggle_changed)
    # 0034: EXPERIMENTAL — asks the ESC for its own stop hold at a standstill (the request does NOT open the clutch:
    # the TCU reads the brake pedal switch, not 0x38D — row 137 and this row's TCU evidence, row 141).
    # LISTED ONLY WHILE ITS OWN VALUE IS ON (fca11_stop_hold_toggle_state): per FORK.md row 137 the hold has never been
    # validated, and the TCU reads the brake pedal switch, not 0x38D (row 141), so the row is out of the way at its
    # default 0 — but the key is BACKUP-flagged and was 1 on route 195, and a restore writes it back with no switch in
    # sight. Showing the row while the value is ON gives the owner that switch. Key, reader, capnp @11 and value stay;
    # the panel never writes the param.
    self.fca11_stop_hold_toggle = toggle_item_sp(tr("Hold Clutch Release (experimental)"), "", param="HyundaiFca11StopHoldClutch",
                                                 callback=self._on_fca11_toggle_changed)
    # fork: ELASTIC FOLLOW DISTANCE (AdaptiveFollowDistance) - the owner's follow law: one gap for every personality
    # (aggressive included), shrinking with speed on the open road instead of the stock 1.25/1.45/1.75 s ladder.
    self.elastic_follow_toggle = toggle_item_sp(tr("Elastic Follow Distance"), "", param="AdaptiveFollowDistance",
                                                 callback=self._on_fca11_toggle_changed)
    # fork: NEVER ACCELERATE TOWARD A LEAD (NoCloseOnLead) - approach a car you can already see by holding/coasting.
    self.no_close_toggle = toggle_item_sp(tr("No Accelerate-To-Lead"), "", param="NoCloseOnLead",
                                          callback=self._on_fca11_toggle_changed)
    self.drive_mode_personality_toggle = toggle_item_sp(
      tr("Drive Mode Personality"), tr("ECO relaxed / NORMAL standard / SPORT aggressive; N blocks openpilot longitudinal"),
      param="DriveModePersonality", callback=self._on_drive_mode_toggle_changed)
    self.items = [self.longitudinal_tuning_item, self.fca11_brake_toggle, self.fca11_shaping_toggle,
                  self.fca11_shadow_toggle, self.pedal_delivery_toggle, self.gas_rise_lag_toggle,
                  self.fca11_dynamic_release_toggle,
                  self.fca11_stop_hold_toggle,
                  self.stop_approach_toggle, self.elastic_follow_toggle, self.no_close_toggle, self.drive_mode_personality_toggle]

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

    # Read once: the pedal switch decides which FCA11 rows can do anything on this car, and it also hides the upstream
    # Custom Longitudinal Tuning row (inert while the pedal is on).
    pedal_raw = ui_state.params.get("HyundaiGasInterceptor")
    brake_raw = ui_state.params.get("HyundaiFca11Brake")
    # R2/N1: the three brake rows open with a sentence built from the ACTUAL values of all four switches. A fixed
    # "This car runs the continuous linear law" was false in 7 of the 8 Shaping/Dynamic/Shadow states, and false again
    # with FCA11 Braking off (where the controller is enabled=False and sends nothing at all).
    shaping_raw = ui_state.params.get("HyundaiFca11AffineGain")
    dynamic_raw = ui_state.params.get("HyundaiFca11DynamicBrakes")
    shadow_raw = ui_state.params.get("HyundaiFca11Shadow")
    brake_values = fca11_brake_state(brake_raw, shaping_raw, dynamic_raw, shadow_raw)
    state_sent = fca11_brake_state_sentence(brake_raw, shaping_raw, dynamic_raw, shadow_raw)
    offroad = ui_state.is_offroad()
    offroad_hint = tr("Enable \"Always Offroad\" in the Device panel, or turn the vehicle off to toggle.") + "<br><br>"

    tuning_param = int(ui_state.params.get("HyundaiLongitudinalTuning") or "0")
    long_enabled = ui_state.has_longitudinal_control

    long_tuning_descs = [
      tr("Your vehicle will use the Default longitudinal tuning."),
      tr("Your vehicle will use the Dynamic longitudinal tuning."),
      tr("Your vehicle will use the Predictive longitudinal tuning."),
    ]
    long_tuning_desc = long_tuning_descs[tuning_param] if tuning_param < len(long_tuning_descs) else long_tuning_descs[0]

    # HIDE the upstream Custom Longitudinal Tuning row while the pedal is on: it is inert there (phase-1 inventory:
    # the CAN stream is byte-identical for 0/1/2 and scc_long is False). Pure visibility rule; key + value untouched.
    tuning_state = longitudinal_tuning_toggle_state(pedal_raw, offroad)
    longitudinal_tuning_disabled = not ui_state.is_offroad() or not long_enabled or not tuning_state.visible
    if longitudinal_tuning_disabled:
      if not ui_state.is_offroad():
        long_tuning_desc = tr("This feature is unavailable while the car is onroad.")
      elif not long_enabled:
        long_tuning_desc = tr("This feature is unavailable because sunnypilot Longitudinal Control (Alpha) is not enabled.")

    self.longitudinal_tuning_item.action_item.set_enabled(not longitudinal_tuning_disabled)
    self.longitudinal_tuning_item.set_description(long_tuning_desc)
    self.longitudinal_tuning_item.show_description(True)
    self.longitudinal_tuning_item.action_item.set_selected_button(tuning_param)
    self.longitudinal_tuning_item.set_visible(self.alpha_long_available and tuning_state.visible)

    # fork: FCA11 Braking (Beta) — production longitudinal braking on top of the comma pedal. Only meaningful when the
    # gas interceptor is on (it is the same car layer), and only settable offroad. Default OFF; the caveat is explicit.
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
    # SETTINGS TIDY (D1-D3): FOUR real laws, measured with the real setup_interfaces + Fca11LongBrake (S1D1H0 the
    # continuous law `4a1fe42f`; S1D0H0 the 0043 shaped law `c6be8954`; S0D1H0 the OLD law + 18x ramp `ac1e6675`;
    # everything else the plain OLD law `8779459c`) - not the two the first brief assumed. The continuous law has NO
    # fast-step bypass (0037 deleted it, FORK row 135): a -3.5 m/s^2 step reaches 20 LSB at 640 ms there, 80 ms on
    # the OLD law.
    shaping_desc = tr(state_sent) + " " + tr(
      "Shaping ON with Dynamic Brakes ON and Shadow OFF gives the continuous linear law: 1 LSB onset, smooth builds "
      "both ways, rate-limited falls, a soft standstill release, and no fast-step bypass - a hard stop ramps in over "
      "about a second (20 LSB at 640 ms), where the plain OLD law opens 4 LSB per frame and is at full strength in "
      "140 ms. Shaping ON with Dynamic Brakes OFF and Shadow OFF gives the 0043 shaped law: open loop, it maps the "
      "plan's ask and never corrects it. Shaping OFF with Dynamic Brakes ON and Shadow OFF gives the OLD law plus the "
      "18x release ramp: a gentle ask starts at 1 LSB and a hard ask is ramped in (-2.8 m/s^2: 20 LSB at 780 ms), but "
      "an emergency ask (-3.0 m/s^2 or harder) still steps like the OLD law - 4 LSB per sent frame, full strength "
      "within 140 ms. Shadow "
      "ON, or Shaping and Dynamic Brakes both OFF, gives the plain OLD law. Requires FCA11 Braking. Takes effect at "
      "the next ignition.")
    if shaping_state.visible and not offroad:
      shaping_desc = offroad_hint + shaping_desc
    self.fca11_shaping_toggle.set_description(shaping_desc)
    self.fca11_shaping_toggle.show_description(True)
    self.fca11_shaping_toggle.set_visible(shaping_state.visible)

    # fork (0045): FCA11 Shadow (Log-Only) - visible with the pedal on, settable offroad only; next ignition.
    shadow_state = fca11_shadow_toggle_state(pedal_raw, offroad)
    self.fca11_shadow_toggle.action_item.set_enabled(shadow_state.enabled)
    # D3: the logged core is the 0043 SHAPED law (fca11_long.py l.1483 `_shadow_core_params` -> l.1881: core =>
    # shape_track False), NOT "the new law": say exactly that.
    # N3: with FCA11 Braking OFF the controller is enabled=False and sends NOTHING, so "ON switches braking back to
    # the plain OLD law" would be false there; Shadow only logs. The state sentence above says that as well.
    shadow_law = tr("ON switches braking back to the plain OLD law - it forces Shaping and Dynamic Brakes off - and "
                    "logs the 0043 shaped law in the background: the open-loop law that maps the plan's ask and never "
                    "corrects it, computed on every frame and never sent to the car.")
    if not brake_values.brake:
      shadow_law = tr("With FCA11 Braking OFF nothing is sent, so this switch only logs: the 0043 shaped law is "
                      "computed on every frame and never sent to the car.")
    shadow_table = tr(
      "Shaping ON with Dynamic Brakes ON and Shadow OFF gives the continuous linear law: 1 LSB onset, smooth builds "
      "both ways, rate-limited falls, a soft standstill release, and no fast-step bypass - a hard stop ramps in over "
      "about a second (20 LSB at 640 ms), where the plain OLD law opens 4 LSB per frame and is at full strength in "
      "140 ms. Shaping ON with Dynamic Brakes OFF and Shadow OFF gives the 0043 shaped law: open loop, it maps the "
      "plan's ask and never corrects it. Shaping OFF with Dynamic Brakes ON and Shadow OFF gives the OLD law plus the "
      "18x release ramp: a gentle ask starts at 1 LSB and a hard ask is ramped in (-2.8 m/s^2: 20 LSB at 780 ms), but "
      "an emergency ask (-3.0 m/s^2 or harder) still steps like the OLD law - 4 LSB per sent frame, full strength "
      "within 140 ms. Shadow "
      "ON, or Shaping and Dynamic Brakes both OFF, gives the plain OLD law. Requires the comma pedal. Takes effect at "
      "the next ignition.")
    # N1: with FCA11 Braking OFF this row is still listed (only Shaping/Dynamic depend on Braking), so the four-state
    # table must say WHICH car it describes - otherwise the row states a law for a car that runs none.
    assume_braking = (tr("The four combinations below assume FCA11 Braking ON. ") if not brake_values.brake else "")
    shadow_desc = tr(state_sent) + " " + shadow_law + " " + assume_braking + shadow_table
    if shadow_state.visible and not offroad:
      shadow_desc = offroad_hint + shadow_desc
    self.fca11_shadow_toggle.set_description(shadow_desc)
    self.fca11_shadow_toggle.show_description(True)
    self.fca11_shadow_toggle.set_visible(shadow_state.visible)

    # fork (17c): Pedal Delivery Law - measured inverse for the light braking asks; pedal on, offroad only.
    delivery_state = pedal_delivery_toggle_state(pedal_raw, offroad)
    self.pedal_delivery_toggle.action_item.set_enabled(delivery_state.enabled)
    delivery_desc = tr("Uses the pedal's MEASURED response instead of the linear law for light braking asks " +
                       "(below about 0.5 m/s^2), so more of what the planner asks actually reaches the road, and the " +
                       "pedal stops dithering at the set speed. Positive asks and every hard stop are unchanged. " +
                       "Measured in NORMAL drive mode: ECO and SPORT keep the stock pedal law (their accelerator " +
                       "map is different). Requires the comma pedal. Takes effect at the next ignition.")
    if delivery_state.visible and not offroad:
      delivery_desc = offroad_hint + delivery_desc
    self.pedal_delivery_toggle.set_description(delivery_desc)
    self.pedal_delivery_toggle.show_description(True)
    self.pedal_delivery_toggle.set_visible(delivery_state.visible)

    # fork (0042-Z): Gas Rise Lag - a rise-only first-order lag on the accel ask; pedal on, offroad only.
    rise_state = gas_rise_lag_toggle_state(pedal_raw, offroad)
    self.gas_rise_lag_toggle.action_item.set_enabled(rise_state.enabled)
    rise_desc = tr("Slows how fast a GAS request is applied (about 0.45 s), so small planner corrections arrive as a " +
                   "smooth build instead of a blip. Lifting off is NEVER delayed: a falling request reaches the " +
                   "pedal on the same frame, and the pedal never sits above where the same request would put it " +
                   "without this. A launch on a ramped request gives up about 0.1-0.15 s to 1 m/s. Requires the comma " +
                   "pedal. Takes effect at the next ignition.")
    if rise_state.visible and not offroad:
      rise_desc = offroad_hint + rise_desc
    self.gas_rise_lag_toggle.set_description(rise_desc)
    self.gas_rise_lag_toggle.show_description(True)
    self.gas_rise_lag_toggle.set_visible(rise_state.visible)

    # fork (18x): Dynamic Brake Release - the FCA11 brake becomes a ramp; needs FCA11 Braking, offroad only.
    dynamic_state = fca11_dynamic_release_toggle_state(pedal_raw, brake_raw, offroad)
    self.fca11_dynamic_release_toggle.action_item.set_enabled(dynamic_state.enabled)
    # D1/D2: same four states as the shaping row. The "emergency ask and a step demand still go at full rate" claim is
    # DELETED - it was pinned on the continuous law, where 0037 removed the bypass. With Shaping OFF the 18x ramp is
    # what runs: a sub-emergency ask (even a -2.8 m/s^2 step) ramps in from 1 LSB, but an EMERGENCY ask
    # (accel <= FCA11_DYN_EMERGENCY_ACCEL, -3.0 m/s^2) keeps the panda's RATE_STEP and is byte-identical to the OLD
    # law there (measured: -3.0 -> 4,8,12,16,20,24,26; -3.5 -> 4,8,12,16,20,24,28,31).
    dynamic_desc = tr(state_sent) + " " + tr(
      "Shaping ON with Dynamic Brakes ON and Shadow OFF gives the continuous linear law: 1 LSB onset, smooth builds "
      "both ways, rate-limited falls, a soft standstill release, and no fast-step bypass - a hard stop ramps in over "
      "about a second (20 LSB at 640 ms), where the plain OLD law opens 4 LSB per frame and is at full strength in "
      "140 ms. Shaping ON with Dynamic Brakes OFF and Shadow OFF gives the 0043 shaped law: open loop, it maps the "
      "plan's ask and never corrects it. Shaping OFF with Dynamic Brakes ON and Shadow OFF gives the OLD law plus "
      "this switch's 18x release ramp: a gentle ask starts at 1 LSB and a hard ask is ramped in (-2.8 m/s^2: 20 LSB "
      "at 780 ms), but an emergency ask (-3.0 m/s^2 or harder) still steps like the OLD law - 4 LSB per sent frame, "
      "full strength within 140 ms. Shadow ON, or Shaping and Dynamic Brakes both OFF, gives the plain OLD law. "
      "Requires FCA11 Braking. Takes effect at the next ignition.")
    if dynamic_state.visible and not offroad:
      dynamic_desc = offroad_hint + dynamic_desc
    self.fca11_dynamic_release_toggle.set_description(dynamic_desc)
    self.fca11_dynamic_release_toggle.show_description(True)
    self.fca11_dynamic_release_toggle.set_visible(dynamic_state.visible)

    # 0034: Hold Clutch Release (experimental) - the FCA11 frame also asks for the ESC's own stop hold at a standstill.
    hold_raw = ui_state.params.get("HyundaiFca11StopHoldClutch")
    hold_state = fca11_stop_hold_toggle_state(pedal_raw, brake_raw, hold_raw, offroad)
    self.fca11_stop_hold_toggle.action_item.set_enabled(hold_state.enabled)
    hold_desc = tr("EXPERIMENTAL. While holding the car stopped, the FCA11 frame also asks the ESC for its own stop " +
                   "hold (the car sits at about 26 Nm of creep torque otherwise). Only sent at walking pace, at a real " +
                   "hold level, with FCA11 Braking on. Unproven: whether the car holds on an incline this way, and how " +
                   "it lets go. This row is only listed while the switch is ON (the stored value is ON), so turning it " +
                   "OFF here hides the row again. Requires FCA11 Braking. Takes effect at the next ignition.")
    if hold_state.visible and not offroad:
      hold_desc = offroad_hint + hold_desc
    self.fca11_stop_hold_toggle.set_description(hold_desc)
    self.fca11_stop_hold_toggle.show_description(True)
    self.fca11_stop_hold_toggle.set_visible(hold_state.visible)

    # fork: Elastic Follow Distance - planner-side, always listed, offroad only (it is a runtime MPC parameter).
    self.elastic_follow_toggle.action_item.set_enabled(offroad)
    elastic_desc = tr("One follow gap for every personality, so Aggressive follows at the same distance as Normal. " +
                      "The gap is continuous, not a 1/2/3 s preset: 1.45 s in town, shrinking to 1.19 s by 72 mph. " +
                      "Aggressive still changes launch and pull behaviour, not the steady-state gap. Takes effect " +
                      "at the next ignition.")
    if not offroad:
      elastic_desc = offroad_hint + elastic_desc
    self.elastic_follow_toggle.set_description(elastic_desc)
    self.elastic_follow_toggle.show_description(True)
    self.elastic_follow_toggle.set_visible(True)

    # fork: No Accelerate-To-Lead - planner-side, always listed, offroad only.
    self.no_close_toggle.action_item.set_enabled(offroad)
    no_close_desc = tr("Approaches a car already in view by holding speed or coasting, never by speeding up to " +
                       "reach the follow gap. Only removes acceleration - it never brakes harder - and it releases " +
                       "once the gap is matched. Takes effect at the next ignition.")
    if not offroad:
      no_close_desc = offroad_hint + no_close_desc
    self.no_close_toggle.set_description(no_close_desc)
    self.no_close_toggle.show_description(True)
    self.no_close_toggle.set_visible(True)

    # fork (17c lane C): Stop-Approach Commitment - planner-side, always listed, offroad only.
    stop_state = stop_approach_toggle_state(offroad)
    self.stop_approach_toggle.action_item.set_enabled(stop_state.enabled)
    stop_desc = tr("On an empty-lane stop, commits early to the stop point the model's own plan already chose, " +
                   "instead of creeping past it. It only ever deepens the ask - never raises it - and does nothing " +
                   "while a lead is present. Takes effect at the next ignition.")
    if not offroad:
      stop_desc = offroad_hint + stop_desc
    self.stop_approach_toggle.set_description(stop_desc)
    self.stop_approach_toggle.show_description(True)
    self.stop_approach_toggle.set_visible(stop_state.visible)
