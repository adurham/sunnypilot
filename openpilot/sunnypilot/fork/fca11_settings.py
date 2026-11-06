"""
fork (0044): PURE visibility / enable decisions for the Hyundai FCA11 settings toggles.

The raylib settings panel (openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py) cannot run
headless, so the decision lives here, free of UI imports, and is unit-tested in fork/tests/test_fca11_settings.py.

  FCA11 Braking (Beta)          HyundaiFca11Brake       visible: pedal on;                 enabled: offroad
  FCA11 Brake Shaping (Beta)    HyundaiFca11AffineGain  visible: pedal on AND braking on;  enabled: offroad
  FCA11 Shadow (Log-Only)       HyundaiFca11Shadow      visible: pedal on;                 enabled: offroad   (0045)
  Pedal Delivery Law            HyundaiPedalDeliveryLaw visible: pedal on;                 enabled: offroad
  Gas Rise Lag                  HyundaiGasRiseLag      visible: pedal on;                 enabled: offroad   (0042-Z)
  Dynamic Brake Release         HyundaiFca11DynamicBrakes visible: pedal on AND braking on; enabled: offroad
  Stop-Approach Commitment      StopApproachCommit      visible: always (planner-side);   enabled: offroad
  Custom Longitudinal Tuning    HyundaiLongitudinalTuning visible: pedal OFF (inert with the pedal on; upstream key)
  Hold Clutch Release           HyundaiFca11StopHoldClutch visible: never (retired row; key + reader untouched)

Both keys are read at CarParams build time, so a change takes effect at the NEXT ignition; offroad-only enabling makes
that consistent. The dependency (shaping needs braking) is ALSO enforced in the car layer (opendbc setup_interfaces), so
a persisted shaping ON with braking off is inert even if this UI rule were bypassed.
"""
from typing import NamedTuple


class ToggleState(NamedTuple):
  visible: bool
  enabled: bool


# R2: the four laws an FCA11 Braking car can run, plus "none" (Braking off => the controller is enabled=False and
# nothing is sent). Tokens only - the panel turns them into words, and the test compares them with the REAL car layer.
LAW_NONE = "none"
LAW_CONTINUOUS = "continuous"
LAW_SHAPED = "shaped"
LAW_RAMP = "ramp"
LAW_OLD = "old"


class BrakeState(NamedTuple):
  law: str
  brake: bool
  shaping: bool
  dynamic: bool
  shadow: bool


def fca11_brake_state(brake_raw, shaping_raw, dynamic_raw, shadow_raw) -> BrakeState:
  """Which FCA11 brake law a car built with these four params will run, from the VALUES, not from a fixed sentence.

  Mirrors the car layer exactly (re-verified against the real `setup_interfaces` + `Fca11LongBrake` over all 16
  Braking x Shaping x Dynamic x Shadow combinations): Braking off => `enabled=False`, no law at all (the reader is
  where the keys land, so nothing else can matter); Shadow on => the plain OLD law (setup_interfaces refuses to write
  fca11AffineGain / fca11DynamicBrakes under Shadow, i.e. it forces both switches off); Shaping AND Dynamic on => the
  continuous linear law (cont_law); Shaping only => the 0043 shaped law; Dynamic only => the OLD law plus the 18x
  release ramp; neither => the plain OLD law."""
  brake, shaping, dynamic, shadow = _on(brake_raw), _on(shaping_raw), _on(dynamic_raw), _on(shadow_raw)
  if not brake:
    law = LAW_NONE
  elif shadow:
    law = LAW_OLD
  elif shaping and dynamic:
    law = LAW_CONTINUOUS
  elif shaping:
    law = LAW_SHAPED
  elif dynamic:
    law = LAW_RAMP
  else:
    law = LAW_OLD
  return BrakeState(law, brake, shaping, dynamic, shadow)


_LAW_WORDS = {
  LAW_NONE: "no FCA11 brake law",
  LAW_CONTINUOUS: "the continuous linear law",
  LAW_SHAPED: "the 0043 shaped law",
  LAW_RAMP: "the OLD law plus the 18x release ramp",
  LAW_OLD: "the plain OLD law",
}


def fca11_brake_state_sentence(brake_raw, shaping_raw, dynamic_raw, shadow_raw) -> str:
  """The car-state sentence the three brake rows open with, built from the ACTUAL param values (R2/N1).

  It names the law AND all four switches, so a reader can check it and the test can compare it with the real
  controller; the Braking-OFF case says that no law runs and that nothing changes, which is what the controller does
  (`enabled=False`, no frame sent) - a static "this car runs the continuous linear law" was false in 7 of 8 states."""
  st = fca11_brake_state(brake_raw, shaping_raw, dynamic_raw, shadow_raw)
  on = lambda b: "ON" if b else "OFF"   # noqa: E731 - one-line formatter, no state
  if st.law == LAW_NONE:
    return ("This car runs no FCA11 brake law: FCA11 Braking OFF, Shaping %s, Dynamic Brakes %s, Shadow %s - nothing "
            "changes while Braking is off." % (on(st.shaping), on(st.dynamic), on(st.shadow)))
  return "This car runs %s: FCA11 Braking ON, Shaping %s, Dynamic Brakes %s, Shadow %s." % (
    _LAW_WORDS[st.law], on(st.shaping), on(st.dynamic), on(st.shadow))


def _on(raw) -> bool:
  """BOOL-param truthiness for a UI read: Params.get() returns a python bool for BOOL keys; tolerate "1"/b"1"/1."""
  if isinstance(raw, bytes):
    raw = raw.decode(errors="ignore")
  return bool(raw is True or str(raw if raw is not None else "0").strip() == "1")


def fca11_brake_toggle_state(pedal_raw, offroad: bool) -> ToggleState:
  pedal_on = _on(pedal_raw)
  return ToggleState(visible=pedal_on, enabled=pedal_on and bool(offroad))


def fca11_shaping_toggle_state(pedal_raw, brake_raw, offroad: bool) -> ToggleState:
  visible = _on(pedal_raw) and _on(brake_raw)
  return ToggleState(visible=visible, enabled=visible and bool(offroad))


def fca11_shadow_toggle_state(pedal_raw, offroad: bool) -> ToggleState:
  """0045: FCA11 Shadow (Log-Only). Visible with the pedal on (independent of FCA11 Braking: the wire stays OLD-LAW as
  Braking says, V3 is computed + logged regardless), settable offroad only (read at CarParams build time => next ignition)."""
  pedal_on = _on(pedal_raw)
  return ToggleState(visible=pedal_on, enabled=pedal_on and bool(offroad))


def pedal_delivery_toggle_state(pedal_raw, offroad: bool) -> ToggleState:
  """17c: Pedal Delivery Law (measured inverse for light braking asks). It lives in the pedal path, so the pedal must be
  on; settable offroad only (CarParamsSP is built at ignition)."""
  pedal_on = _on(pedal_raw)
  return ToggleState(visible=pedal_on, enabled=pedal_on and bool(offroad))


def gas_rise_lag_toggle_state(pedal_raw, offroad: bool) -> ToggleState:
  """0042-Z: Gas Rise Lag. It lives in the pedal path (the accel ask before the pedal law), so the pedal must be on;
  settable offroad only (CarParamsSP is built at ignition). Independent of FCA11 Braking - it is the accelerator."""
  pedal_on = _on(pedal_raw)
  return ToggleState(visible=pedal_on, enabled=pedal_on and bool(offroad))


def fca11_dynamic_release_toggle_state(pedal_raw, brake_raw, offroad: bool) -> ToggleState:
  """18x: Dynamic Brake Release — a property of the FCA11 brake law, so it needs FCA11 Braking on; settable offroad only."""
  visible = _on(pedal_raw) and _on(brake_raw)
  return ToggleState(visible=visible, enabled=visible and bool(offroad))


def fca11_stop_hold_toggle_state(pedal_raw, brake_raw, offroad: bool) -> ToggleState:
  """0034 Hold Clutch Release — RETIRED as a row, and the citation is precise: the ESC's own stop hold has NEVER been
  validated on the car (FORK.md row 137: "Not validated (drive-gated)"), and the reason the pair was expected to
  matter is not in 0x38D at all — the TCU opens the DCT clutch on the BRAKE PEDAL switch, not on any bit we send
  (the lane-U report, UREPORT.md Q2-Q3; summarised in FORK.md row 140). So the row is hidden everywhere. The key, its
  reader, the capnp slot @11 and the stored value stay untouched. To bring the row back, restore the visibility
  expression below (`visible = _on(pedal_raw) and _on(brake_raw)`) and update the tests that pin it hidden."""
  return ToggleState(visible=False, enabled=False)


def stop_approach_toggle_state(offroad: bool) -> ToggleState:
  """17c lane C: Stop-Approach Commitment is planner-side (no pedal dependency), always listed, settable offroad only."""
  return ToggleState(visible=True, enabled=bool(offroad))


def longitudinal_tuning_toggle_state(pedal_raw, offroad: bool) -> ToggleState:
  """UPSTREAM sunnypilot key HyundaiLongitudinalTuning (the Off/Dynamic/Predictive row). It is INERT on a
  gas-interceptor car: with the pedal on, `scc_long` is False so the tuned accel is only published, never sent, and
  this car's tuning config resolves to the 0.50 s the interceptor already sets (the wire is byte-identical for
  0/1/2). So the row is HIDDEN while the pedal is on and comes back the moment it is off - a visibility rule only:
  the key, its reader, initialize_params' feed and the stored value are all untouched (other cars still use it)."""
  pedal_on = _on(pedal_raw)
  visible = not pedal_on
  return ToggleState(visible=visible, enabled=visible and bool(offroad))
