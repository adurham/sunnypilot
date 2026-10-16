"""
Fork: never delete the owner's cruise preferences when openpilot longitudinal is
temporarily unavailable (adurham/sunnypilot). See FORK.md #30.

Why this exists
---------------
The owner's 2022 Elantra N gets openpilot longitudinal from a comma pedal (gas interceptor)
gated by the ``HyundaiGasInterceptor`` param (fork #10). When that param is off for an
ignition — it was disarmed for a parked test — ``CP.openpilotLongitudinalControl`` is False
and ``CP_SP.pcmCruiseSpeed`` is True, i.e. the car looks exactly like a plain stock-ACC car.
Upstream sunnypilot then treats the longitudinal-dependent settings as unsupported and
DELETES them from Params:

  * ``selfdrive/car/interfaces.py`` ``_cleanup_unsupported_params`` — removes DEC /
    CustomAccIncrementsEnabled / SCC-V / SCC-M when ``not openpilotLongitudinalControl and
    pcmCruiseSpeed``;
  * ``selfdrive/ui/sunnypilot/ui_state.py`` ``_enforce_constraints`` — removes ExperimentalMode
    + DEC when there is no longitudinal control, and CustomAcc / SCC-V / SCC-M when there is
    neither longitudinal nor ICBM;
  * ``selfdrive/ui/sunnypilot/layouts/settings/cruise.py`` — the same removals from the
    settings screen;
  * ``selfdrive/selfdrived/selfdrived.py`` / ``selfdrive/ui/layouts/settings/toggles.py`` /
    ``selfdrive/ui/mici/layouts/settings/toggles.py`` — remove ExperimentalMode when there is
    no openpilot longitudinal control.

On the owner's car that condition is TRANSIENT (the interceptor is re-armed on the next
ignition), but the deletion is permanent: after re-arming, DEC and the custom-increment enable
switch stayed gone — the stored 5 mph values themselves survived, so the buttons silently
stepped 1 mph — and SCC vision/map + experimental mode had to be re-toggled by hand.

What this module does
---------------------
One predicate (``preserved``) and one wrapper (``remove_unless_preserved``) that every one of
those cleanup sites calls instead of ``params.remove`` directly. The five keys below are kept
in Params no matter what the current ignition's CarParams say. Nothing else changes:

  * the UI still DISABLES / greys the toggles when the feature is unavailable
    (``sync_toggle`` keeps the toggle showing the stored value while it is disabled, so what
    the screen shows always matches the param);
  * the runtime consumers stay gated on longitudinal being active — they are untouched by this
    module. Experiment mode / DEC only act when ``selfdriveState.experimentalMode`` is set,
    which selfdrived/card compute as ``param and CP.openpilotLongitudinalControl``; SCC-V/SCC-M
    targets are only actuated when ``carControl.longActive`` is true, which controlsd computes
    as ``... and (openpilotLongitudinalControl or not pcmCruiseSpeed)``; the custom-increment
    apply path is only reached on a non-PCM-cruise car. Verified by
    ``fork/tests/test_cruise_prefs.py::TestConsumersInert`` / ``TestUpstreamGates``.

Scope: fork-only file (nothing here exists upstream), so an upstream sync never conflicts on
it. The six call sites are one-line changes from ``params.remove`` to
``remove_unless_preserved``.
"""

# The five cruise preferences the owner chose once and expects to survive a transient loss of
# openpilot longitudinal. Order matches FORK.md #30.
CRUISE_PREF_KEYS = (
  "ExperimentalMode",
  "DynamicExperimentalControl",
  "CustomAccIncrementsEnabled",
  "SmartCruiseControlVision",
  "SmartCruiseControlMap",
)


def preserved(key: str) -> bool:
  """True for the cruise preferences this fork never deletes."""
  return key in CRUISE_PREF_KEYS


def remove_unless_preserved(params, key: str) -> None:
  """``params.remove(key)`` unless it is one of the preserved cruise preferences.

  Drop-in replacement for the ``params.remove(key)`` calls in the upstream cleanup paths listed
  in the module doc. ``params`` is a ``common.params.Params``; duck-typed so tests can pass a
  fake.
  """
  if not preserved(key):
    params.remove(key)


def sync_toggle(params, key: str, item) -> None:
  """Show a toggle widget's STORED value even while it is disabled.

  ``item`` is a ``ListItemSP`` (its ``action_item`` is a ``ToggleAction``-like widget with
  ``set_state``). Upstream clobbers the displayed state to False while the feature is
  unavailable; that makes the screen disagree with the saved param, so the owner cannot see
  what is actually stored.
  """
  item.action_item.set_state(params.get_bool(key))


def maneuver_mode_active(params) -> bool:
  """True while a developer maneuver/debug mode that must suppress experimental mode is on.

  Lateral maneuver mode (controlsd still runs and steers) and joystick debug mode (controlsd does
  not run at all) both need experimental mode OFF at runtime. Upstream enforced this by overwriting
  the stored ``ExperimentalMode`` param from the developer UI; this fork suppresses experimental
  mode in the consumer instead (``experimental_active``), so the owner's stored preference is never
  destroyed by flipping those toggles.

  Scope note: LongitudinalManeuverMode is deliberately NOT here — the original developer write only
  existed in the lateral-maneuver callback, and under longitudinal maneuver mode plannerd (which
  owns DEC/e2e) does not run at all, so experimental mode is already moot.
  """
  return params.get_bool("LateralManeuverMode") or params.get_bool("JoystickDebugMode")


def experimental_active(params, openpilot_longitudinal: bool) -> bool:
  """The runtime experimental-mode state (what card/selfdrived publish as selfdriveState.experimentalMode).

  The stored preference, with two overrides applied:

  * ``openpilot_longitudinal`` must be True — the pre-existing gate (a stock-ACC car ignores the
    stored ExperimentalMode even though this fork keeps the param around); and
  * a developer maneuver mode (lateral maneuver mode / joystick debug mode) forces experimental OFF
    while it is active, WITHOUT touching the stored preference — so toggling the mode on and off
    leaves the owner's ExperimentalMode exactly as he set it.
  """
  return params.get_bool("ExperimentalMode") and openpilot_longitudinal and not maneuver_mode_active(params)
