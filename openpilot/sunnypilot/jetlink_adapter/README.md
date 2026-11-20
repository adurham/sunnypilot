# Jetlink on this fork

Runs the comma large driving model on an iPhone (or Mac/Jetson) over the comma's AUX USB-C port, with the small
model as the always-on fallback. Phase 1 wiring: manager (`jetlinkd` owner), modeld + modeld_v2 (join/attach),
selfdrived (swap events), hardwared (power-off + alert). No UI and no model-manager integration yet: the iPhone app
picks the model.

**Default off.** `JetlinkLink` (INT, default 0) is the setting: 0 off, 1 USB host (Jetson/PC/Mac), 2 iOS. With it
at 0 every hook answers as if jetlink did not exist (`tests/test_default_off.py`). Two things still run with the
link off, by jetlink's design: hardwared binds the shutdown/alert hooks (fail-open: a jetlink failure never blocks
a power-off), and the launch script's boot hook is skipped unless the param file is non-zero.

Pins: `jetlink_repo` (adurham/jetlink, fork of zoompilot/jetlink) at v0.9.0, API 4. Adapter derived from
zoompilot's (MIT, Zeph Leggett).

## Test (nothing here touches the car until it is deployed and the param is written)

Offline, on the workstation (stdlib + the repo venv): `pytest openpilot/sunnypilot/jetlink_adapter`.

On the car (needs the owner's go-ahead for the deploy and the param write), iPhone on iOS 26.1+ running the
Jetlink app built from jetlink v0.9.0, a USB 3 cable in the **AUX** port (never OBD-C), the comma on its own
power port:

1. Offroad: set `JetlinkLink` to 2 (`params.put("JetlinkLink", 2)` or the file `/data/params/d/JetlinkLink`),
   reboot so manager and modeld read it.
2. Open the app, allow Local Network, keep it on screen: wait for "Connected over USB 3".
3. Offroad bench first (Benchmark -> 1 Minute / 10 Minutes in the app), then a parked onroad cycle watching
   `modelV2.big`, then a short drive with the small model as fallback.
4. Roll back: `JetlinkLink` = 0 and reboot.
