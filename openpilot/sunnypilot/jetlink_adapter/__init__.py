"""
Jetlink on this fork: the adapter jetlink runs over, and the hooks manager, modeld, hardwared and selfdrived call.

Derived from zoompilot's openpilot/sunnypilot/jetlink_adapter (Copyright (c) 2026-, Zeph Leggett, MIT License),
trimmed to the pieces this fork wires in phase 1 (manager, modeld, modeld_v2, selfdrived, hardwared; no UI, no
model-manager integration).

jetlink (the jetlink_repo submodule) runs the large driving model on an attached iPhone, Mac or Jetson over USB.
Its stock adapter (jetlink.openpilot.stock) knows stock openpilot's paths; Adapter below is that with this fork's
MADS. The hooks (jetlink.openpilot.hooks) never raise and answer as if the link were off when jetlink is not
checked out, or speaks another API.

DEFAULT OFF: with the JetlinkLink param at 0 (its default) every hook below answers as if jetlink did not exist.

manager imports this module to build the process list, and runs it as jetlinkd, the resident gadget owner, which
stays at about 10 MB for the whole drive. So the top level is the standard library only; everything else is
imported where it is used. tests/test_adapter.py holds the line.
"""
from __future__ import annotations

import functools
import os
import sys
import threading
from collections import namedtuple
from pathlib import Path

# the version of jetlink.openpilot's API this adapter is written to; any other is treated as jetlink being absent,
# with the reason as the offroad alert
API = 4

# the gadget owner, as manager names the process and selfdrived lists it
OWNER = 'jetlinkd'

# the Jetlink setting, stored as an index: jetlink.openpilot.MODES, written out so this module needs no jetlink
MODES = ('off', 'usb', 'ios')

# the params jetlink reads and writes, all declared in params_keys.h. No model-manager slot, catalog or select key:
# phase 1 runs the model picked in the Jetlink app, else jetlink's default big model
_Keys = namedtuple('_Keys', 'link offroad progress spec charge_phone')
KEYS = _Keys(link='JetlinkLink', offroad='IsOffroad', progress='JetlinkProgress', spec='JetlinkSpec',
             charge_phone='JetlinkChargePhone')

_AGNOS = os.path.isfile('/AGNOS')


def _params_dir() -> Path:
  """The params store's directory, by params.cc and hw.h's rule: PARAMS_ROOT, else /data/params on a device and
  $HOME/.comma<OPENPILOT_PREFIX>/params elsewhere, then /<OPENPILOT_PREFIX, or d>. Per call, so it follows a
  prefix as Params does; the environment only, so it never raises."""
  prefix = os.environ.get('OPENPILOT_PREFIX', '')
  root = os.environ.get('PARAMS_ROOT')
  if root is None:
    root = '/data/params' if _AGNOS else os.environ.get('HOME', '') + '/.comma' + prefix + '/params'
  return Path(root) / os.environ.get('OPENPILOT_PREFIX', 'd')


def link_on() -> bool:
  """Is the Jetlink setting on? As jetlink reads it, a file, so that a jetlink this build cannot use tells only
  someone who turned the link on."""
  try:
    return 0 < int((_params_dir() / KEYS.link).read_bytes()) < len(MODES)
  except (OSError, ValueError):
    return False


def chestnut_cable(connected: bool | None = None) -> bool:
  """A cable on the port, for modeld's chestnut guess: `connected`, or the port read now. A Jetson or an iPhone on
  the port is a cable too, so it counts only while Jetlink is off (jetlink docs/porting.md)."""
  from openpilot.common.hardware.usb import cable_connected
  if connected is None:
    connected = cable_connected()
  return connected and not link_on()


def _with_blendful(get_action_from_model, min_lat_control_speed: float):
  """Stock modeld's action function taking Blendful's callback as modeld_v2 calls it. Stock leaves the curvature
  unsmoothed (LAT_SMOOTH_SECONDS 0), so the correction added to its answer is the one modeld_v2 adds before
  smoothing (zoompilot's model_settings.with_blendful)."""
  def action(model_output, prev_action, lat_action_t, long_action_t, v_ego, blendful=None):
    result = get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego)
    if blendful is not None:
      correction = blendful(result.desiredCurvature)
      if v_ego > min_lat_control_speed:
        result.desiredCurvature += correction
    return result
  return action


def _smoothing() -> tuple[float, float, float]:
  """(LAT_SMOOTH_SECONDS, LONG_SMOOTH_SECONDS, MIN_LAT_CONTROL_SPEED) as the joining model reports them: stock
  modeld's, which jetlink's model face (Stock.model_face) hands it."""
  from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS, LONG_SMOOTH_SECONDS, MIN_LAT_CONTROL_SPEED
  return LAT_SMOOTH_SECONDS, LONG_SMOOTH_SECONDS, MIN_LAT_CONTROL_SPEED


def _fork_action_function():
  """modeld_v2's ModelState.get_action_from_model, for a big model that outputs `action` or a plan: the action this
  fork's own big (chestnut) model gets, so a swap to the Jetlink model is not a step in what is commanded. Differs
  from stock modeld's in the curvature's speed floor (max(2.5, v_ego), stock max(1.0, v_ego), a 6x difference at
  1 m/s) and in how a plan's curvature is read. tests/test_adapter.py holds it to modeld_v2's source."""
  from openpilot.cereal import log
  from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, smooth_value
  from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan
  from openpilot.sunnypilot.modeld_v2.fill_model_msg import get_curvature_from_output
  lat_smooth, long_smooth, min_lat_control_speed = _smoothing()

  def action(model_output, prev_action, lat_action_t, long_action_t, v_ego, blendful=None):
    if 'action' not in model_output:
      plan = model_output['plan'][0]
      desired_accel = get_accel_from_plan(plan[:, Plan.VELOCITY][:, 0], plan[:, Plan.ACCELERATION][:, 0], ModelConstants.T_IDXS,
                                          action_t=long_action_t)
      desired_curvature = get_curvature_from_output(model_output, plan, v_ego, lat_action_t, False)
    else:
      desired_accel = model_output['action'][0, 1]
      desired_curvature = model_output['action'][0, 0] / (max(2.5, v_ego)) ** 2
    if blendful is not None:
      desired_curvature += blendful(desired_curvature)

    stop = v_ego < 0.3 and desired_accel < 0.1
    desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, long_smooth)
    if v_ego > min_lat_control_speed:   # smooth curvature, as for a post-FOF bundle
      desired_curvature = smooth_value(desired_curvature, prev_action.desiredCurvature, lat_smooth)
    else:
      desired_curvature = prev_action.desiredCurvature
    return log.ModelDataV2.Action(desiredCurvature=float(desired_curvature), desiredAcceleration=float(desired_accel),
                                  shouldStop=bool(stop))
  return action


def _in_modeld_v2() -> bool:
  """Is this process sunnypilot's modeld_v2 (modeld_tinygrad, run as a script by manager) rather than stock modeld
  (run by manager's launcher, so __main__ is manager)? Each runs its own big model's action rule."""
  main_file = str(getattr(sys.modules.get('__main__'), '__file__', None) or '')
  return 'modeld_v2' in main_file.replace('\\', '/').split('/') or 'openpilot.sunnypilot.modeld_v2.modeld' in sys.modules


@functools.cache
def _adapter_class():
  from jetlink.openpilot.interface import Keys
  from jetlink.openpilot.stock import Stock

  class Adapter(Stock):
    """jetlink.openpilot.interface.Openpilot over this fork: stock openpilot but for what follows."""
    KEYS = Keys(**KEYS._asdict())
    # MADS's state rides on carControlSP
    IN_CONTROL_SERVICES = ('carState', 'carControl', 'carControlSP')

    def chestnut_present(self) -> bool:
      # modeld's own test, the one it runs the big model on: a chestnut whose firmware names itself
      from openpilot.selfdrive.modeld.helpers import chestnut_present
      return chestnut_present()

    def model_constants(self):
      # modeld_tinygrad reads modeld_v2's off the model
      from openpilot.sunnypilot.modeld_v2.constants import ModelConstants
      return ModelConstants

    def action_function(self):
      # the action rule of the modeld this runs in, so the joining model drives as this fork's own big model does.
      # modeld_v2 passes Blendful's correction as a callback on the raw curvature; stock modeld has no Blendful
      if _in_modeld_v2():
        return _fork_action_function()
      from openpilot.selfdrive.modeld.modeld import MIN_LAT_CONTROL_SPEED, get_action_from_model
      return _with_blendful(get_action_from_model, MIN_LAT_CONTROL_SPEED)

    def engaged(self, sm) -> bool:
      # selfdrived's enabled or mads.enabled, as controlsd republishes them. MADS counts with its lateral paused
      # (a stop, a blinker, the brake): it steers again on its own
      return bool(sm['carControl'].enabled or sm['carControlSP'].mads.enabled)

  return Adapter


def adapter():
  """The adapter, for jetlink's entry points that run as their own process: the provisioning run."""
  return _adapter_class()()


def owner_config():
  """What jetlinkd needs: data only, so the owner imports nothing heavy."""
  return _adapter_class().owner_config()


def main() -> None:
  """jetlinkd: hold the USB gadget until manager stops this process."""
  _adapter_class().main()


# -- the hooks ----------------------------------------------------------------

# this process's hooks, made on first use and kept, the link off included: Python does not cache a failed import.
# One per process, so that prepare() and attach() reach the same jetlink
_hooks = None
_making = threading.Lock()


def hooks():
  """jetlink.openpilot.hooks.Hooks over this adapter, or the link off."""
  global _hooks
  with _making:
    if _hooks is None:
      _hooks = _guard()
    return _hooks


def _guard():
  # inside a function, so a jetlink that fails to import lands here too
  try:
    import jetlink
    if getattr(jetlink, '__file__', None) is None:
      return _Off()   # an empty jetlink_repo, which Python takes for a namespace package
    from jetlink.openpilot.hooks import Hooks
  except ModuleNotFoundError as e:
    if e.name == 'jetlink':
      return _Off()   # no checkout: the link does not exist on this device
    if (e.name or '').startswith('jetlink.'):
      return _unusable("jetlink package too old for this build", e)
    return _unusable(f"jetlink failed to load: {e}", e)
  except Exception as e:
    return _unusable(f"jetlink failed to load: {type(e).__name__}: {e}", e)
  try:
    return Hooks(API, adapter, link_on=link_on, log=_cloudlog)
  except Exception as e:
    return _unusable(str(e), e)   # another API, which it names


def _cloudlog():
  from openpilot.common.swaglog import cloudlog
  return cloudlog


def _unusable(why: str, error: Exception) -> _Off:
  try:
    _cloudlog().error("jetlink: %s", why, exc_info=error)
  except Exception:
    pass
  return _Off(why)


class _Off:
  """Every hook with the link off; reason() says why, to someone who turned the link on. The fork's own: jetlink's
  Off cannot answer for a jetlink that is missing or too old to have it, and one answer for every case keeps them
  alike."""
  # by hook; a callable is handed the hook's arguments. A wanted power-off goes ahead: no jetlink must keep a comma
  # on to drain the car's battery
  ANSWERS = {'should_run': False, 'status': None, 'prepare': False, 'attach': lambda model, *_: model,
             'handed_over': False, 'state': 'none', 'shutdown_step': lambda wanted, reason='': wanted,
             'model_state': None, 'lists_big_models': False, 'big_catalog': None, 'listed_ref': None,
             'swap_events': None}

  def __init__(self, why: str = ''):
    self.why = why

  def reason(self) -> str | None:
    return self.why if self.why and link_on() else None

  def __getattr__(self, name: str):
    if name not in self.ANSWERS:
      raise AttributeError(name)
    answer = self.ANSWERS[name]
    return answer if callable(answer) else lambda *args: answer


def _hook(name: str):
  def call(*args):
    return getattr(hooks(), name)(*args)
  call.__name__ = call.__qualname__ = name
  return call


# the hooks by the names the hook lines call, each this process's hooks() at the call: jetlink.openpilot.hooks says
# what each does and answers
HOOK_NAMES = (*_Off.ANSWERS, 'reason')
(should_run, status, prepare, attach, handed_over, state, shutdown_step, model_state, lists_big_models,
 big_catalog, listed_ref, swap_events, reason) = map(_hook, HOOK_NAMES)
