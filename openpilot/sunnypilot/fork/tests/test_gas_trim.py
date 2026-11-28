"""fork/gas_trim.py -- the gas-side integral trim.

The properties that matter are the ones that could hurt: it never touches a brake frame, never lowers an ask,
never runs where the car already over-delivers, and closes the steady-state error when it is allowed to run.
"""
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.fork.gas_trim import GasTrim, KI, TRIM_MAX, V_MIN, RATE


class FakeParams:
  def __init__(self, on, ab=False):
    self.vals = {"GasTrim": on, "GasTrimAB": ab}

  def get_bool(self, key):
    if key not in self.vals:
      raise KeyError(key)          # mirrors UnknownKeyName for a libparams that lacks the key
    return self.vals[key]


class FakeNoKey:
  def get_bool(self, key):
    raise KeyError(key)


def make(on=True, ab=False, start_bit=0):
  t = GasTrim(params=FakeParams(on, ab), ab_start_bit=start_bit)
  t.update_params()
  return t


def settle(t, a_target, a_ego, v=15.0, pedal=False, active=True, seconds=10.0, cap=1.0):
  out = a_target
  for _ in range(int(seconds / DT_MDL)):
    out = t.update(a_target, a_ego, v, pedal, active, cap_factor=cap)
  return out


def test_inert_when_param_off():
  t = GasTrim(params=FakeParams(False))
  t.update_params()
  for _ in range(200):
    assert t.update(0.5, 0.2, 15.0, False, True) == 0.5
  assert t.trim == 0.0
  assert t.applied == 0.0


def test_never_touches_a_zero_or_negative_ask():
  t = make()
  for a in (0.0, -0.1, -1.5, -3.0):
    for _ in range(50):
      assert t.update(a, -2.0, 15.0, False, True) == a


def test_never_lowers_an_ask():
  t = make()
  for a in (0.05, 0.3, 1.2):
    out = settle(t, a, a + 2.0)          # the car is doing MORE than asked -> error is negative
    assert out >= a


def test_inert_below_v_min():
  t = make()
  for _ in range(200):
    assert t.update(0.5, 0.0, V_MIN - 0.1, False, True) == 0.5
  assert t.trim == 0.0


def test_inert_with_the_driver_on_the_pedal():
  t = make()
  for _ in range(200):
    assert t.update(0.5, 0.0, 15.0, True, True) == 0.5
  assert t.trim == 0.0


def test_resets_when_not_active():
  t = make()
  settle(t, 0.5, 0.2, seconds=5.0)
  assert t.trim > 0.0
  t.update(0.5, 0.2, 15.0, False, False)
  assert t.trim == 0.0 and t.applied == 0.0


def test_closes_a_real_steady_state_error():
  """Closed loop against the measured plant: the car delivers ~0.65 of the published ask at 15-20 m/s.
  With the trim off the car would sit at 0.325 for a 0.5 target; with it on the published ask must rise until
  the DELIVERED accel equals the target. Equilibrium: 0.65*(0.5 + trim) = 0.5 -> trim = 0.269."""
  t = make()
  a_target, plant = 0.50, 0.65
  delivered = 0.0
  for _ in range(int(30.0 / DT_MDL)):
    out = t.update(a_target, delivered, 15.0, False, True)
    delivered = plant * out
  assert t.trim == pytest.approx(0.269, abs=0.02)
  assert out == pytest.approx(0.769, abs=0.02)
  assert delivered == pytest.approx(a_target, abs=0.02)


def test_without_the_trim_the_same_plant_stays_short():
  """The control arm: the same plant, param off -> the car never reaches the ask."""
  t = GasTrim(params=FakeParams(False))
  t.update_params()
  a_target, plant = 0.50, 0.65
  delivered = 0.0
  for _ in range(int(30.0 / DT_MDL)):
    out = t.update(a_target, delivered, 15.0, False, True)
    delivered = plant * out
  assert delivered == pytest.approx(0.325, abs=0.01)


def test_trim_is_clamped():
  t = make()
  for _ in range(int(60.0 / DT_MDL)):
    out = t.update(1.5, -3.0, 20.0, False, True)     # huge error, held for a minute
  assert t.trim == pytest.approx(TRIM_MAX)
  assert out == pytest.approx(1.5 + TRIM_MAX)


def test_does_not_integrate_at_the_platform_ceiling():
  t = make()
  for _ in range(200):
    out = t.update(1.95, 0.0, 20.0, False, True)
  assert t.trim == 0.0
  assert out == 1.95


def test_a_sustained_brake_drains_the_trim_and_re_admission_is_one_step():
  t = make()
  settle(t, 0.5, 0.2, seconds=10.0)
  assert t.trim > 0.0 and t.applied > 0.0
  # 2 s of braking: every frame is untouched, and the applied part drains at RATE
  for _ in range(int(2.0 / DT_MDL)):
    assert t.update(-1.0, 0.2, 15.0, False, True) == -1.0
  assert t.applied == 0.0
  assert t.trim > 0.0                      # the integrator state is held, not thrown away
  # re-admission is one step, not the whole trim
  out = t.update(0.5, 0.2, 15.0, False, True)
  assert out - 0.5 <= RATE * DT_MDL + 1e-9


def test_a_brief_non_gas_frame_restores_without_a_new_step():
  """A single brake frame must not drop the trim (that is the dithering path): the applied part only bleeds one
  step, so its return is one step, and the ask never exceeds the stock ask plus the current trim."""
  t = make()
  settle(t, 0.5, 0.2, seconds=10.0)
  applied_before = t.applied
  assert t.update(-1.0, 0.2, 15.0, False, True) == -1.0
  assert t.applied == pytest.approx(max(0.0, applied_before - RATE * DT_MDL))
  out = t.update(0.5, 0.2, 15.0, False, True)
  assert out <= 0.5 + t.trim + 1e-9


def test_ki_and_rate_are_the_documented_ones():
  assert KI == 0.3 and RATE == 0.5 and TRIM_MAX == 0.5 and V_MIN == 10.0


def test_unknown_key_stays_off_and_says_so():
  t = GasTrim(params=FakeNoKey(), ab_start_bit=0)
  t.update_params()
  assert t.enabled is False and t.key_known is False
  for _ in range(50):
    assert t.update(0.5, 0.2, 15.0, False, True) == 0.5


def test_frozen_while_the_no_close_cap_is_active():
  t = make()
  for _ in range(400):
    out = t.update(0.5, 0.2, 15.0, False, True, cap_factor=0.0)
    assert out == 0.5
  assert t.trim == 0.0 and t.applied == 0.0


def test_cap_factor_below_one_also_freezes():
  t = make()
  for _ in range(200):
    t.update(0.5, 0.2, 15.0, False, True, cap_factor=0.6)
  assert t.trim == 0.0


def test_ab_alternates_on_fixed_blocks_and_bleeds_out():
  t = make(ab=True, start_bit=0)
  block = 1200
  # block 0 -> arm 1 (ON), block 1 -> arm 0 (OFF)
  for _ in range(block):
    t.update(0.5, 0.2, 15.0, False, True)
  assert t.arm_effective == 1 and t.trim > 0.0
  trim_at_boundary = t.trim
  outs = [t.update(0.5, 0.2, 15.0, False, True) for _ in range(block)]
  assert t.arm_effective == 0
  assert all(o == 0.5 for o in outs)           # the ask is untouched while the arm is OFF
  assert t.trim == pytest.approx(trim_at_boundary)   # state is held, not integrated
  # the applied part bleeds out at RATE rather than dropping in one frame
  assert t.applied == 0.0


def test_ab_start_bit_shifts_the_first_block():
  t = make(ab=True, start_bit=1)
  t.update(0.5, 0.2, 15.0, False, True)
  assert t.arm_wanted == 0
