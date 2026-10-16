from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.locationd.locationd import ALL_VALID_INVALID_LIMIT, debounce_all_valid


class TestLocationdInputsValid(OpenpilotTestCase):
  """
  locationd's inputs_valid used to be `sm.all_valid() and critical_service_inputs_valid`, so a single message
  flagged invalid (one cameraOdometry with valid=False from a skipped camera frame) dropped
  deviceMotion.inputsOK for a frame and selfdrived raised locationdTemporaryError. These tests pin the
  debounce that rides out that blip while still catching a sustained invalid stream (#38505 / #38929).
  """

  def test_limit_tolerates_a_transient_blip(self):
    # a single-frame blip is the case this exists for, so the limit must be at least 2
    assert ALL_VALID_INVALID_LIMIT >= 2

  def test_starts_bad_until_first_valid_poll(self):
    # main() seeds the counter at the limit so a service that has not arrived yet (e.g. extrinsicsCalibration
    # at segment start) still reads bad from the first poll, matching the pre-debounce behavior
    cnt = ALL_VALID_INVALID_LIMIT
    cnt, all_valid = debounce_all_valid(cnt, False)
    assert not all_valid

    cnt, all_valid = debounce_all_valid(cnt, True)
    assert (cnt, all_valid) == (0, True)

  def test_single_invalid_poll_does_not_flip_inputs(self):
    # the reported failure: exactly one invalid cameraOdometry poll followed by recovery
    cnt, all_valid = debounce_all_valid(0, False)
    assert cnt == 1
    assert all_valid

    cnt, all_valid = debounce_all_valid(cnt, True)
    assert (cnt, all_valid) == (0, True)

  def test_limit_minus_one_invalid_polls_tolerated(self):
    cnt, all_valid = 0, True
    for i in range(ALL_VALID_INVALID_LIMIT - 1):
      cnt, all_valid = debounce_all_valid(cnt, False)
      assert all_valid, f"inputs flipped invalid after only {i + 1} consecutive invalid polls"
    assert cnt == ALL_VALID_INVALID_LIMIT - 1

  def test_limit_consecutive_invalid_polls_flip_inputs(self):
    cnt, all_valid = 0, True
    for _ in range(ALL_VALID_INVALID_LIMIT):
      cnt, all_valid = debounce_all_valid(cnt, False)
    assert not all_valid
    assert cnt == ALL_VALID_INVALID_LIMIT

  def test_valid_poll_resets_counter(self):
    cnt, all_valid = 0, True
    for _ in range(ALL_VALID_INVALID_LIMIT - 1):
      cnt, all_valid = debounce_all_valid(cnt, False)
    assert all_valid

    # one good poll wipes the accumulated count...
    cnt, all_valid = debounce_all_valid(cnt, True)
    assert (cnt, all_valid) == (0, True)

    # ...so the next limit-1 invalid polls are tolerated again
    for _ in range(ALL_VALID_INVALID_LIMIT - 1):
      cnt, all_valid = debounce_all_valid(cnt, False)
      assert all_valid

  def test_alternating_invalid_stream_tolerated(self):
    # an invalid-every-other-message stream never accumulates, since every good poll resets the counter
    cnt, all_valid = 0, True
    for _ in range(10):
      cnt, all_valid = debounce_all_valid(cnt, False)
      assert all_valid
      cnt, all_valid = debounce_all_valid(cnt, True)
      assert all_valid

  def test_sustained_invalid_stream_stays_bad_until_recovery(self):
    cnt, all_valid = 0, True
    for _ in range(ALL_VALID_INVALID_LIMIT):
      cnt, all_valid = debounce_all_valid(cnt, False)
    assert not all_valid

    # stays bad while the stream keeps arriving invalid
    for _ in range(50):
      cnt, all_valid = debounce_all_valid(cnt, False)
      assert not all_valid

    # and clears as soon as a valid poll arrives
    cnt, all_valid = debounce_all_valid(cnt, True)
    assert (cnt, all_valid) == (0, True)
