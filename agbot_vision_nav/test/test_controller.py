import pytest

from agbot_vision_nav.controller import RowCenteringController


def test_sign_convention_negative_offset_turns_left_positive_angular_z():
    # offset_norm < 0 means the traversable centerline is LEFT of image
    # center -> robot must turn left -> positive angular.z (REP-103).
    controller = RowCenteringController(k_p=1.0, angular_z_max=10.0)
    linear_x, angular_z = controller.compute(offset_norm=-0.5, slope_term=0.0, valid=True)
    assert angular_z > 0
    assert angular_z == pytest.approx(0.5)


def test_sign_convention_positive_offset_turns_right_negative_angular_z():
    controller = RowCenteringController(k_p=1.0, angular_z_max=10.0)
    linear_x, angular_z = controller.compute(offset_norm=0.5, slope_term=0.0, valid=True)
    assert angular_z < 0
    assert angular_z == pytest.approx(-0.5)


def test_zero_offset_goes_straight():
    controller = RowCenteringController(k_p=1.0)
    linear_x, angular_z = controller.compute(offset_norm=0.0, slope_term=0.0, valid=True)
    assert angular_z == pytest.approx(0.0)
    assert linear_x == controller.linear_x_cruise


def test_angular_z_is_clamped():
    controller = RowCenteringController(k_p=10.0, angular_z_max=0.3)
    _, angular_z = controller.compute(offset_norm=1.0, slope_term=0.0, valid=True)
    assert angular_z == pytest.approx(-0.3)
    _, angular_z = controller.compute(offset_norm=-1.0, slope_term=0.0, valid=True)
    assert angular_z == pytest.approx(0.3)


def test_slope_term_contributes_with_same_sign_convention():
    controller = RowCenteringController(k_p=0.0, k_slope=1.0, angular_z_max=10.0)
    _, angular_z = controller.compute(offset_norm=0.0, slope_term=0.5, valid=True)
    assert angular_z == pytest.approx(-0.5)


def test_slope_term_disabled_by_default():
    controller = RowCenteringController(k_p=1.0, angular_z_max=10.0)
    assert controller.k_slope == 0.0
    _, angular_z = controller.compute(offset_norm=0.0, slope_term=100.0, valid=True)
    assert angular_z == pytest.approx(0.0)


def test_invalid_frames_hold_last_command_before_stop_threshold():
    controller = RowCenteringController(
        k_p=1.0, linear_x_cruise=0.15, angular_z_max=10.0, invalid_frame_stop_count=5
    )
    linear_x, angular_z = controller.compute(offset_norm=-0.5, slope_term=0.0, valid=True)
    assert (linear_x, angular_z) == (0.15, pytest.approx(0.5))

    # 4 consecutive invalid frames (< stop_count=5) should hold the last command
    for _ in range(4):
        held_linear_x, held_angular_z = controller.compute(
            offset_norm=999.0, slope_term=0.0, valid=False
        )
        assert held_linear_x == pytest.approx(0.15)
        assert held_angular_z == pytest.approx(0.5)


def test_invalid_frames_stop_at_threshold():
    controller = RowCenteringController(
        k_p=1.0, linear_x_cruise=0.15, angular_z_max=10.0, invalid_frame_stop_count=3
    )
    controller.compute(offset_norm=-0.5, slope_term=0.0, valid=True)
    controller.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    controller.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    linear_x, angular_z = controller.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    assert (linear_x, angular_z) == (0.0, 0.0)
    assert controller.consecutive_invalid == 3


def test_valid_frame_after_invalid_resets_invalid_counter():
    controller = RowCenteringController(invalid_frame_stop_count=3)
    controller.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    controller.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    assert controller.consecutive_invalid == 2
    controller.compute(offset_norm=0.0, slope_term=0.0, valid=True)
    assert controller.consecutive_invalid == 0


def test_reset_clears_state():
    controller = RowCenteringController(invalid_frame_stop_count=2)
    controller.compute(offset_norm=0.5, slope_term=0.0, valid=True)
    controller.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    assert controller.consecutive_invalid == 1
    controller.reset()
    assert controller.consecutive_invalid == 0
    linear_x, angular_z = controller.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    # after reset, first invalid frame holds the reset (0,0) last-command
    assert (linear_x, angular_z) == (0.0, 0.0)


def test_first_frame_invalid_with_no_history_returns_zero():
    controller = RowCenteringController(invalid_frame_stop_count=5)
    linear_x, angular_z = controller.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    assert (linear_x, angular_z) == (0.0, 0.0)
