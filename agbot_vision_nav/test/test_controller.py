"""Tests for MPCRowController.

Sign-convention contract (must match centerline_estimator.py and vision_nav_node.py):
  offset_norm < 0  ->  centerline is LEFT of image center
                   ->  robot is RIGHT of the row centerline
                   ->  must turn LEFT  ->  angular_z > 0  (REP-103)

  slope_term > 0   ->  far scan-row midpoint is RIGHT of near midpoint
                   ->  corridor tilts rightward in image
                   ->  robot heading slightly LEFT of row direction
                   ->  must turn RIGHT to correct  ->  angular_z < 0

These signs are unchanged from the old P-controller; the MPC optimizer produces
the same corrective direction because the cost penalizes non-zero state.
"""

import pytest

from agbot_vision_nav.controller import MPCRowController


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make(angular_z_max=10.0, **kwargs):
    """Construct a controller with loose bounds so clamping doesn't interfere."""
    defaults = dict(
        N=6,
        q_offset=10.0,
        q_heading=1.0,
        r_control=0.01,
        r_delta=0.0,
        angular_z_max=angular_z_max,
        delta_angular_z_max=angular_z_max * 2,
        invalid_frame_stop_count=5,
    )
    defaults.update(kwargs)
    return MPCRowController(**defaults)


# ---------------------------------------------------------------------------
# Sign-convention tests (locked in — do not change)
# ---------------------------------------------------------------------------

def test_sign_convention_negative_offset_turns_left_positive_angular_z():
    # offset_norm < 0: centerline LEFT of image -> robot RIGHT of row -> turn LEFT -> angular_z > 0
    ctrl = _make()
    _, az = ctrl.compute(offset_norm=-0.5, slope_term=0.0, valid=True)
    assert az > 0, f"Expected positive angular_z, got {az}"


def test_sign_convention_positive_offset_turns_right_negative_angular_z():
    # offset_norm > 0: centerline RIGHT of image -> robot LEFT of row -> turn RIGHT -> angular_z < 0
    ctrl = _make()
    _, az = ctrl.compute(offset_norm=0.5, slope_term=0.0, valid=True)
    assert az < 0, f"Expected negative angular_z, got {az}"


def test_zero_state_produces_near_zero_command():
    ctrl = _make()
    linear_x, az = ctrl.compute(offset_norm=0.0, slope_term=0.0, valid=True)
    assert abs(az) < 1e-3, f"Expected ~0 angular_z at zero state, got {az}"
    assert linear_x == pytest.approx(ctrl.linear_x_cruise)


def test_heading_left_slope_positive_produces_right_turn():
    # slope_term > 0 -> heading left -> must turn right -> angular_z < 0
    ctrl = _make(q_offset=0.0, q_heading=10.0)   # penalize only heading error
    _, az = ctrl.compute(offset_norm=0.0, slope_term=0.5, valid=True)
    assert az < 0, f"Expected corrective right turn (az<0) for slope_term>0, got {az}"


def test_heading_right_slope_negative_produces_left_turn():
    # slope_term < 0 -> heading right -> must turn left -> angular_z > 0
    ctrl = _make(q_offset=0.0, q_heading=10.0)
    _, az = ctrl.compute(offset_norm=0.0, slope_term=-0.5, valid=True)
    assert az > 0, f"Expected corrective left turn (az>0) for slope_term<0, got {az}"


# ---------------------------------------------------------------------------
# Constraint tests
# ---------------------------------------------------------------------------

def test_angular_z_clamped_by_angular_z_max():
    ctrl = _make(angular_z_max=0.3, q_offset=1000.0, delta_angular_z_max=10.0)
    _, az = ctrl.compute(offset_norm=1.0, slope_term=0.0, valid=True)
    assert az >= -0.3 - 1e-6
    _, az = ctrl.compute(offset_norm=-1.0, slope_term=0.0, valid=True)
    assert az <= 0.3 + 1e-6


def test_rate_constraint_respected_on_first_step():
    # With u_prev=0, the first command must not exceed delta_angular_z_max.
    du_max = 0.05
    ctrl = _make(angular_z_max=1.0, delta_angular_z_max=du_max, q_offset=1000.0)
    _, az = ctrl.compute(offset_norm=-1.0, slope_term=0.0, valid=True)
    assert abs(az) <= du_max + 1e-5, f"|az|={abs(az):.4f} exceeded du_max={du_max}"


def test_rate_constraint_respected_on_second_step():
    du_max = 0.05
    ctrl = _make(angular_z_max=1.0, delta_angular_z_max=du_max, q_offset=1000.0)
    _, az1 = ctrl.compute(offset_norm=-1.0, slope_term=0.0, valid=True)
    _, az2 = ctrl.compute(offset_norm=-1.0, slope_term=0.0, valid=True)
    assert abs(az2 - az1) <= du_max + 1e-5, f"delta={abs(az2-az1):.4f} exceeded {du_max}"


def test_high_r_delta_produces_smoother_first_step_than_low_r_delta():
    # A high smoothness weight should produce a smaller first-step command
    # from rest compared to a controller that ignores rate.
    ctrl_smooth = _make(angular_z_max=5.0, delta_angular_z_max=5.0,
                        r_delta=10.0, r_control=0.0)
    ctrl_agile  = _make(angular_z_max=5.0, delta_angular_z_max=5.0,
                        r_delta=0.0,  r_control=0.0)
    _, az_smooth = ctrl_smooth.compute(offset_norm=-1.0, slope_term=0.0, valid=True)
    _, az_agile  = ctrl_agile.compute(offset_norm=-1.0, slope_term=0.0, valid=True)
    assert abs(az_smooth) <= abs(az_agile) + 1e-5, (
        f"Smooth controller ({az_smooth:.3f}) should not exceed agile ({az_agile:.3f})"
    )


# ---------------------------------------------------------------------------
# Predictive / horizon behavior
# ---------------------------------------------------------------------------

def test_heading_error_drives_anticipatory_command_without_lateral_offset():
    # Even when the robot is currently centered (offset=0), a heading error
    # (slope!=0) should produce a non-zero command because the MPC predicts
    # that the heading error will cause future lateral drift.
    ctrl = _make(q_offset=10.0, q_heading=1.0, alpha=0.2)
    _, az_no_heading    = ctrl.compute(offset_norm=0.0, slope_term=0.0, valid=True)
    ctrl.reset()
    _, az_with_heading  = ctrl.compute(offset_norm=0.0, slope_term=0.3, valid=True)
    assert abs(az_with_heading) > abs(az_no_heading) + 1e-5, (
        "MPC should produce a larger command when heading error predicts future drift"
    )


# ---------------------------------------------------------------------------
# Invalid-frame safety state machine (semantics identical to old P-controller)
# ---------------------------------------------------------------------------

def test_invalid_frames_hold_last_command_before_stop_threshold():
    ctrl = MPCRowController(
        N=4, linear_x_cruise=0.15, angular_z_max=10.0,
        delta_angular_z_max=10.0, invalid_frame_stop_count=5,
    )
    lx, az = ctrl.compute(offset_norm=-0.5, slope_term=0.0, valid=True)
    assert lx == pytest.approx(0.15)
    assert az > 0
    held_lx, held_az = lx, az
    for _ in range(4):     # 4 invalid frames < stop_count=5
        lx2, az2 = ctrl.compute(offset_norm=999.0, slope_term=0.0, valid=False)
        assert lx2 == pytest.approx(held_lx)
        assert az2 == pytest.approx(held_az)


def test_invalid_frames_stop_at_threshold():
    ctrl = MPCRowController(
        N=4, angular_z_max=10.0, delta_angular_z_max=10.0,
        invalid_frame_stop_count=3,
    )
    ctrl.compute(offset_norm=-0.5, slope_term=0.0, valid=True)
    ctrl.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    ctrl.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    lx, az = ctrl.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    assert (lx, az) == (0.0, 0.0)
    assert ctrl.consecutive_invalid == 3


def test_valid_frame_after_invalid_resets_invalid_counter():
    ctrl = MPCRowController(N=4, angular_z_max=10.0, delta_angular_z_max=10.0,
                            invalid_frame_stop_count=3)
    ctrl.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    ctrl.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    assert ctrl.consecutive_invalid == 2
    ctrl.compute(offset_norm=0.0, slope_term=0.0, valid=True)
    assert ctrl.consecutive_invalid == 0


def test_reset_clears_all_state():
    ctrl = MPCRowController(N=4, angular_z_max=10.0, delta_angular_z_max=10.0,
                            invalid_frame_stop_count=2)
    ctrl.compute(offset_norm=0.5, slope_term=0.0, valid=True)
    ctrl.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    assert ctrl.consecutive_invalid == 1
    ctrl.reset()
    assert ctrl.consecutive_invalid == 0
    lx, az = ctrl.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    assert (lx, az) == (0.0, 0.0)   # reset zeroed u_prev, so hold = (0, 0)


def test_first_frame_invalid_with_no_history_returns_zero():
    ctrl = MPCRowController(N=4, angular_z_max=10.0, delta_angular_z_max=10.0,
                            invalid_frame_stop_count=5)
    lx, az = ctrl.compute(offset_norm=0.0, slope_term=0.0, valid=False)
    assert (lx, az) == (0.0, 0.0)
