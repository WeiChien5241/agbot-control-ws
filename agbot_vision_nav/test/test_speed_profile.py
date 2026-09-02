"""Tests for speed_profile: what scales with linear_x_cruise, and what must not.

The failure these pin is the 2026-09-02 sim run -- linear_x_cruise raised
0.15 -> 0.5 with nothing else touched, angular_z saturated at the unchanged
clamp, and the robot drove over every plant. The contract below is what stops
that being repeatable.
"""

import pytest

from agbot_vision_nav.speed_profile import (
    REFERENCE_ANGULAR_Z_MAX,
    REFERENCE_DELTA_ANGULAR_Z_MAX,
    REFERENCE_MPC_ALPHA,
    REFERENCE_SPEED,
    curvature_limit,
    format_launch_args,
    scaled_params,
)


def test_reference_speed_returns_the_params_yaml_values_unchanged():
    """Asking for the tuned speed must be a no-op, or the helper is a re-tune."""
    params = scaled_params(REFERENCE_SPEED)
    assert params["linear_x_cruise"] == pytest.approx(REFERENCE_SPEED)
    assert params["angular_z_max"] == pytest.approx(REFERENCE_ANGULAR_Z_MAX)
    assert params["delta_angular_z_max"] == pytest.approx(
        REFERENCE_DELTA_ANGULAR_Z_MAX
    )
    assert params["mpc_alpha"] == pytest.approx(REFERENCE_MPC_ALPHA)


@pytest.mark.parametrize("speed", [0.05, 0.15, 0.3, 0.5, 0.9, 1.5])
def test_max_path_curvature_is_invariant_across_speeds(speed):
    """THE contract. kappa = angular_z_max / linear_x_cruise is what keeps the
    robot inside a 0.75 m row; holding it fixed is the entire point.

    At the reference it is 1.167 /m (a 0.86 m tightest radius). The 2026-09-02
    run left angular_z_max at 0.175 while tripling the speed, which dropped it
    to 0.35 /m -- a 2.86 m radius, 3.3x less turning per metre driven.
    """
    reference = curvature_limit(scaled_params(REFERENCE_SPEED))
    assert curvature_limit(scaled_params(speed)) == pytest.approx(reference)


def test_the_unscaled_2026_09_02_envelope_is_what_the_helper_rejects():
    """Regression witness: the numbers from the run that failed."""
    failed = {"linear_x_cruise": 0.5, "angular_z_max": REFERENCE_ANGULAR_Z_MAX}
    assert curvature_limit(failed) == pytest.approx(0.35, abs=0.01)
    assert curvature_limit(scaled_params(0.5)) == pytest.approx(1.167, abs=0.01)


def test_mpc_beta_is_never_scaled():
    """theta_dot = angular_z is independent of forward speed. Scaling beta
    would tell the MPC that driving faster makes it turn harder."""
    for speed in (0.15, 0.5, 0.9):
        assert "mpc_beta" not in scaled_params(speed)


def test_distance_valued_detector_thresholds_are_never_scaled():
    """They are speed-invariant BY DESIGN -- moving debounce from frames to
    metres is what fixed the 2026-07-24 field failure. Touching them here
    would undo it."""
    params = scaled_params(0.9, control_period=0.05)
    for knob in (
        "exit_confirm_distance",
        "min_in_row_distance",
        "headland_clearance",
        "traverse_distance",
        "exit_clear_max_distance",
        "blocked_confirm_seconds",
        "max_data_age_sec",
    ):
        assert knob not in params


def test_mpc_dt_is_omitted_unless_a_control_period_is_measured():
    """mpc_dt is a property of the machine, not of the speed. Guessing one is
    worse than leaving params.yaml's value alone."""
    assert "mpc_dt" not in scaled_params(0.5)
    assert scaled_params(0.5, control_period=0.13)["mpc_dt"] == pytest.approx(0.13)


def test_mpc_dt_does_not_double_count_into_mpc_alpha():
    """controller.py already rescales alpha by dt/0.1 internally, so mpc_alpha
    must stay quoted at the 0.1 s reference regardless of the period given."""
    without = scaled_params(0.5)["mpc_alpha"]
    for period in (0.05, 0.13, 0.5):
        assert scaled_params(0.5, control_period=period)["mpc_alpha"] == pytest.approx(
            without
        )


def test_non_positive_inputs_are_rejected():
    for bad in (0.0, -0.3):
        with pytest.raises(ValueError):
            scaled_params(bad)
        with pytest.raises(ValueError):
            scaled_params(0.5, control_period=bad)


def test_launch_args_render_as_pasteable_roslaunch_pairs():
    rendered = format_launch_args(scaled_params(0.5, control_period=0.13))
    assert rendered == (
        "linear_x_cruise:=0.5 angular_z_max:=0.583 "
        "delta_angular_z_max:=0.667 mpc_alpha:=0.333 mpc_dt:=0.13"
    )
    # No float-repr noise: roslaunch would take 0.5830000000000001, but nobody
    # reading the startup config block should have to.
    for token in rendered.split():
        assert "0000" not in token


def test_every_emitted_key_is_a_real_vision_nav_launch_arg():
    """A knob that is not a launch <arg> is silently dropped by roslaunch, so
    the whole profile would appear to apply and do nothing."""
    import os
    import re

    launch = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "launch", "vision_nav.launch"
    )
    with open(launch) as handle:
        declared = set(re.findall(r'<arg\s+name="([^"]+)"', handle.read()))
    for key in scaled_params(0.5, control_period=0.13):
        assert key in declared, "%s is not an <arg> in vision_nav.launch" % key
