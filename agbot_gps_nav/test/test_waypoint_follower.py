"""Tests for the go-to-goal waypoint follower.

Two kinds of test here, and both matter:

  * unit tests pinning one decision each (sign, clamp, state edge), and
  * `drive()`, a closed-loop kinematic simulation. A go-to-goal controller can
    pass every unit test and still orbit its goal forever, so the convergence
    tests actually integrate the unicycle model and check that the robot gets
    there.
"""

import math

import pytest

from agbot_gps_nav.waypoint_follower import (
    STATE_APPROACH, STATE_ARRIVED, STATE_GOTO, STATE_IDLE, WaypointFollower,
)


def drive(follower, goal, start=(0.0, 0.0, 0.0), dt=0.1, max_steps=6000):
    """Closed-loop unicycle simulation. Returns (arrived, pose, steps).

    x' = v cos(yaw), y' = v sin(yaw), yaw' = w -- the same model the Jackal
    approximates. Deliberately simple: no slip, no delay, no noise. It answers
    "does this control law converge", not "how does the robot behave".
    """
    follower.set_goal(goal)
    x, y, yaw = start
    for step in range(max_steps):
        linear_x, angular_z, _state, done = follower.update((x, y, yaw))
        if done:
            return True, (x, y, yaw), step
        yaw += angular_z * dt
        x += linear_x * math.cos(yaw) * dt
        y += linear_x * math.sin(yaw) * dt
    return False, (x, y, yaw), max_steps


# ---- goal handling -------------------------------------------------------

def test_starts_idle_and_still():
    f = WaypointFollower()
    assert f.update((0.0, 0.0, 0.0)) == (0.0, 0.0, STATE_IDLE, False)


def test_no_goal_means_no_motion_however_it_is_asked():
    f = WaypointFollower()
    for pose in [(0.0, 0.0, 0.0), (10.0, -3.0, 2.0), None]:
        linear_x, angular_z, state, done = f.update(pose)
        assert (linear_x, angular_z, state, done) == (0.0, 0.0, STATE_IDLE, False)


def test_setting_a_goal_enters_goto():
    f = WaypointFollower()
    f.set_goal((10.0, 0.0))
    assert f.state == STATE_GOTO
    assert f.goal == (10.0, 0.0)


def test_clear_goal_stops_and_returns_to_idle():
    f = WaypointFollower()
    f.set_goal((10.0, 0.0))
    f.update((0.0, 0.0, 0.0))
    f.clear_goal()
    assert f.goal is None
    assert f.update((0.0, 0.0, 0.0)) == (0.0, 0.0, STATE_IDLE, False)


# ---- sign convention -----------------------------------------------------

def test_goal_to_the_left_turns_left():
    """REP-103: positive angular.z is a LEFT turn. A sign error here sends the
    robot away from every goal and nothing downstream would catch it."""
    f = WaypointFollower()
    f.set_goal((10.0, 1.0))               # ahead and slightly left
    _lx, angular_z, _s, _d = f.update((0.0, 0.0, 0.0))
    assert angular_z > 0


def test_goal_to_the_right_turns_right():
    f = WaypointFollower()
    f.set_goal((10.0, -1.0))
    _lx, angular_z, _s, _d = f.update((0.0, 0.0, 0.0))
    assert angular_z < 0


def test_goal_dead_ahead_does_not_steer():
    f = WaypointFollower()
    f.set_goal((10.0, 0.0))
    _lx, angular_z, _s, _d = f.update((0.0, 0.0, 0.0))
    assert angular_z == pytest.approx(0.0, abs=1e-12)


def test_heading_error_takes_the_short_way_round():
    """The branch cut at +/-180 deg. Robot faces +179 deg; the goal bears
    -178 deg. Those are 3 deg apart ACROSS the cut, and the short way is a
    small LEFT turn (179 -> 180 -> -178). Without angle wrapping the error
    reads as -357 deg and the robot spins almost all the way round the other
    way to arrive at the same heading."""
    f = WaypointFollower()
    f.set_goal((-10.0, -0.35))            # bearing approx -178 deg
    _lx, angular_z, _s, _d = f.update((0.0, 0.0, math.radians(179)))
    assert f.heading_error() == pytest.approx(math.radians(3.0), abs=math.radians(0.5))
    assert 0 < angular_z                   # left, the short way across the cut
    assert abs(angular_z) < 0.2            # and small, because the error is small


# ---- turn in place -------------------------------------------------------

def test_badly_misaligned_turns_in_place_without_driving():
    """Driving forward at 90 deg off covers ground in the wrong direction, and
    close to the goal it produces an orbit the robot can never close."""
    f = WaypointFollower(turn_in_place_deg=30.0)
    f.set_goal((0.0, 10.0))               # 90 deg to the left
    linear_x, angular_z, _s, _d = f.update((0.0, 0.0, 0.0))
    assert linear_x == 0.0
    assert angular_z > 0


def test_goal_directly_behind_turns_rather_than_reversing():
    f = WaypointFollower()
    f.set_goal((-10.0, 0.0))
    linear_x, angular_z, _s, _d = f.update((0.0, 0.0, 0.0))
    assert linear_x == 0.0
    assert angular_z != 0.0


def test_slightly_misaligned_drives_and_steers_together():
    f = WaypointFollower(turn_in_place_deg=30.0)
    f.set_goal((10.0, 1.0))               # about 6 deg off
    linear_x, angular_z, _s, _d = f.update((0.0, 0.0, 0.0))
    assert linear_x > 0
    assert angular_z > 0


@pytest.mark.parametrize("yaw_deg", [0, 45, 90, 135, 180, -45, -90, -135])
def test_never_commands_reverse(yaw_deg):
    """There is no reverse gear in this controller. A negative linear_x would
    mean the derating arithmetic went past zero."""
    f = WaypointFollower()
    f.set_goal((10.0, 5.0))
    linear_x, _az, _s, _d = f.update((0.0, 0.0, math.radians(yaw_deg)))
    assert linear_x >= 0.0


# ---- clamps and derating -------------------------------------------------

def test_angular_z_is_clamped():
    f = WaypointFollower(angular_z_max=0.2, heading_gain=10.0, turn_in_place_deg=89.0)
    f.set_goal((1.0, 1.0))                # 45 deg: inside turn-in-place, steered
    _lx, angular_z, _s, _d = f.update((0.0, 0.0, 0.0))
    assert angular_z == pytest.approx(0.2)


def test_bigger_heading_error_means_slower_forward_speed():
    """kappa = angular_z / linear_x is what determines the path. Cruising at
    full speed through a correction is the mistake HANDOFF3 0f documents."""
    f = WaypointFollower(turn_in_place_deg=45.0)
    f.set_goal((100.0, 0.0))
    straight, _az, _s, _d = f.update((0.0, 0.0, 0.0))
    f.set_goal((100.0, 0.0))
    angled, _az, _s, _d = f.update((0.0, 0.0, math.radians(30)))
    assert 0.0 < angled < straight


# ---- approach and arrival ------------------------------------------------

def test_enters_approach_near_the_goal_and_slows_down():
    f = WaypointFollower(approach_distance=2.0, linear_x_cruise=0.4, approach_speed=0.15)
    f.set_goal((10.0, 0.0))
    far, _az, state_far, _d = f.update((0.0, 0.0, 0.0))
    assert state_far == STATE_GOTO

    near, _az, state_near, _d = f.update((8.5, 0.0, 0.0))   # 1.5 m out
    assert state_near == STATE_APPROACH
    assert near < far


def test_arrives_inside_tolerance():
    f = WaypointFollower(goal_tolerance=0.3)
    f.set_goal((10.0, 0.0))
    linear_x, angular_z, state, done = f.update((9.8, 0.0, 0.0))
    assert (linear_x, angular_z, state, done) == (0.0, 0.0, STATE_ARRIVED, True)


def test_arrived_is_latched():
    """Without the latch, an estimate wobbling across the tolerance boundary
    restarts the drive and the robot creeps around the goal indefinitely."""
    f = WaypointFollower(goal_tolerance=0.3)
    f.set_goal((10.0, 0.0))
    f.update((9.8, 0.0, 0.0))
    for pose in [(9.0, 0.0, 0.0), (5.0, 5.0, 1.0), (0.0, 0.0, 0.0)]:
        linear_x, angular_z, state, done = f.update(pose)
        assert (linear_x, angular_z, state, done) == (0.0, 0.0, STATE_ARRIVED, True)


def test_a_new_goal_releases_the_arrived_latch():
    f = WaypointFollower()
    f.set_goal((10.0, 0.0))
    f.update((9.8, 0.0, 0.0))
    assert f.state == STATE_ARRIVED
    f.set_goal((20.0, 0.0))
    assert f.state == STATE_GOTO
    linear_x, _az, state, done = f.update((10.0, 0.0, 0.0))
    assert state == STATE_GOTO and linear_x > 0 and not done


# ---- missing pose --------------------------------------------------------

def test_missing_pose_stops_but_keeps_the_goal():
    """Losing a pose estimate is not the same as losing the mission. Stop, but
    stay in GOTO so the drive resumes when the estimate returns."""
    f = WaypointFollower()
    f.set_goal((10.0, 0.0))
    f.update((0.0, 0.0, 0.0))
    linear_x, angular_z, state, done = f.update(None)
    assert (linear_x, angular_z, done) == (0.0, 0.0, False)
    assert state == STATE_GOTO
    assert f.goal == (10.0, 0.0)


def test_missing_pose_never_declares_arrival():
    f = WaypointFollower()
    f.set_goal((10.0, 0.0))
    for _ in range(100):
        assert f.update(None)[3] is False


# ---- closed loop ---------------------------------------------------------

@pytest.mark.parametrize("goal", [
    (20.0, 0.0), (0.0, 20.0), (-15.0, 0.0), (0.0, -15.0),
    (-10.0, -10.0), (12.0, -7.0), (3.0, 0.0),
])
def test_converges_to_goals_in_every_direction(goal):
    f = WaypointFollower()
    arrived, pose, _steps = drive(f, goal)
    assert arrived, "did not reach %s, stalled at %s" % (goal, pose)
    assert math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= f.goal_tolerance + 1e-6


@pytest.mark.parametrize("yaw_deg", [0, 90, 180, -90, 45, -135])
def test_converges_from_any_starting_heading(yaw_deg):
    f = WaypointFollower()
    arrived, _pose, _steps = drive(f, (15.0, 5.0), start=(0.0, 0.0, math.radians(yaw_deg)))
    assert arrived


def test_does_not_orbit_a_close_off_axis_goal():
    """The classic go-to-goal failure: a goal just off to the side, close
    enough that the turning circle cannot enclose it. Turning in place when
    badly misaligned is what prevents it -- this test is that guard's reason
    for existing."""
    f = WaypointFollower()
    arrived, _pose, steps = drive(f, (0.5, 1.5))
    assert arrived
    assert steps < 600         # tens of seconds, not an endless circle


def test_reaching_the_goal_takes_a_sane_amount_of_time():
    """20 m at a 0.4 m/s cruise cannot be under 50 s; anything far over ~90 s
    means the derating is fighting itself."""
    f = WaypointFollower()
    arrived, _pose, steps = drive(f, (20.0, 0.0))
    assert arrived
    assert 500 <= steps <= 900        # dt = 0.1 s


# ---- approach speed is a FLOOR, not a second multiplier ------------------

def test_speed_never_drops_below_approach_speed_while_still_driving():
    """The regression this exists for: approach_speed used to be selected
    inside approach_distance and THEN multiplied by a distance derate flooring
    at 0.25, so the last metre ran at 3.75 cm/s. A 2.5 m goal took 25-32 s and
    the first operator to try it concluded the robot had stalled (2026-09-06).
    """
    f = WaypointFollower(linear_x_cruise=0.4, approach_speed=0.15,
                         approach_distance=2.0, goal_tolerance=0.3)
    arrived, _pose, _steps = drive(f, (2.5, 0.0))
    assert arrived

    # Sample the commanded speed straight down the approach, dead ahead so the
    # heading derate is exactly 1.0 and only the distance ramp is under test.
    for x in (0.6, 1.0, 1.5, 1.9, 2.19):        # 1.9 m ... 0.31 m to go
        f.set_goal((2.5, 0.0))
        linear_x, _az, _s, done = f.update((x, 0.0, 0.0))
        assert not done
        assert linear_x >= f.approach_speed - 1e-9, (
            "%.3f m from the goal the command was %.4f m/s, under the %.2f m/s floor"
            % (2.5 - x, linear_x, f.approach_speed))


def test_speed_ramps_down_to_approach_speed_at_the_goal():
    """At the goal the ramp should equal approach_speed exactly -- that is what
    makes it a floor rather than something further derated."""
    # Tolerance far below the sample point, or the tick reports ARRIVED (0.0)
    # instead of the speed we are trying to read.
    f = WaypointFollower(linear_x_cruise=0.4, approach_speed=0.15, approach_distance=2.0,
                         goal_tolerance=1e-9)
    f.set_goal((10.0, 0.0))
    just_short, _az, _s, done = f.update((10.0 - 1e-4, 0.0, 0.0))
    assert not done
    assert just_short == pytest.approx(0.15, abs=1e-3)

    f.set_goal((10.0, 0.0))
    half_way_in, _az, _s, _d = f.update((9.0, 0.0, 0.0))   # 1.0 m of a 2.0 m ramp
    assert half_way_in == pytest.approx(0.15 + (0.4 - 0.15) * 0.5, abs=1e-3)


def test_a_short_goal_does_not_take_longer_than_the_transit_to_it():
    """2.5 m used to cost 25 s. The operator-visible symptom was a robot that
    looked stalled, so the bound here is on TIME, which is what they saw."""
    f = WaypointFollower()
    arrived, _pose, steps = drive(f, (2.5, 0.0))
    assert arrived
    assert steps * 0.1 < 12.0, "2.5 m took %.1f s" % (steps * 0.1)


def test_approach_speed_above_cruise_is_clamped_not_inverted():
    """A misconfigured approach_speed > cruise would otherwise invert the ramp
    and make the robot accelerate into its goal."""
    f = WaypointFollower(linear_x_cruise=0.3, approach_speed=0.9)
    assert f.approach_speed == pytest.approx(0.3)
    f.set_goal((10.0, 0.0))
    far, _az, _s, _d = f.update((0.0, 0.0, 0.0))
    near, _az, _s, _d = f.update((9.0, 0.0, 0.0))
    assert near <= far + 1e-9
