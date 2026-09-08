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

from agbot_gps_nav import geo
from agbot_gps_nav.waypoint_follower import (
    MIN_COURSE_DISTANCE, STATE_ALIGN, STATE_APPROACH, STATE_ARRIVED,
    STATE_FAILED, STATE_GOTO, STATE_HEADING_INIT, STATE_IDLE, WaypointFollower,
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


# ---- approach bearing: arrive pointing the right way ---------------------

def drive_on_axis(follower, goal, bearing, start, dt=0.05, max_steps=40000,
                  approach_distance=None):
    """Closed loop for a goal that carries an approach bearing.

    Returns (arrived, pose, seconds, state_sequence).
    """
    follower.set_goal(goal, approach_bearing=bearing,
                      approach_distance=approach_distance)
    x, y, yaw = start
    states = []
    for step in range(max_steps):
        linear_x, angular_z, state, done = follower.update((x, y, yaw))
        if not states or states[-1] != state:
            states.append(state)
        if done:
            return True, (x, y, yaw), step * dt, states
        yaw += angular_z * dt
        x += linear_x * math.cos(yaw) * dt
        y += linear_x * math.sin(yaw) * dt
    return False, (x, y, yaw), max_steps * dt, states


def test_a_bearing_inserts_a_staging_point_back_along_the_axis():
    f = WaypointFollower()
    f.set_goal((0.704, -3.40), approach_bearing=math.pi / 2, approach_distance=3.0)
    assert f.staging_point == pytest.approx((0.704, -6.40), abs=1e-6)
    assert f.approach_bearing == pytest.approx(math.pi / 2)


def test_no_bearing_means_no_staging_point():
    f = WaypointFollower()
    f.set_goal((10.0, 0.0))
    assert f.staging_point is None
    assert f.approach_bearing is None


@pytest.mark.parametrize("start", [
    (0.7, -12.0, math.pi / 2),      # already lined up, from far back
    (-6.0, -9.0, 0.0),              # off to one side
    (8.0, -8.0, math.pi),           # off to the other, facing away
    (0.7, -12.0, -math.pi / 2),     # lined up but facing backwards
    (-5.0, -14.0, 1.0),
    (6.0, -2.0, 3.0),               # starts PAST the goal
])
def test_arrives_on_the_requested_bearing_from_any_start(start):
    """The row-entrance case. Rows in the maize worlds run along +Y, so the
    bearing is +pi/2. Arriving sideways is useless: vision nav picks up in
    FOLLOW_ROW and needs the corridor already in view."""
    goal, bearing = (0.704, -3.40), math.pi / 2
    f = WaypointFollower()
    arrived, pose, _t, _states = drive_on_axis(f, goal, bearing, start)
    assert arrived
    error = geo.wrap_angle(pose[2] - bearing)
    assert abs(error) < math.radians(6), (
        "arrived %.1f deg off the requested bearing" % math.degrees(error))
    assert math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= f.goal_tolerance + 1e-6


def test_the_bearing_is_what_fixes_the_arrival_heading():
    """Contrast test. Without a bearing the robot arrives pointing however it
    happened to approach -- which for a goal reached from the side is nowhere
    near down the row."""
    goal, bearing = (0.704, -3.40), math.pi / 2
    start = (-8.0, -3.4, 0.0)                 # due west of the goal

    without = WaypointFollower()
    arrived, pose, _steps = drive(without, goal, start=start)
    assert arrived
    naive_error = abs(geo.wrap_angle(pose[2] - bearing))

    with_bearing = WaypointFollower()
    arrived, pose, _t, _s = drive_on_axis(with_bearing, goal, bearing, start)
    assert arrived
    guided_error = abs(geo.wrap_angle(pose[2] - bearing))

    assert naive_error > math.radians(60), "the contrast case is not a contrast"
    assert guided_error < math.radians(6)


def test_the_state_sequence_is_goto_align_approach_arrived():
    f = WaypointFollower()
    _arrived, _pose, _t, states = drive_on_axis(
        f, (0.704, -3.40), math.pi / 2, (-6.0, -9.0, 0.0))
    assert states == [STATE_GOTO, STATE_ALIGN, STATE_APPROACH, STATE_ARRIVED]


def test_align_turns_in_place_and_does_not_drive():
    """ALIGN exists because turn-in-place tolerates up to turn_in_place_deg
    (30) before it fires, and starting the final leg 30 degrees off would leave
    residual heading error at the goal."""
    f = WaypointFollower(align_tolerance_deg=5.0)
    f.set_goal((0.0, 0.0), approach_bearing=math.pi / 2, approach_distance=3.0)
    # sitting exactly on the staging point, facing east instead of north
    linear_x, angular_z, state, done = f.update((0.0, -3.0, 0.0))
    assert state == STATE_ALIGN
    assert linear_x == 0.0
    assert angular_z > 0                       # turn left, toward +pi/2
    assert not done


def test_align_releases_once_inside_its_tolerance():
    f = WaypointFollower(align_tolerance_deg=5.0)
    f.set_goal((0.0, 0.0), approach_bearing=math.pi / 2, approach_distance=3.0)
    _lx, _az, state, _d = f.update((0.0, -3.0, math.radians(88)))
    assert state == STATE_APPROACH


def test_a_new_plain_goal_clears_a_previous_bearing():
    """Otherwise a stale staging point silently steers the next goal."""
    f = WaypointFollower()
    f.set_goal((5.0, 5.0), approach_bearing=0.0)
    assert f.staging_point is not None
    f.set_goal((10.0, 0.0))
    assert f.staging_point is None and f.approach_bearing is None


def test_clear_goal_drops_the_bearing_too():
    f = WaypointFollower()
    f.set_goal((5.0, 5.0), approach_bearing=0.0)
    f.clear_goal()
    assert f.staging_point is None and f.approach_bearing is None


def test_the_final_leg_tracks_the_axis_not_the_goal_point():
    """⚠ The regression this exists for. Steering at the goal POINT corrects any
    lateral offset in the last metre, so the robot swings as it arrives: a leg
    aimed at the point arrived 41 deg off a bearing it had aligned to correctly
    moments earlier (sim, 2026-09-07). Tracking the LINE makes the commanded
    heading equal the bearing once the robot is on it, so the error decays to
    zero along with the offset. Fixing it took 41.2 deg to 0.4 deg."""
    f = WaypointFollower()
    goal, bearing = (0.0, 0.0), math.pi / 2

    # On the axis, short of the goal: steer straight along the bearing.
    f.set_goal(goal, approach_bearing=bearing, approach_distance=3.0)
    f.state = STATE_APPROACH
    _lx, angular_z, _s, _d = f._drive_along_axis((0.0, -1.0, bearing))
    assert angular_z == pytest.approx(0.0, abs=1e-9)

    # Left of the axis: turn RIGHT to get back on it.
    _lx, angular_z, _s, _d = f._drive_along_axis((-0.5, -1.0, bearing))
    assert angular_z < 0

    # Right of the axis: turn LEFT.
    _lx, angular_z, _s, _d = f._drive_along_axis((0.5, -1.0, bearing))
    assert angular_z > 0


def test_axis_correction_saturates():
    """Far off the line the approach must not turn perpendicular to the axis
    and drive across the row."""
    f = WaypointFollower(max_axis_correction_deg=45.0, turn_in_place_deg=90.0)
    f.set_goal((0.0, 0.0), approach_bearing=math.pi / 2, approach_distance=3.0)
    f.state = STATE_APPROACH
    f._drive_along_axis((50.0, -1.0, math.pi / 2))
    assert abs(f.heading_error()) <= math.radians(45) + 1e-9


def test_cross_track_converges_over_the_final_leg():
    """Closed loop: start the approach deliberately off the line and check the
    robot is ON it, and pointing along it, by the time it arrives."""
    goal, bearing = (0.0, 0.0), math.pi / 2
    f = WaypointFollower()
    # start beside the staging point, already roughly aligned
    arrived, pose, _t, _states = drive_on_axis(
        f, goal, bearing, (0.8, -3.0, bearing), approach_distance=3.0)
    assert arrived
    cross = -(pose[0] - goal[0]) * math.sin(bearing) + (pose[1] - goal[1]) * math.cos(bearing)
    assert abs(cross) < 0.35
    assert abs(geo.wrap_angle(pose[2] - bearing)) < math.radians(6)


# ---- receding-goal abort -------------------------------------------------

def test_aborts_when_the_goal_is_receding():
    """⚠ A robot driving AWAY from its goal is broken, and nothing else here
    notices. The geofence is measured from the datum, so a 6.5 m goal missed by
    100 m sits well inside it. Observed in sim 2026-09-07: a bad heading
    estimate sent the robot backwards off the edge of the world while the
    follower reported a calmly growing distance the whole way."""
    f = WaypointFollower(max_goal_distance_growth=5.0)
    f.set_goal((0.0, 0.0))
    # Drive steadily away along +x.
    for x in (1.0, 2.0, 3.0, 4.0, 5.0):
        _lx, _az, state, _d = f.update((x, 0.0, 0.0))
        assert state != STATE_FAILED, "aborted too early, at %.0f m" % x
    _lx, angular_z, state, done = f.update((6.5, 0.0, 0.0))
    assert state == STATE_FAILED
    assert (_lx, angular_z, done) == (0.0, 0.0, False)


def test_abort_is_measured_from_the_closest_approach_not_the_start():
    """Getting within 2 m and then wandering 8 m away is a failure too, and
    comparing against the STARTING distance would excuse it."""
    f = WaypointFollower(max_goal_distance_growth=5.0)
    f.set_goal((0.0, 0.0))
    f.update((40.0, 0.0, 0.0))          # start far out
    f.update((2.0, 0.0, 0.0))           # get close
    _lx, _az, state, _d = f.update((6.0, 0.0, 0.0))
    assert state != STATE_FAILED
    _lx, _az, state, _d = f.update((8.0, 0.0, 0.0))
    assert state == STATE_FAILED


def test_failed_is_latched_and_silent():
    f = WaypointFollower(max_goal_distance_growth=1.0)
    f.set_goal((0.0, 0.0))
    f.update((1.0, 0.0, 0.0))
    f.update((5.0, 0.0, 0.0))
    assert f.state == STATE_FAILED
    for pose in [(0.1, 0.0, 0.0), (0.0, 0.0, 0.0)]:
        linear_x, angular_z, state, done = f.update(pose)
        assert (linear_x, angular_z, state, done) == (0.0, 0.0, STATE_FAILED, False)


def test_a_new_goal_clears_a_failure():
    f = WaypointFollower(max_goal_distance_growth=1.0)
    f.set_goal((0.0, 0.0))
    f.update((1.0, 0.0, 0.0)); f.update((5.0, 0.0, 0.0))
    assert f.state == STATE_FAILED
    f.set_goal((10.0, 0.0))
    assert f.state == STATE_GOTO
    linear_x, _az, _s, _d = f.update((5.0, 0.0, 0.0))
    assert linear_x > 0


def test_normal_approaches_do_not_trip_the_abort():
    """The guard must not fire on the ordinary case where a turn-in-place or a
    curved approach momentarily increases the distance."""
    for goal in [(20.0, 0.0), (-15.0, 0.0), (0.5, 1.5), (12.0, -7.0)]:
        f = WaypointFollower()
        arrived, _pose, _steps = drive(f, goal)
        assert arrived and f.state == STATE_ARRIVED, "spurious abort for %s" % (goal,)


# ---- heading bootstrap ---------------------------------------------------

def test_heading_init_is_off_by_default():
    """Every existing behaviour and test depends on a goal starting in GOTO."""
    f = WaypointFollower()
    assert f.heading_init_distance == 0.0
    f.set_goal((10.0, 0.0))
    assert f.state == STATE_GOTO


def test_heading_init_drives_straight_without_steering():
    """⚠ It deliberately does not steer. Steering on the heading estimate is
    the circularity being broken -- the point is to give the EKF motion so it
    can fix that estimate from GPS displacement."""
    f = WaypointFollower(heading_init_distance=4.0)
    f.set_goal((0.0, 20.0))                  # goal is 90 deg to the left
    linear_x, angular_z, state, _d = f.update((0.0, 0.0, 0.0))
    assert state == STATE_HEADING_INIT
    assert linear_x > 0
    assert angular_z == 0.0, "must not steer during the bootstrap"


def test_heading_init_ends_once_the_yaw_agrees_with_the_course_driven():
    """It ends on a MEASUREMENT, not a distance: whether the estimate has
    converged depends on how wrong it started, which nothing knows up front."""
    f = WaypointFollower(heading_init_distance=4.0, heading_init_tolerance_deg=12.0)
    f.set_goal((20.0, 0.0))
    _lx, _az, state, _d = f.update((0.0, 0.0, 0.0))
    assert state == STATE_HEADING_INIT
    # Short of the minimum, it keeps going whatever the heading says.
    _lx, _az, state, _d = f.update((3.9, 0.0, 0.0))
    assert state == STATE_HEADING_INIT
    # Past the minimum, and the yaw agrees with the course actually driven.
    _lx, _az, state, _d = f.update((4.1, 0.0, 0.0))
    assert state == STATE_GOTO
    assert abs(f.heading_init_error()) < math.radians(1)


def test_heading_init_keeps_driving_while_the_yaw_still_disagrees():
    """⚠ The failure this replaced: a fixed 4 m ended the bootstrap with the
    estimate still 90 deg wrong, and the robot then entered the final approach
    1.0 m off the axis and drove past the row entrance (sim, 2026-09-07)."""
    f = WaypointFollower(heading_init_distance=4.0, heading_init_max_distance=20.0,
                         heading_init_tolerance_deg=12.0)
    f.set_goal((30.0, 0.0))
    # Robot really travels along +x, but believes it is pointing 90 deg off.
    for x in (0.0, 2.0, 4.1, 6.0, 8.0):
        _lx, _az, state, _d = f.update((x, 0.0, math.pi / 2))
        assert state == STATE_HEADING_INIT, "gave up at x=%.1f with yaw 90 deg wrong" % x
    assert abs(f.heading_init_error()) == pytest.approx(math.pi / 2, abs=1e-6)
    # Once the estimate agrees with the course driven, it releases.
    _lx, _az, state, _d = f.update((10.0, 0.0, 0.0))
    assert state == STATE_GOTO


def test_heading_init_gives_up_at_its_max_distance():
    """The backstop. Must fit the open ground ahead -- this drives STRAIGHT."""
    f = WaypointFollower(heading_init_distance=2.0, heading_init_max_distance=6.0,
                         heading_init_tolerance_deg=5.0)
    f.set_goal((30.0, 0.0))
    for x in (0.0, 3.0, 5.9):
        _lx, _az, state, _d = f.update((x, 0.0, math.pi / 2))
        assert state == STATE_HEADING_INIT
    _lx, _az, state, _d = f.update((6.1, 0.0, math.pi / 2))
    assert state == STATE_GOTO, "must not drive straight for ever"


def test_heading_init_drives_at_cruise_not_approach_speed():
    """⚠ The bootstrap is an OBSERVABILITY manoeuvre, and the signal it feeds
    the EKF -- GPS displacement disagreeing with what the yaw predicted --
    grows with ground speed. Run at approach_speed it is both the weakest
    signal available and the longest wait: measured 2026-09-07, the 10 m
    backstop took 67 s at 0.15 m/s and came out 71.9 deg wrong, having laid
    down ~19 deg of fresh gyro drift while it ran."""
    f = WaypointFollower(linear_x_cruise=0.4, approach_speed=0.15,
                         heading_init_distance=4.0)
    f.set_goal((20.0, 0.0))
    linear_x, _az, state, _d = f.update((0.0, 0.0, 0.0))
    assert state == STATE_HEADING_INIT
    assert linear_x == pytest.approx(0.4)


def test_heading_init_converged_reports_which_way_it_ended():
    """⚠ Ending on the MEASUREMENT and ending on the BACKSTOP are opposite
    outcomes. They used to share one loginfo saying "done", which is how a
    transit ran its whole length 72 deg wrong with nothing flagged."""
    good = WaypointFollower(heading_init_distance=4.0,
                            heading_init_tolerance_deg=12.0)
    assert good.heading_init_converged() is None, "None until it has run"
    good.set_goal((20.0, 0.0))
    good.update((0.0, 0.0, 0.0))
    _lx, _az, state, _d = good.update((4.1, 0.0, 0.0))
    assert state == STATE_GOTO
    assert good.heading_init_converged() is True

    gave_up = WaypointFollower(heading_init_distance=2.0,
                               heading_init_max_distance=6.0,
                               heading_init_tolerance_deg=5.0)
    gave_up.set_goal((30.0, 0.0))
    for x in (0.0, 3.0, 5.9, 6.1):
        gave_up.update((x, 0.0, math.pi / 2))
    assert gave_up.heading_init_converged() is False


def test_a_new_goal_clears_the_previous_bootstrap_verdict():
    f = WaypointFollower(heading_init_distance=4.0)
    f.set_goal((20.0, 0.0))
    f.update((0.0, 0.0, 0.0))
    f.update((4.1, 0.0, 0.0))
    assert f.heading_init_converged() is True
    f.set_goal((40.0, 0.0))
    assert f.heading_init_converged() is None


def test_heading_init_does_not_trip_the_receding_goal_abort():
    """The bootstrap legitimately drives AWAY from the goal, and the abort must
    not count that."""
    f = WaypointFollower(heading_init_distance=4.0, max_goal_distance_growth=2.0)
    f.set_goal((-20.0, 0.0))                 # goal is behind: the bootstrap
    for x in (0.0, 1.0, 2.0, 3.0, 4.5):      # drives directly away from it
        _lx, _az, state, _d = f.update((x, 0.0, 0.0))
        assert state != STATE_FAILED
    assert state == STATE_GOTO


def test_still_converges_with_the_bootstrap_enabled():
    f = WaypointFollower(heading_init_distance=4.0)
    arrived, pose, _steps = drive(f, (20.0, 8.0))
    assert arrived
    assert math.hypot(pose[0] - 20.0, pose[1] - 8.0) <= f.goal_tolerance + 1e-6


# ---- arrival on a directed approach is crossing the goal PLANE -----------

def test_arrival_is_crossing_the_plane_not_entering_a_radius():
    """⚠ The regression. A Euclidean radius fails in a way that looks like
    nothing at all: the robot passed 0.46 m to the side of a row entrance, so
    the distance bottomed out at 0.47 m, never reached the 0.30 m tolerance,
    and the follower drove calmly on up the field (sim, 2026-09-07). Crossing
    the plane is a thing that definitely happens; entering a radius is not."""
    f = WaypointFollower(goal_tolerance=0.3, arrival_cross_tolerance=0.35)
    f.set_goal((0.0, 0.0), approach_bearing=math.pi / 2, approach_distance=3.0)
    f.state = STATE_APPROACH
    # 0.3 m off the axis and level with the goal: Euclidean distance is 0.30,
    # right on the boundary, but the plane has been crossed and the offset is
    # inside the corridor.
    _lx, _az, state, done = f.update((0.3, 0.0, math.pi / 2))
    assert state == STATE_ARRIVED and done


def test_crossing_the_plane_too_far_off_axis_restages_instead_of_driving_on():
    f = WaypointFollower(arrival_cross_tolerance=0.35, max_approach_attempts=3)
    f.set_goal((0.0, 0.0), approach_bearing=math.pi / 2, approach_distance=3.0)
    f.state = STATE_APPROACH
    linear_x, _az, state, _d = f.update((1.2, 0.0, math.pi / 2))   # 1.2 m off
    assert state == STATE_GOTO, "must go round again, not carry on up the field"
    assert linear_x == 0.0


def test_repeated_misses_fail_rather_than_looping_for_ever():
    f = WaypointFollower(arrival_cross_tolerance=0.35, max_approach_attempts=3)
    f.set_goal((0.0, 0.0), approach_bearing=math.pi / 2, approach_distance=3.0)
    for _ in range(2):
        f.state = STATE_APPROACH
        f.update((1.2, 0.0, math.pi / 2))
    f.state = STATE_APPROACH
    _lx, _az, state, _d = f.update((1.2, 0.0, math.pi / 2))
    assert state == STATE_FAILED


# ---- the arrival heading check -------------------------------------------

def test_arrival_course_error_is_near_zero_on_a_clean_approach():
    """The whole final leg is a free course-over-ground sample, and on a good
    run the direction driven and the yaw estimate agree."""
    f = WaypointFollower(staging_distance=5.0)
    arrived, _pose, _t, _states = drive_on_axis(
        f, goal=(0.0, 0.0), bearing=math.pi / 2, start=(-4.0, -12.0, 0.0),
        approach_distance=5.0)
    assert arrived
    assert f.approach_course_error() is not None
    assert abs(f.approach_course_error()) < math.radians(5)


def test_arrival_course_error_catches_a_yaw_estimate_that_is_lying():
    """⚠ THE 2026-09-07 FAILURE. GPS pins position, never orientation, so a
    robot with a bad yaw arrives at exactly the right PLACE facing somewhere
    else -- measured ~72 deg off, which handed vision nav a wall of corn.

    ALIGN cannot catch it: it turns until the ESTIMATE reads the bearing, so
    heading-error-vs-bearing is the estimate agreeing with itself. The course
    actually driven is the independent witness. Here the robot really travels
    up the axis while believing it points 70 deg away from that."""
    f = WaypointFollower(arrival_cross_tolerance=0.35)
    f.set_goal((0.0, 0.0), approach_bearing=math.pi / 2, approach_distance=3.0)
    f.state = STATE_APPROACH
    lie = math.pi / 2 - math.radians(70)
    f._approach_start_xy = (0.0, -5.0)          # leg start, 5 m back on axis
    _lx, _az, state, done = f.update((0.0, 0.0, lie))
    assert state == STATE_ARRIVED and done
    assert f.approach_course_error() == pytest.approx(math.radians(70), abs=1e-6)


def test_arrival_course_error_is_none_when_the_leg_is_too_short_to_measure():
    """A course computed over a few centimetres is GPS noise, not a direction.
    None means 'not measured' and the caller must say so rather than assume."""
    f = WaypointFollower(arrival_cross_tolerance=0.35)
    f.set_goal((0.0, 0.0), approach_bearing=math.pi / 2, approach_distance=3.0)
    f.state = STATE_APPROACH
    f._approach_start_xy = (0.0, -MIN_COURSE_DISTANCE / 2.0)
    _lx, _az, state, done = f.update((0.0, 0.0, math.pi / 2))
    assert state == STATE_ARRIVED and done
    assert f.approach_course_error() is None


def test_restaging_starts_a_fresh_heading_measurement():
    """Each attempt must be measured over its OWN leg; carrying the previous
    attempt's start point across a turn-around would measure a chord of the
    loop rather than the approach."""
    f = WaypointFollower(arrival_cross_tolerance=0.35, max_approach_attempts=3)
    f.set_goal((0.0, 0.0), approach_bearing=math.pi / 2, approach_distance=3.0)
    f.state = STATE_APPROACH
    f._approach_start_xy = (1.2, -5.0)
    _lx, _az, state, _d = f.update((1.2, 0.0, math.pi / 2))   # 1.2 m off axis
    assert state == STATE_GOTO
    assert f._approach_start_xy is None


def test_the_cross_tolerance_is_under_half_a_row_spacing():
    """Otherwise 'arrived' could mean 'arrived at the next corridor along'."""
    f = WaypointFollower()
    assert f.arrival_cross_tolerance < 0.75 / 2


def test_approaching_short_of_the_plane_keeps_driving():
    f = WaypointFollower()
    f.set_goal((0.0, 0.0), approach_bearing=math.pi / 2, approach_distance=3.0)
    f.state = STATE_APPROACH
    linear_x, _az, state, done = f.update((0.0, -1.0, math.pi / 2))
    assert state == STATE_APPROACH and not done and linear_x > 0


def test_on_axis_runs_still_converge_end_to_end():
    for start in [(0.7, -12.0, math.pi / 2), (-6.0, -9.0, 0.0), (8.0, -8.0, math.pi)]:
        f = WaypointFollower()
        arrived, pose, _t, _states = drive_on_axis(f, (0.704, -3.40), math.pi / 2, start)
        assert arrived, "regressed for start %s" % (start,)
        assert abs(geo.wrap_angle(pose[2] - math.pi / 2)) < math.radians(8)
