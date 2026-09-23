"""Tests for multi-waypoint routes (Route editing/persistence, RouteRunner).

As in test_waypoint_follower.py, the runner is tested closed-loop on a unicycle
model: a route controller can pass every unit test and still orbit a
pass-through point, and only integrating the motion shows that.
"""

import math

import pytest

from agbot_gps_nav import geo
from agbot_gps_nav.route import Route, RouteRunner, Waypoint
from agbot_gps_nav.waypoint_follower import (
    STATE_ALIGN, STATE_ARRIVED, STATE_FAILED, STATE_HEADING_INIT, STATE_IDLE,
    WaypointFollower,
)

DATUM = (40.494928, -86.996323)


def run(runner, start=(0.0, 0.0, 0.0), dt=0.1, max_steps=20000):
    """Integrate the runner's commands. Returns (done, pose, trace).

    trace is a list of (leg, state, x, y) per step.
    """
    x, y, yaw = start
    trace = []
    for _ in range(max_steps):
        linear_x, angular_z, state, done = runner.update((x, y, yaw))
        trace.append((runner.leg, state, x, y, linear_x))
        if done or state == STATE_FAILED:
            return done, (x, y, yaw), trace
        yaw += angular_z * dt
        x += linear_x * math.cos(yaw) * dt
        y += linear_x * math.sin(yaw) * dt
    return False, (x, y, yaw), trace


# ---- Route: the editable list ----------------------------------------------

def test_append_names_points_in_order_and_undo_pops_the_last():
    route = Route()
    route.append(40.1, -86.1)
    route.append(40.2, -86.2, bearing_deg=90)
    assert [w.name for w in route.waypoints] == ["wp1", "wp2"]
    assert route.undo() == Waypoint(40.2, -86.2, 90.0, "wp2")
    assert len(route) == 1
    route.clear()
    assert len(route) == 0
    assert route.undo() is None


def test_yaml_round_trip(tmp_path):
    route = Route()
    route.append(40.4950, -86.9960)
    route.append(40.4955, -86.9963, bearing_deg=90.0, name="corridor_0")
    path = str(tmp_path / "route.yaml")
    route.save(path, datum=DATUM)
    back = Route.load(path)
    assert back.waypoints == route.waypoints


def test_generated_waypoints_file_schema_loads_as_a_route():
    # A corridor entry from rows_to_waypoints.py can be the last point as-is;
    # the extra keys it carries are ignored.
    spec = {"datum": [40.494928, -86.996323, 0.0],
            "start_pose": {"lat": 40.4947, "lon": -86.9963},
            "waypoints": [{"name": "corridor_0", "lat": 40.4949, "lon": -86.9963,
                           "approach_bearing_deg": 90.0,
                           "first_turn_direction": "right"}]}
    route = Route.from_dict(spec)
    assert route.waypoints == [Waypoint(40.4949, -86.9963, 90.0, "corridor_0")]


def test_to_map_uses_the_navsat_map_frame_and_radians():
    route = Route()
    lat, lon = geo.map_to_latlon(10.0, 20.0, DATUM)
    route.append(lat, lon, bearing_deg=90.0)
    ((x, y, bearing),) = route.to_map(DATUM)
    assert (x, y) == pytest.approx((10.0, 20.0), abs=1e-6)
    assert bearing == pytest.approx(math.pi / 2)


# ---- RouteRunner -------------------------------------------------------------

def test_empty_route_is_refused():
    with pytest.raises(ValueError):
        RouteRunner().start([])


def test_walks_every_point_and_arrives_only_at_the_last():
    runner = RouteRunner(pass_radius=1.0)
    points = [(10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    runner.start(points)
    done, pose, trace = run(runner)
    assert done
    assert geo.distance(pose[:2], points[-1][:2]) <= runner.follower.goal_tolerance
    legs = [t[0] for t in trace]
    assert sorted(set(legs)) == [1, 2, 3]
    assert legs == sorted(legs), "legs must advance monotonically"
    # ARRIVED never appears before the last leg.
    assert all(t[1] != STATE_ARRIVED for t in trace if t[0] < 3)
    for x, y in points[:-1]:
        assert min(math.hypot(t[2] - x, t[3] - y) for t in trace) <= 1.0


def test_pass_through_points_do_not_slow_down():
    runner = RouteRunner(WaypointFollower(heading_init_distance=0.0),
                         pass_radius=1.0)
    runner.start([(10.0, 0.0), (20.0, 0.0)])
    _done, _pose, trace = run(runner)
    # Straight line through wp1: speed stays at cruise across it.
    near_wp1 = [t[4] for t in trace if t[0] == 1 and 8.0 <= t[2] <= 10.0]
    assert near_wp1 and min(near_wp1) == pytest.approx(
        runner.follower.linear_x_cruise)
    # But the last point is still arrived at carefully.
    final = [t[4] for t in trace if t[0] == 2 and t[2] > 19.0]
    assert min(final) < runner.follower.linear_x_cruise


def test_a_wide_miss_still_advances_via_the_plane():
    # pass_radius tiny, so only the perpendicular plane can advance leg 1.
    # Start pointing 20 deg off so the robot curves in and could miss.
    runner = RouteRunner(pass_radius=0.01)
    runner.start([(10.0, 0.0), (10.0, 10.0)])
    done, _pose, trace = run(runner, start=(0.0, -2.0, math.radians(20)))
    assert done
    assert trace[-1][0] == 2


def test_heading_bootstrap_runs_on_the_first_leg_only():
    follower = WaypointFollower(heading_init_distance=2.0)
    runner = RouteRunner(follower, pass_radius=1.0)
    runner.start([(10.0, 0.0), (10.0, 10.0), (0.0, 10.0)])
    _done, _pose, trace = run(runner)
    init_legs = {t[0] for t in trace if t[1] == STATE_HEADING_INIT}
    assert init_legs == {1}


def test_never_advances_out_of_the_heading_bootstrap():
    # Point 1 is 0.5 m ahead -- inside pass_radius from the very first tick --
    # but the bootstrap must still finish before leg 2 starts, because leg 2 is
    # told not to bootstrap.
    follower = WaypointFollower(heading_init_distance=3.0)
    runner = RouteRunner(follower, pass_radius=1.0)
    runner.start([(0.5, 0.0), (20.0, 0.0)])
    _done, _pose, trace = run(runner)
    first_leg2 = next(i for i, t in enumerate(trace) if t[0] == 2)
    assert all(t[1] != STATE_HEADING_INIT for t in trace[first_leg2:])
    assert any(t[1] == STATE_HEADING_INIT for t in trace[:first_leg2])
    assert follower.heading_init_converged() is not None


def test_bearing_applies_to_the_last_point_only():
    runner = RouteRunner(pass_radius=1.0)
    runner.start([(5.0, 5.0, 0.0), (10.0, 20.0, math.pi / 2)])
    done, pose, trace = run(runner)
    assert done
    assert all(t[1] != STATE_ALIGN for t in trace if t[0] == 1)
    assert any(t[1] == STATE_ALIGN for t in trace if t[0] == 2)
    assert abs(geo.wrap_angle(pose[2] - math.pi / 2)) < math.radians(6)


def test_one_point_route_is_a_plain_goal():
    runner = RouteRunner()
    runner.start([(5.0, 0.0)])
    assert runner.is_last_leg and runner.leg == 1 and runner.leg_count == 1
    done, _pose, _trace = run(runner)
    assert done


def test_stop_idles_the_follower():
    runner = RouteRunner()
    runner.start([(5.0, 0.0), (10.0, 0.0)])
    assert runner.active
    runner.stop()
    assert not runner.active and runner.leg == 0
    assert runner.update((0.0, 0.0, 0.0)) == (0.0, 0.0, STATE_IDLE, False)


def test_missing_pose_holds_still_without_losing_the_leg():
    runner = RouteRunner()
    runner.start([(5.0, 0.0), (10.0, 0.0)])
    linear_x, angular_z, _state, done = runner.update(None)
    assert (linear_x, angular_z, done) == (0.0, 0.0, False)
    assert runner.leg == 1
