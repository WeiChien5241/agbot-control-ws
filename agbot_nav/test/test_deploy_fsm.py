"""DeployFSM: who drives, where to, and every way the deployment fails."""
import math

import pytest

from agbot_nav.deploy_fsm import (
    COMMANDER_MOVE_BASE, COMMANDER_SUPERVISOR, COMMANDER_VISION,
    NAV_ACTIVE, NAV_FAILED, NAV_SUCCEEDED,
    STATE_FAILED, STATE_FINISHED, STATE_LOCALIZE, STATE_RETURN,
    STATE_REVERSE_OFF, STATE_ROW_MISSION, STATE_TRANSIT,
    DeployFSM, Estimate, Pose2D, axis_errors,
)

ENTRANCE = (-0.8, -3.9, math.pi / 2)        # corridor mouth, row runs +y
GOOD = Estimate(-0.8, -3.9, math.pi / 2, 0.05, math.radians(1.0))
AT_DROP = Estimate(10.0, -16.0, math.pi, 0.05, math.radians(1.0))


def fsm(**kw):
    return DeployFSM(ENTRANCE, **kw)


def to_localize(f, t=0.0):
    f.start(t)
    f.update(t + 0.1, odom_xy=(0.0, 0.0))
    return f.update(t + 5.0, odom_xy=(-1.6, 0.0))


def to_transit(f):
    to_localize(f)
    f.update(6.0, estimate=AT_DROP)
    return f.update(8.5, estimate=AT_DROP)


def to_row(f):
    to_transit(f)
    return f.update(20.0, estimate=GOOD, nav_state=NAV_SUCCEEDED)


# ---- reverse off the ramp ---------------------------------------------------

def test_start_reverses_with_the_supervisor_in_charge():
    tick = fsm().start(0.0)
    assert tick.state == STATE_REVERSE_OFF
    assert tick.commander == COMMANDER_SUPERVISOR
    assert tick.reverse_speed < 0.0
    assert not tick.vision_enabled and tick.nav_goal is None


def test_reverse_ends_on_odometry_distance_not_time():
    f = fsm(reverse_distance=1.5)
    f.start(0.0)
    f.update(0.1, odom_xy=(5.0, 5.0))           # first sample is the reference
    assert f.update(3.0, odom_xy=(4.0, 5.0)).state == STATE_REVERSE_OFF
    tick = f.update(4.0, odom_xy=(3.4, 5.0))
    assert tick.state == STATE_LOCALIZE
    assert tick.reverse_speed == 0.0 and tick.commander is None


def test_zero_reverse_distance_skips_the_reverse_leg():
    tick = fsm(reverse_distance=0.0).start(0.0)
    assert tick.state == STATE_LOCALIZE
    assert tick.reverse_speed == 0.0 and tick.commander is None


def test_reverse_without_odometry_times_out():
    f = fsm(reverse_timeout_sec=30.0)
    f.start(0.0)
    tick = f.update(31.0, odom_xy=None)
    assert tick.failed and "REVERSE_OFF" in tick.reason


# ---- localize ---------------------------------------------------------------

def test_localize_asks_amcl_for_updates_while_stopped():
    tick = to_localize(fsm())
    assert tick.request_localize_update
    assert tick.commander is None


def test_localize_needs_a_held_confident_estimate():
    f = fsm(localize_hold_sec=2.0)
    to_localize(f)
    assert f.update(6.0, estimate=AT_DROP).state == STATE_LOCALIZE
    # a wobble restarts the hold
    loose = AT_DROP._replace(std_xy=0.5)    # over even the localize limit
    assert f.update(7.0, estimate=loose).state == STATE_LOCALIZE
    assert f.update(8.0, estimate=AT_DROP).state == STATE_LOCALIZE
    assert f.update(9.5, estimate=AT_DROP).state == STATE_LOCALIZE
    tick = f.update(10.1, estimate=AT_DROP)
    assert tick.state == STATE_TRANSIT


def test_localize_accepts_a_spread_the_arrival_would_refuse():
    f = fsm(max_std_xy=0.15, localize_max_std_xy=0.30)
    to_localize(f)
    loose = AT_DROP._replace(std_xy=0.25)
    f.update(6.0, estimate=loose)
    assert f.update(8.5, estimate=loose).state == STATE_TRANSIT


def test_wide_heading_spread_is_not_confident():
    f = fsm()
    to_localize(f)
    wide = AT_DROP._replace(std_yaw=math.radians(15.0))   # over the 10 deg limit
    f.update(6.0, estimate=wide)
    assert f.update(20.0, estimate=wide).state == STATE_LOCALIZE


def test_localize_timeout_fails():
    f = fsm(localize_timeout_sec=30.0)
    to_localize(f)
    assert f.update(40.0, estimate=None).failed


def test_transit_goal_stands_off_the_entrance_on_its_axis():
    f = fsm(entrance_standoff=0.7)
    tick = to_transit(f)
    assert tick.state == STATE_TRANSIT
    assert tick.commander == COMMANDER_MOVE_BASE
    # row runs +y, so standing off moves the goal 0.7 m in -y, same x and yaw
    assert tick.nav_goal.x == pytest.approx(ENTRANCE[0])
    assert tick.nav_goal.y == pytest.approx(ENTRANCE[1] - 0.7)
    assert tick.nav_goal.yaw == pytest.approx(ENTRANCE[2])


def test_arrival_at_the_standoff_point_is_on_axis():
    f = fsm(entrance_standoff=0.7)
    to_transit(f)
    tick = f.update(20.0, estimate=GOOD._replace(y=GOOD.y - 0.7),
                    nav_state=NAV_SUCCEEDED)
    assert tick.state == STATE_ROW_MISSION
    assert f.arrival[0] == pytest.approx(0.0)


def test_drop_off_is_recorded():
    f = fsm()
    to_transit(f)
    assert f.drop_off == Pose2D(AT_DROP.x, AT_DROP.y, AT_DROP.yaw)


def test_goal_is_sent_once_not_every_tick():
    f = fsm()
    to_transit(f)
    assert f.update(9.0, estimate=AT_DROP, nav_state=NAV_ACTIVE).nav_goal is None


# ---- arrival gate -----------------------------------------------------------

def test_on_axis_arrival_hands_over_to_vision():
    f = fsm()
    tick = to_row(f)
    assert tick.state == STATE_ROW_MISSION
    assert tick.commander == COMMANDER_VISION and tick.vision_enabled
    assert f.arrival == pytest.approx((0.0, 0.0))


def test_off_axis_arrival_is_refused():
    f = fsm(max_arrival_cross_track=0.20)
    to_transit(f)
    # 0.3 m to the side of a +y axis is an x offset
    tick = f.update(20.0, estimate=GOOD._replace(x=GOOD.x + 0.3),
                    nav_state=NAV_SUCCEEDED)
    assert tick.failed and not tick.vision_enabled
    assert "off the corridor axis" in tick.reason


def test_along_axis_offset_is_not_cross_track():
    f = fsm()
    to_transit(f)
    tick = f.update(20.0, estimate=GOOD._replace(y=GOOD.y - 0.3),
                    nav_state=NAV_SUCCEEDED)
    assert tick.state == STATE_ROW_MISSION


def test_wrong_heading_arrival_is_refused():
    f = fsm(max_arrival_heading_error_deg=12.0)
    to_transit(f)
    tick = f.update(20.0, estimate=GOOD._replace(yaw=GOOD.yaw + math.radians(15)),
                    nav_state=NAV_SUCCEEDED)
    assert tick.failed


def test_arrival_without_a_confident_estimate_is_refused():
    f = fsm()
    to_transit(f)
    tick = f.update(20.0, estimate=GOOD._replace(std_xy=0.4),
                    nav_state=NAV_SUCCEEDED)
    assert tick.failed and "NOT confident" in tick.reason


def test_move_base_failure_fails_the_transit():
    f = fsm()
    to_transit(f)
    tick = f.update(20.0, estimate=GOOD, nav_state=NAV_FAILED)
    assert tick.failed and tick.cancel_nav


# ---- row mission and the way home ------------------------------------------

def test_mission_done_returns_to_the_recorded_drop_off():
    f = fsm()
    to_row(f)
    tick = f.update(200.0, vision_done=True)
    assert tick.state == STATE_RETURN
    assert tick.commander == COMMANDER_MOVE_BASE and not tick.vision_enabled
    assert tick.nav_goal == f.drop_off
    assert f.update(260.0, nav_state=NAV_SUCCEEDED).state == STATE_FINISHED


def test_return_can_be_disabled():
    f = fsm(return_enabled=False)
    to_row(f)
    tick = f.update(200.0, vision_done=True)
    assert tick.done and tick.commander is None


def test_return_failure_fails():
    f = fsm()
    to_row(f)
    f.update(200.0, vision_done=True)
    assert f.update(220.0, nav_state=NAV_FAILED).failed


def test_at_most_one_commander_in_every_state():
    f = fsm()
    ticks = [f.start(0.0)]
    ticks.append(f.update(0.1, odom_xy=(0.0, 0.0)))
    ticks.append(f.update(5.0, odom_xy=(-1.6, 0.0)))
    ticks.append(f.update(6.0, estimate=AT_DROP))
    ticks.append(f.update(8.5, estimate=AT_DROP))
    ticks.append(f.update(20.0, estimate=GOOD, nav_state=NAV_SUCCEEDED))
    ticks.append(f.update(200.0, vision_done=True))
    for tick in ticks:
        driving = [tick.reverse_speed != 0.0, tick.vision_enabled,
                   tick.commander == COMMANDER_MOVE_BASE]
        assert sum(driving) <= 1, tick


def test_abort_stops_everything():
    f = fsm()
    to_row(f)
    tick = f.abort("operator stop")
    assert tick.failed and tick.cancel_nav
    assert not tick.vision_enabled and tick.reverse_speed == 0.0
    assert tick.commander is None


def test_mission_timeout():
    f = fsm(mission_timeout_sec=100.0)
    to_row(f)
    assert f.update(121.0).failed


def test_axis_errors_sign_convention():
    entrance = Pose2D(0.0, 0.0, 0.0)            # axis along +x
    cross, heading = axis_errors(Estimate(1.0, 0.2, 0.1, 0, 0), entrance)
    assert cross == pytest.approx(0.2)          # left of the axis is positive
    assert heading == pytest.approx(0.1)
