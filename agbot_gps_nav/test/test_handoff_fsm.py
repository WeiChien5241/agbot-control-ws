"""Tests for the GPS-transit -> vision-mission sequencing.

The invariant worth most here is that EXACTLY ONE node is ever enabled. Both
publish /cmd_vel, twist_mux takes that as one input at priority 1 and
arbitrates by priority rather than rate, so two enabled nodes do not contend --
they interleave, and the robot stutters between two commanders.
"""

import pytest

from agbot_gps_nav.handoff_fsm import (
    GPS_ARRIVED, GPS_FAILED, STATE_FAILED, STATE_FINISHED, STATE_IDLE,
    STATE_ROW_MISSION, STATE_TRANSIT, HandoffFSM,
)


def run_to_mission(fsm, t=0.0):
    fsm.start(t)
    return fsm.update(t + 1.0, gps_state=GPS_ARRIVED)


def test_starts_idle_with_nothing_driving():
    fsm = HandoffFSM()
    tick = fsm.update(0.0)
    assert tick.state == STATE_IDLE
    assert not tick.gps_enabled and not tick.vision_enabled


def test_start_runs_gps_and_holds_vision_off():
    fsm = HandoffFSM()
    tick = fsm.start(0.0)
    assert tick.state == STATE_TRANSIT
    assert tick.gps_enabled and not tick.vision_enabled
    assert tick.send_goal, "the goal must be sent when the transit begins"


def test_the_goal_is_sent_once_not_every_tick():
    """Re-sending a goal restarts the follower, including its ARRIVED latch."""
    fsm = HandoffFSM()
    assert fsm.start(0.0).send_goal
    for t in (1.0, 2.0, 3.0):
        assert not fsm.update(t, gps_state="GOTO").send_goal


def test_arrival_hands_over_to_vision():
    fsm = HandoffFSM()
    fsm.start(0.0)
    tick = fsm.update(5.0, gps_state=GPS_ARRIVED)
    assert tick.state == STATE_ROW_MISSION
    assert tick.vision_enabled and not tick.gps_enabled


def test_the_handoff_tick_says_which_node_to_switch_off_first():
    """⚠ Disable the outgoing node BEFORE enabling the incoming one, or there
    is a window with two publishers on /cmd_vel."""
    fsm = HandoffFSM()
    assert fsm.start(0.0).disable_first == "vision"
    assert fsm.update(5.0, gps_state=GPS_ARRIVED).disable_first == "gps"


@pytest.mark.parametrize("scenario", [
    "transit", "mission", "finished", "failed",
])
def test_never_both_enabled(scenario):
    fsm = HandoffFSM(transit_timeout_sec=10.0)
    ticks = [fsm.start(0.0)]
    if scenario == "transit":
        ticks.append(fsm.update(1.0, gps_state="GOTO"))
    elif scenario == "mission":
        ticks.append(fsm.update(1.0, gps_state=GPS_ARRIVED))
        ticks.append(fsm.update(2.0))
    elif scenario == "finished":
        ticks.append(fsm.update(1.0, gps_state=GPS_ARRIVED))
        ticks.append(fsm.update(2.0, vision_done=True))
    else:
        ticks.append(fsm.update(999.0, gps_state="GOTO"))
    for tick in ticks:
        assert not (tick.gps_enabled and tick.vision_enabled), tick


def test_mission_completion_stops_everything():
    fsm = HandoffFSM()
    run_to_mission(fsm)
    tick = fsm.update(10.0, vision_done=True)
    assert tick.state == STATE_FINISHED and tick.done
    assert not tick.gps_enabled and not tick.vision_enabled


def test_finished_is_latched():
    fsm = HandoffFSM()
    run_to_mission(fsm)
    fsm.update(10.0, vision_done=True)
    for t in (11.0, 12.0):
        tick = fsm.update(t, vision_done=False)
        assert tick.state == STATE_FINISHED and tick.done


# ---- the failure paths, which must exist rather than be assumed ----------

def test_a_gps_abort_fails_the_sequence():
    """The follower's receding-goal abort has to reach the supervisor, or the
    sequence sits in TRANSIT forever behind a robot that has given up."""
    fsm = HandoffFSM()
    fsm.start(0.0)
    tick = fsm.update(5.0, gps_state=GPS_FAILED)
    assert tick.state == STATE_FAILED and tick.failed
    assert not tick.gps_enabled and not tick.vision_enabled
    assert "GPS" in tick.reason


def test_transit_timeout():
    fsm = HandoffFSM(transit_timeout_sec=60.0)
    fsm.start(0.0)
    assert fsm.update(59.0, gps_state="GOTO").state == STATE_TRANSIT
    tick = fsm.update(61.0, gps_state="GOTO")
    assert tick.state == STATE_FAILED and "timed out" in tick.reason


def test_mission_timeout_is_measured_from_the_handoff_not_the_start():
    """Otherwise a slow transit eats the row mission's whole budget."""
    fsm = HandoffFSM(transit_timeout_sec=1000.0, mission_timeout_sec=100.0)
    fsm.start(0.0)
    fsm.update(500.0, gps_state=GPS_ARRIVED)      # a long but legal transit
    assert fsm.update(560.0).state == STATE_ROW_MISSION
    assert fsm.update(605.0).state == STATE_FAILED


def test_abort_stops_everything_from_any_state():
    for setup in (lambda f: f.start(0.0),
                  lambda f: run_to_mission(f)):
        fsm = HandoffFSM()
        setup(fsm)
        tick = fsm.abort("operator stop")
        assert tick.state == STATE_FAILED and tick.failed
        assert not tick.gps_enabled and not tick.vision_enabled
        assert tick.reason == "operator stop"


def test_failed_is_latched_and_stays_silent():
    fsm = HandoffFSM()
    fsm.start(0.0)
    fsm.abort("boom")
    for t in (1.0, 2.0):
        tick = fsm.update(t, gps_state=GPS_ARRIVED, vision_done=True)
        assert tick.state == STATE_FAILED
        assert not tick.gps_enabled and not tick.vision_enabled


def test_a_late_arrival_after_failure_does_not_restart_the_sequence():
    fsm = HandoffFSM(transit_timeout_sec=10.0)
    fsm.start(0.0)
    fsm.update(20.0, gps_state="GOTO")            # times out
    assert fsm.state == STATE_FAILED
    assert fsm.update(21.0, gps_state=GPS_ARRIVED).state == STATE_FAILED


# ---- the arrival heading gate --------------------------------------------

def test_arriving_pointed_the_wrong_way_refuses_the_handoff():
    """⚠ THE 2026-09-07 FAILURE. GPS pins position, never orientation, so
    gps_state == ARRIVED on its own says the robot reached the right PLACE and
    nothing about which way it faces. That run arrived 0.41 m from the corridor
    entrance and about 72 deg off it; vision nav was handed a view of solid
    corn, went BLOCKED, backed out, and the mission ended at rows=0/3."""
    fsm = HandoffFSM(max_arrival_heading_error_deg=12.0)
    fsm.start(0.0)
    tick = fsm.update(1.0, gps_state=GPS_ARRIVED,
                      arrival_heading_error_deg=71.9)
    assert tick.state == STATE_FAILED
    assert tick.failed and not tick.vision_enabled and not tick.gps_enabled
    assert "72 deg off" in tick.reason


def test_a_good_arrival_heading_still_hands_over():
    fsm = HandoffFSM(max_arrival_heading_error_deg=12.0)
    fsm.start(0.0)
    tick = fsm.update(1.0, gps_state=GPS_ARRIVED, arrival_heading_error_deg=-4.2)
    assert tick.state == STATE_ROW_MISSION
    assert tick.vision_enabled and not tick.gps_enabled


def test_the_gate_is_symmetric_about_zero():
    """Off to the left is exactly as bad as off to the right."""
    for error in (13.0, -13.0):
        fsm = HandoffFSM(max_arrival_heading_error_deg=12.0)
        fsm.start(0.0)
        tick = fsm.update(1.0, gps_state=GPS_ARRIVED,
                          arrival_heading_error_deg=error)
        assert tick.state == STATE_FAILED, "%.0f deg should have been refused" % error


def test_an_unmeasured_arrival_heading_is_allowed_through():
    """⚠ Deliberate: the gate fires on EVIDENCE OF BEING WRONG, not on the
    absence of evidence. A short or bearingless goal has no leg to measure a
    course over, and failing those would break every plain go-to-a-point use.
    The supervisor logs a warning instead -- unverified, but not silent."""
    fsm = HandoffFSM()
    fsm.start(0.0)
    tick = fsm.update(1.0, gps_state=GPS_ARRIVED, arrival_heading_error_deg=None)
    assert tick.state == STATE_ROW_MISSION


def test_the_gate_default_matches_the_bootstrap_tolerance():
    """Both ask the same question -- does the course driven agree with the yaw
    estimate -- so they should not disagree about what counts as agreement."""
    assert HandoffFSM().max_arrival_heading_error_deg == 12.0
