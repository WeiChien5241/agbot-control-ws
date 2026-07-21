import math

import numpy as np
import pytest

from agbot_vision_nav.centerline_estimator import (
    CLASS_OBSTACLE,
    CLASS_SKY,
    CLASS_TRAVERSABLE,
    estimate_centerline,
)
from agbot_vision_nav.mission_fsm import (
    STATE_BACKOUT,
    STATE_BACKOUT_CLEAR,
    STATE_BACKOUT_TRAVERSE,
    STATE_BACKOUT_TURN_1,
    STATE_BACKOUT_TURN_2,
    STATE_DONE,
    STATE_EXIT_CLEAR,
    STATE_FOLLOW_ROW,
    STATE_REACQUIRE,
    STATE_TRAVERSE,
    STATE_TURN_1,
    STATE_TURN_2,
    MissionFSM,
)
from agbot_vision_nav.row_exit_detector import RowExitDetector

HEIGHT = 100
WIDTH = 200


class StubController:
    """Stands in for MPCRowController: fixed cruise, zero steer."""

    linear_x_cruise = 0.15

    def __init__(self):
        self.reset_calls = 0

    def reset(self):
        self.reset_calls += 1

    def compute(self, offset_norm, slope_term, valid):
        return (self.linear_x_cruise, 0.0) if valid else (0.0, 0.0)


class SteeringStubController(StubController):
    """Stub that steers with the real controller's sign convention:
    offset_norm < 0 (centerline left) -> angular_z > 0 (turn left)."""

    def compute(self, offset_norm, slope_term, valid):
        return (self.linear_x_cruise, -offset_norm) if valid else (0.0, 0.0)


def corridor_result(shift=0):
    mask = np.full((HEIGHT, WIDTH), CLASS_OBSTACLE, dtype=np.uint8)
    mask[:20, :] = CLASS_SKY
    cx = WIDTH // 2 + shift
    mask[20:, cx - 30 : cx + 31] = CLASS_TRAVERSABLE
    return estimate_centerline(mask)


def open_result():
    mask = np.full((HEIGHT, WIDTH), CLASS_TRAVERSABLE, dtype=np.uint8)
    mask[:20, :] = CLASS_SKY
    return estimate_centerline(mask)


def blocked_result():
    """Wall of crop ahead: center columns blocked at every scan row, ground
    still visible at the sides (same signature as test_row_exit_detector)."""
    mask = np.full((HEIGHT, WIDTH), CLASS_TRAVERSABLE, dtype=np.uint8)
    mask[:20, :] = CLASS_SKY
    cx = WIDTH // 2
    mask[20:96, cx - 40 : cx + 41] = CLASS_OBSTACLE
    return estimate_centerline(mask)


def make_fsm(num_rows=3, first_turn_direction="left", **kw):
    return MissionFSM(
        StubController(),
        RowExitDetector(
            exit_detect_frames=3, blocked_detect_frames=3, min_in_row_distance=2.0
        ),
        num_rows=num_rows,
        first_turn_direction=first_turn_direction,
        **kw
    )


def drive_row_to_exit(fsm, start=(0.0, 0.0, 0.0)):
    """Follow a row past arming distance, then feed open frames until exit.

    Drives from `start` along its heading. Returns the pose at which the
    exit fired.
    """
    corridor = corridor_result()
    open_r = open_result()
    x0, y0, yaw = start
    pose = None
    # travel 3 m in-row (past the 2 m arming distance)
    for i in range(30):
        d = 0.1 * (i + 1)
        pose = (x0 + d * math.cos(yaw), y0 + d * math.sin(yaw), yaw)
        lin, ang, state, done = fsm.update(corridor, pose, WIDTH)
        assert state == STATE_FOLLOW_ROW
        assert lin == pytest.approx(0.15)
    # exit signature, debounced over 3 frames
    for _ in range(3):
        _, _, state, done = fsm.update(open_r, pose, WIDTH)
    return pose, state, done


def run_headland(fsm, pose, turn_sign):
    """Drive the FSM through EXIT_CLEAR -> TURN_1 -> TRAVERSE -> TURN_2,
    simulating perfect odometry. Returns the pose entering REACQUIRE."""
    open_r = open_result()
    x, y, yaw = pose

    assert fsm.state == STATE_EXIT_CLEAR
    # advance straight until TURN_1 (clearance 1.0 m)
    for i in range(1, 15):
        p = (x + i * 0.1 * math.cos(yaw), y + i * 0.1 * math.sin(yaw), yaw)
        lin, ang, state, _ = fsm.update(open_r, p, WIDTH)
        if state == STATE_TURN_1:
            x, y = p[0], p[1]
            break
        assert lin > 0 and ang == 0.0
    assert fsm.state == STATE_TURN_1

    # rotate 90 deg in steps
    for i in range(1, 20):
        p = (x, y, yaw + turn_sign * i * 0.1)
        lin, ang, state, _ = fsm.update(open_r, p, WIDTH)
        if state == STATE_TRAVERSE:
            yaw = p[2]
            break
        assert lin == 0.0
        assert math.copysign(1, ang) == turn_sign
    assert fsm.state == STATE_TRAVERSE

    # advance 0.75 m along new heading
    for i in range(1, 15):
        p = (x + i * 0.1 * math.cos(yaw), y + i * 0.1 * math.sin(yaw), yaw)
        lin, ang, state, _ = fsm.update(open_r, p, WIDTH)
        if state == STATE_TURN_2:
            x, y = p[0], p[1]
            break
        assert lin > 0 and ang == 0.0
    assert fsm.state == STATE_TURN_2

    # rotate the second 90 deg, same direction
    for i in range(1, 20):
        p = (x, y, yaw + turn_sign * i * 0.1)
        lin, ang, state, _ = fsm.update(open_r, p, WIDTH)
        if state == STATE_REACQUIRE:
            yaw = p[2]
            break
        assert math.copysign(1, ang) == turn_sign
    assert fsm.state == STATE_REACQUIRE
    return (x, y, yaw)


def test_full_transition_cycle_and_direction_flip():
    fsm = make_fsm(num_rows=3, first_turn_direction="left")
    pose, state, done = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR
    assert not done
    assert fsm.rows_driven == 1

    pose = run_headland(fsm, pose, turn_sign=+1)  # left turns

    # reacquire: corridor for reacquire_frames consecutive frames
    corridor = corridor_result()
    for _ in range(3):
        _, _, state, _ = fsm.update(corridor, pose, WIDTH)
    assert state == STATE_FOLLOW_ROW

    # second transition must turn RIGHT (boustrophedon)
    pose, state, _ = drive_row_to_exit(fsm, start=pose)
    assert state == STATE_EXIT_CLEAR
    assert fsm.rows_driven == 2
    run_headland(fsm, pose, turn_sign=-1)


def test_exit_clear_uses_slow_speed():
    fsm = make_fsm(exit_clear_speed=0.1)
    pose, _, _ = drive_row_to_exit(fsm)
    assert fsm.state == STATE_EXIT_CLEAR
    x, y, yaw = pose
    p = (x + 0.1 * math.cos(yaw), y + 0.1 * math.sin(yaw), yaw)
    lin, ang, state, _ = fsm.update(open_result(), p, WIDTH)
    assert state == STATE_EXIT_CLEAR
    assert lin == pytest.approx(0.1)  # slower than the 0.15 cruise
    assert ang == 0.0


def test_num_rows_termination_skips_turn():
    fsm = make_fsm(num_rows=1)
    _, state, done = drive_row_to_exit(fsm)
    assert state == STATE_DONE
    assert done
    # latched
    lin, ang, state, done = fsm.update(corridor_result(), (99, 99, 0), WIDTH)
    assert (lin, ang) == (0.0, 0.0)
    assert done


def test_reacquire_failure_means_no_rows_left():
    fsm = make_fsm(num_rows=0)  # unlimited
    pose, state, _ = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR  # unlimited: keeps going after exit
    pose = run_headland(fsm, pose, turn_sign=+1)
    # feed open field (no corridor) while creeping past reacquire_max_distance
    open_r = open_result()
    x, y, yaw = pose
    done = False
    for i in range(1, 40):
        p = (x + i * 0.1 * math.cos(yaw), y + i * 0.1 * math.sin(yaw), yaw)
        lin, ang, state, done = fsm.update(open_r, p, WIDTH)
        if done:
            break
    assert state == STATE_DONE
    assert done


def test_maneuver_states_stop_without_odom():
    fsm = make_fsm()
    pose, _, _ = drive_row_to_exit(fsm)
    assert fsm.state == STATE_EXIT_CLEAR
    lin, ang, state, done = fsm.update(open_result(), None, WIDTH)
    assert (lin, ang) == (0.0, 0.0)
    assert state == STATE_EXIT_CLEAR  # holds state, doesn't advance


def test_controller_reset_on_new_row():
    fsm = make_fsm()
    pose, _, _ = drive_row_to_exit(fsm)
    pose = run_headland(fsm, pose, turn_sign=+1)
    resets_before = fsm._controller.reset_calls
    corridor = corridor_result()
    for _ in range(3):
        fsm.update(corridor, pose, WIDTH)
    assert fsm.state == STATE_FOLLOW_ROW
    assert fsm._controller.reset_calls == resets_before + 1


def test_traverse_leg_uses_traverse_distance_not_row_spacing():
    fsm = make_fsm(traverse_distance=0.5)
    assert fsm.row_spacing == pytest.approx(0.75)  # physical value untouched
    pose, _, _ = drive_row_to_exit(fsm)
    open_r = open_result()
    x, y, yaw = pose

    # EXIT_CLEAR -> TURN_1
    for i in range(1, 20):
        p = (x + i * 0.1 * math.cos(yaw), y + i * 0.1 * math.sin(yaw), yaw)
        _, _, state, _ = fsm.update(open_r, p, WIDTH)
        if state == STATE_TURN_1:
            x, y = p[0], p[1]
            break
    assert fsm.state == STATE_TURN_1

    # rotate into TRAVERSE
    for i in range(1, 20):
        p = (x, y, yaw + i * 0.1)
        _, _, state, _ = fsm.update(open_r, p, WIDTH)
        if state == STATE_TRAVERSE:
            yaw = p[2]
            break
    assert fsm.state == STATE_TRAVERSE

    # 0.45 m along the leg: still traversing
    p = (x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw), yaw)
    _, _, state, _ = fsm.update(open_r, p, WIDTH)
    assert state == STATE_TRAVERSE
    # 0.55 m: past traverse_distance but short of row_spacing -> TURN_2
    p = (x + 0.55 * math.cos(yaw), y + 0.55 * math.sin(yaw), yaw)
    _, _, state, _ = fsm.update(open_r, p, WIDTH)
    assert state == STATE_TURN_2


# ---------------------------------------------------------------- back-out --
def drive_row_to_block(fsm, start=(0.0, 0.0, 0.0)):
    """Follow a row past arming, then feed blocked frames until BACKOUT.

    Returns the pose at which the block fired (the robot's block point).
    """
    corridor = corridor_result()
    blocked = blocked_result()
    x0, y0, yaw = start
    pose = None
    for i in range(30):  # 3 m in-row
        d = 0.1 * (i + 1)
        pose = (x0 + d * math.cos(yaw), y0 + d * math.sin(yaw), yaw)
        _, _, state, _ = fsm.update(corridor, pose, WIDTH)
        assert state == STATE_FOLLOW_ROW
    for _ in range(3):  # blocked signature, debounced over 3 frames
        _, _, state, done = fsm.update(blocked, pose, WIDTH)
    return pose, state, done


def run_backout(fsm, pose, turn_sign):
    """Drive the FSM through BACKOUT -> BACKOUT_CLEAR -> BACKOUT_TURN_1 ->
    BACKOUT_TRAVERSE -> BACKOUT_TURN_2, simulating perfect odometry.
    Returns the pose entering REACQUIRE."""
    front = blocked_result()
    x, y, yaw = pose

    assert fsm.state == STATE_BACKOUT
    # reverse along -heading until the block distance is unwound
    for i in range(1, 60):
        p = (x - i * 0.1 * math.cos(yaw), y - i * 0.1 * math.sin(yaw), yaw)
        lin, ang, state, _ = fsm.update(front, p, WIDTH)
        if state == STATE_BACKOUT_CLEAR:
            x, y = p[0], p[1]
            break
        assert lin < 0.0
    assert fsm.state == STATE_BACKOUT_CLEAR

    # keep reversing headland_clearance (1.0 m) past the row entrance
    for i in range(1, 20):
        p = (x - i * 0.1 * math.cos(yaw), y - i * 0.1 * math.sin(yaw), yaw)
        lin, ang, state, _ = fsm.update(front, p, WIDTH)
        if state == STATE_BACKOUT_TURN_1:
            x, y = p[0], p[1]
            break
        assert lin < 0.0 and ang == 0.0
    assert fsm.state == STATE_BACKOUT_TURN_1

    # first 90 deg of the lane change: current turn sign
    for i in range(1, 20):
        p = (x, y, yaw + turn_sign * i * 0.1)
        lin, ang, state, _ = fsm.update(front, p, WIDTH)
        if state == STATE_BACKOUT_TRAVERSE:
            yaw = p[2]
            break
        assert lin == 0.0
        assert math.copysign(1, ang) == turn_sign
    assert fsm.state == STATE_BACKOUT_TRAVERSE

    # sidestep one row spacing (0.75 m), forward
    for i in range(1, 15):
        p = (x + i * 0.1 * math.cos(yaw), y + i * 0.1 * math.sin(yaw), yaw)
        lin, ang, state, _ = fsm.update(front, p, WIDTH)
        if state == STATE_BACKOUT_TURN_2:
            x, y = p[0], p[1]
            break
        assert lin > 0 and ang == 0.0
    assert fsm.state == STATE_BACKOUT_TURN_2

    # second 90 deg COUNTER-rotates (S-shaped lane change)
    for i in range(1, 20):
        p = (x, y, yaw - turn_sign * i * 0.1)
        lin, ang, state, _ = fsm.update(front, p, WIDTH)
        if state == STATE_REACQUIRE:
            yaw = p[2]
            break
        assert math.copysign(1, ang) == -turn_sign
    assert fsm.state == STATE_REACQUIRE
    return (x, y, yaw)


def test_blocked_exit_enters_backout_not_exit_clear():
    fsm = make_fsm(num_rows=3)
    pose, state, done = drive_row_to_block(fsm)
    assert state == STATE_BACKOUT
    assert not done
    assert fsm.rows_driven == 1
    assert len(fsm.blocked_events) == 1
    row, dist = fsm.blocked_events[0]
    assert row == 1
    assert dist == pytest.approx(3.0, abs=0.2)
    # next tick actually reverses
    lin, ang, state, _ = fsm.update(blocked_result(), pose, WIDTH)
    assert lin < 0.0
    assert state == STATE_BACKOUT


def test_backout_rear_steering_signs():
    fsm = MissionFSM(
        SteeringStubController(),
        RowExitDetector(
            exit_detect_frames=3, blocked_detect_frames=3, min_in_row_distance=2.0
        ),
        num_rows=3,
    )
    pose, _, _ = drive_row_to_block(fsm)
    front = blocked_result()
    # rear centerline LEFT of image center -> positive angular_z (turn left);
    # shift keeps the corridor overlapping the center column so the
    # centerline result stays valid
    lin, ang, _, _ = fsm.update(
        front, pose, WIDTH, rear_centerline_result=corridor_result(shift=-20)
    )
    assert lin < 0.0
    assert ang > 0.0
    # rear centerline RIGHT of image center -> negative angular_z
    lin, ang, _, _ = fsm.update(
        front, pose, WIDTH, rear_centerline_result=corridor_result(shift=20)
    )
    assert ang < 0.0


def test_backout_without_rear_result_reverses_straight():
    fsm = make_fsm(num_rows=3, backout_speed=0.12)
    pose, _, _ = drive_row_to_block(fsm)
    lin, ang, state, _ = fsm.update(blocked_result(), pose, WIDTH)
    assert (lin, ang) == (pytest.approx(-0.12), 0.0)
    assert state == STATE_BACKOUT


def test_backout_full_sequence_and_flip_suppression():
    fsm = make_fsm(num_rows=3, first_turn_direction="left")
    pose, _, _ = drive_row_to_block(fsm)  # row 1 blocked
    pose = run_backout(fsm, pose, turn_sign=+1)

    # reacquire the next row: turn sign must NOT flip (same world direction)
    corridor = corridor_result()
    for _ in range(3):
        _, _, state, _ = fsm.update(corridor, pose, WIDTH)
    assert state == STATE_FOLLOW_ROW
    assert fsm._turn_sign == +1

    # row 2 exits normally: headland still turns LEFT, and the flip at the
    # following reacquire happens again (no double suppression)
    pose, state, _ = drive_row_to_exit(fsm, start=pose)
    assert state == STATE_EXIT_CLEAR
    assert fsm.rows_driven == 2
    pose = run_headland(fsm, pose, turn_sign=+1)
    for _ in range(3):
        _, _, state, _ = fsm.update(corridor, pose, WIDTH)
    assert state == STATE_FOLLOW_ROW
    assert fsm._turn_sign == -1


def test_blocked_final_row_backs_out_then_done():
    fsm = make_fsm(num_rows=1)
    pose, state, done = drive_row_to_block(fsm)
    assert state == STATE_BACKOUT
    assert not done  # backs out before stopping
    front = blocked_result()
    x, y, yaw = pose
    done = False
    for i in range(1, 60):
        p = (x - i * 0.1 * math.cos(yaw), y - i * 0.1 * math.sin(yaw), yaw)
        lin, ang, state, done = fsm.update(front, p, WIDTH)
        if done:
            break
        assert lin <= 0.0  # only ever reverses (or holds at transitions)
    assert state == STATE_DONE
    assert done
    assert fsm.blocked_events == [(1, pytest.approx(3.0, abs=0.2))]


def test_backout_progress_reports_distance():
    fsm = make_fsm(num_rows=3)
    pose, _, _ = drive_row_to_block(fsm)
    reversed_m, target = fsm.backout_progress(pose)
    assert reversed_m == pytest.approx(0.0)
    assert target == pytest.approx(3.0, abs=0.2)
    x, y, yaw = pose
    p = (x - 0.5 * math.cos(yaw), y - 0.5 * math.sin(yaw), yaw)
    fsm.update(blocked_result(), p, WIDTH)
    reversed_m, _ = fsm.backout_progress(p)
    assert reversed_m == pytest.approx(0.5)
    assert fsm.backout_progress(None)[0] is None  # no odom -> unknown


def test_backout_states_stop_without_odom():
    fsm = make_fsm(num_rows=3)
    drive_row_to_block(fsm)
    lin, ang, state, done = fsm.update(blocked_result(), None, WIDTH)
    assert (lin, ang) == (0.0, 0.0)
    assert state == STATE_BACKOUT  # holds state, doesn't advance
    assert not done


def test_controller_reset_on_backout_entry():
    fsm = make_fsm(num_rows=3)
    resets_before = fsm._controller.reset_calls
    drive_row_to_block(fsm)
    assert fsm._controller.reset_calls == resets_before + 1


def test_blocked_without_backout_stops_and_done():
    fsm = make_fsm(num_rows=3, backout_enabled=False)
    pose, state, done = drive_row_to_block(fsm)
    assert state == STATE_DONE
    assert done
    assert fsm.blocked_events == [(1, pytest.approx(3.0, abs=0.2))]
    # latched stopped; BACKOUT never entered
    lin, ang, state, done = fsm.update(blocked_result(), pose, WIDTH)
    assert (lin, ang) == (0.0, 0.0)
    assert state == STATE_DONE
    assert done
