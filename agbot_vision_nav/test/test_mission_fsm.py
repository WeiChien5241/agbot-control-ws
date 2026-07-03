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


def corridor_result():
    mask = np.full((HEIGHT, WIDTH), CLASS_OBSTACLE, dtype=np.uint8)
    mask[:20, :] = CLASS_SKY
    cx = WIDTH // 2
    mask[20:, cx - 30 : cx + 31] = CLASS_TRAVERSABLE
    return estimate_centerline(mask)


def open_result():
    mask = np.full((HEIGHT, WIDTH), CLASS_TRAVERSABLE, dtype=np.uint8)
    mask[:20, :] = CLASS_SKY
    return estimate_centerline(mask)


def make_fsm(num_rows=3, first_turn_direction="left", **kw):
    return MissionFSM(
        StubController(),
        RowExitDetector(exit_detect_frames=3, min_in_row_distance=2.0),
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
