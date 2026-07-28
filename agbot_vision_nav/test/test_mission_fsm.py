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
from agbot_vision_nav.row_exit_detector import (
    RowExitDetector,
    normalized_corridor_widths,
)

HEIGHT = 100
WIDTH = 200


class Clock:
    """Fake monotonic clock advanced once per FSM update.

    The BLOCKED signature is debounced in SECONDS (a blocked view stops the
    robot, so meters would never accumulate), and the detector must not read
    the wall clock in tests.
    """

    def __init__(self, step=0.5):
        self.now = 0.0
        self.step = step

    def tick(self):
        self.now += self.step
        return self.now


_clock = Clock()


def update(fsm, *args, **kwargs):
    """fsm.update() with the test clock supplied."""
    kwargs.setdefault("now", _clock.tick())
    return fsm.update(*args, **kwargs)


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
            exit_confirm_distance=0.2,
            blocked_confirm_seconds=1.0,
            min_in_row_distance=2.0,
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
    d = 0.0
    # travel 3 m in-row (past the 2 m arming distance)
    for _ in range(30):
        d += 0.1
        pose = (x0 + d * math.cos(yaw), y0 + d * math.sin(yaw), yaw)
        lin, ang, state, done = update(fsm, corridor, pose, WIDTH)
        assert state == STATE_FOLLOW_ROW
        assert lin == pytest.approx(0.15)
    # Exit signature. The debounce is in METERS, so the robot has to keep
    # driving through it -- feeding open frames from a parked pose never fires.
    for _ in range(20):
        d += 0.1
        pose = (x0 + d * math.cos(yaw), y0 + d * math.sin(yaw), yaw)
        _, _, state, done = update(fsm, open_r, pose, WIDTH)
        if state != STATE_FOLLOW_ROW:
            break
    assert state != STATE_FOLLOW_ROW, "exit never fired"
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
        lin, ang, state, _ = update(fsm, open_r, p, WIDTH)
        if state == STATE_TURN_1:
            x, y = p[0], p[1]
            break
        assert lin > 0 and ang == 0.0
    assert fsm.state == STATE_TURN_1

    # rotate 90 deg in steps
    for i in range(1, 20):
        p = (x, y, yaw + turn_sign * i * 0.1)
        lin, ang, state, _ = update(fsm, open_r, p, WIDTH)
        if state == STATE_TRAVERSE:
            yaw = p[2]
            break
        assert lin == 0.0
        assert math.copysign(1, ang) == turn_sign
    assert fsm.state == STATE_TRAVERSE

    # advance 0.75 m along new heading
    for i in range(1, 15):
        p = (x + i * 0.1 * math.cos(yaw), y + i * 0.1 * math.sin(yaw), yaw)
        lin, ang, state, _ = update(fsm, open_r, p, WIDTH)
        if state == STATE_TURN_2:
            x, y = p[0], p[1]
            break
        assert lin > 0 and ang == 0.0
    assert fsm.state == STATE_TURN_2

    # rotate the second 90 deg, same direction
    for i in range(1, 20):
        p = (x, y, yaw + turn_sign * i * 0.1)
        lin, ang, state, _ = update(fsm, open_r, p, WIDTH)
        if state == STATE_REACQUIRE:
            yaw = p[2]
            break
        assert math.copysign(1, ang) == turn_sign
    assert fsm.state == STATE_REACQUIRE
    return (x, y, yaw)


def reacquire_into_row(fsm, pose, result=None, meters=0.4, step=0.05):
    """Creep forward feeding an in-row view until REACQUIRE latches.

    The latch is confirmed over reacquire_confirm_distance METERS, so the
    robot has to actually move -- feeding frames from a parked pose never
    latches.
    """
    result = corridor_result() if result is None else result
    x, y, yaw = pose
    state = fsm.state
    for i in range(1, int(round(meters / step)) + 1):
        p = (x + i * step * math.cos(yaw), y + i * step * math.sin(yaw), yaw)
        _, _, state, _ = update(fsm, result, p, WIDTH)
        if state != STATE_REACQUIRE:
            return p, state
    return p, state


def test_full_transition_cycle_and_direction_flip():
    fsm = make_fsm(num_rows=3, first_turn_direction="left")
    pose, state, done = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR
    assert not done
    assert fsm.rows_driven == 1

    pose = run_headland(fsm, pose, turn_sign=+1)  # left turns

    # reacquire: creep forward with an in-row view until it latches
    pose, state = reacquire_into_row(fsm, pose)
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
    lin, ang, state, _ = update(fsm, open_result(), p, WIDTH)
    assert state == STATE_EXIT_CLEAR
    assert lin == pytest.approx(0.1)  # slower than the 0.15 cruise
    assert ang == 0.0


def test_num_rows_termination_skips_turn():
    fsm = make_fsm(num_rows=1)
    _, state, done = drive_row_to_exit(fsm)
    assert state == STATE_DONE
    assert done
    # latched
    lin, ang, state, done = update(fsm, corridor_result(), (99, 99, 0), WIDTH)
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
        lin, ang, state, done = update(fsm, open_r, p, WIDTH)
        if done:
            break
    assert state == STATE_DONE
    assert done


def test_maneuver_states_stop_without_odom():
    fsm = make_fsm()
    pose, _, _ = drive_row_to_exit(fsm)
    assert fsm.state == STATE_EXIT_CLEAR
    lin, ang, state, done = update(fsm, open_result(), None, WIDTH)
    assert (lin, ang) == (0.0, 0.0)
    assert state == STATE_EXIT_CLEAR  # holds state, doesn't advance


def test_controller_reset_on_new_row():
    fsm = make_fsm()
    pose, _, _ = drive_row_to_exit(fsm)
    pose = run_headland(fsm, pose, turn_sign=+1)
    resets_before = fsm._controller.reset_calls
    reacquire_into_row(fsm, pose)
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
        _, _, state, _ = update(fsm, open_r, p, WIDTH)
        if state == STATE_TURN_1:
            x, y = p[0], p[1]
            break
    assert fsm.state == STATE_TURN_1

    # rotate into TRAVERSE
    for i in range(1, 20):
        p = (x, y, yaw + i * 0.1)
        _, _, state, _ = update(fsm, open_r, p, WIDTH)
        if state == STATE_TRAVERSE:
            yaw = p[2]
            break
    assert fsm.state == STATE_TRAVERSE

    # 0.45 m along the leg: still traversing
    p = (x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw), yaw)
    _, _, state, _ = update(fsm, open_r, p, WIDTH)
    assert state == STATE_TRAVERSE
    # 0.55 m: past traverse_distance but short of row_spacing -> TURN_2
    p = (x + 0.55 * math.cos(yaw), y + 0.55 * math.sin(yaw), yaw)
    _, _, state, _ = update(fsm, open_r, p, WIDTH)
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
        _, _, state, _ = update(fsm, corridor, pose, WIDTH)
        assert state == STATE_FOLLOW_ROW
    for _ in range(3):  # blocked signature, debounced over 3 frames
        _, _, state, done = update(fsm, blocked, pose, WIDTH)
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
        lin, ang, state, _ = update(fsm, front, p, WIDTH)
        if state == STATE_BACKOUT_CLEAR:
            x, y = p[0], p[1]
            break
        assert lin < 0.0
    assert fsm.state == STATE_BACKOUT_CLEAR

    # keep reversing headland_clearance (1.0 m) past the row entrance
    for i in range(1, 20):
        p = (x - i * 0.1 * math.cos(yaw), y - i * 0.1 * math.sin(yaw), yaw)
        lin, ang, state, _ = update(fsm, front, p, WIDTH)
        if state == STATE_BACKOUT_TURN_1:
            x, y = p[0], p[1]
            break
        assert lin < 0.0 and ang == 0.0
    assert fsm.state == STATE_BACKOUT_TURN_1

    # first 90 deg of the lane change: current turn sign
    for i in range(1, 20):
        p = (x, y, yaw + turn_sign * i * 0.1)
        lin, ang, state, _ = update(fsm, front, p, WIDTH)
        if state == STATE_BACKOUT_TRAVERSE:
            yaw = p[2]
            break
        assert lin == 0.0
        assert math.copysign(1, ang) == turn_sign
    assert fsm.state == STATE_BACKOUT_TRAVERSE

    # sidestep one row spacing (0.75 m), forward
    for i in range(1, 15):
        p = (x + i * 0.1 * math.cos(yaw), y + i * 0.1 * math.sin(yaw), yaw)
        lin, ang, state, _ = update(fsm, front, p, WIDTH)
        if state == STATE_BACKOUT_TURN_2:
            x, y = p[0], p[1]
            break
        assert lin > 0 and ang == 0.0
    assert fsm.state == STATE_BACKOUT_TURN_2

    # second 90 deg COUNTER-rotates (S-shaped lane change)
    for i in range(1, 20):
        p = (x, y, yaw - turn_sign * i * 0.1)
        lin, ang, state, _ = update(fsm, front, p, WIDTH)
        if state == STATE_REACQUIRE:
            yaw = p[2]
            break
        assert math.copysign(1, ang) == -turn_sign
    assert fsm.state == STATE_REACQUIRE
    return (x, y, yaw)


def test_rear_exit_detector_mirrors_flank_margin():
    # The rear open-exit watcher must use the same flank-edge margin as the
    # front detector so backing out into open field stops the reverse leg
    # exactly as before.
    det = RowExitDetector(exit_flank_edge_margin=0.07)
    fsm = MissionFSM(StubController(), det, num_rows=3)
    assert fsm.rear_exit_detector.exit_flank_edge_margin == 0.07


def test_blocked_exit_enters_backout_not_exit_clear():
    fsm = make_fsm(num_rows=3)
    pose, state, done = drive_row_to_block(fsm)
    assert state == STATE_BACKOUT
    assert not done
    assert fsm.rows_driven == 0  # a blocked row does not count as driven
    assert len(fsm.blocked_events) == 1
    row, dist = fsm.blocked_events[0]
    assert row == 1  # reported index = attempt number (rows_driven + 1)
    assert dist == pytest.approx(3.0, abs=0.2)
    # next tick actually reverses
    lin, ang, state, _ = update(fsm, blocked_result(), pose, WIDTH)
    assert lin < 0.0
    assert state == STATE_BACKOUT


def test_backout_rear_steering_signs():
    fsm = MissionFSM(
        SteeringStubController(),
        RowExitDetector(
            exit_confirm_distance=0.2,
            blocked_confirm_seconds=1.0,
            min_in_row_distance=2.0,
        ),
        num_rows=3,
    )
    pose, _, _ = drive_row_to_block(fsm)
    front = blocked_result()
    # rear centerline LEFT of image center -> positive angular_z (turn left);
    # shift keeps the corridor overlapping the center column so the
    # centerline result stays valid
    lin, ang, _, _ = update(fsm, 
        front, pose, WIDTH, rear_centerline_result=corridor_result(shift=-20)
    )
    assert lin < 0.0
    assert ang > 0.0
    # rear centerline RIGHT of image center -> negative angular_z
    lin, ang, _, _ = update(fsm, 
        front, pose, WIDTH, rear_centerline_result=corridor_result(shift=20)
    )
    assert ang < 0.0


def test_backout_without_rear_result_reverses_straight():
    fsm = make_fsm(num_rows=3, backout_speed=0.12)
    pose, _, _ = drive_row_to_block(fsm)
    lin, ang, state, _ = update(fsm, blocked_result(), pose, WIDTH)
    assert (lin, ang) == (pytest.approx(-0.12), 0.0)
    assert state == STATE_BACKOUT


def test_backout_full_sequence_and_flip_suppression():
    fsm = make_fsm(num_rows=3, first_turn_direction="left")
    pose, _, _ = drive_row_to_block(fsm)  # row 1 blocked
    pose = run_backout(fsm, pose, turn_sign=+1)

    # reacquire the next row: turn sign must NOT flip (same world direction)
    pose, state = reacquire_into_row(fsm, pose)
    assert state == STATE_FOLLOW_ROW
    assert fsm._turn_sign == +1

    # the next physical row exits normally: headland still turns LEFT, and
    # the flip at the following reacquire happens again (no double
    # suppression). The blocked row did NOT count, so this open exit is the
    # first driven row.
    pose, state, _ = drive_row_to_exit(fsm, start=pose)
    assert state == STATE_EXIT_CLEAR
    assert fsm.rows_driven == 1
    pose = run_headland(fsm, pose, turn_sign=+1)
    pose, state = reacquire_into_row(fsm, pose)
    assert state == STATE_FOLLOW_ROW
    assert fsm._turn_sign == -1


def test_blocked_row_does_not_count_continues_to_next_row():
    # A blocked row is a FAILED row: it must not count toward num_rows.
    # Even with num_rows=1, a block on the first row backs out and S-turns
    # into the next physical row instead of ending the mission (regression,
    # 2026-07-22 field run: robot backed out then went straight to DONE).
    fsm = make_fsm(num_rows=1)
    pose, state, done = drive_row_to_block(fsm)
    assert state == STATE_BACKOUT
    assert not done
    front = blocked_result()
    x, y, yaw = pose
    for i in range(1, 60):
        p = (x - i * 0.1 * math.cos(yaw), y - i * 0.1 * math.sin(yaw), yaw)
        lin, ang, state, done = update(fsm, front, p, WIDTH)
        assert not done  # a block never ends the mission on its own
        if state == STATE_BACKOUT_TURN_1:
            break
    # backed fully out and started the S-turn into the next row -- NOT DONE
    assert fsm.state == STATE_BACKOUT_TURN_1
    assert fsm.rows_driven == 0
    assert fsm.blocked_events == [(1, pytest.approx(3.0, abs=0.2))]


def test_blocked_middle_row_still_requires_full_num_rows():
    # User's field scenario: num_rows=2, obstacle in row 2. The blocked row
    # must not satisfy the count -- the robot backs out, S-turns, and only
    # finishes after TWO successful (open-exit) rows.
    fsm = make_fsm(num_rows=2, first_turn_direction="left")
    # row 1 open exit: counts (rows_driven 1), headland into row 2
    pose, state, _ = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR
    assert fsm.rows_driven == 1
    pose = run_headland(fsm, pose, turn_sign=+1)
    pose, _ = reacquire_into_row(fsm, pose)
    assert fsm.state == STATE_FOLLOW_ROW

    # row 2 blocked: does NOT count, back out + S-turn into row 3
    pose, state, done = drive_row_to_block(fsm, start=pose)
    assert state == STATE_BACKOUT
    assert not done
    assert fsm.rows_driven == 1  # still only one successful row
    # after row 1's headland the boustrophedon sign has flipped to -1
    pose = run_backout(fsm, pose, turn_sign=-1)
    pose, _ = reacquire_into_row(fsm, pose)
    assert fsm.state == STATE_FOLLOW_ROW

    # the next physical row (row 3) open exit: NOW rows_driven reaches 2 and
    # the mission completes
    pose, state, done = drive_row_to_exit(fsm, start=pose)
    assert fsm.rows_driven == 2
    assert state == STATE_DONE
    assert done
    assert fsm.blocked_events == [(2, pytest.approx(3.0, abs=0.2))]


def test_backout_ends_early_when_rear_sees_open_field():
    # Regression (sim, 2026-07-21): d_block includes the pre-row approach
    # (row 1 starts at the spawn point), so odometry-only reversing
    # overshot the row entrance by meters and REACQUIRE never found the
    # next row. The rear camera seeing open field must end the reverse leg.
    fsm = make_fsm(num_rows=3)
    pose, _, _ = drive_row_to_block(fsm)  # backout target ~3 m
    front = blocked_result()
    open_rear = open_result()
    x, y, yaw = pose
    p = (x - 0.5 * math.cos(yaw), y - 0.5 * math.sin(yaw), yaw)
    # rear still shows a corridor: keep reversing
    lin, _, state, _ = update(fsm, 
        front, p, WIDTH, rear_centerline_result=corridor_result()
    )
    assert state == STATE_BACKOUT and lin < 0.0
    # Rear opens up. The rear watcher debounces in METERS too, so the robot
    # has to keep reversing through the open view -- but the leg still ends
    # far short of the 3 m odometry target.
    reversed_m = 0.5
    for _ in range(10):
        reversed_m += 0.1
        p = (x - reversed_m * math.cos(yaw), y - reversed_m * math.sin(yaw), yaw)
        _, _, state, _ = update(
            fsm, front, p, WIDTH, rear_centerline_result=open_rear
        )
        if state != STATE_BACKOUT:
            break
    assert fsm.state == STATE_BACKOUT_CLEAR
    assert reversed_m < 1.5   # far short of the ~3 m odometry bound


def test_backout_rear_corridor_keeps_reversing_to_odometry_bound():
    fsm = make_fsm(num_rows=3)
    pose, _, _ = drive_row_to_block(fsm)
    front = blocked_result()
    corridor = corridor_result()
    x, y, yaw = pose
    for i in range(1, 26):  # 2.5 m reversed, still under the ~3 m target
        p = (x - i * 0.1 * math.cos(yaw), y - i * 0.1 * math.sin(yaw), yaw)
        lin, _, state, _ = update(fsm, 
            front, p, WIDTH, rear_centerline_result=corridor
        )
        assert state == STATE_BACKOUT
        assert lin < 0.0


def test_backout_progress_reports_distance():
    fsm = make_fsm(num_rows=3)
    pose, _, _ = drive_row_to_block(fsm)
    reversed_m, target = fsm.backout_progress(pose)
    assert reversed_m == pytest.approx(0.0)
    assert target == pytest.approx(3.0, abs=0.2)
    x, y, yaw = pose
    p = (x - 0.5 * math.cos(yaw), y - 0.5 * math.sin(yaw), yaw)
    update(fsm, blocked_result(), p, WIDTH)
    reversed_m, _ = fsm.backout_progress(p)
    assert reversed_m == pytest.approx(0.5)
    assert fsm.backout_progress(None)[0] is None  # no odom -> unknown


def test_backout_states_stop_without_odom():
    fsm = make_fsm(num_rows=3)
    drive_row_to_block(fsm)
    lin, ang, state, done = update(fsm, blocked_result(), None, WIDTH)
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
    lin, ang, state, done = update(fsm, blocked_result(), pose, WIDTH)
    assert (lin, ang) == (0.0, 0.0)
    assert state == STATE_DONE
    assert done


# ------------------------------------ EXIT_CLEAR back-dating and revocation --
def near_flanked_result():
    """Far rows open, NEAREST row still corn-flanked -- the geometry of a
    false exit fired at a mid-row gap: the corridor widened, but there is
    still corn immediately beside the robot."""
    mask = np.full((HEIGHT, WIDTH), CLASS_TRAVERSABLE, dtype=np.uint8)
    mask[:20, :] = CLASS_SKY
    mask[85:, :40] = CLASS_OBSTACLE
    mask[85:, -40:] = CLASS_OBSTACLE
    return estimate_centerline(mask)


def far_corn_near_open_result():
    """Nearest row wide open, FAR rows blocked -- the view while genuinely
    exiting toward the corn block across the headland."""
    mask = np.full((HEIGHT, WIDTH), CLASS_TRAVERSABLE, dtype=np.uint8)
    mask[:20, :] = CLASS_SKY
    mask[20:80, :] = CLASS_OBSTACLE
    return estimate_centerline(mask)


def creep(fsm, result, pose, meters, step=0.05):
    """Drive `meters` forward from `pose` feeding `result`; returns the pose
    reached and the number of steps taken (stops early on a state change)."""
    x, y, yaw = pose
    state0 = fsm.state
    steps = int(round(meters / step))
    for i in range(1, steps + 1):
        p = (x + i * step * math.cos(yaw), y + i * step * math.sin(yaw), yaw)
        _, _, state, _ = update(fsm, result, p, WIDTH)
        if state != state0:
            return p, i
    return (x + steps * step * math.cos(yaw),
            y + steps * step * math.sin(yaw), yaw), steps


def test_exit_clear_back_dates_to_first_sighting():
    """headland_clearance must be measured from where the exit was FIRST seen,
    not from where it confirmed -- otherwise exit_confirm_distance is added on
    top and the robot overruns the row end by that much extra."""
    fsm = make_fsm(num_rows=3, headland_clearance=1.0)
    pose, state, _ = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR
    # 0.2 m of the clearance was already driven during confirmation.
    assert fsm._exit_clear_offset == pytest.approx(0.2, abs=0.05)
    _, steps = creep(fsm, open_result(), pose, meters=1.5)
    assert fsm.state == STATE_TURN_1
    assert steps * 0.05 == pytest.approx(0.8, abs=0.1)   # 1.0 - 0.2


def test_exit_clear_min_distance_is_always_driven():
    """When the confirmation distance already exceeds headland_clearance,
    back-dating must not collapse EXIT_CLEAR to zero travel."""
    fsm = MissionFSM(
        StubController(),
        RowExitDetector(
            exit_confirm_distance=1.5,
            blocked_confirm_seconds=1.0,
            min_in_row_distance=2.0,
        ),
        num_rows=3,
        headland_clearance=1.0,
        exit_clear_min_distance=0.2,
    )
    pose, state, _ = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR
    assert fsm._exit_clear_offset > 1.0        # already past the clearance
    _, steps = creep(fsm, open_result(), pose, meters=1.0)
    assert fsm.state == STATE_TURN_1
    assert steps * 0.05 == pytest.approx(0.2, abs=0.06)


def test_exit_revoked_when_near_row_stays_corn_flanked():
    """The 2026-07-24 field failure made survivable: a false open exit is
    withdrawn instead of committing the robot into the corn."""
    fsm = make_fsm(num_rows=3)
    pose, state, _ = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR and fsm.rows_driven == 1

    pose, _ = creep(fsm, near_flanked_result(), pose, meters=0.45)
    assert fsm.state == STATE_FOLLOW_ROW
    assert fsm.rows_driven == 0                 # the row was NOT driven
    assert len(fsm.revoked_exits) == 1

    # The detector must still be ARMED: the row was never left, so its entry
    # reference has to survive the revert (a naive _enter(FOLLOW_ROW) would
    # reset it and disarm the exit for another min_in_row_distance meters).
    _, _ = creep(fsm, open_result(), pose, meters=0.4)
    assert fsm.state == STATE_EXIT_CLEAR
    assert fsm.rows_driven == 1


def test_far_row_corn_across_headland_does_not_revoke():
    """A genuine exit sees the corn block on the far side of the headland in
    its FAR scan rows. Keying revocation on those (or on 'corn anywhere beside
    the corridor') would revoke every real exit; only the near row counts."""
    fsm = make_fsm(num_rows=3)
    pose, state, _ = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR
    creep(fsm, far_corn_near_open_result(), pose, meters=0.45)
    assert fsm.state in (STATE_EXIT_CLEAR, STATE_TURN_1)
    assert fsm.revoked_exits == []
    assert fsm.rows_driven == 1


def test_no_revocation_after_the_window_closes():
    fsm = make_fsm(num_rows=3, headland_clearance=2.0, exit_revoke_distance=0.3)
    pose, state, _ = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR
    pose, _ = creep(fsm, open_result(), pose, meters=0.4)   # past the window
    creep(fsm, near_flanked_result(), pose, meters=0.6)
    assert fsm.state == STATE_EXIT_CLEAR
    assert fsm.revoked_exits == []


def test_revocation_can_be_disabled():
    fsm = make_fsm(num_rows=3, exit_revoke_enabled=False)
    pose, _, _ = drive_row_to_exit(fsm)
    creep(fsm, near_flanked_result(), pose, meters=0.45)
    assert fsm.state != STATE_FOLLOW_ROW
    assert fsm.revoked_exits == []


# ------------------------------------------- separate exit-detector scan rows --
def test_exit_scan_rows_feed_the_detector_only():
    """With exit_scan_row_fractions configured, the node measures a second
    centerline for the detector. Steering must keep using the steering rows."""
    fsm = MissionFSM(
        SteeringStubController(),
        RowExitDetector(
            exit_confirm_distance=0.2,
            blocked_confirm_seconds=1.0,
            min_in_row_distance=2.0,
        ),
        num_rows=3,
    )
    steering_view = corridor_result(shift=20)   # off-center: demands a steer
    exit_view = open_result()
    assert steering_view.offset_norm > 0.0

    x, y, yaw = 0.0, 0.0, 0.0
    state = STATE_FOLLOW_ROW
    for i in range(1, 31):                      # 3 m in-row, both views normal
        p = (x + i * 0.1, y, yaw)
        _, ang, state, _ = update(fsm, steering_view, p, WIDTH)
    assert state == STATE_FOLLOW_ROW
    assert ang == pytest.approx(-steering_view.offset_norm)

    # Exit rows now see open field while the steering rows still see a row.
    for i in range(31, 40):
        p = (x + i * 0.1, y, yaw)
        _, ang, state, _ = update(
            fsm, steering_view, p, WIDTH, exit_centerline_result=exit_view
        )
        if state != STATE_FOLLOW_ROW:
            break
    assert fsm.state == STATE_EXIT_CLEAR        # detector read the exit rows


# ------------------------------------------------------------- REACQUIRE --
def low_camera_row_result():
    """In-row view from a LOW camera: corridor ~0.7 of the image width, corn
    on both sides. The old latch test (mean width < reacquire_max_width 0.6)
    can never pass on this, which is why REACQUIRE crept blind for 2 m."""
    mask = np.full((HEIGHT, WIDTH), CLASS_OBSTACLE, dtype=np.uint8)
    mask[:20, :] = CLASS_SKY
    margin = int(WIDTH * 0.15)          # corridor spans 70% of the width
    mask[20:, margin : WIDTH - margin] = CLASS_TRAVERSABLE
    return estimate_centerline(mask)


def enter_reacquire(fsm=None):
    """Drive a full row + headland so the FSM is sitting in REACQUIRE."""
    fsm = make_fsm(num_rows=3) if fsm is None else fsm
    pose, _, _ = drive_row_to_exit(fsm)
    pose = run_headland(fsm, pose, turn_sign=+1)
    assert fsm.state == STATE_REACQUIRE
    return fsm, pose


def test_low_camera_row_latches_reacquire():
    """The 2026-07-28 regression. Corridor width ~0.7 -- wider than the old
    0.6 bar -- but corn on both sides, so it is unambiguously a row."""
    fsm, pose = enter_reacquire()
    view = low_camera_row_result()
    widths = [
        w for w in normalized_corridor_widths(view, WIDTH) if w is not None
    ]
    assert min(widths) > 0.6          # the old rule could never have latched
    _, state = reacquire_into_row(fsm, pose, result=view)
    assert state == STATE_FOLLOW_ROW


def test_open_headland_does_not_latch_reacquire():
    fsm, pose = enter_reacquire()
    _, state = reacquire_into_row(fsm, pose, result=open_result(), meters=0.3)
    assert state == STATE_REACQUIRE


def test_low_traversable_fraction_no_longer_blocks_the_latch():
    """The old test also required centerline_result.valid, i.e.
    traversable_fraction >= 0.10. The sim headland measured 0.09, so REACQUIRE
    was failing on a second, unrelated count."""
    mask = np.full((HEIGHT, WIDTH), CLASS_OBSTACLE, dtype=np.uint8)
    mask[:20, :] = CLASS_SKY
    mask[88:, 60:140] = CLASS_TRAVERSABLE      # thin strip: frac well under 0.10
    view = estimate_centerline(mask)
    assert view.traversable_fraction < 0.10
    assert not view.valid

    fsm, pose = enter_reacquire()
    _, state = reacquire_into_row(fsm, pose, result=view)
    assert state == STATE_FOLLOW_ROW


def test_reacquire_confirms_over_distance_not_frames():
    fsm, pose = enter_reacquire()
    view = corridor_result()
    # well under reacquire_confirm_distance (0.12 m), however many frames
    _, state = reacquire_into_row(fsm, pose, result=view, meters=0.05, step=0.005)
    assert state == STATE_REACQUIRE
    _, state = reacquire_into_row(fsm, pose, result=view, meters=0.4)
    assert state == STATE_FOLLOW_ROW


def test_reacquire_latches_at_same_distance_at_any_frame_rate():
    latched_at = []
    for step in (0.04, 0.004):      # 2 Hz and 20 Hz at 0.08 m/s
        fsm, pose = enter_reacquire()
        view = corridor_result()
        x, y, yaw = pose
        i = 0
        while fsm.state == STATE_REACQUIRE:
            i += 1
            assert i * step < 1.0, "never latched"
            update(fsm, view, (x + i * step, y, yaw), WIDTH)
        latched_at.append(i * step)
    slow, fast = latched_at
    assert abs(slow - fast) <= 0.04 + 1e-9      # one coarse sample
    assert all(0.12 <= d <= 0.12 + 0.04 for d in latched_at)


def test_reacquire_steers_toward_an_off_centre_row():
    """REACQUIRE used to command angular_z = 0.0 for up to 2.0 m, holding
    whatever lateral error the headland turn left behind -- which is how it
    nearly drove into the corn."""
    fsm = MissionFSM(
        SteeringStubController(),
        RowExitDetector(
            exit_confirm_distance=0.2,
            blocked_confirm_seconds=1.0,
            min_in_row_distance=2.0,
        ),
        num_rows=3,
    )
    fsm, pose = enter_reacquire(fsm)
    off_centre = corridor_result(shift=20)
    assert off_centre.offset_norm > 0.0
    x, y, yaw = pose
    _, ang, state, _ = update(fsm, off_centre, (x + 0.02, y, yaw), WIDTH)
    assert state == STATE_REACQUIRE
    assert ang == pytest.approx(-off_centre.offset_norm)   # steering back


def test_reacquire_drives_straight_without_a_row():
    fsm, pose = enter_reacquire()
    x, y, yaw = pose
    lin, ang, state, _ = update(fsm, open_result(), (x + 0.02, y, yaw), WIDTH)
    assert state == STATE_REACQUIRE
    assert lin > 0.0 and ang == 0.0


def test_reacquire_steering_can_be_disabled():
    fsm = MissionFSM(
        SteeringStubController(),
        RowExitDetector(
            exit_confirm_distance=0.2,
            blocked_confirm_seconds=1.0,
            min_in_row_distance=2.0,
        ),
        num_rows=3,
        reacquire_steering_enabled=False,
    )
    fsm, pose = enter_reacquire(fsm)
    x, y, yaw = pose
    _, ang, _, _ = update(fsm, corridor_result(shift=20), (x + 0.02, y, yaw), WIDTH)
    assert ang == 0.0


# ------------------------------------------------------ operator override --
def test_resume_after_override_does_not_bank_the_manual_excursion():
    """While a human drives, the caller stops updating the FSM, so the robot
    can move metres. Every distance debounce works on deltas between samples,
    so without clearing them the first frame back would credit that whole
    excursion at once and fire a spurious exit the instant the operator lets
    go of the stick."""
    fsm = make_fsm(num_rows=3)
    corridor = corridor_result()
    open_r = open_result()

    # drive in-row far enough to arm the exit detector
    for i in range(1, 31):
        update(fsm, corridor, (0.1 * i, 0.0, 0.0), WIDTH)
    assert fsm.state == STATE_FOLLOW_ROW

    # operator takes over and drives 5 m by hand, then releases
    fsm.resume_after_override()

    # first frames back see open field at a pose 5 m further on
    _, _, state, _ = update(fsm, open_r, (8.0, 0.0, 0.0), WIDTH)
    assert state == STATE_FOLLOW_ROW           # the 5 m jump credited nothing
    assert fsm._detector.last_status.open_distance == 0.0
    # and it still exits normally once it genuinely drives through open field
    for i in range(1, 10):
        _, _, state, _ = update(fsm, open_r, (8.0 + 0.1 * i, 0.0, 0.0), WIDTH)
        if state != STATE_FOLLOW_ROW:
            break
    assert state == STATE_EXIT_CLEAR


def test_resume_after_override_keeps_the_detector_armed():
    """The robot never left the row, so the arming distance must survive --
    clearing the row-entry pose would disarm the exit for another 2 m."""
    fsm = make_fsm(num_rows=3)
    corridor = corridor_result()
    for i in range(1, 31):
        update(fsm, corridor, (0.1 * i, 0.0, 0.0), WIDTH)
    assert fsm._detector.last_status.open_armed

    fsm.resume_after_override()
    update(fsm, corridor, (3.1, 0.0, 0.0), WIDTH)
    assert fsm._detector.last_status.open_armed
