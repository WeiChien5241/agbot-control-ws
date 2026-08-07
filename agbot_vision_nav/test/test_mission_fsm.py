import math

import numpy as np
import pytest

from agbot_vision_nav.centerline_estimator import (
    CLASS_OBSTACLE,
    CLASS_SKY,
    CLASS_TRAVERSABLE,
    CenterlineResult,
    ScanRowResult,
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


# --------------------------------------------------------------------------
# Rear-camera-steered EXIT_CLEAR (2026-08-06).
#
# The front camera has just said "open field ahead"; the REAR camera is then
# looking straight back down the row being left, which is the best available
# reference for the row axis. The headland leg steers from it and turns only
# when the REAR view ALSO opens -- the tail has cleared the last plants.
# Replaces a blind odometry leg that clipped end-of-row corn in the field
# (2026-08-05) and left the robot mis-aligned for the following TURN_2.
# --------------------------------------------------------------------------


def make_rear_steered_fsm(controller=None, **kw):
    kw.setdefault("exit_clear_rear_steering", True)
    return MissionFSM(
        controller if controller is not None else StubController(),
        RowExitDetector(
            exit_confirm_distance=0.2,
            blocked_confirm_seconds=1.0,
            min_in_row_distance=2.0,
        ),
        num_rows=3,
        **kw
    )


# Projection constants for the synthetic views below, from the scan rows'
# ground distances (params.yaml scan_row_fractions [0.65, 0.78, 0.92] /
# weights [0.2, 0.3, 0.5], imaging roughly 3 / 2 / 1 m):
#     C1 = 0.2/3 + 0.3/2 + 0.5/1     lateral -> offset
#     C3 = 1/1 - 1/3                 lateral -> slope (far row minus near row)
# C2 (heading -> offset) is a free scale here; heading contributes NOTHING to
# slope, because with the camera on the row axis every point of that axis
# projects to the same column.
_C1 = 0.7167
_C3 = 0.6667


def rear_view(lateral=0.0, heading=0.0, clipped=False):
    """A synthetic REAR CenterlineResult for a known robot pose error.

    lateral > 0: robot LEFT of the row axis. heading > 0: yawed LEFT.

        rear offset = -C1*lateral + heading      (mirror flips the lateral
                                                  term only)
        rear slope  = +C3*lateral                (heading-free)

    Built by hand rather than from a mask because the sign table below has to
    vary the two error sources INDEPENDENTLY -- which is the whole point.

    The scan row it carries is narrow (no open-exit signature, so the leg does
    not terminate mid-test) and, unless `clipped`, has both edges INSIDE the
    image -- which is what makes the midpoint a real measurement and is
    required before the leg will steer at all.
    """
    row = (
        ScanRowResult(90, 0, 140, 70.0, -0.3, 1.0, 0.0)     # runs off the left
        if clipped
        else ScanRowResult(90, 60, 140, 100.0, 0.0, 0.0, 0.0)
    )
    return CenterlineResult(
        offset_norm=-_C1 * lateral + heading,
        slope_term=_C3 * lateral,
        valid=True,
        traversable_fraction=0.5,
        scan_rows=(row,),
        obstacle_fraction=0.0,
    )


def drive_exit_clear(fsm, pose, rear_result, steps, step=0.1, front_result=None):
    """Advance EXIT_CLEAR along its heading, feeding `rear_result` as the rear
    camera.

    The node infers on the REAR camera for the whole leg, so the FRONT result
    is None -- ⚠ None, never a blank result: "did not look" and "looked and
    saw no corridor beside me" mean opposite things to the revocation test.
    front_result stands in for the fallback frames the node takes when the
    rear camera stops delivering. Returns (last pose, last command, state).
    """
    x, y, yaw = pose
    lin = ang = 0.0
    state = fsm.state
    for i in range(1, steps + 1):
        p = (x + i * step * math.cos(yaw), y + i * step * math.sin(yaw), yaw)
        lin, ang, state, _ = update(
            fsm,
            front_result,
            p,
            WIDTH,
            rear_centerline_result=rear_result,
        )
        pose = p
        if state != STATE_EXIT_CLEAR:
            break
    return pose, (lin, ang), state


def steer_once_in_exit_clear(rear, **kw):
    """angular_z commanded by one rear tick of a rear-steered EXIT_CLEAR."""
    fsm = make_rear_steered_fsm(SteeringStubController(), **kw)
    pose, state, _ = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR
    _, (_, ang), _ = drive_exit_clear(fsm, pose, rear, 1)
    return ang


@pytest.mark.parametrize(
    "name,lateral,heading,expect_left",
    [
        # Off to one side, pointing straight down the row: steer back onto
        # the axis. Driving FORWARD, so left of the axis means turn right.
        ("left of axis", +0.30, 0.0, False),
        ("right of axis", -0.30, 0.0, True),
        # Yawed, sitting ON the axis: unwind the yaw. ⚠ These two are the
        # cases the old "negate the state" rule got backwards. The vanishing
        # point moves to image-RIGHT in the rear view for a yaw to the left --
        # the SAME direction as in the front view, because the camera rotates
        # with the robot -- so negating made heading feedback divergent, which
        # is what walked the robot off the row in sim (2026-08-07).
        ("yawed left", 0.0, +0.20, False),
        ("yawed right", 0.0, -0.20, True),
    ],
)
def test_exit_clear_rear_steering_signs(name, lateral, heading, expect_left):
    """⚠ THE sign tripwire, replacing test_..._sign_is_negated.

    No single sign flip can serve this leg: the 180-degree mirror inverts the
    LATERAL term but not the heading term, so the rear result is converted to
    the equivalent FRONT measurement (rear_to_front_state) instead. Getting it
    wrong steers the robot away from the row it is leaving, at the one moment
    nothing else is watching.
    """
    ang = steer_once_in_exit_clear(rear_view(lateral, heading))
    assert (ang > 0.0) == expect_left, (
        "%s: expected a turn %s, got angular_z=%+.3f"
        % (name, "left" if expect_left else "right", ang)
    )


def test_exit_clear_does_not_steer_on_a_border_clipped_rear_row():
    """⚠ The measurement gate, and the reason the leg ran away in sim.

    A corridor clipped by the image border averages a real corn boundary with
    an imaginary one at the frame edge, so its midpoint is fiction -- and in a
    HEADLAND that is the normal case, not the exception. Sim 2026-08-07 logged
    `edges=1.00/0.00` (corridor running off the left border) with
    `angular_z=+0.175` held for the entire leg: full authority, on nothing.
    """
    clipped = rear_view(lateral=+0.30, clipped=True)
    assert steer_once_in_exit_clear(clipped) == 0.0
    # Same pose error, measured properly, does steer.
    assert steer_once_in_exit_clear(rear_view(lateral=+0.30)) != 0.0


def test_exit_clear_and_backout_treat_the_same_rear_view_differently():
    """The two rear-camera legs are NOT sign-flips of each other, and the one
    that is field-proven is BACKOUT (2026-08-05).

    Reversing flips the lateral dynamics as well as the view, so BACKOUT feeds
    the rear result through unchanged. EXIT_CLEAR drives forward and must
    convert it. A pure heading error is where they agree (turn rate acts on
    heading regardless of which way the robot is moving); a pure lateral
    error is where they must differ.
    """
    lateral = rear_view(lateral=+0.30)    # robot left of the axis
    heading = rear_view(heading=+0.20)    # yawed left, on the axis

    def backout_steer(rear):
        fsm = MissionFSM(
            SteeringStubController(),
            RowExitDetector(
                exit_confirm_distance=0.2,
                blocked_confirm_seconds=1.0,
                min_in_row_distance=2.0,
            ),
            num_rows=3,
        )
        fsm._backout_target = 5.0
        fsm._enter(STATE_BACKOUT, (0.0, 0.0, 0.0))
        _, ang, _, _ = update(
            fsm, corridor_result(), (0.05, 0.0, 0.0), WIDTH,
            rear_centerline_result=rear,
        )
        return ang

    # Lateral: reversing toward the axis means turning the other way.
    assert steer_once_in_exit_clear(lateral) < 0.0
    assert backout_steer(lateral) > 0.0
    # Heading: both must unwind the same yaw error the same way.
    assert steer_once_in_exit_clear(heading) < 0.0
    assert backout_steer(heading) < 0.0


def test_exit_clear_does_not_turn_until_the_rear_view_opens():
    """headland_clearance no longer ends the leg in this mode. Driving well
    past it with corn still behind must NOT start the turn."""
    fsm = make_rear_steered_fsm(headland_clearance=1.0, exit_clear_max_distance=10.0)
    pose, state, _ = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR
    _, (lin, _), state = drive_exit_clear(fsm, pose, corridor_result(), 30)
    assert state == STATE_EXIT_CLEAR      # 3.0 m driven, 3x headland_clearance
    assert lin == pytest.approx(fsm.exit_clear_speed)


def test_exit_clear_turns_after_post_rear_distance():
    fsm = make_rear_steered_fsm(exit_clear_post_rear_distance=0.3)
    pose, _, _ = drive_row_to_exit(fsm)
    _, _, state = drive_exit_clear(fsm, pose, open_result(), 40)
    assert state == STATE_TURN_1


def test_exit_clear_drives_post_rear_distance_before_turning():
    """The rear signature fires when the CAMERA is level with the row end;
    there is still a bumper's worth of robot behind it."""
    fsm = make_rear_steered_fsm(
        exit_clear_post_rear_distance=0.5, exit_clear_max_distance=10.0
    )
    pose, _, _ = drive_row_to_exit(fsm)
    x, y, yaw = pose
    open_r = open_result()
    opened_at = None
    travelled = None
    for i in range(1, 40):
        p = (x + i * 0.1 * math.cos(yaw), y + i * 0.1 * math.sin(yaw), yaw)
        _, _, state, _ = update(
            fsm, None, p, WIDTH, rear_centerline_result=open_r
        )
        if opened_at is None and fsm._exit_clear_rear_open_at is not None:
            opened_at = fsm._exit_clear_rear_open_at
        if state == STATE_TURN_1:
            travelled = math.hypot(p[0] - x, p[1] - y)
            break
    assert opened_at is not None
    assert travelled is not None
    assert travelled >= opened_at + 0.5 - 1e-6


def test_exit_clear_falls_back_to_open_loop_without_rear_frames():
    """The rear camera fails SILENTLY (wrong topic, unplugged, dead). A row
    end is exactly where that would otherwise strand the robot, so a leg that
    never sees a single rear frame reverts to the headland_clearance
    terminator -- the pre-2026-08 behaviour."""
    fsm = make_rear_steered_fsm(headland_clearance=1.0)
    pose, state, _ = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR
    _, _, state = drive_exit_clear(fsm, pose, None, 30)
    assert state == STATE_TURN_1
    # and it turned at headland_clearance, not after exit_clear_max_distance
    assert fsm._exit_clear_rear_frames == 0


def test_exit_clear_max_distance_turns_rather_than_driving_on():
    """The ceiling on the rear-terminated leg.

    ⚠ It used to un-count the row and drop back to FOLLOW_ROW instead -- in
    the middle of a headland, where the exit detector has to re-arm over
    min_in_row_distance (2 m) of open field before it can fire again. That is
    how the sim robot reached the world edge (2026-08-07). The false-exit
    backstop is revocation, on the front frames the leg alternates with.
    """
    fsm = make_rear_steered_fsm(exit_clear_max_distance=1.0)
    pose, _, _ = drive_row_to_exit(fsm)
    assert fsm.rows_driven == 1
    _, _, state = drive_exit_clear(fsm, pose, corridor_result(), 20)
    assert state == STATE_TURN_1
    assert fsm.rows_driven == 1
    assert fsm.revoked_exits == []


def test_exit_clear_still_revokes_when_front_frames_arrive():
    """Revocation needs the FRONT camera, which this leg gives up -- but the
    node falls back to front frames whenever the rear camera stops delivering,
    and on those the ordinary backstop must still run.

    It cannot be rebuilt on the rear view: just after a GENUINE exit the rear
    near row legitimately still has corn on both sides, so a rear revocation
    would revoke every real exit.
    """
    fsm = make_rear_steered_fsm(exit_revoke_fail_distance=0.25)
    pose, _, _ = drive_row_to_exit(fsm)
    assert fsm.rows_driven == 1
    row_entry = fsm._row_entry_xy
    # Rear never opens, front says "still in a row": a false exit.
    _, _, state = drive_exit_clear(
        fsm, pose, corridor_result(), 20, front_result=corridor_result()
    )
    assert state == STATE_FOLLOW_ROW
    assert fsm.rows_driven == 0
    assert len(fsm.revoked_exits) == 1
    # ⚠ gotcha 1d: the row was never left, so its entry reference must
    # survive -- re-stamping it would disarm the exit detector for another
    # min_in_row_distance metres inside a row the robot is still in.
    assert fsm._row_entry_xy == row_entry
    assert fsm._entry_xy == row_entry


def test_rear_tick_must_not_be_read_as_revocation_evidence():
    """⚠ None and "an invalid result" are NOT interchangeable here.

    On a rear tick there is no front perception at all. Handing revocation a
    blank front result instead would read as "no corridor beside me", which
    it counts against the exit exactly like corn -- revoking every genuine
    exit within exit_revoke_fail_distance. Driving the leg with rear ticks
    only must therefore never revoke.
    """
    fsm = make_rear_steered_fsm(
        exit_revoke_fail_distance=0.1, exit_clear_max_distance=10.0
    )
    pose, _, _ = drive_row_to_exit(fsm)
    x, y, yaw = pose
    for i in range(1, 12):
        p = (x + i * 0.1 * math.cos(yaw), y + i * 0.1 * math.sin(yaw), yaw)
        _, _, state, _ = update(
            fsm, None, p, WIDTH, rear_centerline_result=corridor_result()
        )
    assert state == STATE_EXIT_CLEAR
    assert fsm.revoked_exits == []


def test_exit_clear_back_dates_the_rear_open_point():
    """The rear open point is where the streak BEGAN, not where it confirmed.

    Using the confirmation point added the detector's whole
    exit_confirm_distance to the leg, and exit_clear_post_rear_distance on top
    of that -- the same compounding overshoot the FRONT leg was fixed for in
    2026-07, and most of why this leg ran ~2 m in sim (2026-08-07).
    """
    fsm = make_rear_steered_fsm()
    fsm.exit_clear_detector.exit_confirm_distance = 0.4
    pose, _, _ = drive_row_to_exit(fsm)
    x, y, yaw = pose
    open_r = open_result()
    for i in range(1, 30):
        p = (x + i * 0.1 * math.cos(yaw), y + i * 0.1 * math.sin(yaw), yaw)
        _, _, state, _ = update(
            fsm, None, p, WIDTH, rear_centerline_result=open_r
        )
        if fsm._exit_clear_rear_open_at is not None:
            travelled = math.hypot(p[0] - x, p[1] - y)
            break
    assert fsm._exit_clear_rear_open_at is not None
    # Confirmed after 0.4 m of evidence, but credited to where it started.
    assert fsm._exit_clear_rear_open_at < travelled - 0.3


def test_exit_clear_confirms_faster_than_the_front_detector():
    """The rear check is a SECOND look at something the front camera already
    confirmed, so it does not re-pay the front's evidence distance.

    The front detector decides whether the row ended at all, from inside the
    row, where a mid-row gap looks identical to a row end -- 0.4 m of driving
    buys that. This one only asks where the robot is now. Charging 0.4 m again
    doubled the leg and nearly took the robot out of the world (sim,
    2026-08-08).
    """
    fsm = make_rear_steered_fsm(exit_clear_rear_confirm_distance=0.1)
    assert fsm.exit_clear_detector.exit_confirm_distance == 0.1
    assert fsm.exit_clear_detector.exit_confirm_distance < (
        fsm._detector.exit_confirm_distance
    )
    # Every OTHER threshold is still the front detector's.
    for name in (
        "exit_width_threshold",
        "exit_open_rows_required",
        "exit_flank_edge_margin",
        "exit_flank_min_clear_fraction",
        "exit_detect_min_frames",
        "exit_leak_ratio",
    ):
        assert getattr(fsm.exit_clear_detector, name) == getattr(
            fsm._detector, name
        ), name


def test_backout_keeps_the_front_confirm_distance():
    """⚠ The back-out reverse is field-proven (2026-08-05) and must NOT
    inherit a threshold shortened for the headland leg. Different watcher, on
    purpose."""
    fsm = make_rear_steered_fsm(exit_clear_rear_confirm_distance=0.05)
    assert fsm.rear_exit_detector is not fsm.exit_clear_detector
    assert fsm.rear_exit_detector.exit_confirm_distance == (
        fsm._detector.exit_confirm_distance
    )


def test_active_rear_detector_follows_the_state():
    """The HUD reads one number; it has to be the one explaining what the
    robot is waiting for right now."""
    fsm = make_rear_steered_fsm()
    fsm.state = STATE_EXIT_CLEAR
    assert fsm.active_rear_detector is fsm.exit_clear_detector
    fsm.state = STATE_BACKOUT
    assert fsm.active_rear_detector is fsm.rear_exit_detector


def test_rear_exit_detector_inherits_the_front_leak_ratio():
    """Every other threshold is copied from the front detector; leaving this
    one at the constructor default made the rear watcher silently diverge the
    moment exit_leak_ratio was tuned in params.yaml -- in the one place it
    would be hardest to notice."""
    fsm = MissionFSM(
        StubController(),
        RowExitDetector(exit_leak_ratio=0.25, exit_confirm_distance=0.2),
        num_rows=3,
    )
    assert fsm.rear_exit_detector.exit_leak_ratio == 0.25
    assert fsm.exit_clear_detector.exit_leak_ratio == 0.25


def test_exit_clear_open_loop_mode_is_unchanged():
    """exit_clear_rear_steering=false must reproduce the field-proven leg
    exactly: straight, headland_clearance-terminated, revocable."""
    fsm = make_rear_steered_fsm(
        exit_clear_rear_steering=False, headland_clearance=1.0
    )
    pose, state, _ = drive_row_to_exit(fsm)
    assert state == STATE_EXIT_CLEAR
    # A rear result is supplied and must be ignored outright: the leg steers
    # straight off the FRONT view and ends on headland_clearance.
    x, y, yaw = pose
    open_r = open_result()
    rear_off_centre = corridor_result(shift=20)
    for i in range(1, 30):
        p = (x + i * 0.1 * math.cos(yaw), y + i * 0.1 * math.sin(yaw), yaw)
        lin, ang, state, _ = update(
            fsm, open_r, p, WIDTH, rear_centerline_result=rear_off_centre
        )
        if state == STATE_TURN_1:
            travelled = math.hypot(p[0] - x, p[1] - y)
            break
        assert ang == 0.0, "open-loop leg must not steer"
        assert lin == pytest.approx(fsm.exit_clear_speed)
    assert state == STATE_TURN_1
    assert travelled <= 1.0 + 1e-6      # headland_clearance, back-dated
    assert fsm._exit_clear_rear_frames == 0     # rear detector never consulted


def test_rear_steered_exit_clear_does_not_disturb_the_backout_branch():
    """The back-out is the one rear-camera path with field validation behind
    it; enabling rear exit steering must not touch it."""
    fsm = make_rear_steered_fsm()
    corridor = corridor_result()
    blocked = blocked_result()
    pose = None
    for i in range(1, 31):
        pose = (i * 0.1, 0.0, 0.0)
        _, _, state, _ = update(fsm, corridor, pose, WIDTH)
    for i in range(31, 60):
        pose = (i * 0.1, 0.0, 0.0)
        _, _, state, _ = update(fsm, blocked, pose, WIDTH)
        if state == STATE_BACKOUT:
            break
    assert state == STATE_BACKOUT
    assert fsm.rows_driven == 0
    assert len(fsm.blocked_events) == 1
