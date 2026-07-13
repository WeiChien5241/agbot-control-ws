"""Headland-turn mission state machine for multi-row navigation.

Pure Python, no rospy. Composes the existing MPCRowController (in-row
driving) and RowExitDetector (end-of-row detection); does not modify either.

State graph (boustrophedon coverage):

  FOLLOW_ROW --open exit--> EXIT_CLEAR --> TURN_1 (90 deg)
      ^                                        |
      |                                        v
  REACQUIRE <-- TURN_2 (90 deg, same dir) <-- TRAVERSE (row_spacing m)

Blocked-row back-out branch (rows are too tight to turn around in; the
robot reverses out the end it entered, steering from the REAR camera):

  FOLLOW_ROW --blocked exit--> BACKOUT (reverse d_block, rear-steered)
      --> BACKOUT_CLEAR (reverse headland_clearance more, straight)
      --> BACKOUT_TURN_1 (90 deg, turn_sign)
      --> BACKOUT_TRAVERSE (row_spacing m, forward)
      --> BACKOUT_TURN_2 (90 deg, MINUS turn_sign: S-shaped lane change)
      --> REACQUIRE

  The next row is entered from the SAME end and traveled in the SAME world
  direction as the blocked attempt, so the boustrophedon turn-direction
  flip at the following REACQUIRE is suppressed exactly once. A blocked
  FINAL row (num_rows reached) still backs fully out of the crop
  (BACKOUT -> BACKOUT_CLEAR) before stopping in DONE.

  With backout_enabled=False (no rear camera), the BACKOUT states are
  unreachable: a blocked signal stops the robot in place and ends the
  mission in DONE, with the blocked row recorded in blocked_events.

- All maneuver segments are closed-loop on wheel odometry: turns integrate
  measured yaw until 90 degrees is swept; EXIT_CLEAR / TRAVERSE integrate
  measured displacement. The TRAVERSE leg is exactly one row spacing, which
  is what guarantees the robot re-enters a NEW row rather than the one it
  just exited -- no guessing.
- Turn direction alternates after every completed transition (two lefts
  into this row means two rights into the next).
- Termination: `num_rows` corridors driven (exit of the final corridor goes
  straight to DONE without turning), or -- with num_rows == 0 (unlimited) --
  when REACQUIRE finds no corridor within reacquire_max_distance, meaning
  there are no rows left. REACQUIRE failure is a terminal DONE in either
  mode: better to stop than wander an open field.
- Safety: if odometry is missing during any maneuver state, command zero.
  In FOLLOW_ROW the MPC's own invalid-frame handling still applies, and the
  ROS node's watchdog remains the outer net.
"""

import math

from agbot_vision_nav.row_exit_detector import (
    EXIT_NONE,
    EXIT_ROW_END_BLOCKED,
    normalized_corridor_widths,
)

STATE_FOLLOW_ROW = "FOLLOW_ROW"
STATE_EXIT_CLEAR = "EXIT_CLEAR"
STATE_TURN_1 = "TURN_1"
STATE_TRAVERSE = "TRAVERSE"
STATE_TURN_2 = "TURN_2"
STATE_REACQUIRE = "REACQUIRE"
STATE_DONE = "DONE"
STATE_BACKOUT = "BACKOUT"
STATE_BACKOUT_CLEAR = "BACKOUT_CLEAR"
STATE_BACKOUT_TURN_1 = "BACKOUT_TURN_1"
STATE_BACKOUT_TRAVERSE = "BACKOUT_TRAVERSE"
STATE_BACKOUT_TURN_2 = "BACKOUT_TURN_2"

# States of the blocked-row back-out branch (exported for the ROS node,
# which switches inference to the rear camera while in STATE_BACKOUT).
BACKOUT_STATES = frozenset(
    {
        STATE_BACKOUT,
        STATE_BACKOUT_CLEAR,
        STATE_BACKOUT_TURN_1,
        STATE_BACKOUT_TRAVERSE,
        STATE_BACKOUT_TURN_2,
    }
)


def _wrap_angle(a):
    """Wrap to (-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class MissionFSM:
    """Multi-row mission controller: in-row MPC + odometry headland turns."""

    def __init__(
        self,
        controller,
        detector,
        num_rows=3,
        first_turn_direction="left",
        row_spacing=0.75,
        headland_clearance=1.0,
        turn_rate=0.4,
        yaw_tolerance_deg=5.0,
        reacquire_speed=0.08,
        reacquire_max_width=0.6,
        reacquire_frames=3,
        reacquire_max_distance=1.5,
        backout_speed=0.10,
        backout_enabled=True,
        exit_clear_speed=0.10,
    ):
        if first_turn_direction not in ("left", "right"):
            raise ValueError("first_turn_direction must be 'left' or 'right'")
        self._controller = controller
        self._detector = detector
        self.num_rows = num_rows
        self.row_spacing = row_spacing
        self.headland_clearance = headland_clearance
        self.turn_rate = abs(turn_rate)
        self.yaw_tolerance = math.radians(yaw_tolerance_deg)
        self.reacquire_speed = reacquire_speed
        self.reacquire_max_width = reacquire_max_width
        self.reacquire_frames = reacquire_frames
        self.reacquire_max_distance = reacquire_max_distance
        self.backout_speed = abs(backout_speed)
        self.backout_enabled = backout_enabled
        # EXIT_CLEAR runs slower than cruise: the post-exit leg is where
        # overshoot (world edge, headland obstacles) hurts most.
        self.exit_clear_speed = abs(exit_clear_speed)

        # +1 = left (positive angular.z, REP-103), -1 = right
        self._turn_sign = 1 if first_turn_direction == "left" else -1

        self.state = STATE_FOLLOW_ROW
        self.rows_driven = 0
        self.blocked_events = []   # (row_index, distance_m) per blocked row
        self._entry_xy = None      # (x, y) at state entry, for distance legs
        self._last_yaw = None      # previous yaw sample, for sweep integration
        self._swept = 0.0          # accumulated yaw swept in current turn
        self._reacquire_hits = 0   # consecutive corridor-looking frames
        self._backout_target = None    # meters to reverse in BACKOUT
        self._done_after_backout = False
        self._suppress_flip = False    # skip one turn-sign flip at REACQUIRE

    # ------------------------------------------------------------ helpers --
    def _enter(self, state, odom_pose):
        self.state = state
        self._entry_xy = (odom_pose[0], odom_pose[1]) if odom_pose else None
        self._last_yaw = odom_pose[2] if odom_pose else None
        self._swept = 0.0
        self._reacquire_hits = 0
        if state == STATE_FOLLOW_ROW:
            self._controller.reset()
            self._detector.reset()
        elif state == STATE_BACKOUT:
            # Clear the MPC rate-limiter history from forward driving before
            # the controller starts steering the reverse leg.
            self._controller.reset()

    def _distance_from_entry(self, odom_pose):
        if odom_pose is None or self._entry_xy is None:
            return None
        dx = odom_pose[0] - self._entry_xy[0]
        dy = odom_pose[1] - self._entry_xy[1]
        return math.hypot(dx, dy)

    def _integrate_yaw(self, odom_pose):
        """Accumulate swept yaw across samples (wrap-safe); returns |swept|."""
        if odom_pose is None:
            return abs(self._swept)
        yaw = odom_pose[2]
        if self._last_yaw is not None:
            self._swept += _wrap_angle(yaw - self._last_yaw)
        self._last_yaw = yaw
        return abs(self._swept)

    def _corridor_looks_like_row(self, centerline_result, image_width):
        if not centerline_result.valid:
            return False
        widths = [
            w
            for w in normalized_corridor_widths(centerline_result, image_width)
            if w is not None
        ]
        if not widths:
            return False
        mean_width = sum(widths) / len(widths)
        return mean_width < self.reacquire_max_width

    # ------------------------------------------------------------- update --
    def update(self, centerline_result, odom_pose, image_width,
               rear_centerline_result=None):
        """Advance one tick. Returns (linear_x, angular_z, state, done).

        Args:
            centerline_result: CenterlineResult for the current FRONT frame.
            odom_pose: (x, y, yaw) from odometry, or None if unavailable.
            image_width: mask width in pixels.
            rear_centerline_result: CenterlineResult from the rear camera,
                or None. Only used in STATE_BACKOUT; the sign conventions of
                the front controller apply unchanged (the 180-deg image
                mirror and the reversed motion cancel exactly).
        """
        if self.state == STATE_FOLLOW_ROW:
            # Lazily record the row-entry pose (covers mission start, where
            # the initial state was set without an _enter() transition, and
            # any start before the first odometry message arrived).
            if self._entry_xy is None and odom_pose is not None:
                self._entry_xy = (odom_pose[0], odom_pose[1])
            linear_x, angular_z = self._controller.compute(
                centerline_result.offset_norm,
                centerline_result.slope_term,
                centerline_result.valid,
            )
            exit_signal = self._detector.update(
                centerline_result, image_width, self._distance_from_entry(odom_pose)
            )
            if exit_signal != EXIT_NONE:
                self.rows_driven += 1
                mission_complete = (
                    self.num_rows > 0 and self.rows_driven >= self.num_rows
                )
                if exit_signal == EXIT_ROW_END_BLOCKED:
                    # Blocked ahead (mid-row obstacle or crop wall at the row
                    # end): reverse out the end we entered. d_block is never
                    # None here -- the detector only fires when armed, which
                    # requires odometry.
                    d_block = self._distance_from_entry(odom_pose)
                    self.blocked_events.append((self.rows_driven, d_block))
                    if not self.backout_enabled:
                        # No rear camera: nothing safe left to do. Stop in
                        # place and end the mission; the blocked row is in
                        # blocked_events for the final report.
                        self._enter(STATE_DONE, odom_pose)
                        return 0.0, 0.0, self.state, True
                    self._backout_target = d_block
                    self._done_after_backout = mission_complete
                    self._suppress_flip = True
                    self._enter(STATE_BACKOUT, odom_pose)
                    return 0.0, 0.0, self.state, False
                if mission_complete:
                    self._enter(STATE_DONE, odom_pose)
                    return 0.0, 0.0, self.state, True
                self._enter(STATE_EXIT_CLEAR, odom_pose)
                return 0.0, 0.0, self.state, False
            return linear_x, angular_z, self.state, False

        if self.state == STATE_DONE:
            return 0.0, 0.0, self.state, True

        # All remaining states are odometry-closed-loop maneuvers.
        if odom_pose is None:
            return 0.0, 0.0, self.state, False

        if self.state == STATE_EXIT_CLEAR:
            if self._distance_from_entry(odom_pose) >= self.headland_clearance:
                self._enter(STATE_TURN_1, odom_pose)
                return 0.0, 0.0, self.state, False
            return self.exit_clear_speed, 0.0, self.state, False

        if self.state == STATE_BACKOUT:
            if self._distance_from_entry(odom_pose) >= self._backout_target:
                self._enter(STATE_BACKOUT_CLEAR, odom_pose)
                return 0.0, 0.0, self.state, False
            # Steer from the rear camera; the front controller's sign
            # conventions hold unchanged while reversing (mirror + reversed
            # motion cancel). Without a usable rear result, reverse straight
            # -- the leg is odometry-bounded either way.
            angular_z = 0.0
            if rear_centerline_result is not None and rear_centerline_result.valid:
                _, angular_z = self._controller.compute(
                    rear_centerline_result.offset_norm,
                    rear_centerline_result.slope_term,
                    True,
                )
            return -self.backout_speed, angular_z, self.state, False

        if self.state == STATE_BACKOUT_CLEAR:
            if self._distance_from_entry(odom_pose) >= self.headland_clearance:
                if self._done_after_backout:
                    self._enter(STATE_DONE, odom_pose)
                    return 0.0, 0.0, self.state, True
                self._enter(STATE_BACKOUT_TURN_1, odom_pose)
                return 0.0, 0.0, self.state, False
            return -self.backout_speed, 0.0, self.state, False

        # 90-degree in-place turns: {state: (sign multiplier, next state)}.
        # BACKOUT_TURN_2 counter-rotates (S-shaped lane change into the next
        # row, entered from the same end the blocked row was).
        turn_table = {
            STATE_TURN_1: (1, STATE_TRAVERSE),
            STATE_TURN_2: (1, STATE_REACQUIRE),
            STATE_BACKOUT_TURN_1: (1, STATE_BACKOUT_TRAVERSE),
            STATE_BACKOUT_TURN_2: (-1, STATE_REACQUIRE),
        }
        if self.state in turn_table:
            sign_mult, next_state = turn_table[self.state]
            swept = self._integrate_yaw(odom_pose)
            if swept >= math.pi / 2.0 - self.yaw_tolerance:
                self._enter(next_state, odom_pose)
                return 0.0, 0.0, self.state, False
            return (
                0.0,
                sign_mult * self._turn_sign * self.turn_rate,
                self.state,
                False,
            )

        if self.state in (STATE_TRAVERSE, STATE_BACKOUT_TRAVERSE):
            next_turn = (
                STATE_TURN_2 if self.state == STATE_TRAVERSE else STATE_BACKOUT_TURN_2
            )
            if self._distance_from_entry(odom_pose) >= self.row_spacing:
                self._enter(next_turn, odom_pose)
                return 0.0, 0.0, self.state, False
            return self._controller.linear_x_cruise, 0.0, self.state, False

        if self.state == STATE_REACQUIRE:
            if self._corridor_looks_like_row(centerline_result, image_width):
                self._reacquire_hits += 1
            else:
                self._reacquire_hits = 0
            if self._reacquire_hits >= self.reacquire_frames:
                # New row acquired: flip turn direction for the next headland
                # -- unless this row was reached via a back-out, in which case
                # it is traveled in the SAME world direction as the blocked
                # attempt and the flip must be skipped exactly once.
                if self._suppress_flip:
                    self._suppress_flip = False
                else:
                    self._turn_sign = -self._turn_sign
                self._enter(STATE_FOLLOW_ROW, odom_pose)
                return 0.0, 0.0, self.state, False
            if self._distance_from_entry(odom_pose) >= self.reacquire_max_distance:
                # No corridor where a row should be: no rows left. Stop.
                self._enter(STATE_DONE, odom_pose)
                return 0.0, 0.0, self.state, True
            return self.reacquire_speed, 0.0, self.state, False

        raise RuntimeError("unknown state: %s" % self.state)
