"""Headland-turn mission state machine for multi-row navigation.

Pure Python, no rospy. Composes the existing MPCRowController (in-row
driving) and RowExitDetector (end-of-row detection); does not modify either.

State graph (boustrophedon coverage):

  FOLLOW_ROW --open exit--> EXIT_CLEAR --> TURN_1 (90 deg)
      ^                    (rear-steered;         |
      |                     ends when the REAR    |
      |                     camera sees the row   v
  REACQUIRE <-- TURN_2 <-- TRAVERSE   open behind the robot)

Blocked-row back-out branch (rows are too tight to turn around in; the
robot reverses out the end it entered, steering from the REAR camera):

  FOLLOW_ROW --blocked exit--> BACKOUT (reverse, rear-steered; ends when
          the REAR camera sees the row open up behind the robot, bounded
          above by the odometry distance d_block recorded since row entry
          -- d_block alone overshoots badly on row 1, where FOLLOW_ROW
          started at the spawn point well before the row entrance)
      --> BACKOUT_CLEAR (reverse headland_clearance more, straight)
      --> BACKOUT_TURN_1 (90 deg, turn_sign)
      --> BACKOUT_TRAVERSE (traverse_distance m, forward)
      --> BACKOUT_TURN_2 (90 deg, MINUS turn_sign: S-shaped lane change)
      --> REACQUIRE

  The next row is entered from the SAME end and traveled in the SAME world
  direction as the blocked attempt, so the boustrophedon turn-direction
  flip at the following REACQUIRE is suppressed exactly once. A blocked
  row does NOT count toward num_rows (a failed row is not a driven row):
  the robot always backs out and continues to the next physical row, so a
  block never ends the mission on its own. The mission ends only when
  num_rows SUCCESSFUL (open-exit) rows have been driven, or when REACQUIRE
  finds no further row.

  With backout_enabled=False (no rear camera), the BACKOUT states are
  unreachable: a blocked signal stops the robot in place and ends the
  mission in DONE, with the blocked row recorded in blocked_events.

- EXIT_CLEAR has TWO modes, chosen by exit_clear_rear_steering:

  * REAR-STEERED (preferred, needs the rear camera). The front camera has
    just said "open field ahead"; the REAR camera is now looking straight
    back down the row being left, which is the best available reference for
    the row axis. So the headland leg steers from the rear view, and the turn
    starts only when the REAR camera ALSO sees open field -- positive
    evidence that the robot's tail has cleared the last plants. Blind
    odometry clipped the end-of-row corn in the field (2026-08-05) and left
    the robot mis-aligned, which is also what put its nose into a plant at
    the following TURN_2.
    ⚠ SIGN: the rear result is converted to the equivalent FRONT measurement
    by rear_to_front_state() before the controller sees it. It is NOT a
    negation -- the mirror flips the lateral term but not the heading one, so
    negating inverts heading feedback and drives the robot off the row axis
    (sim, 2026-08-07). BACKOUT is different again and passes the rear result
    through unchanged; see rear_to_front_state's docstring for why all three
    are consistent.
    ⚠ headland_clearance does NOT terminate this mode -- the rear camera
    does, bounded above by exit_clear_max_distance. It remains the terminator
    of the open-loop mode below.

  * OPEN-LOOP (the original, and the automatic fallback whenever no usable
    rear frame arrives during the leg -- a dead or mis-topiced rear camera
    must not strand the robot at a row end). Drives straight for
    headland_clearance, back-dated to where the exit was first seen so the
    detector's confirmation distance is not added on top (0.4 + 0.75 =
    1.15 m of overshoot), with exit_clear_min_distance always driven after
    the exit confirmed.

- A false open exit is caught by REVOCATION: for the first
  exit_revoke_distance meters, a nearest FRONT scan row that stays
  corn-flanked for a continuous exit_revoke_fail_distance falls back to
  FOLLOW_ROW and un-counts the row (the 2026-07-24 field failure drove into
  the corn because the transition was a one-way commit). It needs the front
  camera, so in rear-steered mode it only runs on the fallback frames that
  arrive when the rear camera is not delivering; there, exit_clear_max_distance
  is the bound instead -- drive that far without the rear view opening and the
  robot turns anyway. Revocation cannot be rebuilt on the rear view: just
  after a GENUINE exit the rear near row legitimately still has corn on both
  sides, so a rear revocation would revoke every real exit.
- All maneuver segments are closed-loop on wheel odometry: turns integrate
  measured yaw until 90 degrees is swept; EXIT_CLEAR / TRAVERSE integrate
  measured displacement. The TRAVERSE leg is traverse_distance (default
  0.6 m, deliberately a bit under the 0.75 m row spacing so the nose stays
  clear of the next row's corn at TURN_2); the small undershoot is closed
  by REACQUIRE + the MPC once the new row is entered.
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
    EXIT_ROW_END_OPEN,
    RowExitDetector,
    nearest_row_corridor_is_bounded,
    nearest_row_flank_clear,
)

STATE_FOLLOW_ROW = "FOLLOW_ROW"
STATE_EXIT_CLEAR = "EXIT_CLEAR"
STATE_TURN_1 = "TURN_1"
STATE_TRAVERSE = "TRAVERSE"
STATE_TURN_2 = "TURN_2"
STATE_REACQUIRE = "REACQUIRE"
STATE_NUDGE = "NUDGE"
# Reported instead of STATE_REACQUIRE while REACQUIRE is centring rather than
# searching. A LABEL, not a state: self.state stays STATE_REACQUIRE, so every
# transition and test keyed on it is unchanged. It exists because the two
# phases are the two halves of the defect this state was rebuilt to fix, and
# a CSV that renders them identically cannot show which half is slow.
STATE_REACQUIRE_CENTER = "REACQUIRE (CENTER)"
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


def rear_to_front_state(offset_norm, slope_term, kappa):
    """Re-express a REAR-camera centerline result as the equivalent FRONT one.

    ⚠ This replaces the "negate the state" rule, which was wrong. Write the
    two measurements in terms of the robot's lateral offset e (positive =
    robot LEFT of the row axis) and heading error theta (positive = yawed
    left). Projecting the row axis through a pinhole camera on a mirrored
    mount gives:

        front:  offset = +c1*e + c2*theta      slope = -c3*e
        rear:   offset = -c1*e + c2*theta      slope = +c3*e

    The 180-degree mirror flips the sign of the LATERAL term but NOT of the
    heading term: a yaw to the left moves the vanishing point to image-RIGHT
    in both views, because both cameras rotate with the robot. So no single
    sign flip can convert one into the other -- negating fixes e and inverts
    theta (which is what drove the robot off the row in sim, 2026-08-07),
    while not negating does the reverse.

    BACKOUT is unaffected and still feeds the rear result through UNCHANGED:
    reversing also flips the sign of the lateral dynamics (e_dot = v*sin
    theta with v < 0), so both terms line up there by themselves.

    Note slope is a THETA-FREE readout of e, which is what makes the
    inversion possible at all:

        e     = slope_rear / c3
        theta = (offset_rear + c1*e) / c2
        =>  offset_front = offset_rear + 2*(c1/c3) * slope_rear
            slope_front  = -slope_rear

    kappa = 2*c1/c3 depends only on the RATIO of the scan rows' ground
    distances, which is identical for both cameras (the rear mount is an
    exact geometric mirror of the front one), so it is not a per-robot
    calibration. See exit_clear_rear_offset_gain for the default's derivation.
    """
    return offset_norm + kappa * slope_term, -slope_term


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
        traverse_distance=0.6,
        traverse_speed=None,
        headland_clearance=1.0,
        turn_rate=0.4,
        backout_turn_rate=None,
        yaw_tolerance_deg=5.0,
        reacquire_speed=0.08,
        reacquire_confirm_distance=0.12,
        reacquire_steering_enabled=True,
        reacquire_max_distance=1.5,
        reacquire_center_tolerance=0.06,
        reacquire_center_confirm_distance=0.10,
        reacquire_center_speed=0.15,
        reacquire_center_max_distance=0.5,
        nudge_max_attempts=2,
        nudge_distance=0.12,
        nudge_speed=0.08,
        nudge_trigger_seconds=0.5,
        nudge_min_corridor_rows=2,
        nudge_min_healthy_distance=0.5,
        backout_speed=0.10,
        backout_enabled=True,
        exit_clear_speed=0.10,
        exit_revoke_enabled=True,
        exit_revoke_distance=0.5,
        exit_revoke_fail_distance=0.25,
        exit_clear_min_distance=0.2,
        exit_clear_rear_steering=False,
        exit_clear_post_rear_distance=0.2,
        exit_clear_max_distance=1.5,
        exit_clear_rear_offset_gain=2.0,
        exit_clear_rear_confirm_distance=0.1,
    ):
        if first_turn_direction not in ("left", "right"):
            raise ValueError("first_turn_direction must be 'left' or 'right'")
        self._controller = controller
        self._detector = detector
        # Rear-view open-exit watcher for BACKOUT: reversing ends as soon as
        # the rear camera sees the corridor widen to open field (the robot
        # has actually left the row), instead of always unwinding the full
        # odometry distance. Same open thresholds as the front detector, but
        # armed immediately -- the exit behind may be arbitrarily close.
        # Public: the ROS node renders its status on the debug HUD.
        self.rear_exit_detector = RowExitDetector(
            exit_width_threshold=detector.exit_width_threshold,
            exit_confirm_distance=detector.exit_confirm_distance,
            min_in_row_distance=0.0,
            exit_open_rows_required=detector.exit_open_rows_required,
            exit_flank_edge_margin=detector.exit_flank_edge_margin,
            exit_flank_min_clear_fraction=detector.exit_flank_min_clear_fraction,
            exit_detect_min_frames=detector.exit_detect_min_frames,
            # Must be copied like the rest: leaving it at the constructor
            # default made the rear detector silently diverge from the front
            # one the moment exit_leak_ratio was tuned in params.yaml, and the
            # rear leg is exactly where that would be hardest to notice.
            exit_leak_ratio=detector.exit_leak_ratio,
        )
        # Rear-view open-exit watcher for EXIT_CLEAR. Same detector, same
        # thresholds, ONE deliberate difference: a much shorter confirmation
        # distance.
        #
        # The two are answering different questions. The FRONT detector is
        # deciding whether the row has ended AT ALL, from inside the row, with
        # nothing else to corroborate it -- a mid-row gap looks the same as a
        # row end, so it pays 0.4 m of driving for the evidence. By the time
        # this one runs, that question has already been answered and paid for.
        # All it is asked is "has my tail passed the last plants yet", which is
        # a geometric fact about where the robot is, not a judgement about what
        # kind of place it is in. Charging another 0.4 m for it made the leg
        # twice as long as it needed to be (sim, 2026-08-08: the robot nearly
        # left the world waiting for the confirmation of something the front
        # camera had already confirmed).
        #
        # exit_detect_min_frames still applies, so no single flickery frame can
        # fire it, and exit_clear_post_rear_distance is the deliberate margin
        # for turning slightly early. That is the right place to pay for
        # caution here -- it is measured from the row end, whereas confirmation
        # distance is dead time.
        #
        # ⚠ Kept SEPARATE from rear_exit_detector on purpose: that one
        # terminates the BACKOUT reverse, which is field-proven (2026-08-05)
        # and must not inherit a threshold change made for the headland leg.
        self.exit_clear_detector = RowExitDetector(
            exit_width_threshold=detector.exit_width_threshold,
            exit_confirm_distance=exit_clear_rear_confirm_distance,
            min_in_row_distance=0.0,
            exit_open_rows_required=detector.exit_open_rows_required,
            exit_flank_edge_margin=detector.exit_flank_edge_margin,
            exit_flank_min_clear_fraction=detector.exit_flank_min_clear_fraction,
            exit_detect_min_frames=detector.exit_detect_min_frames,
            exit_leak_ratio=detector.exit_leak_ratio,
        )
        self.num_rows = num_rows
        self.row_spacing = row_spacing
        # Length of the TRAVERSE/BACKOUT_TRAVERSE leg. Intentionally SHORTER
        # than row_spacing: stopping ~0.15 m before the next row's centerline
        # keeps the nose away from the second row's corn at TURN_2 (field
        # near-miss, 2026-07); REACQUIRE + the MPC close the remaining
        # lateral offset.
        self.traverse_distance = traverse_distance
        # Speed of the TRAVERSE/BACKOUT_TRAVERSE leg. It used to BE
        # linear_x_cruise, read straight off the controller -- which coupled a
        # blind odometry leg to the operator's in-row speed knob, so a 0.9 m/s
        # test run also crossed the headland at 0.9 m/s. None keeps the old
        # behaviour so a caller that never heard of this knob is unchanged.
        self.traverse_speed = (
            abs(traverse_speed) if traverse_speed is not None else None
        )
        self.headland_clearance = headland_clearance
        self.turn_rate = abs(turn_rate)
        # The back-out S-turn starts next to whatever blocked the robot and has
        # no validation at the headland turns' raised rate, so it keeps its
        # own. None = share turn_rate (pre-2026-09-12 behaviour).
        self.backout_turn_rate = (
            abs(backout_turn_rate) if backout_turn_rate is not None
            else abs(turn_rate)
        )
        self.yaw_tolerance = math.radians(yaw_tolerance_deg)
        self.reacquire_speed = reacquire_speed
        # Meters of sustained in-row view to latch the new row. 0.12 m is the
        # field-proven 3 frames at 2 Hz and 0.08 m/s, expressed so it means the
        # same thing at any inference rate.
        self.reacquire_confirm_distance = reacquire_confirm_distance
        # Steer while creeping instead of driving dead straight: REACQUIRE
        # could otherwise cover up to reacquire_max_distance misaligned, which
        # is how it nearly put the robot into the corn (sim, 2026-07-28).
        self.reacquire_steering_enabled = reacquire_steering_enabled
        self.reacquire_max_distance = reacquire_max_distance
        # REACQUIRE phase B: latching a row and being centred in it are two
        # different events, and treating them as one is what handed FOLLOW_ROW
        # up to 0.26 of offset_norm at every headland (field, 2026-09-09).
        self.reacquire_center_tolerance = abs(reacquire_center_tolerance)
        self.reacquire_center_confirm_distance = reacquire_center_confirm_distance
        self.reacquire_center_speed = abs(reacquire_center_speed)
        self.reacquire_center_max_distance = reacquire_center_max_distance
        # Occlusion nudge. A leaf on the lens and a crop wall produce the same
        # blocked signature; the only thing that separates them is moving. 0
        # attempts disables the mechanism entirely.
        self.nudge_max_attempts = max(0, int(nudge_max_attempts))
        self.nudge_distance = abs(nudge_distance)
        self.nudge_speed = abs(nudge_speed)
        self.nudge_trigger_seconds = abs(nudge_trigger_seconds)
        self.nudge_min_corridor_rows = max(0, int(nudge_min_corridor_rows))
        self.nudge_min_healthy_distance = abs(nudge_min_healthy_distance)
        self.backout_speed = abs(backout_speed)
        self.backout_enabled = backout_enabled
        # EXIT_CLEAR runs slower than cruise: the post-exit leg is where
        # overshoot (world edge, headland obstacles) hurts most.
        self.exit_clear_speed = abs(exit_clear_speed)
        # EXIT_CLEAR is revocable for its first exit_revoke_distance meters: a
        # false open exit then costs a steering wobble instead of a collision.
        # The test looks at the NEAREST scan row only -- the ground immediately
        # beside the robot. The far rows legitimately see the corn block across
        # the headland during a genuine exit, so keying revocation on them (or
        # on "any corn anywhere beside the corridor") would revoke every real
        # exit. Mid-row, the near row is corn-flanked again within ~0.15 m.
        self.exit_revoke_enabled = exit_revoke_enabled
        self.exit_revoke_distance = exit_revoke_distance
        self.exit_revoke_fail_distance = exit_revoke_fail_distance
        # Always drive at least this far after the exit confirms, even when
        # back-dating has already consumed all of headland_clearance.
        self.exit_clear_min_distance = exit_clear_min_distance
        # Steer the headland leg from the REAR camera and end it on the rear
        # open-exit signature instead of on headland_clearance. Requires the
        # rear camera; the ROS node ANDs this with rear_camera_enabled the
        # same way it does backout_enabled.
        self.exit_clear_rear_steering = exit_clear_rear_steering
        # Extra travel after the rear view opens, so the far corner sweeps
        # clear of the last plant during TURN_1. The rear signature fires when
        # the camera sees open field, which is when the CAMERA is level with
        # the row end -- there is still a bumper's worth of robot behind it.
        self.exit_clear_post_rear_distance = exit_clear_post_rear_distance
        # Hard ceiling on the rear-terminated leg: driving this far without the
        # rear view ever opening means the rear evidence is not coming, so the
        # robot turns on it rather than driving on. It used to UN-COUNT the row
        # and drop back to FOLLOW_ROW instead -- in the middle of a headland,
        # where the exit detector then has to re-arm over min_in_row_distance
        # (2.0 m) of open field before it can fire again. That is how the sim
        # robot reached the world edge (2026-08-07). It is the bound on the
        # leg, not a false-exit test: the front-camera revocation below is
        # that, and it cannot run while the node is looking backwards.
        self.exit_clear_max_distance = exit_clear_max_distance
        # kappa in rear_to_front_state(). 2*c1/c3, where c1 and c3 come from the
        # scan rows' ground distances: with scan_row_fractions
        # [0.65, 0.78, 0.92] / weights [0.2, 0.3, 0.5] imaging roughly 3/2/1 m,
        # c1 = 0.2/3 + 0.3/2 + 0.5/1 = 0.717 and c3 = 1/1 - 1/3 = 0.667, so
        # kappa = 2.15. 0.0 reproduces the broken lateral behaviour (it drops
        # the term that makes the reconstruction work), so raise rather than
        # lower it if the leg under-corrects.
        self.exit_clear_rear_offset_gain = exit_clear_rear_offset_gain

        self._first_turn_direction = first_turn_direction
        self.reset()

    def reset(self, first_turn_direction=None):
        """Return to a clean mission: row 1, FOLLOW_ROW, nothing accumulated.

        ⚠ THIS IS NOT PAUSE, AND THE DIFFERENCE IS THE WHOLE POINT. `~pause`
        exists so a mission SURVIVES an operator interruption -- rows_driven,
        the boustrophedon turn direction, the exit detector's arming distance
        and the row-entry pose all stay exactly as they were. reset() is for
        the opposite case: the robot has been somewhere else entirely (driven
        to a row entrance under GPS, say) and the previous mission's state is
        not merely stale, it is wrong.

        ⚠ The row-entry pose is cleared rather than set, because update() stamps
        it lazily on the first FOLLOW_ROW tick when it is None. That is what
        makes the exit detector arm over min_in_row_distance from WHERE THE
        ROBOT IS NOW instead of from wherever the node happened to start. Get
        this wrong and a node enabled after a 40 m transit has already "driven"
        far more than the arming distance, so the very first frame can fire an
        exit in the middle of row 1.

        The constructor calls this, so the field list cannot drift out of sync
        with initial state -- there is only one definition of "a fresh mission".
        """
        if first_turn_direction is not None:
            self._first_turn_direction = first_turn_direction
        # +1 = left (positive angular.z, REP-103), -1 = right. Restored from the
        # configured direction, not from whatever the last row left it at, or a
        # reset mission turns the wrong way out of row 1.
        self._turn_sign = 1 if self._first_turn_direction == "left" else -1

        self.state = STATE_FOLLOW_ROW
        self.rows_driven = 0
        self.blocked_events = []   # (row_index, distance_m) per blocked row
        self.revoked_exits = []    # (row_index, distance_m) per revoked exit
        self._entry_xy = None      # (x, y) at state entry, for distance legs
        # (x, y) where the CURRENT row was entered. Distinct from _entry_xy,
        # which is re-stamped on every maneuver: the exit detector's arming
        # distance must survive a revoked exit, so the row reference cannot be
        # reset when EXIT_CLEAR falls back to FOLLOW_ROW.
        self._row_entry_xy = None
        self._exit_clear_offset = 0.0   # m already driven since first sighting
        # EXIT_CLEAR rear-steering bookkeeping. _rear_open_at is the travel at
        # which the rear view first read open (None = not yet); _rear_frames
        # counts usable rear results this leg, and staying at 0 is what
        # demotes the leg back to the open-loop terminator.
        self._exit_clear_rear_open_at = None
        self._exit_clear_rear_frames = 0
        self._revoke_fail = 0.0         # m of continuous near-row corn
        self._revoke_last_distance = None
        self._last_yaw = None      # previous yaw sample, for sweep integration
        self._swept = 0.0          # accumulated yaw swept in current turn
        self._reacquire_distance = 0.0   # m of in-row view banked (leaky)
        self._reacquire_last = None      # previous REACQUIRE distance sample
        # REACQUIRE phase B: set once the row is latched, from which point the
        # state is centring rather than searching -- and can no longer end the
        # mission. _reacquire_center_distance banks travel spent inside
        # reacquire_center_tolerance.
        self._reacquire_latched = False
        self._reacquire_center_distance = 0.0
        self._reacquire_center_last = None
        # True when phase B gave up on reacquire_center_max_distance rather
        # than on the tolerance. The ROS node turns this into a warning; it is
        # a list so a repeated failure across rows is visible, not overwritten.
        self.uncentered_handoffs = []  # (row_index, |offset_norm| at handoff)
        self._backout_target = None    # meters to reverse in BACKOUT
        self._suppress_flip = False    # skip one turn-sign flip at REACQUIRE
        # Occlusion nudge bookkeeping. _healthy_distance is the "there WAS a
        # corridor" precondition; _nudge_attempts is the budget, spent on entry
        # and refunded the moment the view comes back; _nudge_resume_xy is the
        # row-entry pose NUDGE must hand back unchanged (see _resume_follow_row).
        self._healthy_distance = 0.0
        self._healthy_last = None
        self._nudge_attempts = 0
        self.nudge_events = []     # (row_index, distance_in_row) per nudge

        # Every distance- and time-debounced accumulator lives in these, and
        # each works on the DELTA between consecutive samples. Left alone, the
        # first frame after a reset credits the whole transit at once.
        self._controller.reset()
        self._detector.reset()
        self.rear_exit_detector.reset()
        self.exit_clear_detector.reset()

    # ------------------------------------------------------------ helpers --
    @property
    def active_rear_detector(self):
        """Whichever rear watcher the current state is using, for the HUD.

        Two exist because the questions differ (see the constructor); the ROS
        node just wants the one whose numbers explain what the robot is
        waiting for right now.
        """
        if self.state == STATE_EXIT_CLEAR:
            return self.exit_clear_detector
        return self.rear_exit_detector

    def _enter(self, state, odom_pose):
        self.state = state
        self._entry_xy = (odom_pose[0], odom_pose[1]) if odom_pose else None
        self._last_yaw = odom_pose[2] if odom_pose else None
        self._swept = 0.0
        self._reacquire_distance = 0.0
        self._reacquire_last = None
        self._reacquire_latched = False
        self._reacquire_center_distance = 0.0
        self._reacquire_center_last = None
        if state == STATE_EXIT_CLEAR:
            self._revoke_fail = 0.0
            # 0.0, not None: the leg starts at zero travel, so the first
            # sample's whole delta is travel this leg is evidence for. Leaving
            # it None threw that sample away, which matters most in
            # rear-terminated mode -- the front camera only gets every other
            # frame there, so revocation would need three of them to bank two
            # frames' worth of distance and could time out first.
            self._revoke_last_distance = 0.0
            self._exit_clear_rear_open_at = None
            self._exit_clear_rear_frames = 0
            if self.exit_clear_rear_steering:
                # Same reasons as BACKOUT: clear the MPC rate-limiter history
                # before the steering reference changes, and clear the rear
                # accumulator so this leg does not inherit evidence banked by
                # a previous headland.
                self._controller.reset()
                self.exit_clear_detector.reset()
        if state == STATE_FOLLOW_ROW:
            self._row_entry_xy = self._entry_xy
            self._controller.reset()
            self._detector.reset()
            # A new row: the occlusion budget and the "I had a corridor"
            # evidence both belong to the row that just ended.
            self._healthy_distance = 0.0
            self._healthy_last = None
            self._nudge_attempts = 0
        elif state == STATE_BACKOUT:
            # Clear the MPC rate-limiter history from forward driving before
            # the controller starts steering the reverse leg.
            self._controller.reset()
            self.rear_exit_detector.reset()

    def _resume_follow_row(self, odom_pose, distance_in_row, now):
        """Return to FOLLOW_ROW from NUDGE without starting a new row.

        ⚠ Deliberately NOT _enter(STATE_FOLLOW_ROW, ...). That call re-stamps
        _row_entry_xy and resets the detector, which is right when a headland
        turn has delivered the robot into a genuinely new row and catastrophic
        here: the nudge happens mid-row, and re-stamping would re-arm the exit
        detector over another min_in_row_distance (2.0 m) and throw away the
        open evidence banked on the way up the row. rows_driven, the turn sign
        and the row-entry pose all belong to the row the robot never left.

        The detector IS resynced, because it was not updated during the nudge:
        its accumulators work on deltas between consecutive samples, so the
        first frame back would otherwise bank the whole nudge -- _MAX_DT of
        blocked seconds and nudge_distance of travel -- in one step. Resync
        moves the references forward without crediting anything, so the
        blocked evidence banked BEFORE the nudge survives and a real dead end
        still confirms promptly.
        """
        self.state = STATE_FOLLOW_ROW
        self._entry_xy = self._row_entry_xy
        self._last_yaw = odom_pose[2] if odom_pose else None
        self._swept = 0.0
        # The robot drove blind and straight; whatever the rate limiter was
        # holding predates the occlusion.
        self._controller.reset()
        self._detector.resync(now, distance_in_row)
        # Same for the nudge's own travel: it was not corridor evidence.
        self._healthy_last = distance_in_row

    def _distance_from(self, reference_xy, odom_pose):
        if odom_pose is None or reference_xy is None:
            return None
        dx = odom_pose[0] - reference_xy[0]
        dy = odom_pose[1] - reference_xy[1]
        return math.hypot(dx, dy)

    def _distance_from_entry(self, odom_pose):
        return self._distance_from(self._entry_xy, odom_pose)

    def _distance_in_row(self, odom_pose):
        """Distance since the CURRENT ROW was entered (drives exit arming)."""
        return self._distance_from(self._row_entry_xy, odom_pose)

    def _integrate_yaw(self, odom_pose):
        """Accumulate swept yaw across samples (wrap-safe); returns |swept|."""
        if odom_pose is None:
            return abs(self._swept)
        yaw = odom_pose[2]
        if self._last_yaw is not None:
            self._swept += _wrap_angle(yaw - self._last_yaw)
        self._last_yaw = yaw
        return abs(self._swept)

    def backout_progress(self, odom_pose):
        """(meters reversed so far or None, target meters) while in
        STATE_BACKOUT; used by the ROS node's back-out telemetry log."""
        return (
            self._distance_from_entry(odom_pose),
            self._backout_target if self._backout_target is not None else 0.0,
        )

    def exit_clear_progress(self, odom_pose):
        """(meters travelled or None, meters at which the rear view opened or
        None, rear frames seen this leg) while in STATE_EXIT_CLEAR; used by
        the ROS node's headland telemetry log."""
        return (
            self._distance_from_entry(odom_pose),
            self._exit_clear_rear_open_at,
            self._exit_clear_rear_frames,
        )

    def _revoke_exit(self, centerline_result, image_width, travelled, odom_pose):
        """Undo a just-fired open exit that the near scan row contradicts.

        Judged ONLY on the nearest scan row -- the ground immediately beside
        the robot. Mid-row, a false exit puts corn back alongside within
        ~0.15 m of travel; in the headland that row stays open no matter what
        the far rows see of the corn block across the way. Returns True if the
        FSM fell back to FOLLOW_ROW this tick.
        """
        if not self.exit_revoke_enabled or travelled is None:
            return False
        if travelled >= self.exit_revoke_distance:
            return False

        delta = (
            max(0.0, travelled - self._revoke_last_distance)
            if self._revoke_last_distance is not None
            else 0.0
        )
        self._revoke_last_distance = travelled

        near_clear = nearest_row_flank_clear(
            centerline_result,
            image_width,
            self._detector.exit_flank_edge_margin,
            self._detector.exit_flank_min_clear_fraction,
        )
        # None = the near row has no corridor at all (something non-traversable
        # straight ahead); that is not an open exit either, so it counts
        # against the exit exactly like corn in the flanks does.
        if near_clear:
            self._revoke_fail = 0.0
            return False
        self._revoke_fail += delta
        if self._revoke_fail < self.exit_revoke_fail_distance:
            return False

        self._withdraw_exit(odom_pose)
        return True

    def _withdraw_exit(self, odom_pose):
        """Un-count a fired exit and resume row-following.

        Shared by both false-exit backstops: revocation (open-loop mode) and
        the exit_clear_max_distance timeout (rear-steered mode).

        Falls back WITHOUT _enter(): that would re-stamp _row_entry_xy to here
        and disarm the exit detector for another min_in_row_distance meters.
        The row was never left, so its entry reference must survive.
        """
        self.rows_driven = max(0, self.rows_driven - 1)
        self.revoked_exits.append(
            (self.rows_driven + 1, self._distance_in_row(odom_pose))
        )
        self.state = STATE_FOLLOW_ROW
        self._entry_xy = self._row_entry_xy
        self._last_yaw = odom_pose[2] if odom_pose else None
        self._swept = 0.0
        self._reacquire_distance = 0.0
        self._reacquire_last = None
        self._exit_clear_offset = 0.0
        self._exit_clear_rear_open_at = None
        self._exit_clear_rear_frames = 0
        self._controller.reset()
        self._detector.reset()

    def _exit_clear_rear_steered(self, rear_centerline_result,
                                 exit_centerline_result, image_width,
                                 travelled, odom_pose, now):
        """EXIT_CLEAR steered and terminated by the REAR camera.

        The node infers on the REAR camera for the whole leg, so
        exit_centerline_result is normally None. It is not None only on the
        fallback frames the node takes when the rear camera stops delivering,
        and there the ordinary revocation test is exactly what should run.

        Returns the same (linear_x, angular_z, state, done) tuple as update().
        """
        if exit_centerline_result is not None:
            if self._revoke_exit(exit_centerline_result, image_width,
                                 travelled, odom_pose):
                return 0.0, 0.0, self.state, False

        if rear_centerline_result is not None:
            # Counts frames, not valid results: an invalid mask still proves
            # the camera is alive, which is the only thing this counter is
            # asked. Zero here means no rear frames arrived at all.
            self._exit_clear_rear_frames += 1
            rear_signal = self.exit_clear_detector.update(
                rear_centerline_result, image_width, travelled, now=now
            )
            if (
                rear_signal == EXIT_ROW_END_OPEN
                and self._exit_clear_rear_open_at is None
            ):
                # Back-date to where the rear view FIRST read open, exactly as
                # the front path does with open_streak_start. Using the
                # confirmation point instead added the detector's whole
                # exit_confirm_distance (0.4 m) on top of the leg, and then
                # exit_clear_post_rear_distance on top of that -- the same
                # compounding overshoot the front leg was fixed for in
                # 2026-07, and most of why this leg ran ~2 m in sim.
                streak_start = self.exit_clear_detector.open_streak_start
                self._exit_clear_rear_open_at = (
                    streak_start if streak_start is not None else travelled
                )

        # The tail is out: drive exit_clear_post_rear_distance more so the
        # corner sweeps clear of the last plant, then turn.
        if self._exit_clear_rear_open_at is not None:
            if travelled >= max(
                self.exit_clear_min_distance,
                self._exit_clear_rear_open_at + self.exit_clear_post_rear_distance,
            ):
                self._enter(STATE_TURN_1, odom_pose)
                return 0.0, 0.0, self.state, False

        elif self._exit_clear_rear_frames == 0:
            # No rear frame has arrived this whole leg -- camera unplugged,
            # dead, or on the wrong topic (it fails SILENTLY, and only ever
            # matters at moments like this one). Degrade to the open-loop
            # terminator rather than stranding the robot at a row end.
            if (
                travelled >= self.exit_clear_min_distance
                and travelled + self._exit_clear_offset >= self.headland_clearance
            ):
                self._enter(STATE_TURN_1, odom_pose)
                return 0.0, 0.0, self.state, False

        elif travelled >= self.exit_clear_max_distance:
            # The rear camera is alive and has watched this much headland go
            # by without the row ever opening behind. Turn on the ceiling: the
            # front detector did confirm an open exit, revocation above has
            # had its say, and continuing straight is what put the sim robot
            # over the world edge.
            self._enter(STATE_TURN_1, odom_pose)
            return 0.0, 0.0, self.state, False

        # Steer from the rear view, re-expressed as the equivalent FRONT
        # measurement so the field-tuned MPC applies unchanged. See
        # rear_to_front_state: the mirror flips the lateral term but not the
        # heading one, so the old "negate the state" rule inverted heading
        # feedback and drove the robot OFF the row axis. Transforming the
        # STATE rather than the output keeps the controller's _u_prev and its
        # rate-limit constraint in the same frame as the published command.
        #
        # ⚠ `valid` means something WEAKER here than it does in a row. It is a
        # pixel count over the lower half, and it cannot see that the corridor
        # ran off the side of the image -- the scan stops at the first
        # non-traversable pixel OR at the border, and x_mid averages the two
        # kinds of boundary as if they were the same thing. In a row that is a
        # bias; in a HEADLAND, where the ground genuinely continues past the
        # frame, it is a fiction, and the MPC steered on it at full authority
        # for a whole leg (sim, 2026-08-07: `edges=1.00/0.00`,
        # `angular_z=+0.175` held to the world edge). So the leg asks the one
        # extra question `valid` cannot answer, and otherwise treats an
        # unusable rear frame exactly as the front controller treats an
        # unusable front frame: no steering this tick.
        angular_z = 0.0
        if (
            rear_centerline_result is not None
            and rear_centerline_result.valid
            and nearest_row_corridor_is_bounded(
                rear_centerline_result, image_width
            )
        ):
            offset, slope = rear_to_front_state(
                rear_centerline_result.offset_norm,
                rear_centerline_result.slope_term,
                self.exit_clear_rear_offset_gain,
            )
            _, angular_z = self._controller.compute(offset, slope, True)
        return self.exit_clear_speed, angular_z, self.state, False

    # --------------------------------------------------- occlusion nudge --
    @property
    def nudge_attempts(self):
        """Nudges spent on the CURRENT occlusion (0 once the view returns)."""
        return self._nudge_attempts

    def _track_corridor_health(self, distance_in_row):
        """Bank meters driven with a real corridor in view, for the nudge gate.

        The nudge's precondition is the user-facing one: there WAS a long
        enough traversable path, and then it vanished. Without it the robot
        would creep forward into something it has never once seen a corridor
        past -- a row entered straight into an obstacle, say.

        ⚠ The accumulator is HELD, not cleared, once a nudge cycle is under
        way (_nudge_attempts > 0). Clearing it on the very frames the nudge
        exists to handle would let exactly one nudge ever fire: after the
        first one the view is still occluded, so the evidence would be gone
        and nudge_max_attempts could never be reached.
        """
        status = self._detector.last_status
        if distance_in_row is None or status is None:
            self._healthy_last = distance_in_row
            return
        delta = (
            max(0.0, distance_in_row - self._healthy_last)
            if self._healthy_last is not None
            else 0.0
        )
        self._healthy_last = distance_in_row
        if (status.corridor_rows or 0) >= self.nudge_min_corridor_rows:
            self._healthy_distance += delta
            # The view is back: refund the budget so a second leaf later in
            # the same row is handled like the first one.
            self._nudge_attempts = 0
        elif self._nudge_attempts == 0:
            self._healthy_distance = 0.0

    def _should_nudge(self, distance_in_row):
        """True when this frame's blocked signature is worth testing by moving.

        The blocked test is read off the detector's own status rather than
        recomputed, so it cannot drift from row_exit_detector's definition:
        no corridor at any scan row, plus enough obstacle pixels to say
        something is actually there.

        Timing comes free from the detector's existing leaky blocked timer.
        nudge_trigger_seconds of banked evidence is the debounce -- at the
        field robot's ~58 Hz a single garbage frame is 17 ms and must never
        move the robot -- and it is far below blocked_confirm_seconds, so the
        nudge always gets its chance before the back-out commits.
        """
        if self.nudge_max_attempts <= 0 or distance_in_row is None:
            return False
        if self._nudge_attempts >= self.nudge_max_attempts:
            return False
        if self._healthy_distance < self.nudge_min_healthy_distance:
            return False
        status = self._detector.last_status
        if status is None or not status.blocked_armed:
            return False
        blocked_signature = (status.corridor_rows or 0) == 0 and (
            status.obstacle_fraction or 0.0
        ) >= self._detector.blocked_min_obstacle_fraction
        if not blocked_signature:
            return False
        return (status.blocked_seconds or 0.0) >= self.nudge_trigger_seconds

    def _looks_like_row(self, centerline_result, image_width):
        """True when the near scan row has a corridor with corn on BOTH sides.

        The exact inverse of the open-exit test, and deliberately not a width
        threshold. The previous rule -- mean corridor width < 0.6 -- is a
        camera-height constant: normal in-row width is ~0.5 on the tall mount
        but ~0.7 on the low one, so on the low camera it could never be
        satisfied inside a row. The FSM then crept the full
        reacquire_max_distance blind (2.0 m at 0.08 m/s = 25 s) and nearly
        drove into the corn (sim, 2026-07-28). Corn is corn at any camera
        height. It also drops the old `valid` requirement
        (traversable_fraction >= 0.10), a second unrelated way to fail to
        latch: this test is strictly more specific than a global pixel count,
        needing a traversable run at image centre AND corn in both strips.
        """
        return (
            nearest_row_flank_clear(
                centerline_result,
                image_width,
                self._detector.exit_flank_edge_margin,
                self._detector.exit_flank_min_clear_fraction,
            )
            is False
        )

    # ------------------------------------------------------------- update --
    def update(self, centerline_result, odom_pose, image_width,
               rear_centerline_result=None, now=None,
               exit_centerline_result=None):
        """Advance one tick. Returns (linear_x, angular_z, state, done).

        Args:
            centerline_result: CenterlineResult for the current FRONT frame,
                or None when this tick carries no front perception at all
                (the node is inferring on the rear camera). ⚠ None is NOT the
                same as an invalid result: "looked and saw no corridor beside
                me" counts as evidence against an exit in the revocation test,
                while "did not look" must count as nothing.
            odom_pose: (x, y, yaw) from odometry, or None if unavailable.
            image_width: mask width in pixels.
            rear_centerline_result: CenterlineResult from the rear camera,
                or None. Used in STATE_BACKOUT, and in STATE_EXIT_CLEAR when
                exit_clear_rear_steering is on -- the same open-exit detector
                the front camera runs, on the same thresholds, pointed
                backwards. ⚠ The two treat it
                differently: BACKOUT reverses and passes it through UNCHANGED
                (reversing flips the lateral dynamics, which is what makes the
                mirrored view work as-is); EXIT_CLEAR drives FORWARD and must
                convert it with rear_to_front_state() first. Neither is a
                negation -- see that function for the geometry.
            now: timestamp in seconds for the detector's BLOCKED accumulator
                (the ROS node passes rospy.get_time()).
            exit_centerline_result: CenterlineResult measured on the exit
                detector's own scan rows, when those are configured separately
                from the steering rows. Defaults to centerline_result. Only
                the exit detector and the revocation test read it; steering
                and REACQUIRE always use the steering rows.
        """
        if exit_centerline_result is None:
            exit_centerline_result = centerline_result

        if centerline_result is None and self.state in (
            STATE_FOLLOW_ROW,
            STATE_REACQUIRE,
        ):
            # No front perception on this tick (the node is inferring on the
            # rear camera) and these two states have nothing to drive on.
            # Only reachable if a state transition raced the camera switch;
            # stopping for one frame is the safe reading.
            return 0.0, 0.0, self.state, False

        if self.state == STATE_FOLLOW_ROW:
            # Lazily record the row-entry pose (covers mission start, where
            # the initial state was set without an _enter() transition, and
            # any start before the first odometry message arrived).
            if self._row_entry_xy is None and odom_pose is not None:
                self._row_entry_xy = (odom_pose[0], odom_pose[1])
                if self._entry_xy is None:
                    self._entry_xy = self._row_entry_xy
            linear_x, angular_z = self._controller.compute(
                centerline_result.offset_norm,
                centerline_result.slope_term,
                centerline_result.valid,
            )
            distance_in_row = self._distance_in_row(odom_pose)
            exit_signal = self._detector.update(
                exit_centerline_result, image_width, distance_in_row, now=now
            )
            self._track_corridor_health(distance_in_row)
            if exit_signal == EXIT_NONE and self._should_nudge(distance_in_row):
                # A leaf on the lens and a crop wall ahead look identical in
                # one frame. Move a little and look again instead of standing
                # still, which is the one action that guarantees a leaf stays
                # put. If the occlusion survives the budget, the blocked
                # evidence banked so far is still there and BLOCKED fires
                # below on a later frame exactly as it always did.
                self._nudge_attempts += 1
                self.nudge_events.append((self.rows_driven + 1, distance_in_row))
                self._enter(STATE_NUDGE, odom_pose)
                return 0.0, 0.0, self.state, False
            if exit_signal != EXIT_NONE:
                if exit_signal == EXIT_ROW_END_BLOCKED:
                    # Blocked ahead (mid-row obstacle or crop wall at the row
                    # end): a failed row does NOT count toward num_rows, so
                    # rows_driven is left untouched -- the robot backs out and
                    # goes on to the next physical row. The reported row index
                    # is the attempt number = rows_driven + 1. d_block is never
                    # None here -- the detector only fires when armed, which
                    # requires odometry.
                    d_block = distance_in_row
                    self.blocked_events.append((self.rows_driven + 1, d_block))
                    if not self.backout_enabled:
                        # No rear camera: nothing safe left to do. Stop in
                        # place and end the mission; the blocked row is in
                        # blocked_events for the final report.
                        self._enter(STATE_DONE, odom_pose)
                        return 0.0, 0.0, self.state, True
                    self._backout_target = d_block
                    self._suppress_flip = True
                    self._enter(STATE_BACKOUT, odom_pose)
                    return 0.0, 0.0, self.state, False
                # Open exit: a row successfully driven. Count it and end the
                # mission once num_rows of them are done. (On the final row
                # there is no EXIT_CLEAR and therefore no revocation window --
                # stopping is the safe failure mode for a false positive.)
                self.rows_driven += 1
                if self.num_rows > 0 and self.rows_driven >= self.num_rows:
                    self._enter(STATE_DONE, odom_pose)
                    return 0.0, 0.0, self.state, True
                # Back-date the headland leg to where the exit was FIRST seen,
                # so the confirmation distance is not added on top of
                # headland_clearance (0.4 + 0.75 = 1.15 m of overshoot).
                streak_start = self._detector.open_streak_start
                self._exit_clear_offset = (
                    max(0.0, distance_in_row - streak_start)
                    if streak_start is not None and distance_in_row is not None
                    else 0.0
                )
                self._enter(STATE_EXIT_CLEAR, odom_pose)
                return 0.0, 0.0, self.state, False
            return linear_x, angular_z, self.state, False

        if self.state == STATE_DONE:
            return 0.0, 0.0, self.state, True

        # All remaining states are odometry-closed-loop maneuvers.
        if odom_pose is None:
            return 0.0, 0.0, self.state, False

        if self.state == STATE_EXIT_CLEAR:
            travelled = self._distance_from_entry(odom_pose)
            if self.exit_clear_rear_steering:
                return self._exit_clear_rear_steered(
                    rear_centerline_result, exit_centerline_result,
                    image_width, travelled, odom_pose, now,
                )
            if self._revoke_exit(exit_centerline_result, image_width, travelled,
                                 odom_pose):
                return 0.0, 0.0, self.state, False
            # headland_clearance is measured from where the exit was first
            # seen (_exit_clear_offset), but at least exit_clear_min_distance
            # is always driven after it confirmed.
            if (
                travelled >= self.exit_clear_min_distance
                and travelled + self._exit_clear_offset >= self.headland_clearance
            ):
                self._enter(STATE_TURN_1, odom_pose)
                return 0.0, 0.0, self.state, False
            return self.exit_clear_speed, 0.0, self.state, False

        if self.state == STATE_BACKOUT:
            if self._distance_from_entry(odom_pose) >= self._backout_target:
                self._enter(STATE_BACKOUT_CLEAR, odom_pose)
                return 0.0, 0.0, self.state, False
            # Rear-view exit check: the row opening up behind the robot
            # means it is out -- stop unwinding odometry (which overshoots
            # whenever d_block includes pre-row approach, e.g. row 1 from
            # the spawn point) and move on to BACKOUT_CLEAR.
            if rear_centerline_result is not None:
                rear_signal = self.rear_exit_detector.update(
                    rear_centerline_result,
                    image_width,
                    self._distance_from_entry(odom_pose),
                    now=now,
                )
                if rear_signal == EXIT_ROW_END_OPEN:
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
                # A blocked row never ends the mission on its own -- always
                # S-turn into the next physical row.
                self._enter(STATE_BACKOUT_TURN_1, odom_pose)
                return 0.0, 0.0, self.state, False
            return -self.backout_speed, 0.0, self.state, False

        if self.state == STATE_NUDGE:
            if self._distance_from_entry(odom_pose) >= self.nudge_distance:
                self._resume_follow_row(
                    odom_pose, self._distance_in_row(odom_pose), now
                )
                return 0.0, 0.0, self.state, False
            # angular_z is 0.0 on purpose: there is no corridor in the mask to
            # steer on -- that is the whole reason this state exists -- and
            # steering on a reading taken through a leaf is worse than holding
            # the heading the robot already had in the row.
            return self.nudge_speed, 0.0, self.state, False

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
            # The back-out S-turn keeps its own rate: it is the one turn that
            # starts beside whatever blocked the robot, and the headland
            # turns' raised rate has no validation there.
            rate = (
                self.backout_turn_rate
                if self.state in (STATE_BACKOUT_TURN_1, STATE_BACKOUT_TURN_2)
                else self.turn_rate
            )
            return (
                0.0,
                sign_mult * self._turn_sign * rate,
                self.state,
                False,
            )

        if self.state in (STATE_TRAVERSE, STATE_BACKOUT_TRAVERSE):
            next_turn = (
                STATE_TURN_2 if self.state == STATE_TRAVERSE else STATE_BACKOUT_TURN_2
            )
            if self._distance_from_entry(odom_pose) >= self.traverse_distance:
                self._enter(next_turn, odom_pose)
                return 0.0, 0.0, self.state, False
            # NOT linear_x_cruise. This leg is blind and odometry-bounded, and
            # cruise is an operator knob pushed to 0.9 m/s in speed testing;
            # inheriting it sent the headland crossing up there too.
            speed = (
                self.traverse_speed
                if self.traverse_speed is not None
                else self._controller.linear_x_cruise
            )
            return speed, 0.0, self.state, False

        if self.state == STATE_REACQUIRE:
            travelled = self._distance_from_entry(odom_pose)
            delta = (
                max(0.0, travelled - self._reacquire_last)
                if self._reacquire_last is not None
                else 0.0
            )
            self._reacquire_last = travelled
            in_row = self._looks_like_row(exit_centerline_result, image_width)

            # Phase B -- centring. Latching a row and being fit to hand it to
            # FOLLOW_ROW are two different events; until 2026-09-12 they were
            # the same one, and REACQUIRE handed over whatever lateral error
            # the headland turn had left (field 2026-09-09: typically 0.10 of
            # offset_norm, worst 0.26, and the two worst handoffs of the
            # morning are the two rows that needed a rescue).
            if self._reacquire_latched:
                centered = (
                    abs(centerline_result.offset_norm)
                    <= self.reacquire_center_tolerance
                )
                if centered:
                    self._reacquire_center_distance += delta
                else:
                    self._reacquire_center_distance = 0.0
                give_up = travelled >= self.reacquire_center_max_distance
                if (
                    self._reacquire_center_distance
                    >= self.reacquire_center_confirm_distance - 1e-9
                    or give_up
                ):
                    if give_up:
                        # Latched a row and never centred in it. Recorded, not
                        # swallowed: the node logs it as a warning, because the
                        # next thing that happens is FOLLOW_ROW inheriting the
                        # error this state exists to remove.
                        self.uncentered_handoffs.append(
                            (
                                self.rows_driven + 1,
                                abs(centerline_result.offset_norm),
                            )
                        )
                    self._enter(STATE_FOLLOW_ROW, odom_pose)
                    return 0.0, 0.0, self.state, False
                angular_z = 0.0
                if self.reacquire_steering_enabled:
                    _, angular_z = self._controller.compute(
                        centerline_result.offset_norm,
                        centerline_result.slope_term,
                        True,
                    )
                # Faster than the search creep: the row is found, there is
                # nothing left to look for, and reacquire_center_speed is the
                # speed angular_z_max and the MPC are actually tuned at.
                return (
                    self.reacquire_center_speed,
                    angular_z,
                    STATE_REACQUIRE_CENTER,
                    False,
                )

            # Phase A -- searching.
            if in_row:
                self._reacquire_distance += delta
            else:
                self._reacquire_distance = 0.0
            if self._reacquire_distance >= self.reacquire_confirm_distance - 1e-9:
                # New row acquired: flip turn direction for the next headland
                # -- unless this row was reached via a back-out, in which case
                # it is traveled in the SAME world direction as the blocked
                # attempt and the flip must be skipped exactly once.
                if self._suppress_flip:
                    self._suppress_flip = False
                else:
                    self._turn_sign = -self._turn_sign
                # Stay in REACQUIRE and centre. The odometry reference is
                # re-stamped so reacquire_center_max_distance measures phase B
                # alone rather than inheriting the search creep.
                self._reacquire_latched = True
                self._reacquire_center_distance = 0.0
                self._entry_xy = (odom_pose[0], odom_pose[1])
                self._reacquire_last = 0.0
                return (
                    self.reacquire_center_speed,
                    0.0,
                    STATE_REACQUIRE_CENTER,
                    False,
                )
            if travelled >= self.reacquire_max_distance:
                # No corridor where a row should be: no rows left. Stop.
                # ⚠ Reachable in phase A ONLY. "I can see a row but am not yet
                # centred in it" must never be reported as "there are no rows
                # left" -- phase B has its own, non-terminal backstop above.
                self._enter(STATE_DONE, odom_pose)
                return 0.0, 0.0, self.state, True
            # Steer while creeping. Driving dead straight for up to
            # reacquire_max_distance is how REACQUIRE nearly put the robot into
            # the corn: entering a row slightly off-centre, it held the error
            # instead of closing it. Only steer once a row is actually visible;
            # angular_z_max clamps whatever the MPC makes of a headland view.
            angular_z = 0.0
            if self.reacquire_steering_enabled and in_row:
                _, angular_z = self._controller.compute(
                    centerline_result.offset_norm,
                    centerline_result.slope_term,
                    True,
                )
            return self.reacquire_speed, angular_z, self.state, False

        raise RuntimeError("unknown state: %s" % self.state)
