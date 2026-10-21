"""Sequencing for a LiDAR-localized deployment: trailer -> row -> trailer.

Pure python -- no rospy. `scripts/deploy_supervisor.py` is the ROS shell around
this; every decision about who drives and every way the sequence can fail is
here, where it can be tested without a simulator. Same shape as
agbot_gps_nav's handoff_fsm, which it does not replace (the GPS stack is kept).

    IDLE -start-> REVERSE_OFF -> LOCALIZE -> TRANSIT -> ROW_MISSION -> RETURN -> FINISHED
                       |             |          |            |            |
                       +-------------+--- timeout / nav failure / gate ---+--> FAILED

INTERACTIVE (no entrance given): the operator picks the row entrance on the
map instead (RViz "2D Nav Goal", arrow down the row -> `set_entrance`), and
the deployment REPEATS -- after the way home, or after a failure, it waits
for the next goal instead of ending. That is how a goal is tested anywhere
on the map without restarting the stack:

    LOCALIZE -> WAIT_GOAL -goal-> TRANSIT -> ROW_MISSION -> RETURN -> WAIT_GOAL
                                     \___________\_____________\--> FAILED -goal-> ...

REVERSE_OFF  The supervisor itself reverses the robot `reverse_distance` m
             straight off the ramp on wheel odometry. Blind on purpose: the
             trailer is right in front of the robot and no planner should be
             turning near it. Real layout: trailer heading north, robot backs
             off still facing north.
LOCALIZE     Stopped. AMCL must hold its estimate below
             `localize_max_std_xy` / `localize_max_std_yaw_deg` for
             `localize_hold_sec`. AMCL only updates on motion, so each tick
             asks for a no-motion update. The pose it settles on is RECORDED as
             the return point: the trailer parks somewhere different every
             deployment, so the way home is measured, never stored.
             ⚠ These limits are LOOSER than the arrival ones on purpose. At
             the drop-off the nearest mapped features can be 15 m away and
             the trailer (unmapped by design -- it moves) fills the scan; the
             sim measured AMCL at ~0.2 m / 4 deg there, and forcing no-motion
             updates made it WORSE (33 deg), not better. The strict check is
             at the row entrance, where the corn is close and the pose
             matters.
TRANSIT      move_base drives to the row entrance (a map-frame pose whose yaw
             is the row bearing), or rather to a point `entrance_standoff` m
             BEHIND it on the row axis -- see below.
ROW_MISSION  Vision nav drives the rows; move_base has no goal.
WAIT_GOAL    Interactive only. Stopped, nobody commanding, no timeout.
RETURN       Vision off, move_base drives back to the recorded drop-off pose.
             Driving back UP the ramp is out of scope until the trailer team
             says what the ramp is.

⚠ EXACTLY ONE COMMANDER AT A TIME. The supervisor's reverse leg, move_base and
vision nav all publish /cmd_vel, which twist_mux takes as ONE input -- it
arbitrates by priority, not rate, so two publishers on it interleave. Every
tick reports the FULL desired state (`commander`) rather than edge events, and
a state that hands over always names the outgoing commander as switched off in
the same tick, which the caller applies before anything is switched on.

⚠ ARRIVING IS NOT THE SAME AS ARRIVING ON THE ROW AXIS. move_base's SUCCEEDED
means "inside its goal tolerance", and its goal tolerance is not the handoff
tolerance. The AMCL pose at arrival is checked against the corridor axis
(cross-track and heading) AND AMCL must still be confident there; anything
else refuses the handoff. Refusing at the entrance is cheap; one row spacing
later is not. (Same lesson as handoff_fsm, 2026-09-07.)
"""

import collections
import math

# ⚠ WHY THE GOAL STANDS OFF THE ENTRANCE (measured 2026-10-13, sim): the
# LiDAR marks corn LEAVES, not stalks, and a 1.3 m plant's leaves reach ~0.6 m
# out of the row. With the robot's inscribed radius and inflation on top,
# everything within ~0.9 m of the first stalks was lethal in move_base's
# costmap -- including an entrance placed 0.5 m out, so GlobalPlanner refused
# the goal ("Failed to get a plan") 3 m short of it. move_base therefore aims
# `entrance_standoff` further back along the axis, and vision nav drives the
# rest. The arrival check is unaffected: cross-track and heading are the same
# anywhere on the axis.

STATE_IDLE = "IDLE"
STATE_REVERSE_OFF = "REVERSE_OFF"
STATE_LOCALIZE = "LOCALIZE"
STATE_WAIT_GOAL = "WAIT_GOAL"
STATE_TRANSIT = "TRANSIT"
STATE_ROW_MISSION = "ROW_MISSION"
STATE_RETURN = "RETURN"
STATE_FINISHED = "FINISHED"
STATE_FAILED = "FAILED"

NAV_ACTIVE = "ACTIVE"
NAV_SUCCEEDED = "SUCCEEDED"
NAV_FAILED = "FAILED"

COMMANDER_SUPERVISOR = "supervisor"
COMMANDER_MOVE_BASE = "move_base"
COMMANDER_VISION = "vision"

Pose2D = collections.namedtuple("Pose2D", "x y yaw")
# AMCL's estimate: pose plus its 1-sigma spread (m, rad).
Estimate = collections.namedtuple("Estimate", "x y yaw std_xy std_yaw")

DeployTick = collections.namedtuple(
    "DeployTick",
    "state commander reverse_speed vision_enabled nav_goal cancel_nav "
    "request_localize_update done failed reason",
)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def axis_errors(estimate, entrance):
    """(cross_track m, heading_error rad) of `estimate` vs the corridor axis.

    The axis passes through `entrance` along `entrance.yaw`. Cross-track is
    positive to the LEFT of the axis (REP-103).
    """
    dx = estimate.x - entrance.x
    dy = estimate.y - entrance.y
    cross = -dx * math.sin(entrance.yaw) + dy * math.cos(entrance.yaw)
    return cross, wrap(estimate.yaw - entrance.yaw)


class DeployFSM(object):
    """Decides who drives, where to, and when the deployment has failed."""

    def __init__(self, entrance=None, entrance_standoff=0.7,
                 reverse_distance=1.5, reverse_speed=0.15,
                 max_std_xy=0.15, max_std_yaw_deg=8.0,
                 localize_max_std_xy=0.30, localize_max_std_yaw_deg=10.0,
                 localize_hold_sec=2.0,
                 max_arrival_cross_track=0.20,
                 max_arrival_heading_error_deg=12.0,
                 return_enabled=True,
                 reverse_timeout_sec=30.0, localize_timeout_sec=30.0,
                 transit_timeout_sec=300.0, mission_timeout_sec=1800.0,
                 return_timeout_sec=300.0):
        # No entrance = the operator gives one (and then another) at run time.
        self.interactive = entrance is None
        self.entrance_standoff = float(entrance_standoff)
        self.entrance = None
        self.transit_goal = None
        if entrance is not None:
            self._set_entrance(entrance)
        self.reverse_distance = float(reverse_distance)
        # Stored positive; the tick reports it as a NEGATIVE linear.x.
        self.reverse_speed = abs(float(reverse_speed))
        # Arrival confidence. 8 deg, not tighter: AMCL's heading SPREAD in the
        # sim sat at 6.0 deg at a corridor mouth the robot had really reached
        # 0.03 m / -1.4 deg off-axis (2026-10-13) -- a 3 deg limit refused a
        # perfect handoff. The 12 deg arrival heading limit is the budget;
        # this keeps most of it for honest uncertainty.
        self.max_std_xy = float(max_std_xy)
        self.max_std_yaw = math.radians(float(max_std_yaw_deg))
        self.localize_max_std_xy = float(localize_max_std_xy)
        self.localize_max_std_yaw = math.radians(float(localize_max_std_yaw_deg))
        self.localize_hold_sec = float(localize_hold_sec)
        # 0.20 m: the Jackal is ~0.43 m wide in a 0.75 m corridor, so ~0.16 m
        # of clearance a side; the same number agbot_gps_nav's mission launch
        # uses for its arrival plane.
        self.max_arrival_cross_track = float(max_arrival_cross_track)
        self.max_arrival_heading_error = math.radians(
            float(max_arrival_heading_error_deg))
        self.return_enabled = bool(return_enabled)
        self.timeouts = {
            STATE_REVERSE_OFF: float(reverse_timeout_sec),
            STATE_LOCALIZE: float(localize_timeout_sec),
            STATE_TRANSIT: float(transit_timeout_sec),
            STATE_ROW_MISSION: float(mission_timeout_sec),
            STATE_RETURN: float(return_timeout_sec),
        }

        self.state = STATE_IDLE
        self.reason = ""
        self.drop_off = None          # Pose2D recorded at the end of LOCALIZE
        self.arrival = None           # (cross_track, heading_error) at the row
        self._entered_at = None
        self._reverse_start = None    # odom (x, y) where the reverse began
        self._confident_since = None

    # ---- transitions ------------------------------------------------------

    def start(self, now):
        """Begin: the robot is at the ramp foot, facing the trailer."""
        self.reason = ""
        self.drop_off = None
        self.arrival = None
        self._reverse_start = None
        if self.reverse_distance <= 0.0:
            # Driven off the trailer by hand: nothing to reverse.
            return self._enter(STATE_LOCALIZE, now)
        return self._enter(STATE_REVERSE_OFF, now)

    def abort(self, reason):
        """Stop everything. Used for an operator stop or an external fault.

        Terminal unless interactive: there the operator can send a new goal.
        """
        self.state = STATE_FAILED
        self.reason = reason
        return self._tick(cancel_nav=True)

    def set_entrance(self, entrance, now):
        """The operator's row entrance (x, y, row bearing), map frame.

        Returns (tick or None, message). A tick means the caller applies it
        now; None means nothing changes this instant (refused, or held until
        LOCALIZE finishes).
        """
        if not self.interactive:
            return None, ("ignored: the entrance is fixed by the launch args "
                          "(start deploy_mission without entrance_* to pick it "
                          "on the map)")
        if self.state in (STATE_TRANSIT, STATE_ROW_MISSION, STATE_RETURN):
            return None, ("ignored: busy in %s -- wait for WAIT_GOAL, or stop "
                          "the robot with the joystick" % self.state)
        self._set_entrance(entrance)
        e = self.entrance
        message = "row entrance (%.2f, %.2f) facing %.0f deg" % (
            e.x, e.y, math.degrees(e.yaw))
        if self.state == STATE_WAIT_GOAL or (
                self.state == STATE_FAILED and self.drop_off is not None):
            self.reason = ""
            return self._enter(STATE_TRANSIT, now), message
        if self.state == STATE_FAILED:
            # Failed before the drop-off was ever recorded: localize first,
            # or there would be no way home.
            self.reason = ""
            return self._enter(STATE_LOCALIZE, now), message + "; localizing first"
        return None, message + "; held until localized"

    def _set_entrance(self, entrance):
        self.entrance = e = Pose2D(*entrance)
        self.transit_goal = Pose2D(
            e.x - self.entrance_standoff * math.cos(e.yaw),
            e.y - self.entrance_standoff * math.sin(e.yaw),
            e.yaw)

    def update(self, now, odom_xy=None, estimate=None, nav_state=None,
               vision_done=False):
        """Advance one tick and return the full desired state.

        odom_xy     (x, y) of /odometry/filtered -- the reverse leg's distance.
        estimate    Estimate from AMCL, or None before its first pose.
        nav_state   NAV_ACTIVE / NAV_SUCCEEDED / NAV_FAILED for the goal sent
                    in THIS state, or None before one is known.
        vision_done the row mission's /vision_nav_node/mission_done.
        """
        now = float(now)
        if self.state in (STATE_IDLE, STATE_FINISHED, STATE_FAILED,
                          STATE_WAIT_GOAL):
            return self._tick()

        elapsed = now - self._entered_at
        if elapsed > self.timeouts[self.state]:
            return self.abort("%s timed out after %.0f s" % (self.state, elapsed))

        if self.state == STATE_REVERSE_OFF:
            if odom_xy is None:
                return self._tick()
            if self._reverse_start is None:
                self._reverse_start = tuple(odom_xy)
            driven = math.hypot(odom_xy[0] - self._reverse_start[0],
                                odom_xy[1] - self._reverse_start[1])
            if driven >= self.reverse_distance:
                return self._enter(STATE_LOCALIZE, now)
            return self._tick()

        if self.state == STATE_LOCALIZE:
            if self._confident(estimate, self.localize_max_std_xy,
                               self.localize_max_std_yaw):
                if self._confident_since is None:
                    self._confident_since = now
                if now - self._confident_since >= self.localize_hold_sec:
                    self.drop_off = Pose2D(estimate.x, estimate.y, estimate.yaw)
                    if self.entrance is None:
                        return self._enter(STATE_WAIT_GOAL, now)
                    return self._enter(STATE_TRANSIT, now)
            else:
                self._confident_since = None
            return self._tick()

        if self.state == STATE_TRANSIT:
            if nav_state == NAV_FAILED:
                return self.abort("move_base gave up on the way to the row")
            if nav_state != NAV_SUCCEEDED:
                return self._tick()
            if estimate is None or not self._confident(estimate):
                return self.abort(
                    "arrived at the row entrance with AMCL NOT confident "
                    "(std %s) -- the pose the arrival check would use cannot "
                    "be trusted. NOT handing over to vision nav."
                    % ("unknown" if estimate is None else
                       "%.2f m / %.1f deg" % (estimate.std_xy,
                                              math.degrees(estimate.std_yaw))))
            cross, heading = axis_errors(estimate, self.entrance)
            self.arrival = (cross, heading)
            if (abs(cross) > self.max_arrival_cross_track
                    or abs(heading) > self.max_arrival_heading_error):
                return self.abort(
                    "arrived %.2f m off the corridor axis, %+.1f deg off its "
                    "bearing (limits %.2f m / %.0f deg). NOT handing over."
                    % (cross, math.degrees(heading),
                       self.max_arrival_cross_track,
                       math.degrees(self.max_arrival_heading_error)))
            return self._enter(STATE_ROW_MISSION, now)

        if self.state == STATE_ROW_MISSION:
            if vision_done:
                if self.return_enabled:
                    return self._enter(STATE_RETURN, now)
                return self._enter(self._finished_state(), now)
            return self._tick()

        if self.state == STATE_RETURN:
            if nav_state == NAV_FAILED:
                return self.abort("move_base gave up on the way back to the trailer")
            if nav_state == NAV_SUCCEEDED:
                return self._enter(self._finished_state(), now)
            return self._tick()

        return self._tick()

    # ---- helpers ----------------------------------------------------------

    def _finished_state(self):
        return STATE_WAIT_GOAL if self.interactive else STATE_FINISHED

    def _confident(self, estimate, max_std_xy=None, max_std_yaw=None):
        """AMCL's spread under the limits; the ARRIVAL limits by default."""
        if max_std_xy is None:
            max_std_xy = self.max_std_xy
        if max_std_yaw is None:
            max_std_yaw = self.max_std_yaw
        return (estimate is not None
                and estimate.std_xy <= max_std_xy
                and estimate.std_yaw <= max_std_yaw)

    def _enter(self, state, now):
        self.state = state
        self._entered_at = float(now)
        self._confident_since = None
        goal = None
        if state == STATE_TRANSIT:
            goal = self.transit_goal
        elif state == STATE_RETURN:
            goal = self.drop_off
        return self._tick(nav_goal=goal)

    def _tick(self, nav_goal=None, cancel_nav=False):
        commander = {
            STATE_REVERSE_OFF: COMMANDER_SUPERVISOR,
            STATE_TRANSIT: COMMANDER_MOVE_BASE,
            STATE_ROW_MISSION: COMMANDER_VISION,
            STATE_RETURN: COMMANDER_MOVE_BASE,
        }.get(self.state)
        return DeployTick(
            state=self.state,
            commander=commander,
            reverse_speed=(-self.reverse_speed
                           if self.state == STATE_REVERSE_OFF else 0.0),
            vision_enabled=self.state == STATE_ROW_MISSION,
            nav_goal=nav_goal,
            cancel_nav=cancel_nav,
            request_localize_update=self.state == STATE_LOCALIZE,
            done=self.state == STATE_FINISHED,
            # Terminal only when nothing can follow it.
            failed=self.state == STATE_FAILED and not self.interactive,
            reason=self.reason,
        )
