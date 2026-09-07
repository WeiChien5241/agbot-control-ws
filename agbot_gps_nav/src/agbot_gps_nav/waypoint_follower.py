"""Go-to-goal waypoint follower: drive from where we are to a point in the map
frame, then stop.

Pure python -- no rospy, no numpy. Unit-tested without ROS, like every other
algorithmic module in this workspace.

WHY NOT move_base. The v1 assumption is flat ground with no obstacles between
the trailer and the row. move_base's costmaps want a laser this robot does not
have (jackal_navigation's costmap_common_params.yaml requires front/scan), and
a global planner buys nothing when the correct path is a straight line. If
obstacle avoidance is ever needed, that is when move_base earns its place.

THE STATES

    IDLE ---(goal)--> GOTO ---(within approach_distance)--> APPROACH
                       ^                                       |
                       |                                  (within tolerance)
                    (new goal)                                 v
                       +----------------------------------- ARRIVED

  GOTO       cruise. Turn in place when badly misaligned, otherwise drive and
             steer at the same time.
  APPROACH   the last approach_distance metres, ramping from cruise DOWN TO
             approach_speed. Slower, because overshooting the row entrance is
             the expensive failure -- it is the one leg where the robot is
             aiming at a gap in a wall of corn.
  ARRIVED    zero velocity, LATCHED. It does not fall back to GOTO if the
             estimate wobbles across the tolerance boundary afterwards, which
             is what would make the robot creep around forever near the goal.

⚠ approach_speed is a FLOOR, not a second multiplier. The first version picked
approach_speed inside approach_distance and THEN multiplied it by a distance
derate flooring at 0.25, so the last metre ran at 3.75 cm/s and a 2.5 m goal
took 25-32 s -- long enough that the first operator to try it concluded the
robot had stalled and killed the run (2026-09-06). Two independent slowdowns
compounding is the bug; the careful approach itself was never the problem. The
ramp below reaches exactly approach_speed at the goal and never goes under it.

SIGN CONVENTION, REP-103: x forward, z up, positive angular.z turns LEFT. A
goal to the robot's left gives a positive heading error and therefore a
positive angular.z. Same convention as MPCRowController, deliberately.

WHAT IS NOT HERE. There is no HEADING_INIT state yet. It belongs with the
heading estimator (a short straight drive at mission start, so GPS
course-over-ground can pin the odom-yaw-to-ENU offset), and adding the state
before the thing that fills it in would just be a state that does nothing.
"""

import math

from agbot_gps_nav import geo

STATE_IDLE = "IDLE"
STATE_HEADING_INIT = "HEADING_INIT"
STATE_GOTO = "GOTO"
STATE_ALIGN = "ALIGN"
STATE_APPROACH = "APPROACH"
STATE_ARRIVED = "ARRIVED"
STATE_FAILED = "FAILED"


class WaypointFollower(object):
    """Drive to a goal point in the map frame.

    All thresholds are constructor keyword args with the same defaults as
    config/params.yaml, matching MissionFSM's pattern.
    """

    def __init__(self,
                 linear_x_cruise=0.4,
                 approach_speed=0.15,
                 angular_z_max=0.6,
                 heading_gain=1.2,
                 turn_in_place_deg=30.0,
                 turn_in_place_rate=0.4,
                 approach_distance=2.0,
                 goal_tolerance=0.3,
                 slow_down_deg=60.0,
                 staging_distance=3.0,
                 staging_tolerance=0.25,
                 align_tolerance_deg=5.0,
                 axis_gain=1.0,
                 max_axis_correction_deg=45.0,
                 max_goal_distance_growth=5.0,
                 heading_init_distance=0.0,
                 heading_init_max_distance=8.0,
                 heading_init_tolerance_deg=12.0,
                 arrival_cross_tolerance=0.35,
                 max_approach_attempts=3):
        self.linear_x_cruise = float(linear_x_cruise)
        # Clamped to cruise: approach_speed above cruise would invert the ramp
        # below and make the robot SPEED UP into the goal.
        self.approach_speed = min(float(approach_speed), float(linear_x_cruise))
        self.angular_z_max = float(angular_z_max)
        self.heading_gain = float(heading_gain)
        self.turn_in_place_rad = math.radians(float(turn_in_place_deg))
        self.turn_in_place_rate = float(turn_in_place_rate)
        self.approach_distance = float(approach_distance)
        self.goal_tolerance = float(goal_tolerance)
        self.slow_down_rad = math.radians(float(slow_down_deg))
        # How far before the goal the on-axis leg begins, when a goal carries an
        # approach bearing. Long enough that arriving at the staging point
        # slightly off-axis still leaves room to converge; short enough not to
        # be a detour.
        self.staging_distance = float(staging_distance)
        self.staging_tolerance = float(staging_tolerance)
        self.align_tolerance_rad = math.radians(float(align_tolerance_deg))
        # ⚠ Abort if the goal is RECEDING. A robot driving away from its goal is
        # broken, and nothing else in this controller notices: the geofence is
        # measured from the datum, so a 6.5 m goal missed by 100 m sits well
        # inside it. Observed in sim 2026-09-07, where a bad heading estimate
        # sent the robot backwards off the edge of the world while the follower
        # reported a calmly growing "distance to goal" the whole way.
        self.max_goal_distance_growth = float(max_goal_distance_growth)
        # ⚠ HEADING BOOTSTRAP. GPS gives position, never orientation, and this
        # robot's map-frame yaw is gyro-integrated from zero at boot -- so at
        # startup it is wrong by however far the robot happens to be pointing.
        # The dual EKF DOES recover yaw, but only from MOTION: it compares
        # GPS-observed displacement against what the current yaw predicts.
        # Measured in the maize world from a 92 deg error: the estimate decayed
        # 99 -> 19 deg over the drive, which is real convergence but not fast
        # enough on a 6.5 m leg -- the robot missed the row entrance by 5.9 m.
        # Driving straight first buys that convergence before any steering
        # decision depends on it. 0.0 disables it (blank-world testing, or a
        # robot whose heading is already trusted).
        self.heading_init_distance = float(heading_init_distance)
        # ⚠ The bootstrap ends on a MEASUREMENT, not on a distance. A fixed 4 m
        # was tried first and was not enough: from a 92 deg error the robot
        # thrashed through the transit and entered the final approach 1.0 m off
        # the axis, too far to null in 3 m, and drove past the row entrance
        # (sim, 2026-09-07). How far the estimate needs is a property of how
        # wrong it started, which nothing knows in advance.
        #
        # The test is course over ground: while driving straight, the direction
        # the robot ACTUALLY moved (from GPS-fused positions) is the truth its
        # yaw estimate should agree with. When they agree, the estimate has
        # converged and steering on it is safe. heading_init_distance is the
        # MINIMUM to drive first (a course computed over a few centimetres is
        # noise); the max is the backstop, and must fit the open ground ahead.
        self.heading_init_max_distance = float(heading_init_max_distance)
        self.heading_init_tolerance_rad = math.radians(
            float(heading_init_tolerance_deg))
        self._heading_init_error = None
        # ⚠ Arrival on a DIRECTED approach is crossing the goal's plane, not
        # getting within a radius of the point. Euclidean distance was tried
        # first and it fails in a way that looks like nothing: the robot passed
        # 0.46 m to the side of a row entrance, so the distance bottomed out at
        # 0.47 m, never reached the 0.30 m tolerance, and the follower drove
        # calmly on up the field (sim, 2026-09-07). Crossing the plane is a
        # thing that definitely happens; getting within a radius is not.
        # The cross tolerance is what makes it safe: half a row spacing, so
        # "arrived" cannot mean "arrived at the next corridor".
        self.arrival_cross_tolerance = float(arrival_cross_tolerance)
        self.max_approach_attempts = int(max_approach_attempts)
        self._approach_attempts = 0
        self.axis_gain = float(axis_gain)
        self.max_axis_correction_rad = math.radians(float(max_axis_correction_deg))

        self.state = STATE_IDLE
        self._goal = None
        self._approach_bearing = None
        self._staging = None
        self._last_distance = None
        self._last_heading_error = None

    # ---- goal handling ---------------------------------------------------

    def set_goal(self, goal_xy, approach_bearing=None, approach_distance=None):
        """Accept a new goal (x, y) in the map frame, and start driving.

        A new goal always restarts from GOTO, including out of ARRIVED -- that
        is what makes ARRIVED's latch safe to have.

        `approach_bearing` (radians, ENU, CCW from east) turns the goal from a
        POINT into a point plus a DIRECTION. A row entrance is exactly that: it
        is no use arriving there sideways, because vision nav has to pick up
        with the corridor in view.

        Given a bearing, the follower inserts a STAGING POINT
        `approach_distance` metres back along it, drives there first, turns to
        the bearing, and only then runs the final leg ON THE ROW AXIS. Arrival
        heading is then forced by geometry rather than trusted from the heading
        estimate -- which matters, because that estimate is unbounded at rest
        and was measured 74 degrees wrong on a cold start (HANDOFF3 0g).
        """
        self._goal = (float(goal_xy[0]), float(goal_xy[1]))
        self._last_distance = None
        self._last_heading_error = None
        self._closest_distance = None
        self._init_start_xy = None
        self._approach_attempts = 0
        if approach_bearing is None:
            self._approach_bearing = None
            self._staging = None
        else:
            bearing = geo.wrap_angle(float(approach_bearing))
            back = (float(approach_distance) if approach_distance is not None
                    else self.staging_distance)
            self._approach_bearing = bearing
            self._staging = (self._goal[0] - back * math.cos(bearing),
                             self._goal[1] - back * math.sin(bearing))
        self.state = (STATE_HEADING_INIT if self.heading_init_distance > 0.0
                      else STATE_GOTO)

    def clear_goal(self):
        """Drop the current goal and stop. Used by the node's pause path."""
        self._goal = None
        self._approach_bearing = None
        self._staging = None
        self.state = STATE_IDLE
        self._last_distance = None
        self._last_heading_error = None

    @property
    def goal(self):
        return self._goal

    @property
    def approach_bearing(self):
        return self._approach_bearing

    @property
    def staging_point(self):
        """The inserted on-axis waypoint, or None when the goal has no bearing."""
        return self._staging

    # ---- telemetry, for the node's status line ---------------------------

    def distance_remaining(self):
        return self._last_distance

    def heading_error(self):
        return self._last_heading_error

    def heading_init_error(self):
        """Last course-over-ground vs estimated-yaw disagreement, or None.

        This is the number that decides when the bootstrap has done its job,
        and it is worth logging: a bootstrap that ends on its max distance with
        this still large means the heading never converged.
        """
        return self._heading_init_error

    # ---- the tick --------------------------------------------------------

    def update(self, pose, now=None):
        """Advance one tick. Returns (linear_x, angular_z, state, done).

        `pose` is (x, y, yaw) in the map frame, or None when no pose estimate is
        available. `now` is accepted and ignored: unlike MissionFSM this
        controller has no time-based debounce, and taking the argument keeps the
        two update() signatures interchangeable for a future supervisor.

        A None pose stops the robot but does NOT change state -- losing a frame
        is not the same as losing the goal, and the mission should resume when
        the estimate comes back.
        """
        del now  # see docstring

        if self._goal is None or self.state in (STATE_IDLE, STATE_ARRIVED,
                                               STATE_FAILED):
            return 0.0, 0.0, self.state, self.state == STATE_ARRIVED

        if pose is None:
            return 0.0, 0.0, self.state, False

        pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        distance = geo.distance(pose[:2], self._goal)
        self._last_distance = distance

        if self.state == STATE_HEADING_INIT:
            # Drive straight. The DIRECTION does not matter -- any motion gives
            # the filter the GPS-vs-prediction disagreement it needs -- so this
            # deliberately does not steer, because steering on the heading we
            # are trying to establish is the circularity being broken.
            if self._init_start_xy is None:
                self._init_start_xy = pose[:2]
            gone = geo.distance(pose[:2], self._init_start_xy)
            if gone >= self.heading_init_distance:
                # Course over ground: the direction the robot ACTUALLY went.
                # That is the truth the yaw estimate has to agree with.
                course = geo.bearing_to(self._init_start_xy, pose[:2])
                self._heading_init_error = geo.wrap_angle(course - pose[2])
                converged = (abs(self._heading_init_error)
                             <= self.heading_init_tolerance_rad)
            else:
                converged = False
            if not converged and gone < self.heading_init_max_distance:
                self._last_heading_error = 0.0
                return self.approach_speed, 0.0, STATE_HEADING_INIT, False
            self.state = STATE_GOTO
            # The bootstrap legitimately drives away from the goal, so the
            # receding-goal abort must not count it.
            self._closest_distance = None

        # Receding-goal abort. Measured against the CLOSEST approach so far,
        # not the starting distance: a robot that got within 2 m and is now
        # 8 m out has failed just as surely as one that never got close, and
        # comparing against the start would excuse it.
        if self._closest_distance is None or distance < self._closest_distance:
            self._closest_distance = distance
        elif distance > self._closest_distance + self.max_goal_distance_growth:
            self.state = STATE_FAILED
            return 0.0, 0.0, STATE_FAILED, False

        if self._staging is not None:
            return self._update_on_axis(pose)
        return self._update_direct(pose)

    # ---- goal with no bearing: drive straight at it -----------------------

    def _update_direct(self, pose):
        x, y, _yaw = pose
        distance = geo.distance((x, y), self._goal)
        if distance <= self.goal_tolerance:
            self.state = STATE_ARRIVED
            self._last_heading_error = 0.0
            return 0.0, 0.0, STATE_ARRIVED, True

        self.state = STATE_APPROACH if distance <= self.approach_distance else STATE_GOTO
        return self._drive_toward(self._goal, pose)

    # ---- goal with a bearing: staging point, align, then the axis ---------

    def _update_on_axis(self, pose):
        """GOTO(staging) -> ALIGN -> APPROACH(on-axis) -> ARRIVED.

        ALIGN is a real state rather than leaning on the turn-in-place rule,
        because that rule tolerates up to `turn_in_place_deg` (30) of error
        before it fires. Starting the final leg 30 degrees off would leave
        residual heading error at the goal, and the whole point of the bearing
        is that the arrival heading is tight.
        """
        x, y, yaw = pose

        if self.state == STATE_GOTO:
            if geo.distance((x, y), self._staging) > self.staging_tolerance:
                return self._drive_toward(self._staging, pose)
            self.state = STATE_ALIGN

        if self.state == STATE_ALIGN:
            error = geo.wrap_angle(self._approach_bearing - yaw)
            self._last_heading_error = error
            if abs(error) > self.align_tolerance_rad:
                return (0.0, math.copysign(self.turn_in_place_rate, error),
                        STATE_ALIGN, False)
            self.state = STATE_APPROACH

        # Arrival is crossing the plane through the goal perpendicular to the
        # approach bearing -- see arrival_cross_tolerance in __init__.
        bearing = self._approach_bearing
        dx, dy = x - self._goal[0], y - self._goal[1]
        along = dx * math.cos(bearing) + dy * math.sin(bearing)
        cross = -dx * math.sin(bearing) + dy * math.cos(bearing)

        if along >= -self.goal_tolerance:
            if abs(cross) <= self.arrival_cross_tolerance:
                self.state = STATE_ARRIVED
                return 0.0, 0.0, STATE_ARRIVED, True
            # Crossed the plane, but too far off the axis to call it arrived.
            # Go round again from the staging point rather than driving on:
            # continuing is what sent the robot up the field last time.
            self._approach_attempts += 1
            if self._approach_attempts >= self.max_approach_attempts:
                self.state = STATE_FAILED
                return 0.0, 0.0, STATE_FAILED, False
            self.state = STATE_GOTO
            self._closest_distance = None
            return 0.0, 0.0, STATE_GOTO, False

        return self._drive_along_axis(pose)

    # ---- the shared steering and speed law --------------------------------

    def _drive_toward(self, target, pose):
        """Steer and pace straight at `target`. Does NOT decide state."""
        distance = geo.distance(pose[:2], target)
        heading_error = geo.wrap_angle(
            geo.bearing_to(pose[:2], target) - pose[2])
        linear_x, angular_z = self._command(distance, heading_error)
        return linear_x, angular_z, self.state, False

    def _drive_along_axis(self, pose):
        """Track the row AXIS, not the goal point.

        ⚠ This is the difference between arriving on the requested bearing and
        merely arriving. Steering at the goal point means any lateral offset is
        corrected in the last metre, so the robot swings as it arrives: measured
        in sim, a final leg aimed at the point arrived 41 degrees off a bearing
        it had aligned to correctly moments earlier. Tracking the line instead
        makes the heading error decay to zero as the offset does, because the
        commanded heading IS the bearing once the robot is on the line.
        """
        x, y, yaw = pose
        bearing = self._approach_bearing
        # Signed cross-track: positive means the robot is LEFT of the axis.
        dx, dy = x - self._goal[0], y - self._goal[1]
        cross = -dx * math.sin(bearing) + dy * math.cos(bearing)
        # Turn back toward the line, saturating so the approach never departs
        # more than max_axis_correction from the axis even far off it.
        correction = -math.atan(self.axis_gain * cross)
        correction = max(-self.max_axis_correction_rad,
                         min(self.max_axis_correction_rad, correction))
        heading_error = geo.wrap_angle(bearing + correction - yaw)
        linear_x, angular_z = self._command(
            geo.distance((x, y), self._goal), heading_error)
        return linear_x, angular_z, self.state, False

    def _command(self, distance, heading_error):
        """(distance to goal, heading error) -> (linear_x, angular_z)."""
        self._last_heading_error = heading_error

        # Badly misaligned: turn in place. Driving forward while 90 deg off
        # covers ground in the wrong direction, and near the goal it produces
        # the classic orbit -- circling a point it can never turn tightly
        # enough to reach.
        if abs(heading_error) > self.turn_in_place_rad:
            return 0.0, math.copysign(self.turn_in_place_rate, heading_error)

        angular_z = self.heading_gain * heading_error
        angular_z = max(-self.angular_z_max, min(self.angular_z_max, angular_z))

        # Base speed: cruise, ramped linearly down to approach_speed over the
        # last approach_distance metres, so the robot decelerates into the goal
        # rather than stopping dead the instant it crosses the tolerance. At
        # distance 0 this is exactly approach_speed -- the ramp IS the floor,
        # which is why there is no second derate here (see the module docstring).
        if distance < self.approach_distance:
            base = self.approach_speed + (
                self.linear_x_cruise - self.approach_speed
            ) * (distance / self.approach_distance)
        else:
            base = self.linear_x_cruise

        # Derate with heading error, so a correcting turn is tighter than a
        # cruising one. kappa = angular_z / linear_x is what actually determines
        # the path, and the same coupling bit this workspace once already
        # (HANDOFF3 0f: raising speed without raising the turn limit drove the
        # robot over every plant). Bounded below by turn_in_place_deg <
        # slow_down_deg, so this cannot approach zero while still driving.
        linear_x = base * max(0.0, 1.0 - abs(heading_error) / self.slow_down_rad)

        return linear_x, angular_z
