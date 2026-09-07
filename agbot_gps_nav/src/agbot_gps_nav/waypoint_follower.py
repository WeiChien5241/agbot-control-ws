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
STATE_GOTO = "GOTO"
STATE_APPROACH = "APPROACH"
STATE_ARRIVED = "ARRIVED"


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
                 slow_down_deg=60.0):
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

        self.state = STATE_IDLE
        self._goal = None
        self._last_distance = None
        self._last_heading_error = None

    # ---- goal handling ---------------------------------------------------

    def set_goal(self, goal_xy):
        """Accept a new goal (x, y) in the map frame, and start driving.

        A new goal always restarts from GOTO, including out of ARRIVED -- that
        is what makes ARRIVED's latch safe to have.
        """
        self._goal = (float(goal_xy[0]), float(goal_xy[1]))
        self.state = STATE_GOTO
        self._last_distance = None
        self._last_heading_error = None

    def clear_goal(self):
        """Drop the current goal and stop. Used by the node's pause path."""
        self._goal = None
        self.state = STATE_IDLE
        self._last_distance = None
        self._last_heading_error = None

    @property
    def goal(self):
        return self._goal

    # ---- telemetry, for the node's status line ---------------------------

    def distance_remaining(self):
        return self._last_distance

    def heading_error(self):
        return self._last_heading_error

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

        if self._goal is None or self.state in (STATE_IDLE, STATE_ARRIVED):
            return 0.0, 0.0, self.state, self.state == STATE_ARRIVED

        if pose is None:
            return 0.0, 0.0, self.state, False

        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        distance = geo.distance((x, y), self._goal)
        heading_error = geo.wrap_angle(geo.bearing_to((x, y), self._goal) - yaw)
        self._last_distance = distance
        self._last_heading_error = heading_error

        if distance <= self.goal_tolerance:
            self.state = STATE_ARRIVED
            return 0.0, 0.0, STATE_ARRIVED, True

        self.state = STATE_APPROACH if distance <= self.approach_distance else STATE_GOTO

        # Badly misaligned: turn in place. Driving forward while 90 deg off
        # covers ground in the wrong direction, and near the goal it produces
        # the classic orbit -- circling a point it can never turn tightly
        # enough to reach.
        if abs(heading_error) > self.turn_in_place_rad:
            rate = math.copysign(self.turn_in_place_rate, heading_error)
            return 0.0, rate, self.state, False

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

        return linear_x, angular_z, self.state, False
