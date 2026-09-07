#!/usr/bin/python3
"""GPS waypoint navigation node -- the ONLY file in this package that imports
rospy. Everything it decides lives in src/agbot_gps_nav/ and is unit-tested
without ROS.

WHAT IT DOES. Takes a goal (from RViz, from mapviz, or as a lat/lon), converts
it into the georeferenced `map` frame, and drives there on /cmd_vel until it
arrives. Then it stops. That is the whole job: open-sky transit from the
trailer to the front of a row, where vision nav takes over.

TOPICS

  in   /odometry/filtered/global  nav_msgs/Odometry     map-frame pose. NOT
                                                        /odometry/filtered --
                                                        that is the stock
                                                        odom-frame EKF.
       gps/fix                    sensor_msgs/NavSatFix fix age + quality gate
                                                        only; never used for
                                                        control directly.
       /move_base_simple/goal     PoseStamped           RViz "2D Nav Goal"
       ~goal_wgs84                PointStamped          x=LONGITUDE, y=latitude
  out  /cmd_vel                   Twist
       ~status                    String (latched)

⚠ ONE SUBSCRIBER SERVES BOTH GEO GOAL SOURCES. mapviz's point_click_publisher
emits a PointStamped in a "wgs84" frame with x=longitude and y=latitude -- note
the order, it is the opposite of how everyone says it out loud. Publishing that
message by hand is also the documented way to type a lat/lon, so there is one
code path rather than two.

SAFETY. This robot drives itself across open ground with nobody in front of it,
so four gates stand between a goal and a moving robot, and all four publish a
zero twist rather than simply declining to publish:

  fix age        a stale fix means the receiver died; keep driving on the last
                 known position and the robot leaves the field.
  fix quality    ⚠ the hector sim plugin reports status 0 (STATUS_FIX), never 2
                 (STATUS_GBAS_FIX / RTK). A gate written for the field rejects
                 all sim data, so min_fix_status defaults to 0 and the FIELD
                 value is 2. Set it before rolling.
  geofence       a bad datum or a bad fix otherwise drives the robot
                 arbitrarily far in a straight line. With no heading estimator
                 yet, this is the only thing standing between a heading-sign
                 error and a robot in the next county.
  pose age       no map-frame pose means no idea where we are.

CMD_VEL. Publishes to /cmd_vel, which twist_mux gives priority 1 while both
joystick inputs sit at 9 and 10 -- so a human with the pad always outranks this
node. twist_mux also times every input out after 0.5 s, hence the 10 Hz
keep-alive republish. Same arrangement as vision_nav_node.
"""

import math
import threading

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse

from agbot_gps_nav import geo
from agbot_gps_nav.waypoint_follower import (
    STATE_ARRIVED, STATE_FAILED, STATE_HEADING_INIT, STATE_IDLE, WaypointFollower)


def _quaternion_to_yaw(q):
    """Yaw from a quaternion. Avoids a tf dependency, like vision_nav_node."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GpsNavNode(object):

    def __init__(self):
        rospy.init_node("gps_nav_node")

        datum = rospy.get_param("~datum", None)
        if datum is None or len(datum) < 2:
            rospy.logfatal(
                "~datum is required and must be [lat, lon, yaw] -- load "
                "agbot_gps_nav/config/gps_datum.yaml into this node's namespace.")
            raise rospy.ROSInitException("no datum")
        self._datum = (float(datum[0]), float(datum[1]))

        self._follower = WaypointFollower(
            linear_x_cruise=rospy.get_param("~linear_x_cruise", 0.4),
            approach_speed=rospy.get_param("~approach_speed", 0.15),
            angular_z_max=rospy.get_param("~angular_z_max", 0.6),
            heading_gain=rospy.get_param("~heading_gain", 1.2),
            turn_in_place_deg=rospy.get_param("~turn_in_place_deg", 30.0),
            turn_in_place_rate=rospy.get_param("~turn_in_place_rate", 0.4),
            approach_distance=rospy.get_param("~approach_distance", 2.0),
            goal_tolerance=rospy.get_param("~goal_tolerance", 0.3),
            slow_down_deg=rospy.get_param("~slow_down_deg", 60.0),
            staging_distance=rospy.get_param("~staging_distance", 3.0),
            staging_tolerance=rospy.get_param("~staging_tolerance", 0.25),
            align_tolerance_deg=rospy.get_param("~align_tolerance_deg", 5.0),
            axis_gain=rospy.get_param("~axis_gain", 1.0),
            max_axis_correction_deg=rospy.get_param(
                "~max_axis_correction_deg", 45.0),
            max_goal_distance_growth=rospy.get_param(
                "~max_goal_distance_growth", 5.0),
            heading_init_distance=rospy.get_param("~heading_init_distance", 0.0),
            heading_init_max_distance=rospy.get_param(
                "~heading_init_max_distance", 8.0),
            heading_init_tolerance_deg=rospy.get_param(
                "~heading_init_tolerance_deg", 12.0),
            arrival_cross_tolerance=rospy.get_param(
                "~arrival_cross_tolerance", 0.35),
            max_approach_attempts=rospy.get_param("~max_approach_attempts", 3),
        )
        # RViz's 2D Nav Goal carries an orientation (you drag to set it), but a
        # plain click sends a meaningless one, and honouring that would impose
        # "arrive facing east" on every casual click plus the staging detour it
        # implies. Off by default; ~goal_pose below is the unambiguous channel.
        self._use_goal_orientation = bool(
            rospy.get_param("~use_goal_orientation", False))

        self._control_rate = float(rospy.get_param("~control_rate", 20.0))
        self._max_fix_age_sec = float(rospy.get_param("~max_fix_age_sec", 2.0))
        self._max_pose_age_sec = float(rospy.get_param("~max_pose_age_sec", 1.0))
        self._min_fix_status = int(rospy.get_param("~min_fix_status", 0))
        self._geofence_radius_m = float(rospy.get_param("~geofence_radius_m", 200.0))
        self._require_fix = bool(rospy.get_param("~require_fix", True))
        self._refusal_hold_sec = float(rospy.get_param("~refusal_hold_sec", 5.0))

        self._state_lock = threading.Lock()
        self._pose = None                 # (x, y, yaw) in map
        self._pose_time = None
        self._fix = None                  # (status, lat, lon)
        self._fix_time = None
        self._paused = False
        self._enabled = bool(rospy.get_param("~start_enabled", True))
        self._block_reason = None
        self._last_state = STATE_IDLE
        # A refusal is a one-shot event, but ~status is republished at
        # control_rate, so without a hold the message is overwritten within
        # 50 ms and the operator never sees why nothing happened.
        # (text, stamp) as ONE field: the control thread reads this without the
        # lock, and two separate fields can be seen half-written.
        self._refusal = None

        cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self._cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self._status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)

        odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered/global")
        fix_topic = rospy.get_param("~gps_fix_topic", "/gps/fix")
        rospy.Subscriber(odom_topic, Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber(fix_topic, NavSatFix, self._fix_cb, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._rviz_goal_cb,
                         queue_size=1)
        rospy.Subscriber("~goal_wgs84", PointStamped, self._wgs84_goal_cb, queue_size=1)
        rospy.Subscriber("~goal_pose", PoseStamped, self._pose_goal_cb, queue_size=1)

        rospy.Service("~pause", SetBool, self._pause_srv)
        rospy.Service("~set_enabled", SetBool, self._set_enabled_srv)

        self._log_config(cmd_vel_topic, odom_topic, fix_topic)
        self._timer = rospy.Timer(rospy.Duration(1.0 / self._control_rate),
                                  self._control_cb)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("gps_nav_node ready; waiting for a goal on "
                      "/move_base_simple/goal or ~goal_wgs84")

    # ---- callbacks -------------------------------------------------------

    def _odom_cb(self, msg):
        yaw = _quaternion_to_yaw(msg.pose.pose.orientation)
        with self._state_lock:
            self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
            self._pose_time = rospy.Time.now()

    def _fix_cb(self, msg):
        with self._state_lock:
            self._fix = (msg.status.status, msg.latitude, msg.longitude)
            self._fix_time = rospy.Time.now()

    def _rviz_goal_cb(self, msg):
        """RViz 2D Nav Goal. Already in the map frame, so no conversion.

        The frame is checked rather than assumed: RViz publishes whatever its
        Fixed Frame is set to, and a goal clicked in `odom` looks identical to
        one clicked in `map` while meaning somewhere else entirely.
        """
        frame = msg.header.frame_id.lstrip("/")
        if frame and frame != "map":
            rospy.logwarn("ignoring goal in frame '%s' -- set RViz's Fixed Frame "
                          "to 'map'. A goal expressed in '%s' names a different "
                          "physical place, and nothing downstream could tell.",
                          frame, frame)
            return
        goal = (msg.pose.position.x, msg.pose.position.y)
        bearing = (_quaternion_to_yaw(msg.pose.orientation)
                   if self._use_goal_orientation else None)
        lat, lon = geo.enu_to_latlon(goal[0], goal[1], self._datum)
        rospy.loginfo("goal from RViz: map (%.2f, %.2f) = %.7f, %.7f%s",
                      goal[0], goal[1], lat, lon,
                      "" if bearing is None
                      else "  approach bearing %.0f deg" % math.degrees(bearing))
        self._accept_goal(goal, approach_bearing=bearing)

    def _wgs84_goal_cb(self, msg):
        """mapviz click, or a hand-published lat/lon.

        ⚠ x is LONGITUDE and y is LATITUDE. That is mapviz's convention (x is
        the east-west axis), and it is the reverse of how the pair is spoken.
        """
        lat, lon = msg.point.y, msg.point.x
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            rospy.logwarn("ignoring goal_wgs84 (%.6f, %.6f): out of range. This "
                          "message carries x=LONGITUDE, y=latitude -- swapped?",
                          msg.point.x, msg.point.y)
            return
        goal = geo.latlon_to_enu(lat, lon, self._datum)
        rospy.loginfo("goal from WGS84: lat %.7f, lon %.7f = map (%.2f, %.2f)",
                      lat, lon, goal[0], goal[1])
        self._accept_goal(goal, hint=self._swap_hint(lat, lon))

    def _pose_goal_cb(self, msg):
        """A goal whose ORIENTATION IS ALWAYS the approach bearing.

        This is the channel the mission supervisor uses, and it is deliberately
        separate from /move_base_simple/goal: there, the orientation is a UI
        artefact that may or may not mean anything, so honouring it is opt-in.
        Here it is the entire point, and a caller that publishes to this topic
        has said so by choosing it.
        """
        frame = msg.header.frame_id.lstrip("/")
        if frame and frame != "map":
            rospy.logwarn("ignoring goal_pose in frame '%s' -- this topic is "
                          "map-frame only.", frame)
            return
        goal = (msg.pose.position.x, msg.pose.position.y)
        bearing = _quaternion_to_yaw(msg.pose.orientation)
        rospy.loginfo("goal from ~goal_pose: map (%.2f, %.2f), approach bearing "
                      "%.0f deg", goal[0], goal[1], math.degrees(bearing))
        self._accept_goal(goal, approach_bearing=bearing)

    def _accept_goal(self, goal_xy, hint=None, approach_bearing=None):
        """Common goal entry point. Refuses anything outside the geofence."""
        distance = math.hypot(goal_xy[0], goal_xy[1])
        if distance > self._geofence_radius_m:
            rospy.logerr("REFUSING goal %.1f m from the datum; geofence is %.1f m. "
                         "Either the datum is wrong or the goal is.%s",
                         distance, self._geofence_radius_m,
                         (" " + hint) if hint else "")
            self._refuse("REFUSED: goal %.0f m out, geofence %.0f m%s"
                         % (distance, self._geofence_radius_m,
                            ("; " + hint) if hint else ""))
            return
        with self._state_lock:
            self._follower.set_goal(goal_xy, approach_bearing=approach_bearing)
            self._block_reason = None
            self._refusal = None

    def _refuse(self, text):
        """Record a refusal so it survives on ~status long enough to be read."""
        with self._state_lock:
            self._refusal = (text, rospy.Time.now())
        self._set_status(text)

    def _swap_hint(self, lat, lon):
        """If reading this pair the other way round would land inside the
        geofence, say so. A swapped lat/lon whose latitude happens to be a
        legal one (|lat| <= 90) cannot be caught by a range check -- this is
        the check that actually names the mistake."""
        try:
            east, north = geo.latlon_to_enu(lon, lat, self._datum)
        except (ValueError, TypeError):
            return None
        if math.hypot(east, north) <= self._geofence_radius_m:
            return ("swapping them lands inside the geofence -- this message "
                    "carries x=LONGITUDE, y=latitude")
        return None

    def _pause_srv(self, req):
        """Hold the robot, but stay the active driver.

        Paused KEEPS PUBLISHING ZEROS, on purpose: a paused node is still the
        one in charge of the robot, and zeros hold it stopped immediately
        rather than waiting out twist_mux's 0.5 s timeout. Compare
        ~set_enabled, which hands the robot to someone else and therefore goes
        silent.
        """
        with self._state_lock:
            self._paused = bool(req.data)
            paused = self._paused
        if paused:
            self._publish_twist(0.0, 0.0)
        rospy.loginfo("gps nav %s", "PAUSED" if paused else "RESUMED")
        return SetBoolResponse(success=True, message="paused" if paused else "running")

    def _set_enabled_srv(self, req):
        """Hand the robot to, or take it back from, another autonomy node.

        ⚠ Disabled means SILENT on cmd_vel, not zeros. This node and
        vision_nav_node both publish /cmd_vel, which twist_mux takes as ONE
        input at priority 1 -- it arbitrates by PRIORITY, not rate, so two
        publishers on that topic are not arbitrated at all, they interleave. A
        "stopped" node emitting zeros at 20 Hz would chop the active node's
        commands to pieces. Twist_mux's timeout is harmless here because the
        OTHER node is publishing.

        ⚠ Note the polarity, which is the opposite of ~pause: data=True means
        GO, whereas ~pause data=True means STOP.
        """
        enabled = bool(req.data)
        with self._state_lock:
            changed = enabled != self._enabled
            self._enabled = enabled
            if not enabled:
                # Drop the goal: whatever it was, this node is no longer the
                # one driving toward it, and a stale goal would resume the
                # instant someone re-enabled the node.
                self._follower.clear_goal()
        if not enabled:
            self._publish_twist(0.0, 0.0, force=True)   # one zero, then silence
        if changed:
            rospy.logwarn("gps nav %s", "ENABLED" if enabled else "DISABLED")
        return SetBoolResponse(success=True,
                               message="enabled" if enabled else "disabled")

    # ---- gates -----------------------------------------------------------

    def _blocking_reason(self, now):
        """Return a human-readable reason the robot must not move, or None.

        Order matters only for which message the operator sees first; every one
        of these produces a stop.
        """
        if self._paused:
            return "paused"

        if self._pose is None:
            return "no map-frame pose yet on /odometry/filtered/global"
        pose_age = (now - self._pose_time).to_sec()
        if pose_age > self._max_pose_age_sec:
            return "pose stale by %.1f s" % pose_age

        if self._require_fix:
            if self._fix is None:
                return "no GNSS fix received yet"
            fix_age = (now - self._fix_time).to_sec()
            if fix_age > self._max_fix_age_sec:
                return "GNSS fix stale by %.1f s" % fix_age
            if self._fix[0] < self._min_fix_status:
                return ("fix status %d below required %d"
                        % (self._fix[0], self._min_fix_status))

        distance = math.hypot(self._pose[0], self._pose[1])
        if distance > self._geofence_radius_m:
            return ("GEOFENCE: %.1f m from datum, limit %.1f m"
                    % (distance, self._geofence_radius_m))
        return None

    # ---- control loop ----------------------------------------------------

    def _control_cb(self, _event):
        now = rospy.Time.now()
        with self._state_lock:
            if not self._enabled:
                return          # silent; another node owns the robot
            reason = self._blocking_reason(now)
            if reason is not None:
                changed = reason != self._block_reason
                self._block_reason = reason
                pose = None
            else:
                if self._block_reason is not None:
                    rospy.loginfo("clear: %s -- resuming", self._block_reason)
                self._block_reason = None
                changed = False
                pose = self._pose

            # A blocked tick passes pose=None, which stops the robot without
            # touching the goal: the drive resumes when the gate clears.
            linear_x, angular_z, state, done = self._follower.update(pose, now.to_sec())
            distance = self._follower.distance_remaining()
            init_done = (self._last_state == STATE_HEADING_INIT
                         and state != STATE_HEADING_INIT)
            arrived_now = done and self._last_state != STATE_ARRIVED
            failed_now = state == STATE_FAILED and self._last_state != STATE_FAILED
            self._last_state = state

        if reason is not None and changed:
            rospy.logwarn("holding: %s", reason)

        self._publish_twist(linear_x, angular_z)

        if init_done:
            error = self._follower.heading_init_error()
            rospy.loginfo("heading bootstrap done: yaw vs course driven = %s",
                          "%.1f deg" % math.degrees(error) if error is not None
                          else "not measured")
        if arrived_now:
            rospy.loginfo("ARRIVED (%.2f m from goal). Stopped.", distance or 0.0)
        if failed_now:
            rospy.logerr("ABORTED: the goal is RECEDING (%.1f m away, closest "
                         "approach was %.1f m). The robot is driving away from "
                         "it -- suspect the heading estimate. Stopped; send a "
                         "new goal to clear.", distance or 0.0,
                         self._follower._closest_distance or 0.0)

        # A recent refusal outranks the routine state line: it is the answer
        # to "why is nothing happening", and it is only true for an instant.
        refusal = self._refusal
        if refusal is not None:
            if (now - refusal[1]).to_sec() <= self._refusal_hold_sec:
                self._set_status(refusal[0])
                return
            self._refusal = None

        if reason is not None:
            self._set_status("HOLD %s" % reason)
        elif distance is None:
            self._set_status(state)
        else:
            axis = "" if self._follower.approach_bearing is None else " (on-axis)"
            if state == STATE_FAILED:
                self._set_status("FAILED: goal receding, %.1f m away" % distance)
            else:
                self._set_status("%s%s %.1f m to goal" % (state, axis, distance))

    def _publish_twist(self, linear_x, angular_z, force=False):
        """Publish a command, unless this node has been disabled.

        ⚠ Disabled means SILENT, not zero -- see _set_enabled_srv. `force` is
        for the single zero published at the moment of disabling, which is what
        actually stops the robot.
        """
        if not force and not self._enabled:
            return
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def _set_status(self, text):
        self._status_pub.publish(String(data=text))

    def _on_shutdown(self):
        self._publish_twist(0.0, 0.0)

    # ---- startup config block -------------------------------------------

    def _log_config(self, cmd_vel_topic, odom_topic, fix_topic):
        """One-shot dump of the config that actually resolved.

        Reads back the merged private namespace rather than restating the
        constructor args, so it reports what rosparam holds and not what this
        file's defaults say -- the same reason vision_nav_node does it.
        """
        try:
            f = self._follower
            rospy.loginfo("---- gps_nav config in effect ----")
            rospy.loginfo("datum:    %.7f, %.7f  (map origin; every goal is "
                          "relative to this)", self._datum[0], self._datum[1])
            rospy.loginfo("speed:    cruise=%.2f approach=%.2f ang_max=%.2f "
                          "gain=%.2f", f.linear_x_cruise, f.approach_speed,
                          f.angular_z_max, f.heading_gain)
            rospy.loginfo("arrival:  approach_dist=%.2f m tolerance=%.2f m "
                          "turn_in_place=%.0f deg", f.approach_distance,
                          f.goal_tolerance, math.degrees(f.turn_in_place_rad))
            rospy.loginfo("          on a bearing, arrival is CROSSING THE GOAL "
                          "PLANE within %.2f m of the axis, up to %d attempts",
                          f.arrival_cross_tolerance, f.max_approach_attempts)
            rospy.loginfo("on-axis:  staging=%.2f m (tol %.2f) align=%.0f deg | "
                          "RViz goal orientation %s",
                          f.staging_distance, f.staging_tolerance,
                          math.degrees(f.align_tolerance_rad),
                          "USED" if self._use_goal_orientation else "ignored")
            rospy.loginfo("bootstrap: %s",
                          ("straight %.1f-%.1f m until the yaw agrees with the "
                           "course driven to within %.0f deg"
                           % (f.heading_init_distance, f.heading_init_max_distance,
                              math.degrees(f.heading_init_tolerance_rad)))
                          if f.heading_init_distance > 0
                          else "DISABLED (heading_init_distance 0) -- fine in a "
                               "blank world, NOT on the robot")
            rospy.loginfo("topics:   pose=%s fix=%s cmd=%s",
                          odom_topic, fix_topic, cmd_vel_topic)
            rospy.loginfo("enabled:  %s at startup (~set_enabled to hand over; "
                          "disabled = SILENT on cmd_vel, not zeros)",
                          "YES" if self._enabled else "NO")
            rospy.loginfo("safety:   geofence=%.0f m fix_age<%.1f s pose_age<%.1f s "
                          "min_fix_status=%d%s",
                          self._geofence_radius_m, self._max_fix_age_sec,
                          self._max_pose_age_sec, self._min_fix_status,
                          "" if self._require_fix else "  (FIX GATE DISABLED)")
            if self._min_fix_status < 2:
                rospy.loginfo("          min_fix_status < 2 accepts a NON-RTK fix. "
                              "Correct for simulation (hector reports 0); set 2 "
                              "before any field run.")
            rospy.loginfo("heading:  from odometry yaw via navsat_transform "
                          "(use_odometry_yaw), so it is WRONG at boot and is "
                          "recovered only by MOVING -- the EKF compares GPS "
                          "displacement against what the yaw predicts.")
            rospy.loginfo("----------------------------------")
        except Exception as exc:                 # never let logging kill startup
            rospy.logwarn("could not log config: %s", exc)


def main():
    node = GpsNavNode()
    del node
    rospy.spin()


if __name__ == "__main__":
    main()
