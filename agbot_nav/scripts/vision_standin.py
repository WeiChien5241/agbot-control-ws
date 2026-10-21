#!/usr/bin/env python3
"""SIM ONLY: a stand-in for vision_nav_node when the real one cannot run.

    rosrun agbot_nav vision_standin.py __name:=vision_nav_node _row_distance:=9.0

The dev laptop has no torch / lightly_train / weights (CLAUDE.md), so the real
row controller cannot run there. This offers the same interface the deploy
supervisor uses -- `~set_enabled` (std_srvs/SetBool, true = go) and a LATCHED
`~mission_done` (std_msgs/Bool, reset to False on enable). Disabled, it
publishes nothing (a disabled node goes silent, never zeros -- see
agbot_gps_nav's twist_mux note). It has NO camera, NO segmentation and NO
MPC, so there is no /vision_nav_node/debug/image while it drives.

Two modes:
  ~oracle false  ONE row, STRAIGHT and BLIND: `~row_distance` m of odometry.
                 A test of the TRANSIT alone: it only clears the plants if
                 the handoff really was on the axis and pointing along it. A
                 5.6 deg handoff error put it 0.8 m into the corn over 8 m
                 (2026-10-14).
  ~oracle true   a perfect vision nav, steering from GAZEBO GROUND TRUTH:
                 `~num_rows` corridors, boustrophedon, like the real mission
                 FSM. Row k is the entrance's axis (/deploy_supervisor/
                 entrance, whatever the operator clicked) shifted k *
                 `~row_spacing` toward `~first_turn_direction`, driven
                 `~row_distance` m from where row 1 started (through the row
                 and out into the headland) and alternately back. Between
                 rows: turn in place, cross one row spacing, turn in place.
                 Cheating by design; sim only. ⚠ It steers on the CLICKED
                 axis: a sloppy 2D Nav Goal arrow drives it into the corn,
                 where the real vision nav would follow the actual rows.
"""
import math

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

TURN_RATE = 0.6          # rad/s ceiling while turning in place
TURN_DONE = math.radians(3.0)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VisionStandin(object):

    def __init__(self):
        rospy.init_node("vision_nav_node")
        self._distance = rospy.get_param("~row_distance", 9.0)
        self._speed = float(rospy.get_param("~speed", 0.3))
        self._oracle = rospy.get_param("~oracle", False)
        self._num_rows = int(rospy.get_param("~num_rows", 1)) if self._oracle else 1
        self._spacing = float(rospy.get_param("~row_spacing", 0.76))
        turn = str(rospy.get_param("~first_turn_direction", "left")).strip().lower()
        self._side = 1.0 if turn == "left" else -1.0       # REP-103: left = +y
        self._axis = None                                  # (x, y, yaw) of row 1
        if self._oracle:
            rospy.Subscriber("/deploy_supervisor/entrance", PoseStamped,
                             self._entrance_cb, queue_size=1)
            rospy.wait_for_service("/gazebo/get_model_state")
            self._truth = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self._enabled = False
        self._start = None
        self._odom = None
        self._done = False
        self._legs = []
        self._cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._done_pub = rospy.Publisher("~mission_done", Bool, queue_size=1, latch=True)
        self._done_pub.publish(Bool(data=False))
        rospy.Subscriber("/odometry/filtered", Odometry, self._odom_cb, queue_size=1)
        rospy.Service("~set_enabled", SetBool, self._set_enabled)
        rospy.Timer(rospy.Duration(0.1), self._tick)
        if self._oracle:
            rospy.logwarn("vision_standin: SIM STAND-IN steering from ground truth, "
                          "%d row(s) of %.1f m at %.2f m/s, first turn %s -- this is "
                          "NOT vision nav (no camera, no MPC, no debug image)",
                          self._num_rows, self._distance, self._speed, turn)
        else:
            rospy.logwarn("vision_standin: SIM STAND-IN, drives %.1f m straight and "
                          "blind when enabled -- this is NOT vision nav", self._distance)

    def _odom_cb(self, msg):
        self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _entrance_cb(self, msg):
        p = msg.pose
        self._axis = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _set_enabled(self, req):
        enabled = bool(req.data)
        if enabled and not self._enabled:
            self._start = None
            self._legs = []
            self._done = False
            self._done_pub.publish(Bool(data=False))
        if not enabled and self._enabled:
            self._cmd.publish(Twist())              # one stop, then silent
        self._enabled = enabled
        return SetBoolResponse(success=True, message="enabled" if enabled else "disabled")

    def _finish(self, message):
        self._cmd.publish(Twist())
        self._done = True
        self._done_pub.publish(Bool(data=True))
        rospy.loginfo("vision_standin: %s, mission done", message)

    def _tick(self, _event):
        if not self._enabled or self._done:
            return
        if self._oracle:
            self._tick_oracle()
            return
        if self._odom is None:
            return
        if self._start is None:
            self._start = self._odom
        driven = math.hypot(self._odom[0] - self._start[0], self._odom[1] - self._start[1])
        if driven >= self._distance:
            self._finish("drove %.2f m" % driven)
            return
        cmd = Twist()
        cmd.linear.x = self._speed
        self._cmd.publish(cmd)

    # ---- oracle: a plan of legs, each followed on ground truth --------------

    def _plan(self, x, y):
        """Legs from the robot's true start (x, y), in the entrance frame.

        ("line", ox, oy, heading, length): follow the line through (ox, oy)
        along heading until `length` m of progress along it.
        ("turn", heading): turn in place to it.
        """
        ax, ay, ayaw = self._axis
        c, s = math.cos(ayaw), math.sin(ayaw)
        u0 = (x - ax) * c + (y - ay) * s          # along-axis start, ~ -standoff
        u1 = u0 + self._distance                  # far headland end

        def world(u, v):
            return ax + u * c - v * s, ay + u * s + v * c

        legs = []
        for k in range(self._num_rows):
            v = k * self._spacing * self._side
            forward = k % 2 == 0
            u_from, u_to = (u0, u1) if forward else (u1, u0)
            heading = ayaw if forward else wrap(ayaw + math.pi)
            ox, oy = world(u_from, v)
            legs.append(("line", ox, oy, heading, abs(u_to - u_from), "row %d" % (k + 1)))
            if k + 1 < self._num_rows:
                across = wrap(ayaw + self._side * math.pi / 2.0)
                ox, oy = world(u_to, v)
                legs.append(("turn", across))
                legs.append(("line", ox, oy, across, self._spacing, "headland"))
                legs.append(("turn", wrap(heading + math.pi)))
        return legs

    def _tick_oracle(self):
        if self._axis is None:
            rospy.logwarn_throttle(2.0, "vision_standin: no /deploy_supervisor/entrance yet")
            return
        pose = self._truth("jackal", "").pose
        x, y, yaw = pose.position.x, pose.position.y, yaw_of(pose.orientation)
        if self._start is None:
            self._start = (x, y)
            self._legs = self._plan(x, y)
        cmd = Twist()
        while self._legs:
            leg = self._legs[0]
            if leg[0] == "turn":
                err = wrap(leg[1] - yaw)
                if abs(err) < TURN_DONE:
                    self._legs.pop(0)
                    continue
                cmd.angular.z = max(-TURN_RATE, min(TURN_RATE, 2.0 * err))
                break
            _, ox, oy, heading, length, name = leg
            dx, dy = x - ox, y - oy
            along = dx * math.cos(heading) + dy * math.sin(heading)
            if along >= length:
                self._legs.pop(0)
                if name != "headland":
                    rospy.loginfo("vision_standin: %s done", name)
                continue
            cross = -dx * math.sin(heading) + dy * math.cos(heading)  # left > 0
            target = heading - math.atan2(cross, 1.0)    # aim 1 m ahead on the line
            err = wrap(target - yaw)
            cmd.linear.x = self._speed if abs(err) < math.radians(30) else 0.0
            cmd.angular.z = max(-TURN_RATE, min(TURN_RATE, 1.5 * err))
            break
        if not self._legs:
            self._finish("drove %d row(s)" % self._num_rows)
            return
        self._cmd.publish(cmd)


if __name__ == "__main__":
    VisionStandin()
    rospy.spin()
