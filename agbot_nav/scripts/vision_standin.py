#!/usr/bin/env python3
"""SIM ONLY: a stand-in for vision_nav_node when the real one cannot run.

    rosrun agbot_nav vision_standin.py __name:=vision_nav_node _row_distance:=9.0

The dev laptop has no torch / lightly_train / weights (CLAUDE.md), so the real
row controller cannot run there. This offers the same interface the deploy
supervisor uses -- `~set_enabled` (std_srvs/SetBool, true = go) and a LATCHED
`~mission_done` (std_msgs/Bool, reset to False on enable) -- and, when enabled,
drives STRAIGHT and BLIND for `~row_distance` metres of odometry, then stops
and reports done.

That makes it a test of the TRANSIT, not of row following: a straight blind
run down a 0.75 m corridor only clears the plants if the handoff pose really
was on the axis and pointing along it. Disabled, it publishes nothing (a
disabled node goes silent, never zeros -- see agbot_gps_nav's twist_mux note).
"""
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse


class VisionStandin(object):

    def __init__(self):
        rospy.init_node("vision_nav_node")
        self._distance = rospy.get_param("~row_distance", 9.0)
        self._speed = rospy.get_param("~speed", 0.3)
        self._enabled = False
        self._start = None
        self._odom = None
        self._done = False
        self._cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._done_pub = rospy.Publisher("~mission_done", Bool, queue_size=1, latch=True)
        self._done_pub.publish(Bool(data=False))
        rospy.Subscriber("/odometry/filtered", Odometry, self._odom_cb, queue_size=1)
        rospy.Service("~set_enabled", SetBool, self._set_enabled)
        rospy.Timer(rospy.Duration(0.1), self._tick)
        rospy.logwarn("vision_standin: SIM STAND-IN, drives %.1f m straight and blind "
                      "when enabled -- this is NOT vision nav", self._distance)

    def _odom_cb(self, msg):
        self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _set_enabled(self, req):
        enabled = bool(req.data)
        if enabled and not self._enabled:
            self._start = None
            self._done = False
            self._done_pub.publish(Bool(data=False))
        if not enabled and self._enabled:
            self._cmd.publish(Twist())              # one stop, then silent
        self._enabled = enabled
        return SetBoolResponse(success=True, message="enabled" if enabled else "disabled")

    def _tick(self, _event):
        if not self._enabled or self._done or self._odom is None:
            return
        if self._start is None:
            self._start = self._odom
        driven = math.hypot(self._odom[0] - self._start[0], self._odom[1] - self._start[1])
        if driven >= self._distance:
            self._cmd.publish(Twist())
            self._done = True
            self._done_pub.publish(Bool(data=True))
            rospy.loginfo("vision_standin: drove %.2f m, mission done", driven)
            return
        cmd = Twist()
        cmd.linear.x = self._speed
        self._cmd.publish(cmd)


if __name__ == "__main__":
    VisionStandin()
    rospy.spin()
