#!/usr/bin/env python3
"""Runs a deployment: reverse off the trailer -> localize -> move_base to the
row -> vision nav drives the rows -> move_base back to the drop-off point.

The only rospy file around `agbot_nav.deploy_fsm`, which makes every decision
and is unit-tested; this is plumbing.

    roslaunch agbot_nav deploy_mission.launch ...

⚠ EXACTLY ONE COMMANDER ON /cmd_vel. This node (reverse leg), move_base and
vision nav all publish /cmd_vel, which twist_mux takes as ONE input and does
not arbitrate between publishers. The FSM names the commander on every tick;
this node switches the outgoing one off BEFORE switching anything on --
vision is disabled before a move_base goal is sent, a goal is cancelled
before vision is enabled -- and it publishes reverse commands only while it
is the commander.
"""

import math
import os
import sys

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, SetBool

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from agbot_nav.deploy_fsm import (  # noqa: E402
    COMMANDER_SUPERVISOR, NAV_ACTIVE, NAV_FAILED, NAV_SUCCEEDED,
    DeployFSM, Estimate,
)

_ACTIVE = (GoalStatus.PENDING, GoalStatus.ACTIVE)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DeploySupervisor(object):

    def __init__(self):
        rospy.init_node("deploy_supervisor")
        p = rospy.get_param
        entrance = (p("~entrance_x"), p("~entrance_y"),
                    math.radians(p("~entrance_yaw_deg")))
        self._fsm = DeployFSM(
            entrance,
            entrance_standoff=p("~entrance_standoff", 0.7),
            reverse_distance=p("~reverse_distance", 1.5),
            reverse_speed=p("~reverse_speed", 0.15),
            max_std_xy=p("~max_std_xy", 0.15),
            max_std_yaw_deg=p("~max_std_yaw_deg", 8.0),
            localize_max_std_xy=p("~localize_max_std_xy", 0.30),
            localize_max_std_yaw_deg=p("~localize_max_std_yaw_deg", 10.0),
            localize_hold_sec=p("~localize_hold_sec", 2.0),
            max_arrival_cross_track=p("~max_arrival_cross_track", 0.20),
            max_arrival_heading_error_deg=p("~max_arrival_heading_error_deg", 12.0),
            return_enabled=p("~return_enabled", True),
            reverse_timeout_sec=p("~reverse_timeout_sec", 30.0),
            localize_timeout_sec=p("~localize_timeout_sec", 30.0),
            transit_timeout_sec=p("~transit_timeout_sec", 300.0),
            mission_timeout_sec=p("~mission_timeout_sec", 1800.0),
            return_timeout_sec=p("~return_timeout_sec", 300.0),
        )

        self._odom_xy = None
        self._estimate = None
        self._vision_done = False
        self._vision_on = None          # last state we set
        self._goal_active = False
        self._last_state = None
        self._finished = False

        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        rospy.Subscriber("/odometry/filtered", Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._amcl_cb, queue_size=1)
        rospy.Subscriber("/vision_nav_node/mission_done", Bool, self._done_cb, queue_size=1)

        self._nav = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("waiting for move_base ...")
        self._nav.wait_for_server()
        rospy.loginfo("waiting for /vision_nav_node/set_enabled ...")
        rospy.wait_for_service("/vision_nav_node/set_enabled")
        self._vision_enable = rospy.ServiceProxy("/vision_nav_node/set_enabled", SetBool)
        self._nomotion = rospy.ServiceProxy("/request_nomotion_update", Empty)

        rospy.on_shutdown(self._on_shutdown)
        f = self._fsm
        rospy.loginfo("---- deploy supervisor ----")
        rospy.loginfo("row entrance (map): (%.2f, %.2f) facing %.0f deg; "
                      "move_base stops %.2f m short of it on the axis",
                      f.entrance.x, f.entrance.y, math.degrees(f.entrance.yaw),
                      f.entrance_standoff)
        rospy.loginfo("reverse off the trailer: %.2f m at %.2f m/s",
                      f.reverse_distance, f.reverse_speed)
        rospy.loginfo("localized when AMCL std <= %.2f m / %.1f deg for %.1f s",
                      f.localize_max_std_xy, math.degrees(f.localize_max_std_yaw),
                      f.localize_hold_sec)
        rospy.loginfo("handoff refused unless AMCL std <= %.2f m / %.1f deg and "
                      "within %.2f m / %.0f deg of the corridor axis",
                      f.max_std_xy, math.degrees(f.max_std_yaw),
                      f.max_arrival_cross_track,
                      math.degrees(f.max_arrival_heading_error))
        rospy.loginfo("return to the drop-off point: %s",
                      "yes" if f.return_enabled else "no")
        rospy.loginfo("---------------------------")

        self._apply(self._fsm.start(rospy.get_time()))
        rospy.Timer(rospy.Duration(0.1), self._tick)

    # ---- inputs ----------------------------------------------------------

    def _odom_cb(self, msg):
        self._odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _amcl_cb(self, msg):
        p, cov = msg.pose.pose, msg.pose.covariance
        self._estimate = Estimate(p.position.x, p.position.y, yaw_of(p.orientation),
                                  math.sqrt(max(cov[0], cov[7])), math.sqrt(cov[35]))

    def _done_cb(self, msg):
        # ⚠ The topic is LATCHED: a True left over from an earlier mission
        # would end this one the instant vision is enabled. Only count it
        # while vision is on; enabling clears it (the node also re-publishes
        # False on enable).
        if self._vision_on:
            self._vision_done = bool(msg.data)

    def _nav_state(self):
        if not self._goal_active:
            return None
        state = self._nav.get_state()
        if state in _ACTIVE:
            return NAV_ACTIVE
        if state == GoalStatus.SUCCEEDED:
            return NAV_SUCCEEDED
        return NAV_FAILED

    # ---- the loop --------------------------------------------------------

    def _tick(self, _event):
        if self._finished:
            return
        tick = self._fsm.update(rospy.get_time(), odom_xy=self._odom_xy,
                                estimate=self._estimate,
                                nav_state=self._nav_state(),
                                vision_done=self._vision_done)
        self._apply(tick)

    def _apply(self, tick):
        if tick.state != self._last_state:
            rospy.loginfo("%s (commander: %s)%s", tick.state, tick.commander or "none",
                          ("  -- " + tick.reason) if tick.reason else "")
            if tick.state == "ROW_MISSION" and self._fsm.arrival:
                cross, heading = self._fsm.arrival
                rospy.loginfo("arrived %.3f m off the corridor axis, %+.1f deg "
                              "off its bearing (AMCL)", cross, math.degrees(heading))
            if self._last_state == "REVERSE_OFF":
                self._cmd_pub.publish(Twist())       # one stop, then silent
            self._last_state = tick.state

        # ⚠ Off before on. Anything that should stop driving stops first.
        if tick.cancel_nav and self._goal_active:
            self._nav.cancel_all_goals()
            self._goal_active = False
        if not tick.vision_enabled:
            self._set_vision(False)
        if tick.nav_goal is not None:
            self._send_goal(tick.nav_goal)
        if tick.vision_enabled:
            if self._goal_active:                  # belt and braces
                self._nav.cancel_all_goals()
                self._goal_active = False
            self._set_vision(True)

        if tick.commander == COMMANDER_SUPERVISOR:
            cmd = Twist()
            cmd.linear.x = tick.reverse_speed
            self._cmd_pub.publish(cmd)

        if tick.request_localize_update:
            try:
                self._nomotion()
            except rospy.ServiceException:
                pass

        self._status_pub.publish(String(
            data="%s%s" % (tick.state, (" -- " + tick.reason) if tick.reason else "")))

        if tick.done:
            self._finished = True
            rospy.loginfo("DEPLOYMENT COMPLETE: back at the drop-off point "
                          "(%.2f, %.2f)." % (self._fsm.drop_off.x, self._fsm.drop_off.y)
                          if self._fsm.return_enabled else "DEPLOYMENT COMPLETE.")
            rospy.signal_shutdown("deployment complete")
        elif tick.failed:
            self._finished = True
            rospy.logerr("DEPLOYMENT FAILED: %s", tick.reason)
            rospy.signal_shutdown("deployment failed")

    def _send_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose.x
        goal.target_pose.pose.position.y = pose.y
        goal.target_pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        self._nav.send_goal(goal)
        self._goal_active = True
        rospy.loginfo("move_base goal: (%.2f, %.2f) facing %.0f deg",
                      pose.x, pose.y, math.degrees(pose.yaw))

    def _set_vision(self, enabled):
        if self._vision_on == enabled:
            return
        if enabled:
            self._vision_done = False
        try:
            self._vision_enable(enabled)
            self._vision_on = enabled
        except rospy.ServiceException as exc:
            rospy.logerr("could not %s vision nav: %s",
                         "enable" if enabled else "disable", exc)

    def _on_shutdown(self):
        """Leave nothing driving, whatever happened."""
        try:
            self._nav.cancel_all_goals()
        except Exception:                           # shutting down; best effort
            pass
        try:
            self._vision_enable(False)
        except Exception:
            pass
        try:
            self._cmd_pub.publish(Twist())
        except Exception:
            pass


def main():
    node = DeploySupervisor()
    del node
    rospy.spin()


if __name__ == "__main__":
    main()
