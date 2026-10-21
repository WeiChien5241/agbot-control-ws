#!/usr/bin/env python3
"""SIM ONLY: re-seed AMCL from Gazebo truth whenever the operator sends a goal.

    (started by deploy_mission.launch when sim:=true)

⚠ Why (measured 2026-10-21, field world, both with and without the LiDAR):
the sim Jackal SLIDES FORWARD while parked -- zero /cmd_vel, wheels all but
still, ~3 cm in its first minute and accelerating, 0.5 m in a later minute.
Wheel odometry cannot see a slide and AMCL only updates on odometry motion,
so after a few minutes in WAIT_GOAL AMCL was 0.5 m behind the robot and
move_base drove it into the pen wall planning from the stale pose. The real
robot does not do this, which is why this is a sim node and the supervisor
knows nothing about it.

It cancels the ARTIFACT, nothing else: on every 2D Nav Goal (the moment the
robot is about to move after a wait) it publishes /initialpose = the true
pose, mapped into the map frame through the map<-world offset measured when
AMCL first settled. So it works on a SLAM map whose origin is not Gazebo's,
as long as the first localization was right -- the same assumption the
deployment already makes.
"""
import math

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

STD_XY = 0.05
STD_YAW = math.radians(2.0)
SETTLED_STD_XY = 0.15          # AMCL "settled" for measuring the offset


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CreepGuard(object):

    def __init__(self):
        rospy.init_node("sim_creep_guard")
        rospy.wait_for_service("/gazebo/get_model_state")
        self._truth = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self._offset = None            # (dx, dy, dyaw): map = R(dyaw) * world + d
        self._pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped,
                                    queue_size=1)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._amcl_cb,
                         queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._goal_cb,
                         queue_size=1)
        rospy.logwarn("sim_creep_guard: SIM ONLY -- re-seeds AMCL from Gazebo truth "
                      "on every 2D Nav Goal (the parked sim Jackal slides)")

    def _true_pose(self):
        pose = self._truth("jackal", "").pose
        return pose.position.x, pose.position.y, yaw_of(pose.orientation)

    def _amcl_cb(self, msg):
        if self._offset is not None:
            return
        cov = msg.pose.covariance
        if math.sqrt(max(cov[0], cov[7])) > SETTLED_STD_XY:
            return
        p = msg.pose.pose
        ax, ay, ayaw = p.position.x, p.position.y, yaw_of(p.orientation)
        tx, ty, tyaw = self._true_pose()
        dyaw = ayaw - tyaw
        c, s = math.cos(dyaw), math.sin(dyaw)
        self._offset = (ax - (c * tx - s * ty), ay - (s * tx + c * ty), dyaw)
        rospy.loginfo("sim_creep_guard: map <- world offset (%.2f, %.2f, %.1f deg)",
                      self._offset[0], self._offset[1], math.degrees(dyaw))

    def _goal_cb(self, _msg):
        if self._offset is None:
            rospy.logwarn("sim_creep_guard: AMCL never settled; not re-seeding")
            return
        tx, ty, tyaw = self._true_pose()
        dx, dy, dyaw = self._offset
        c, s = math.cos(dyaw), math.sin(dyaw)
        x, y, yaw = c * tx - s * ty + dx, s * tx + c * ty + dy, tyaw + dyaw
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = STD_XY ** 2
        msg.pose.covariance[35] = STD_YAW ** 2
        self._pub.publish(msg)
        rospy.loginfo("sim_creep_guard: AMCL re-seeded at the true pose (%.2f, %.2f, "
                      "%.0f deg)", x, y, math.degrees(yaw))


if __name__ == "__main__":
    CreepGuard()
    rospy.spin()
