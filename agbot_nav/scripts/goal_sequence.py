#!/usr/bin/env python3
"""Drive a list of map-frame poses through move_base, one after another.

    rosrun agbot_nav goal_sequence.py  x,y,yaw_deg  x,y,yaw_deg ...

Used for the sim mapping lap (slam_toolbox + move_base) and for transit
tests. Prints each goal's result and the time it took, and exits non-zero on
the first goal move_base does not reach.
"""
import math
import sys

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


def parse(arg):
    x, y, yaw_deg = (float(v) for v in arg.split(","))
    return x, y, math.radians(yaw_deg)


def make_goal(x, y, yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
    return goal


def main():
    rospy.init_node("goal_sequence")
    argv = rospy.myargv()[1:]
    if not argv:
        print(__doc__)
        return 2
    goals = [parse(a) for a in argv]
    timeout = rospy.get_param("~goal_timeout", 300.0)

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("waiting for move_base")
    client.wait_for_server()

    for i, (x, y, yaw) in enumerate(goals):
        t0 = rospy.Time.now()
        rospy.loginfo("goal %d/%d: (%.2f, %.2f, %.0f deg)",
                      i + 1, len(goals), x, y, math.degrees(yaw))
        client.send_goal(make_goal(x, y, yaw))
        finished = client.wait_for_result(rospy.Duration(timeout))
        state = client.get_state()
        dt = (rospy.Time.now() - t0).to_sec()
        if not finished or state != GoalStatus.SUCCEEDED:
            client.cancel_all_goals()
            rospy.logerr("goal %d FAILED (state %d) after %.1f s", i + 1, state, dt)
            return 1
        rospy.loginfo("goal %d reached in %.1f s", i + 1, dt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
