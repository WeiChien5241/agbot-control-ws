#!/usr/bin/env python3
"""SIM ONLY: AMCL's pose vs Gazebo ground truth, once or as a CSV log.

    rosrun agbot_nav amcl_truth.py                 # one line, then exit
    rosrun agbot_nav amcl_truth.py _log:=true      # every 1 s until Ctrl-C

Assumes the map frame IS the Gazebo world frame, which holds for maps from
gt_occupancy_map.py. Reports position error (m), heading error (deg) and
AMCL's own 1-sigma spread, so "AMCL is wrong" and "AMCL knows it is unsure"
can be told apart.
"""
import math

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    rospy.init_node("amcl_truth", anonymous=True)
    model = rospy.get_param("~model", "jackal")
    log = rospy.get_param("~log", False)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    rospy.wait_for_service("/gazebo/get_model_state")

    latest = {}
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                     lambda m: latest.__setitem__("amcl", m), queue_size=1)
    if log:
        print("t,true_x,true_y,true_yaw_deg,amcl_x,amcl_y,amcl_yaw_deg,"
              "err_xy,err_yaw_deg,std_xy,std_yaw_deg")
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        if "amcl" in latest:
            msg = latest["amcl"]
            truth = get_state(model, "").pose
            p = msg.pose.pose
            cov = msg.pose.covariance
            err_xy = math.hypot(p.position.x - truth.position.x,
                                p.position.y - truth.position.y)
            ty, ay = yaw_of(truth.orientation), yaw_of(p.orientation)
            err_yaw = math.degrees(math.atan2(math.sin(ay - ty), math.cos(ay - ty)))
            std_xy = math.sqrt(max(cov[0], cov[7]))
            std_yaw = math.degrees(math.sqrt(cov[35]))
            row = (rospy.get_time(), truth.position.x, truth.position.y,
                   math.degrees(ty), p.position.x, p.position.y, math.degrees(ay),
                   err_xy, err_yaw, std_xy, std_yaw)
            if log:
                print(",".join("%.3f" % v for v in row), flush=True)
            else:
                print("truth (%.2f, %.2f, %.1f deg)  amcl (%.2f, %.2f, %.1f deg)  "
                      "error %.3f m / %+.2f deg  amcl std %.3f m / %.2f deg"
                      % row[1:])
                return
        rate.sleep()


if __name__ == "__main__":
    main()
