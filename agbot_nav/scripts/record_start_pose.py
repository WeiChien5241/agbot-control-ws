#!/usr/bin/env python3
"""Record where the robot was, IN THE MAP FRAME, when mapping started.

slam_toolbox (and gmapping) start the map frame ON TOP OF odom, and odom
starts where the robot BOOTED, not where mapping started. In the sim the two
coincide, so "initial pose 0 0 0" worked there; on the robot (2026-10-20) the
taped start spot was (0.245, -2.623, 16.5 deg) in the map, and AMCL started
2.6 m and 70 deg off.

Writes `x y yaw` (m, m, rad) to ~/.ros/agbot_mapping_start_pose once, from
the first map -> base_link transform; save_map.sh copies it next to the map
and prints the matching transit.launch arguments.
"""
import math
import os

import rospy
import tf2_ros


def main():
    rospy.init_node("record_start_pose")
    out = os.path.expanduser(rospy.get_param("~file", "~/.ros/agbot_mapping_start_pose"))
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    while not rospy.is_shutdown():
        try:
            t = buf.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0)).transform
            break
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            continue
    else:
        return
    q = t.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    with open(out, "w") as fh:
        fh.write("%.3f %.3f %.4f\n" % (t.translation.x, t.translation.y, yaw))
    rospy.loginfo("mapping start pose in map: x %.3f y %.3f yaw %.1f deg -> %s",
                  t.translation.x, t.translation.y, math.degrees(yaw), out)


if __name__ == "__main__":
    main()
