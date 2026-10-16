#!/usr/bin/env python3
"""List which rosbags are worth copying for offline LiDAR mapping / AMCL work.

Reads only each bag's index (fast even on multi-GB bags) unless --path is
given, which also integrates /odometry/filtered to show how far the robot
actually drove. Run it where the bags are (an external drive), then copy only
the bags marked USEFUL.

    python3 survey_bags.py /mnt/d/bags            # every *.bag under it
    python3 survey_bags.py --path a.bag b.bag     # plus distance driven

A bag is USEFUL for mapping when it has the horizontal Velodyne (points or
packets), odometry and TF, and runs for at least --min-duration seconds.
"""
import argparse
import math
import os
import sys

import rosbag

LIDAR = ("/ns1/velodyne_points", "/ns1/velodyne_packets")
ODOM = ("/odometry/filtered",)
TF = ("/tf",)
EXTRAS = {
    "tf_static": ("/tf_static",),
    "gps": ("/navsat/fix", "/gps/fix"),
    "imu": ("/imu/data", "/gx5/imu/data"),
    "cam": ("/usb_cam/image_raw/compressed", "/camera/image_raw"),
    "joy": ("/bluetooth_teleop/joy",),
}


def find_bags(paths):
    for p in paths:
        if os.path.isdir(p):
            for root, _, files in os.walk(p):
                for f in sorted(files):
                    if f.endswith(".bag"):
                        yield os.path.join(root, f)
        else:
            yield p


def odom_path(bag):
    prev, dist = None, 0.0
    for _, m, _ in bag.read_messages(topics=list(ODOM)):
        p = m.pose.pose.position
        if prev is not None:
            dist += math.hypot(p.x - prev[0], p.y - prev[1])
        prev = (p.x, p.y)
    return dist


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="bag files or directories")
    ap.add_argument("--path", action="store_true",
                    help="also integrate /odometry/filtered (reads the odometry messages)")
    ap.add_argument("--min-duration", type=float, default=60.0)
    args = ap.parse_args()

    for f in find_bags(args.paths):
        try:
            bag = rosbag.Bag(f)
        except Exception as e:  # unindexed / truncated bags are common on old drives
            print("%-60s  UNREADABLE (%s)" % (f, e))
            continue
        topics = set(bag.get_type_and_topic_info().topics)
        dur = bag.get_end_time() - bag.get_start_time() if bag.get_message_count() else 0.0
        has = lambda names: any(t in topics for t in names)
        core = has(LIDAR) and has(ODOM) and has(TF)
        useful = core and dur >= args.min_duration
        extras = ",".join(k for k, v in EXTRAS.items() if has(v)) or "-"
        line = "%-60s %6.0fs %7.0fMB  lidar=%s odom=%s tf=%s  [%s]" % (
            f, dur, os.path.getsize(f) / 1e6,
            "y" if has(LIDAR) else "n", "y" if has(ODOM) else "n",
            "y" if has(TF) else "n", extras)
        if args.path and has(ODOM):
            line += "  drove %.1fm" % odom_path(bag)
        print(("USEFUL  " if useful else "        ") + line)
        bag.close()


if __name__ == "__main__":
    sys.exit(main())
