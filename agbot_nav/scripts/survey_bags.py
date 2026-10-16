#!/usr/bin/env python3
"""List which rosbags are worth copying for offline LiDAR mapping / AMCL work.

Reads only each bag's summary records (a few KB, fast even on multi-GB bags) unless --path is
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
import struct
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


def _read_record(f):
    """One bag v2.0 record -> (header fields dict, data bytes)."""
    (hlen,) = struct.unpack("<i", f.read(4))
    raw, fields, i = f.read(hlen), {}, 0
    while i < hlen:
        (flen,) = struct.unpack("<i", raw[i:i + 4])
        name, _, value = raw[i + 4:i + 4 + flen].partition(b"=")
        fields[name.decode()] = value
        i += 4 + flen
    (dlen,) = struct.unpack("<i", f.read(4))
    return fields, f.read(dlen)


def quick_info(path):
    """Topics and duration from the bag's summary records only.

    rosbag.Bag() loads every per-message index entry in Python, which takes
    minutes per GB over a Windows (drvfs) mount. The summary at index_pos --
    one connection record per topic, one chunk-info record per chunk -- is a
    few KB and holds everything this survey needs. Returns None for a bag
    that was never indexed (recording cut off; `rosbag reindex` fixes it).
    """
    with open(path, "rb") as f:
        if f.read(13) != b"#ROSBAG V2.0\n":
            raise ValueError("not a v2.0 bag")
        fields, _ = _read_record(f)
        (index_pos,) = struct.unpack("<Q", fields["index_pos"])
        (conn_count,) = struct.unpack("<i", fields["conn_count"])
        (chunk_count,) = struct.unpack("<i", fields["chunk_count"])
        if index_pos == 0:
            return None
        f.seek(index_pos)
        topics = set()
        for _ in range(conn_count):
            fields, _ = _read_record(f)
            topics.add(fields["topic"].decode())
        start, end = float("inf"), 0.0
        for _ in range(chunk_count):
            fields, _ = _read_record(f)
            s, sn = struct.unpack("<II", fields["start_time"])
            e, en = struct.unpack("<II", fields["end_time"])
            start, end = min(start, s + sn * 1e-9), max(end, e + en * 1e-9)
        return topics, (end - start if chunk_count else 0.0)


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
            info = quick_info(f)
        except Exception as e:  # truncated / corrupt bags are common on old drives
            print("        %-60s  UNREADABLE (%s)" % (f, e), flush=True)
            continue
        if info is None:
            print("        %-60s  UNINDEXED (recording cut off; rosbag reindex)" % f, flush=True)
            continue
        topics, dur = info
        has = lambda names: any(t in topics for t in names)
        core = has(LIDAR) and has(ODOM) and has(TF)
        useful = core and dur >= args.min_duration
        extras = ",".join(k for k, v in EXTRAS.items() if has(v)) or "-"
        line = "%-60s %6.0fs %7.0fMB  lidar=%s odom=%s tf=%s  [%s]" % (
            f, dur, os.path.getsize(f) / 1e6,
            "y" if has(LIDAR) else "n", "y" if has(ODOM) else "n",
            "y" if has(TF) else "n", extras)
        if args.path and has(ODOM):
            with rosbag.Bag(f) as bag:
                line += "  drove %.1fm" % odom_path(bag)
        print(("USEFUL  " if useful else "        ") + line, flush=True)


if __name__ == "__main__":
    sys.exit(main())
