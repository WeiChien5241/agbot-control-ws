#!/usr/bin/env python3
"""List which rosbags are worth copying for offline LiDAR mapping / AMCL work.

Reads only each bag's summary records (a few KB, fast even on multi-GB bags).
With --path it also SAMPLES /odometry/filtered and the GPS fix from up to
--samples chunks spread over the bag, to show how far the robot actually
drove and where (a bag can be 15 minutes of the robot sitting still).
Run it where the bags are (an external drive), then copy only what it ranks.

    python3 survey_bags.py /mnt/d/bags                 # every *.bag under it
    python3 survey_bags.py --path /mnt/d/bags          # plus distance + location

A bag is USEFUL for mapping when it has the horizontal Velodyne (points or
packets), odometry and TF, and runs for at least --min-duration seconds.

⚠ The --path distance is sampled (straight lines between ~--samples odometry
poses), so it UNDER-counts a wiggly drive. It is for ranking bags, not for
quoting; bag_distance.py in agbot_vision_nav integrates every message.
"""
import argparse
import math
import os
import struct
import sys

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

LIDAR = ("/ns1/velodyne_points", "/ns1/velodyne_packets")
ODOM = ("/odometry/filtered",)
TF = ("/tf",)
GPS = ("/navsat/fix", "/gps/fix")
EXTRAS = {
    "tf_static": ("/tf_static",),
    "gps": GPS,
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


def _parse_header(raw):
    fields, i = {}, 0
    while i < len(raw):
        (flen,) = struct.unpack("<i", raw[i:i + 4])
        name, _, value = raw[i + 4:i + 4 + flen].partition(b"=")
        fields[name.decode()] = value
        i += 4 + flen
    return fields


def _read_record(f):
    """One bag v2.0 record -> (header fields dict, data bytes)."""
    (hlen,) = struct.unpack("<i", f.read(4))
    fields = _parse_header(f.read(hlen))
    (dlen,) = struct.unpack("<i", f.read(4))
    return fields, f.read(dlen)


def _time(v):
    s, ns = struct.unpack("<II", v)
    return s + ns * 1e-9


class BagSummary(object):
    """Topics, duration and chunk layout from the bag's summary records only.

    rosbag.Bag() loads every per-message index entry in Python, which takes
    minutes per GB over a Windows (drvfs) mount. The summary at index_pos --
    one connection record per topic, one chunk-info record per chunk -- is a
    few KB and holds everything the survey needs, including which chunks
    hold which topic, so a topic can be sampled by reading a few chunks.
    `indexed` is False for a bag whose recording was cut off
    (`rosbag reindex` fixes it).
    """

    def __init__(self, path):
        self.path = path
        self.topics, self.conns, self.chunks = set(), {}, []
        self.duration, self.indexed = 0.0, True
        with open(path, "rb") as f:
            if f.read(13) != b"#ROSBAG V2.0\n":
                raise ValueError("not a v2.0 bag")
            fields, _ = _read_record(f)
            (index_pos,) = struct.unpack("<Q", fields["index_pos"])
            (conn_count,) = struct.unpack("<i", fields["conn_count"])
            (chunk_count,) = struct.unpack("<i", fields["chunk_count"])
            if index_pos == 0:
                self.indexed = False
                return
            f.seek(index_pos)
            for _ in range(conn_count):
                fields, _ = _read_record(f)
                (cid,) = struct.unpack("<i", fields["conn"])
                topic = fields["topic"].decode()
                self.conns[cid] = topic
                self.topics.add(topic)
            start, end = float("inf"), 0.0
            for _ in range(chunk_count):
                fields, data = _read_record(f)
                (pos,) = struct.unpack("<Q", fields["chunk_pos"])
                ids = {struct.unpack("<i", data[i:i + 4])[0] for i in range(0, len(data), 8)}
                self.chunks.append((pos, ids))
                start = min(start, _time(fields["start_time"]))
                end = max(end, _time(fields["end_time"]))
            self.duration = end - start if chunk_count else 0.0

    def has(self, names):
        return any(t in self.topics for t in names)

    def sample(self, names, msg_class, n):
        """First message of the topic in up to n chunks spread over the bag."""
        ids = {c for c, t in self.conns.items() if t in names}
        holding = [pos for pos, cids in self.chunks if cids & ids]
        if not holding:
            return []
        step = max(1, len(holding) // n)
        out = []
        with open(self.path, "rb") as f:
            for pos in holding[::step]:
                f.seek(pos)
                fields, data = _read_record(f)
                if fields.get("compression", b"none") != b"none":
                    continue  # bz2/lz4 chunks: skip rather than decompress
                i = 0
                while i < len(data):
                    (hlen,) = struct.unpack("<i", data[i:i + 4])
                    h = _parse_header(data[i + 4:i + 4 + hlen])
                    (dlen,) = struct.unpack("<i", data[i + 4 + hlen:i + 8 + hlen])
                    body = data[i + 8 + hlen:i + 8 + hlen + dlen]
                    i += 8 + hlen + dlen
                    if h.get("op") == b"\x02" and struct.unpack("<i", h["conn"])[0] in ids:
                        m = msg_class()
                        m.deserialize(body)
                        out.append(m)
                        break
        return out


def path_and_place(summary, n):
    poses = summary.sample(ODOM, Odometry, n)
    dist = sum(math.hypot(b.pose.pose.position.x - a.pose.pose.position.x,
                          b.pose.pose.position.y - a.pose.pose.position.y)
               for a, b in zip(poses, poses[1:]))
    fixes = [m for m in summary.sample(GPS, NavSatFix, n)
             if m.status.status >= 0 and abs(m.latitude) > 1e-6]
    if not fixes:
        return dist, None
    lat = sorted(m.latitude for m in fixes)[len(fixes) // 2]
    lon = sorted(m.longitude for m in fixes)[len(fixes) // 2]
    return dist, (lat, lon, max(m.status.status for m in fixes))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="bag files or directories")
    ap.add_argument("--path", action="store_true",
                    help="also sample odometry + GPS: distance driven and where")
    ap.add_argument("--samples", type=int, default=40,
                    help="chunks to sample per bag for --path (default 40)")
    ap.add_argument("--min-duration", type=float, default=60.0)
    args = ap.parse_args()

    for f in find_bags(args.paths):
        try:
            s = BagSummary(f)
        except Exception as e:  # truncated / corrupt bags are common on old drives
            print("        %-60s  UNREADABLE (%s)" % (f, e), flush=True)
            continue
        if not s.indexed:
            print("        %-60s  UNINDEXED (recording cut off; rosbag reindex)" % f, flush=True)
            continue
        core = s.has(LIDAR) and s.has(ODOM) and s.has(TF)
        useful = core and s.duration >= args.min_duration
        extras = ",".join(k for k, v in EXTRAS.items() if s.has(v)) or "-"
        line = "%-60s %6.0fs %7.0fMB  lidar=%s odom=%s tf=%s  [%s]" % (
            f, s.duration, os.path.getsize(f) / 1e6,
            "y" if s.has(LIDAR) else "n", "y" if s.has(ODOM) else "n",
            "y" if s.has(TF) else "n", extras)
        if args.path and useful:
            dist, place = path_and_place(s, args.samples)
            line += "  drove~%.0fm" % dist
            if place:
                line += "  at %.5f,%.5f gps_status=%d" % place
        print(("USEFUL  " if useful else "        ") + line, flush=True)


if __name__ == "__main__":
    sys.exit(main())
