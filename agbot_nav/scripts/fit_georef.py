#!/usr/bin/env python3
"""Georeference a saved map from a bag: fit lat/lon <-> map once, offline.

    rosrun agbot_nav fit_georef.py ~/bags/mapping_*.bag --out maps/lab_georef.yaml
    # sim (hector GNSS):
    rosrun agbot_nav fit_georef.py run.bag --fix-topic /navsat/fix --min-status 0

The bag needs /tf (map -> odom -> base_link: the SLAM or AMCL run itself),
/tf_static, and the GPS fix. Each good fix is paired with where the map puts
the ANTENNA at the fix's stamp; the fit is agbot_nav.georef.fit_georef.

ANTENNA LEVER ARM. GPS measures the antenna, the map tracks base_link, and on
a turn in place a 0.2 m arm sweeps a 0.4 m circle. `--lever x,y` (metres in
base_link) wins; otherwise base_link -> `--antenna-frame` is read from the
bag's TF (the sim's navsat_link is where hector's receiver really is). ⚠ On
the real robot navsat_link is Clearpath's default mount, not necessarily
where the Reach antenna is -- tape-measure it and pass --lever.
⚠ In SIM pass --lever 0,0: Gazebo lumps fixed-joint links into the chassis,
so hector's navsat_link body IS base_link and the fix reports the robot's
centre (measured 2026-10-26: with the TF arm the RMS was 0.21 m, with 0,0
it was 0.04 m).

Prints the fit, the residuals and the map origin's lat/lon, and writes YAML
that `agbot_nav.georef.Georef.from_dict` loads.
"""
import argparse
import math
import sys

import numpy as np
import rosbag
import rospy
import tf2_ros
import yaml

from agbot_nav.georef import GeorefError, fit_georef, thin_by_distance


def _load_tf(bag):
    buf = tf2_ros.BufferCore(rospy.Duration(1.0e6))
    n = 0
    for topic, msg, _ in bag.read_messages(topics=["/tf", "/tf_static"]):
        for tr in msg.transforms:
            if topic == "/tf_static":
                buf.set_transform_static(tr, "bag")
            else:
                buf.set_transform(tr, "bag")
            n += 1
    return buf, n


def _xy_yaw(tr):
    t, q = tr.transform.translation, tr.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return t.x, t.y, yaw


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--fix-topic", default="/gps/fix")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--antenna-frame", default="navsat_link")
    ap.add_argument("--lever", help="antenna x,y in base_link (m); overrides TF")
    ap.add_argument("--min-status", type=int, default=2,
                    help="NavSatStatus floor; 2 = RTK fixed on our Reach driver")
    ap.add_argument("--max-std", type=float, default=0.05,
                    help="drop fixes whose reported horizontal std exceeds this (m)")
    ap.add_argument("--min-step", type=float, default=0.25,
                    help="keep one pair per this much antenna travel (m)")
    ap.add_argument("--reject", type=float, default=0.5)
    ap.add_argument("--datum", help="lat,lon of the ENU plane (default: first fix)")
    ap.add_argument("--out", help="write the fit here (YAML)")
    args = ap.parse_args()

    bag = rosbag.Bag(args.bag)
    buf, ntf = _load_tf(bag)
    print("%s: %d TF transforms" % (args.bag, ntf))

    if args.lever:
        lever = tuple(float(v) for v in args.lever.split(","))
        lever_src = "--lever"
    else:
        try:
            tr = buf.lookup_transform_core(args.base_frame, args.antenna_frame,
                                           rospy.Time(0))
            lever = (tr.transform.translation.x, tr.transform.translation.y)
            lever_src = "TF %s -> %s" % (args.base_frame, args.antenna_frame)
        except tf2_ros.TransformException:
            lever, lever_src = (0.0, 0.0), "NONE (no TF, no --lever): assumed at base_link"
    print("antenna lever arm: (%.3f, %.3f) m from %s" % (lever[0], lever[1], lever_src))

    counts = {"fixes": 0, "status": 0, "std": 0, "no_tf": 0}
    latlon, map_xy, stamps = [], [], []
    for _, msg, _ in bag.read_messages(topics=[args.fix_topic]):
        counts["fixes"] += 1
        if msg.status.status < args.min_status or not math.isfinite(msg.latitude):
            counts["status"] += 1
            continue
        if msg.position_covariance_type > 0:
            std = math.sqrt(max(msg.position_covariance[0], msg.position_covariance[4]))
            if std > args.max_std:
                counts["std"] += 1
                continue
        try:
            tr = buf.lookup_transform_core(args.map_frame, args.base_frame,
                                           msg.header.stamp)
        except tf2_ros.TransformException:
            counts["no_tf"] += 1
            continue
        x, y, yaw = _xy_yaw(tr)
        c, s = math.cos(yaw), math.sin(yaw)
        map_xy.append((x + c * lever[0] - s * lever[1], y + s * lever[0] + c * lever[1]))
        latlon.append((msg.latitude, msg.longitude))
        stamps.append(msg.header.stamp.to_sec())
    bag.close()

    print("fixes: %(fixes)d  dropped: status %(status)d, std %(std)d, "
          "no map TF %(no_tf)d" % counts)
    if counts["fixes"] == 0:
        print("no messages on %s -- wrong --fix-topic? (sim: /navsat/fix)" % args.fix_topic)
        return 1

    keep = thin_by_distance(map_xy, args.min_step)
    latlon = [latlon[i] for i in keep]
    map_xy = [map_xy[i] for i in keep]
    stamps = [stamps[i] for i in keep]
    print("pairs after %.2f m thinning: %d" % (args.min_step, len(keep)))

    datum = tuple(float(v) for v in args.datum.split(",")) if args.datum else None
    try:
        g, res, kept = fit_georef(latlon, map_xy, datum=datum, reject=args.reject)
    except GeorefError as exc:
        print("FIT REFUSED: %s" % exc)
        return 1

    print("\nfit: map = R(yaw) * ENU + t   (ENU about %.8f, %.8f)" % g.datum)
    print("  yaw  %+.3f deg   (map +x points at compass %.1f deg)"
          % (math.degrees(g.yaw), g.map_yaw_to_compass(0.0)))
    print("  t    (%+.3f, %+.3f) m" % (g.tx, g.ty))
    print("  pairs used %d of %d, span %.1f m" % (g.n, len(kept), g.span))
    print("  residual RMS %.3f m, max %.3f m, p95 %.3f m"
          % (g.rms, g.max_err, float(np.percentile(res[kept], 95))))
    if (~kept).any():
        t0 = stamps[0]
        print("  rejected %d pair(s): " % (~kept).sum() + ", ".join(
            "t=%.1fs %.2fm" % (stamps[i] - t0, res[i]) for i in np.where(~kept)[0][:10]))
    lat0, lon0 = g.map_to_latlon(0.0, 0.0)
    print("  map origin (0, 0) is lat %.8f lon %.8f" % (lat0, lon0))
    if g.rms > 0.10:
        print("  ⚠ RMS > 0.10 m: the map is bent or the TF/GPS timing is off; "
              "do not hand a row entrance over on this fit")

    if args.out:
        d = g.to_dict()
        d["source_bag"] = args.bag
        d["lever_arm"] = [float(lever[0]), float(lever[1])]
        with open(args.out, "w") as f:
            f.write("# fit_georef.py: map = R(yaw_rad) * ENU(datum) + (x, y)\n")
            yaml.safe_dump(d, f, default_flow_style=None, sort_keys=False)
        print("wrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
