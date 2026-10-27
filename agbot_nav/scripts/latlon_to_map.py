#!/usr/bin/env python3
"""Surveyed lat/lon (+ row bearing) -> a pose in a georeferenced map.

    rosrun agbot_nav latlon_to_map.py maps/lab_georef.yaml LAT LON [BEARING_DEG]

BEARING_DEG is a COMPASS bearing (degrees clockwise from true north) of the
direction to drive DOWN the row. Prints the map pose and the matching
deploy_mission.launch args, e.g.

    roslaunch agbot_nav deploy_mission.launch ... $(rosrun agbot_nav latlon_to_map.py --args f.yaml 40.42 -86.92 90)

The reverse (`--reverse X Y`) prints the lat/lon of a map point, for checking a
fit against a spot you can stand on.
"""
import argparse
import math
import sys

import yaml

from agbot_nav.georef import Georef


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("georef", help="YAML written by fit_georef.py")
    ap.add_argument("a", type=float, help="LAT (or map X with --reverse)")
    ap.add_argument("b", type=float, help="LON (or map Y with --reverse)")
    ap.add_argument("bearing", type=float, nargs="?",
                    help="compass bearing down the row, deg CW from north")
    ap.add_argument("--args", action="store_true",
                    help="print only the entrance_* launch args")
    ap.add_argument("--reverse", action="store_true", help="map x y -> lat lon")
    opt = ap.parse_args()

    with open(opt.georef) as f:
        g = Georef.from_dict(yaml.safe_load(f))

    if opt.reverse:
        lat, lon = g.map_to_latlon(opt.a, opt.b)
        print("lat %.8f lon %.8f" % (lat, lon))
        return 0

    x, y = g.latlon_to_map(opt.a, opt.b)
    yaw_deg = math.degrees(g.compass_to_map_yaw(opt.bearing)) if opt.bearing is not None else 0.0
    launch = "entrance_x:=%.3f entrance_y:=%.3f entrance_yaw_deg:=%.1f" % (x, y, yaw_deg)
    if opt.bearing is not None:
        # the same axis in this map, so later RViz clicks snap onto these rows
        launch += " row_bearing_deg:=%.1f" % yaw_deg
    if opt.args:
        print(launch)
        return 0
    print("map (%.3f, %.3f)" % (x, y) + ("" if opt.bearing is None else
          ", facing %.1f deg (compass %.1f)" % (yaw_deg, opt.bearing)))
    if g.rms is not None:
        print("fit RMS %.3f m over %s pairs" % (g.rms, g.n))
    print(launch)
    return 0


if __name__ == "__main__":
    sys.exit(main())
