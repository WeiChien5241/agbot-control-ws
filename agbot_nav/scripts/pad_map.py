#!/usr/bin/env python3
"""Pad a saved map_server map with UNKNOWN cells on every side, in place.

    rosrun agbot_nav pad_map.py maps/NAME.yaml 30      # 30 m each side

Why: slam_toolbox (Karto) sizes its grid from the bounding box of real LiDAR
returns. In open ground that is little more than the corn -- built from the
trailer side, the sim map was ~12 x 17 m and the trailer sat OUTSIDE it.
AMCL and move_base's global costmap (whose size is taken from the map) then
have no map where the robot starts, and every plan fails with "The robot's
start position is off the global costmap".

Unknown, not free: the planner already treats unknown as traversable
(global_costmap.yaml track_unknown_space: false, GlobalPlanner allow_unknown),
and AMCL draws nothing from unknown cells, so padding claims nothing about
ground the LiDAR never saw.

No ROS needed -- plain numpy + PIL on the yaml/pgm pair map_saver writes.
"""
import os
import sys

import numpy as np
import yaml
from PIL import Image

UNKNOWN = 205   # map_saver's trinary value for unknown


def pad_map(yaml_path, pad_m):
    with open(yaml_path) as handle:
        spec = yaml.safe_load(handle)
    image_path = spec["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image_path)

    grid = np.array(Image.open(image_path))
    res = float(spec["resolution"])
    pad = int(round(pad_m / res))

    padded = np.full((grid.shape[0] + 2 * pad, grid.shape[1] + 2 * pad),
                     UNKNOWN, dtype=grid.dtype)
    padded[pad:pad + grid.shape[0], pad:pad + grid.shape[1]] = grid
    Image.fromarray(padded).save(image_path)

    # origin is the map's LOWER-left corner; image row 0 is the TOP, so only
    # the origin's x and y both move back by the pad.
    origin = list(spec["origin"])
    origin[0] -= pad * res
    origin[1] -= pad * res
    spec["origin"] = origin
    with open(yaml_path, "w") as handle:
        yaml.safe_dump(spec, handle, default_flow_style=None)

    return grid.shape, padded.shape, origin


def main(argv):
    if len(argv) != 3:
        print(__doc__)
        return 2
    before, after, origin = pad_map(argv[1], float(argv[2]))
    print("padded %s: %dx%d -> %dx%d cells, origin now (%.2f, %.2f)"
          % (argv[1], before[1], before[0], after[1], after[0], origin[0], origin[1]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
