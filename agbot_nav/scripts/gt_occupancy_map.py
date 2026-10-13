#!/usr/bin/env python3
"""SIM ONLY: rasterize a virtual_maize_field world's ground truth into a map.

    rosrun agbot_nav gt_occupancy_map.py ~/.ros/virtual_maize_field/gt_map.csv \
        $(rospack find agbot_nav)/maps/sim_gps_tall --world-size 47.85

Writes NAME.pgm + NAME.yaml (map_server format) whose map frame IS the Gazebo
world frame, so an AMCL pose can be compared with /gazebo/get_model_state
directly and goals can be written in world coordinates.

Why not a slam_toolbox map: in the sim, online SLAM is not a fair test of
mapping. The maize models are short and the open headland gives the LiDAR a
handful of plant returns at 15 m, while the simulated Jackal creeps ~1-2 mm/s
with its wheels still, which odometry cannot see. On 2026-10-13 slam_toolbox
"corrected" itself ~10 m off during a mapping lap. This map removes mapping
from the question so the sim can test what it CAN test -- AMCL + move_base +
the deployment sequence on a correct map. Mapping is validated on the robot.

Plants become occupied disks of --plant-radius (default 0.15 m: the stalk plus
the leaves the 0.25-1.35 m scan slice actually hits). Everything else is
FREE: the ground truth is complete, unlike a LiDAR map.

⚠ No wall at the world edge by default (--border 0). AMCL's beam model
raycasts the map, and a wall the LiDAR can never see -- 7 m from the trailer
pose -- is a predicted return that every scan contradicts.
"""
import argparse
import csv
import os

import numpy as np
import yaml
from PIL import Image

FREE, OCCUPIED = 254, 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("gt_csv")
    ap.add_argument("out_base", help="output path without extension")
    ap.add_argument("--world-size", type=float, default=47.85,
                    help="heightmap side length in m (generated.world <size>)")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--plant-radius", type=float, default=0.15)
    ap.add_argument("--border", type=float, default=0.0,
                    help="lethal band inside the world edge, m (0 = none)")
    args = ap.parse_args()

    with open(args.gt_csv) as handle:
        plants = [(float(r["X"]), float(r["Y"])) for r in csv.DictReader(handle)
                  if r["kind"] == "crop"]

    res = args.resolution
    half = args.world_size / 2.0
    n = int(round(args.world_size / res))
    grid = np.full((n, n), FREE, dtype=np.uint8)   # row 0 = TOP = +y edge

    # cell centres in world coordinates
    xs = -half + (np.arange(n) + 0.5) * res
    ys = half - (np.arange(n) + 0.5) * res
    border = int(round(args.border / res))
    if border > 0:
        grid[:border, :] = grid[-border:, :] = OCCUPIED
        grid[:, :border] = grid[:, -border:] = OCCUPIED

    r = args.plant_radius
    for px, py in plants:
        cols = np.nonzero(np.abs(xs - px) <= r)[0]
        rows = np.nonzero(np.abs(ys - py) <= r)[0]
        for row in rows:
            for col in cols:
                if (xs[col] - px) ** 2 + (ys[row] - py) ** 2 <= r * r:
                    grid[row, col] = OCCUPIED

    base = os.path.abspath(args.out_base)
    Image.fromarray(grid).save(base + ".pgm")
    spec = {
        "image": os.path.basename(base) + ".pgm",
        "resolution": res,
        "origin": [-half, -half, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with open(base + ".yaml", "w") as handle:
        yaml.safe_dump(spec, handle, default_flow_style=None)
    print("%d plants -> %s.{pgm,yaml}: %dx%d cells at %.2f m, origin (%.2f, %.2f)"
          % (len(plants), base, n, n, res, -half, -half))


if __name__ == "__main__":
    main()
