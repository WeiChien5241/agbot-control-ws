#!/usr/bin/python3
"""Turn a generated maize world into GPS row-entrance waypoints.

    python3 rows_to_waypoints.py ~/.ros/virtual_maize_field/gt_map.csv \
        --datum-file ../config/gps_datum.yaml -o ../config/waypoints_maize_gps.yaml

No ROS. Reads `gt_map.csv`, which virtual_maize_field writes AFTER its final
coordinate fix-up, so its numbers are exactly the plant poses in the .world --
that is why this parses the CSV rather than the SDF.

WHY GENERATE RATHER THAN TYPE. Row positions come out of a seeded procedural
generator. Regenerating the world moves every plant a few centimetres, and
hand-copied waypoints then point at slightly the wrong gaps with nothing to say
so. Regenerating the waypoints alongside the world keeps them honest.

⚠ The map frame's origin is the datum, and the Gazebo world origin is placed at
that same datum (agbot_bringup/scripts/load_robot_description.sh exports it as
GAZEBO_WORLD_LAT/LON), so world coordinates and map coordinates are the same
numbers. If that ever stops being true, this script's lat/lon are wrong and
nothing downstream would notice.
"""

import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from agbot_gps_nav import geo  # noqa: E402


def read_crops(path):
    """(x, y) of every crop in gt_map.csv."""
    points = []
    with open(path) as handle:
        for row in csv.DictReader(handle):
            if row.get("kind", "").strip() == "crop":
                points.append((float(row["X"]), float(row["Y"])))
    if not points:
        raise SystemExit("no rows of kind 'crop' in %s" % path)
    return points


def cluster_rows(points, gap=0.35):
    """Group plants into rows by x.

    `gap` is under half the 0.75 m row spacing, so it separates rows without
    splitting one row's own jitter.
    """
    rows = []
    for x, y in sorted(points):
        if rows and abs(x - rows[-1][-1][0]) <= gap:
            rows[-1].append((x, y))
        else:
            rows.append([(x, y)])
    return rows


def corridors(rows, entry_offset):
    """One waypoint per gap between adjacent rows.

    The mouth of a corridor is where it becomes bounded on BOTH sides, i.e. the
    later of its two rows' first plants -- not the earlier one, which would put
    the waypoint beside a gap that is still open on one flank.
    """
    out = []
    for index in range(len(rows) - 1): 
        left, right = rows[index], rows[index + 1]
        centre_x = (sum(x for x, _ in left) / len(left)
                    + sum(x for x, _ in right) / len(right)) / 2.0
        mouth_y = max(min(y for _, y in left), min(y for _, y in right))
        # Rows run along +Y in every straight virtual_maize_field world, but
        # measure it rather than assume it.
        span = max(y for _, y in left) - min(y for _, y in left)
        bearing = math.atan2(span, 0.0)
        # ⚠ Which way the boustrophedon must turn out of THIS corridor. Rows
        # are ordered by increasing x and the robot enters heading along the
        # bearing (+Y), so the next corridor is to its RIGHT (+x) from the
        # leftmost one and to its LEFT from the rightmost. Turning the wrong
        # way out of an edge corridor drives straight out of the field: seen in
        # sim 2026-09-07, where corridor_0 with the default "left" ended the
        # mission at rows=1/3 in open ground.
        turn = "right" if index == 0 else "left"
        out.append({
            "name": "corridor_%d" % index,
            "first_turn_direction": turn,
            "map_xy": [round(centre_x, 3), round(mouth_y - entry_offset, 3)],
            "approach_bearing_deg": round(math.degrees(bearing), 1),
            "row_x": [round(left[0][0], 3), round(right[0][0], 3)],
            "mouth_y": round(mouth_y, 3),
        })
    return out


def load_datum(path):
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if line.startswith("datum:"):
                inner = line.split(":", 1)[1].strip().strip("[]")
                parts = [float(p) for p in inner.split(",")]
                return parts[0], parts[1]
    raise SystemExit("no 'datum:' line in %s" % path)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("gt_map", help="path to gt_map.csv from the generated world")
    parser.add_argument("--datum-file", required=True, help="gps_datum.yaml")
    parser.add_argument("-o", "--output", required=True, help="waypoints yaml to write")
    parser.add_argument("--entry-offset", type=float, default=0.5,
                        help="metres to stop SHORT of the first plants, so vision "
                             "nav takes over with the corridor ahead rather than "
                             "already inside it (default: 0.5)")
    parser.add_argument("--start-offset", type=float, default=7.0,
                        help="metres back from the mouth for the simulated trailer "
                             "start pose (default: 7.0)")
    args = parser.parse_args()

    datum = load_datum(args.datum_file)
    rows = cluster_rows(read_crops(args.gt_map))
    entries = corridors(rows, args.entry_offset)

    lines = [
        "# GENERATED by scripts/rows_to_waypoints.py -- do not hand-edit.",
        "#   source: %s" % args.gt_map,
        "#   datum:  %.6f, %.6f" % datum,
        "#",
        "# Row entrances for the generated maize world, as lat/lon plus the map",
        "# coordinates they came from. Each carries an approach bearing: a row",
        "# entrance is a point PLUS a direction, because vision nav picks up in",
        "# FOLLOW_ROW and needs the corridor already in view.",
        "#",
        "# ⚠ first_turn_direction must be passed to vision nav to MATCH the",
        "# corridor. Turning the wrong way out of an edge corridor drives",
        "# straight out of the field.",
        "#",
        "# Regenerate whenever the world is regenerated -- the layout is seeded",
        "# and procedural, so plants move and hand-copied numbers go quietly",
        "# stale.",
        "datum: [%.6f, %.6f, 0.0]" % (datum[0], datum[1]),
        "rows_detected: %d" % len(rows),
        "",
        "# Simulated trailer: %.1f m back from the first corridor's mouth, on its"
        % args.start_offset,
        "# axis. Spawn the robot here to give the GPS transit somewhere to start.",
    ]
    first = entries[0]
    start_y = first["mouth_y"] - args.start_offset
    slat, slon = geo.enu_to_latlon(first["map_xy"][0], start_y, datum)
    lines += [
        "start_pose:",
        "  map_xy: [%.3f, %.3f]" % (first["map_xy"][0], start_y),
        "  yaw_deg: %.1f" % first["approach_bearing_deg"],
        "  lat: %.8f" % slat,
        "  lon: %.8f" % slon,
        "",
        "waypoints:",
    ]
    for entry in entries:
        lat, lon = geo.enu_to_latlon(entry["map_xy"][0], entry["map_xy"][1], datum)
        lines += [
            "  - name: %s" % entry["name"],
            "    lat: %.8f" % lat,
            "    lon: %.8f" % lon,
            "    approach_bearing_deg: %.1f" % entry["approach_bearing_deg"],
            "    first_turn_direction: %s" % entry["first_turn_direction"],
            "    # map (%.3f, %.3f); between rows at x=%.3f and x=%.3f; first"
            % (entry["map_xy"][0], entry["map_xy"][1],
               entry["row_x"][0], entry["row_x"][1]),
            "    # plants at y=%.3f" % entry["mouth_y"],
        ]
    with open(args.output, "w") as handle:
        handle.write("\n".join(lines) + "\n")

    print("%d rows -> %d corridors; wrote %s" % (len(rows), len(entries), args.output))
    for entry in entries:
        print("  %-12s map (%7.3f, %7.3f)  bearing %5.1f deg  first turn %s"
              % (entry["name"], entry["map_xy"][0], entry["map_xy"][1],
                 entry["approach_bearing_deg"], entry["first_turn_direction"]))


if __name__ == "__main__":
    main()
