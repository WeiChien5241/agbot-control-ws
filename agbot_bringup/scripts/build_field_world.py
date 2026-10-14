#!/usr/bin/env python3
"""Build a Gazebo maize world from a few BLOCKS OF ROWS described in YAML.

    rosrun agbot_bringup build_field_world.py config/agbot_field_layout.yaml
    rosrun agbot_bringup switch_maize_world.sh <snapshot>

virtual_maize_field generates one parallel-row field at a time; a real field
with a perpendicular strip, a cross alley and a headland in particular places
cannot be expressed in it. This places every plant from the block
description instead and writes a snapshot in the SAME layout
virtual_maize_field uses, so switch_maize_world.sh and agbot_gazebo.launch
work unchanged:

    ~/.ros/virtual_maize_field_snapshots/<snapshot>/
        generated.world                    the world
        virtual_maize_field_heightmap.png  the ground: flat, `ground.size` m
        gt_map.csv                         plant positions -> AMCL map
        robot_spawner.launch               spawn pose, which switch prints
        field_preview.png                  top-down picture of the layout

The sky, light and ground TEXTURES are copied from an existing snapshot
(--template, default `gps`) so the camera sees the visuals vision nav was
validated on. The ground itself is a new flat heightmap, a square of
`ground.size` m centred on `ground.center` (Gazebo heightmaps must be
square); the builder refuses plants or a spawn within 1 m of its edge.

Also writes agbot_nav/maps/<snapshot>.{pgm,yaml} via gt_occupancy_map.py
unless --no-map.
"""
import argparse
import math
import os
import re
import shutil
import subprocess
import sys

import numpy as np
import yaml

SNAPSHOTS = os.path.join(os.environ.get("ROS_HOME", os.path.expanduser("~/.ros")),
                         "virtual_maize_field_snapshots")
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))
GROUND_Z = 0.30        # ground height under the plants (m)
HEIGHT_RANGE = 0.305   # the template heightmap's z range (m)
GROUND_PIXEL = 250     # flat ground: 250/255 * 0.305 = 0.299 m, the template's
HEIGHTMAP_PX = 129     # Gazebo wants 2^n + 1 pixels a side
MODELS = {"tall": "agbot_maize_tall_0%d", "stock": "maize_0%d"}


def place_block(block, defaults, rng):
    """Yield (x, y) for every plant of one block."""
    spacing = float(block.get("row_spacing", defaults["row_spacing"]))
    step = float(block.get("plant_spacing", defaults["plant_spacing"]))
    jitter = float(block.get("jitter", defaults["jitter"]))
    x0, y0 = (float(v) for v in block["start"])
    heading = math.radians(float(block["heading"]))
    along = np.array([math.cos(heading), math.sin(heading)])
    left = np.array([-math.sin(heading), math.cos(heading)])
    sign = 1.0 if block.get("side", "left") == "left" else -1.0
    n_plants = int(math.floor(float(block["length"]) / step)) + 1
    for row in range(int(block["rows"])):
        base = np.array([x0, y0]) + sign * row * spacing * left
        for k in range(n_plants):
            p = base + k * step * along + rng.normal(0.0, jitter, 2)
            yield float(p[0]), float(p[1])


def split_template(world_text):
    """(head, tail) of a virtual_maize_field world, without its plants."""
    first = world_text.index("<include>")
    gui = world_text.index("<gui")
    return world_text[:first], world_text[gui:]


def write_preview(path, plants, spec):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    xs, ys = zip(*plants)
    fig, ax = plt.subplots(figsize=(9, 9))
    size = float(spec["ground"]["size"])
    cx, cy = (float(v) for v in spec["ground"]["center"])
    ax.add_patch(Rectangle((cx - size / 2, cy - size / 2), size, size, fill=False,
                           ec="saddlebrown", lw=2, ls="--", label="ground edge"))
    ax.scatter(xs, ys, s=6, c="green", label="corn (%d plants)" % len(plants))

    def arrow(pose, colour, label):
        x, y, yaw = pose
        ax.annotate("", xy=(x + 0.9 * math.cos(math.radians(yaw)),
                            y + 0.9 * math.sin(math.radians(yaw))),
                    xytext=(x, y),
                    arrowprops=dict(arrowstyle="->", color=colour, lw=2.5))
        ax.plot([x], [y], "o", color=colour, label=label)

    arrow(spec["entrance"], "orange", "row entrance")
    arrow(spec["spawn"], "blue", "robot start")
    ex, ey = spec["entrance"][:2]
    sx, sy = spec["spawn"][:2]
    ax.plot([sx, ex], [sy, ey], ":", color="grey")
    ax.text((sx + ex) / 2, (sy + ey) / 2, " %.1f m" % math.hypot(ex - sx, ey - sy),
            color="grey")
    if spec.get("trailer"):
        tx, ty, th = spec["trailer"]
        rect = Rectangle((-2.0, -1.0), 4.0, 2.0, color="grey", alpha=0.5,
                         label="trailer")
        t = matplotlib.transforms.Affine2D().rotate_deg(th).translate(tx, ty)
        rect.set_transform(t + ax.transData)
        ax.add_patch(rect)
    for block in spec["blocks"]:
        x0, y0 = block["start"]
        ax.text(x0, y0 + 0.3, block.get("name", ""), fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)  ->  EAST")
    ax.set_ylabel("y (m)  ->  NORTH")
    ax.set_title("%s: top-down layout (1 m grid)" % spec["snapshot"])
    ax.legend(loc="upper right", fontsize=8)
    ax.xaxis.set_major_locator(matplotlib.ticker.MultipleLocator(1.0))
    ax.yaxis.set_major_locator(matplotlib.ticker.MultipleLocator(1.0))
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("layout")
    ap.add_argument("--template", default="gps",
                    help="snapshot whose ground/sky/light are reused")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--no-map", action="store_true")
    args = ap.parse_args()

    with open(args.layout) as handle:
        spec = yaml.safe_load(handle)
    name = spec["snapshot"]
    rng = np.random.default_rng(args.seed)
    defaults = spec.get("defaults", {})

    plants = []
    for block in spec["blocks"]:
        plants.extend(place_block(block, defaults, rng))
    size = float(spec["ground"]["size"])
    cx, cy = (float(v) for v in spec["ground"]["center"])
    inside = size / 2.0 - 1.0
    for label, (x, y) in [("plant", p) for p in plants] + [("spawn", tuple(spec["spawn"][:2]))]:
        if abs(x - cx) > inside or abs(y - cy) > inside:
            sys.exit("%s at (%.2f, %.2f) is within 1 m of the ground edge "
                     "(%.0f m square centred on (%.2f, %.2f)); enlarge or move "
                     "`ground`" % (label, x, y, size, cx, cy))

    template_dir = os.path.join(SNAPSHOTS, args.template)
    with open(os.path.join(template_dir, "generated.world")) as handle:
        head, tail = split_template(handle.read())
    # The template's heightmap appears twice (visual + collision): resize and
    # re-centre both.
    # ⚠ Only the THREE-number <size>: the texture blocks inside the same
    # <heightmap> have one-number <size> tags of their own.
    head, n = re.subn(r"<size>\s*[0-9.]+\s+[0-9.]+\s+[0-9.]+\s*</size>",
                      "<size>%.3f %.3f %.3f</size>" % (size, size, HEIGHT_RANGE), head)
    if n != 2:
        sys.exit("expected 2 heightmap <size> tags in the template, found %d" % n)
    head = head.replace("<pos>0 0 0</pos>", "<pos>%.3f %.3f 0</pos>" % (cx, cy))

    model = MODELS[spec.get("plant_model", "tall")]
    body = []
    for i, (x, y) in enumerate(plants):
        body.append(
            "    <include>\n"
            "      <uri>model://%s</uri>\n"
            "      <pose>%.4f %.4f %.3f 0 0 %.3f</pose>\n"
            "      <name>plant_%04d</name>\n"
            "      <static>false</static>\n"
            "    </include>\n"
            % (model % rng.integers(1, 3), x, y, GROUND_Z,
               rng.uniform(0, 2 * math.pi), i))
    sx, sy, syaw = spec["spawn"]
    tail = tail.replace(
        tail[tail.index("<pose"):tail.index("</pose>") + len("</pose>")],
        "<pose frame=''>%.2f %.2f 14 0 1.1 1.5708</pose>" % (cx, cy - 7.0), 1)

    out = os.path.join(SNAPSHOTS, name)
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out)
    from PIL import Image
    Image.fromarray(np.full((HEIGHTMAP_PX, HEIGHTMAP_PX), GROUND_PIXEL, dtype=np.uint8)
                    ).save(os.path.join(out, "virtual_maize_field_heightmap.png"))
    with open(os.path.join(out, "generated.world"), "w") as handle:
        handle.write(head + "\n".join(body) + "\n    " + tail)
    with open(os.path.join(out, "gt_map.csv"), "w") as handle:
        handle.write("X,Y,kind\n")
        handle.writelines("%.6f,%.6f,crop\n" % p for p in plants)
    with open(os.path.join(out, "robot_spawner.launch"), "w") as handle:
        handle.write(
            "<launch>\n  <!-- Spawn pose from %s; switch_maize_world.sh prints it. -->\n"
            "  <node name=\"urdf_spawner\" pkg=\"gazebo_ros\" type=\"spawn_model\"\n"
            "        args=\"-urdf -model robot_model -param robot_description "
            "-x %.3f -y %.3f -z 0.35 -R 0 -P 0 -Y %.4f \" />\n</launch>\n"
            % (os.path.basename(args.layout), sx, sy, math.radians(syaw)))
    write_preview(os.path.join(out, "field_preview.png"), plants, spec)

    print("%s: %d plants in %d blocks -> %s" % (name, len(plants), len(spec["blocks"]), out))
    ex, ey, eyaw = spec["entrance"]
    print("spawn:    x:=%.3f y:=%.3f z:=0.35 yaw:=%.4f" % (sx, sy, math.radians(syaw)))
    print("entrance: entrance_x:=%.3f entrance_y:=%.3f entrance_yaw_deg:=%.1f "
          "(%.1f m from the spawn)" % (ex, ey, eyaw, math.hypot(ex - sx, ey - sy)))

    if not args.no_map:
        map_base = os.path.join(SRC, "agbot_nav", "maps", name)
        subprocess.run([sys.executable, os.path.join(SRC, "agbot_nav", "scripts",
                                                     "gt_occupancy_map.py"),
                        os.path.join(out, "gt_map.csv"), map_base,
                        "--world-size", str(size), "--center", str(cx), str(cy)],
                       check=True)


if __name__ == "__main__":
    main()
