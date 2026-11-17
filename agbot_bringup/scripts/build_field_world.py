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
        gt_map.csv                         plant + wall positions -> AMCL map
        robot_spawner.launch               spawn pose, which switch prints
        field_preview.png                  top-down picture of the layout

The sky, light and ground TEXTURES are copied from an existing snapshot
(--template, default `gps`) so the camera sees the visuals vision nav was
validated on. The ground itself is a new flat heightmap, a square of
`ground.size` m centred on `ground.center` (Gazebo heightmaps must be
square); the builder refuses plants or a spawn within 1 m of its edge.

Optional `walls:` are static boxes (a fence, a pen around the start) that
the LiDAR sees cleanly. They are written to gt_map.csv as `wall` points every
WALL_STEP m, so the ground-truth map carries them too.

Optional `trailer:` puts a static trailer with a ramp in the world, placed
so that the robot, backed straight down the ramp, stops exactly at `spawn`
(see trailer_geometry). It is deliberately NOT in gt_map.csv: the real
trailer parks somewhere different every time, so the map is built without it
and AMCL has to live with it as an unmapped object, as on the robot.

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
WALL_STEP = 0.05      # m between `wall` points in gt_map.csv
MODELS = {"tall": "agbot_maize_tall_0%d", "stock": "maize_0%d"}
JACKAL_HALF_LENGTH = 0.254   # m, base_link to either bumper
JACKAL_DECK_Z = 0.10         # m above a surface to spawn the robot on it


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


def wall_specs(spec):
    """[(name, (x1, y1), (x2, y2), height, thickness)] from `walls:`."""
    cfg = spec.get("walls") or {}
    height = float(cfg.get("height", 1.5))
    thickness = float(cfg.get("thickness", 0.10))
    out = []
    for i, wall in enumerate(cfg.get("segments", [])):
        out.append((wall.get("name", "wall_%02d" % i),
                    tuple(float(v) for v in wall["from"]),
                    tuple(float(v) for v in wall["to"]), height, thickness))
    return out


def wall_sdf(name, a, b, height, thickness):
    """A static grey box from a to b standing on the ground."""
    length = math.hypot(b[0] - a[0], b[1] - a[1]) + thickness   # close the corners
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    box = "<box><size>%.3f %.3f %.3f</size></box>" % (length, thickness, height)
    return (
        "    <model name=\"%s\">\n"
        "      <static>true</static>\n"
        "      <pose>%.4f %.4f %.3f 0 0 %.4f</pose>\n"
        "      <link name=\"link\">\n"
        "        <collision name=\"collision\"><geometry>%s</geometry></collision>\n"
        "        <visual name=\"visual\"><geometry>%s</geometry>\n"
        "          <material><script><uri>file://media/materials/scripts/gazebo.material</uri>"
        "<name>Gazebo/Grey</name></script></material>\n"
        "        </visual>\n"
        "      </link>\n"
        "    </model>\n"
        % (name, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2, GROUND_Z + height / 2, yaw,
           box, box))


def wall_points(a, b):
    """Points every WALL_STEP m along a wall, both ends included."""
    n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / WALL_STEP)))
    return [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
            for k in range(n + 1)]


def trailer_geometry(spec):
    """Where the trailer goes, from `spawn` and the `trailer:` dimensions.

    The robot parks on the deck facing the trailer's front, backs straight
    down the ramp at the rear without turning, and stops at `spawn` -- so the
    trailer is on the line through `spawn` along the spawn heading, AHEAD of
    it. Distances along that heading, from `spawn`:
        ramp foot  = half robot length + ramp_foot_gap
        deck rear  = ramp foot + the ramp's horizontal run
        robot      = deck rear + robot_setback      (where it is parked)
    Returns a dict (all metres / radians, world frame) or None.
    """
    tr = spec.get("trailer")
    if not tr:
        return None
    sx, sy, syaw_deg = (float(v) for v in spec["spawn"])
    yaw = math.radians(syaw_deg)
    h = float(tr["deck_height"])
    ramp = float(tr["ramp_length"])
    if not 0.0 < h < ramp:
        sys.exit("trailer: need 0 < deck_height < ramp_length")
    theta = math.asin(h / ramp)
    run = ramp * math.cos(theta)
    s_foot = JACKAL_HALF_LENGTH + float(tr.get("ramp_foot_gap", 0.5))
    s_rear = s_foot + run
    s_robot = s_rear + float(tr.get("robot_setback", 0.55))
    deck_len = float(tr.get("deck_length", 3.0))
    if s_robot + JACKAL_HALF_LENGTH > s_rear + deck_len:
        sys.exit("trailer: robot_setback puts the robot off the front of the deck")

    def along(s):
        return sx + s * math.cos(yaw), sy + s * math.sin(yaw)

    return dict(
        yaw=yaw, theta=theta, deck_height=h, ramp_length=ramp, run=run,
        deck_length=deck_len, deck_width=float(tr.get("deck_width", 1.3)),
        ramp_width=float(tr.get("ramp_width", 1.0)),
        origin=along(s_rear),          # model frame: deck rear edge, on the ground
        robot=along(s_robot) + (yaw,),
        # Wheel travel from the parked pose to `spawn`: the straight-line
        # distance plus what the slope adds over its horizontal run.
        reverse_wheel_distance=s_robot + (ramp - run),
    )


def trailer_sdf(g):
    """A static deck box plus one tilted plank, origin at the deck's rear edge.

    The plank's TOP edge sits exactly on the deck's rear edge (no lip for the
    wheels to catch) and its foot continues 0.10 m into the ground.
    """
    th, h, t = g["theta"], g["deck_height"], 0.04
    plank = g["ramp_length"] + 0.10
    # centre of the plank's top surface, then down its normal by t/2
    tx = -(plank / 2.0) * math.cos(th)
    tz = h - (plank / 2.0) * math.sin(th)
    px, pz = tx + (t / 2.0) * math.sin(th), tz - (t / 2.0) * math.cos(th)
    ox, oy = g["origin"]

    def box(name, pose, size, colour):
        geo = "<box><size>%.4f %.4f %.4f</size></box>" % size
        return (
            "      <link name=\"%s\">\n"
            "        <pose>%.6f 0 %.6f 0 %.6f 0</pose>\n"
            "        <collision name=\"collision\"><geometry>%s</geometry>\n"
            "          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>\n"
            "        </collision>\n"
            "        <visual name=\"visual\"><geometry>%s</geometry>\n"
            "          <material><script><uri>file://media/materials/scripts/gazebo.material</uri>"
            "<name>%s</name></script></material>\n"
            "        </visual>\n"
            "      </link>\n" % ((name,) + pose + (geo, geo, colour)))

    return (
        "    <model name=\"agbot_trailer_ramp\">\n"
        "      <static>true</static>\n"
        "      <pose>%.4f %.4f %.3f 0 0 %.4f</pose>\n" % (ox, oy, GROUND_Z, g["yaw"])
        + box("deck", (g["deck_length"] / 2.0, h / 2.0, 0.0),
              (g["deck_length"], g["deck_width"], h), "Gazebo/Grey")
        + box("ramp", (px, pz, -th), (plank, g["ramp_width"], t), "Gazebo/DarkGrey")
        + "    </model>\n")


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
    g = trailer_geometry(spec)
    if g:
        rot = matplotlib.transforms.Affine2D().rotate(g["yaw"]).translate(*g["origin"])
        for x0, length, width, colour, label in (
                (0.0, g["deck_length"], g["deck_width"], "grey", "trailer deck"),
                (-g["run"], g["run"], g["ramp_width"], "darkgrey", "ramp")):
            rect = Rectangle((x0, -width / 2.0), length, width, color=colour,
                             alpha=0.6, label=label)
            rect.set_transform(rot + ax.transData)
            ax.add_patch(rect)
        rx, ry, ryaw = g["robot"]
        arrow((rx, ry, math.degrees(ryaw)), "purple", "robot parked on the trailer")
    for i, (_, a, b, _, _) in enumerate(wall_specs(spec)):
        ax.plot([a[0], b[0]], [a[1], b[1]], "-", color="dimgrey", lw=3,
                label="wall" if i == 0 else None)
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

    walls = wall_specs(spec)
    half = size / 2.0
    for name_w, a, b, _, _ in walls:
        for x, y in (a, b):
            if abs(x - cx) > half or abs(y - cy) > half:
                sys.exit("wall %s end (%.2f, %.2f) is off the ground" % (name_w, x, y))

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
    trailer = trailer_geometry(spec)
    if trailer:
        # ⚠ The ramp needs a 1 ms physics step (the template's is 5 ms).
        # Measured 2026-11-16, this world, trailer_deploy at -0.1 m/s down the
        # 16 deg ramp, Gazebo truth:
        #   5 ms + ramp_traction   yawed 18-33 deg in 7-11 deg SNAPS on the
        #                          slope and ended 0.9 m off, at the corn
        #   1 ms, stock wheels     straight, but slid 1 m in 2 s on the ramp
        #                          and ended against pen_south, wheels spinning
        #   1 ms + ramp_traction   1 deg, no slide, 0.10 m from `spawn`
        # RTF on the laptop was still 0.99 with the LiDAR on.
        for tag, value in (("max_step_size", "0.001"),
                           ("real_time_update_rate", "1000")):
            head, n = re.subn(r"<%s>[^<]*</%s>" % (tag, tag),
                              "<%s>%s</%s>" % (tag, value, tag), head)
            if n != 1:
                sys.exit("expected one <%s> in the template, found %d" % (tag, n))

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
    body.extend(wall_sdf(*w) for w in walls)
    if trailer:
        body.append(trailer_sdf(trailer))
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
        for _, a, b, _, _ in walls:
            handle.writelines("%.6f,%.6f,wall\n" % p for p in wall_points(a, b))
    with open(os.path.join(out, "robot_spawner.launch"), "w") as handle:
        handle.write(
            "<launch>\n  <!-- Spawn pose from %s; switch_maize_world.sh prints it. -->\n"
            "  <node name=\"urdf_spawner\" pkg=\"gazebo_ros\" type=\"spawn_model\"\n"
            "        args=\"-urdf -model robot_model -param robot_description "
            "-x %.3f -y %.3f -z 0.35 -R 0 -P 0 -Y %.4f \" />\n</launch>\n"
            % (os.path.basename(args.layout), sx, sy, math.radians(syaw)))
    write_preview(os.path.join(out, "field_preview.png"), plants, spec)

    print("%s: %d plants in %d blocks, %d walls -> %s"
          % (name, len(plants), len(spec["blocks"]), len(walls), out))
    ex, ey, eyaw = spec["entrance"]
    print("spawn:    x:=%.3f y:=%.3f z:=0.35 yaw:=%.4f" % (sx, sy, math.radians(syaw)))
    print("entrance: entrance_x:=%.3f entrance_y:=%.3f entrance_yaw_deg:=%.1f "
          "(%.1f m from the spawn)" % (ex, ey, eyaw, math.hypot(ex - sx, ey - sy)))

    if trailer:
        rx, ry, ryaw = trailer["robot"]
        print("trailer:  deck %.2f m high, ramp %.2f m at %.1f deg; robot parked on it:"
              % (trailer["deck_height"], trailer["ramp_length"],
                 math.degrees(trailer["theta"])))
        print("          x:=%.3f y:=%.3f z:=%.2f yaw:=%.4f   (agbot_gazebo.launch)"
              % (rx, ry, GROUND_Z + trailer["deck_height"] + JACKAL_DECK_Z, ryaw))
        print("          backing straight off to the spawn = %.2f m of wheel travel "
              "(%.1f s at 0.1 m/s)" % (trailer["reverse_wheel_distance"],
                                       trailer["reverse_wheel_distance"] / 0.1))

    if not args.no_map:
        map_base = os.path.join(SRC, "agbot_nav", "maps", name)
        subprocess.run([sys.executable, os.path.join(SRC, "agbot_nav", "scripts",
                                                     "gt_occupancy_map.py"),
                        os.path.join(out, "gt_map.csv"), map_base,
                        "--world-size", str(size), "--center", str(cx), str(cy)],
                       check=True)


if __name__ == "__main__":
    main()
