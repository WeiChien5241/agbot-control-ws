"""Build a roslaunch argument list from operator-panel form fields.

Pure Python, no rospy and no Qt, so the one rule that matters here can be
unit-tested: A BLANK FIELD IS NOT PASSED.

That rule is the repo's configuration contract, not a convenience. Launch
<arg>s default to the empty string and their <param> tags are conditional, so
config/params.yaml holds the tuning and a launch argument overrides it only
when actually supplied. A panel that helpfully passed its own default for
every field would re-pin all of them on every run and silently defeat that --
which is exactly the failure that made editing params.yaml a no-op before
2026-07-30, and cost a session to find. The panel must be able to launch a
run that is tuned entirely by the yaml.

model_path follows the same rule, and it is worth being precise about why: it
has no params.yaml entry (the launch file owns it), but it DOES have a launch
default of $(find agbot_vision_nav)/config/exported_best.pt. So blank means
"use the weights that ship with the package", which is the normal case on the
robot -- and requiring it here would make the panel harder to use than the
command line it replaces.
"""

PKG = "agbot_vision_nav"
CAMERAS_LAUNCH = "cameras.launch"
VISION_NAV_LAUNCH = "vision_nav.launch"

# Fields passed through verbatim when non-blank. Booleans are handled
# separately because a checkbox has no "blank" state -- it is always either
# true or false, so it always carries an opinion and is always passed.
PASSTHROUGH_FIELDS = (
    "num_rows",
    "first_turn_direction",
    "linear_x_cruise",
    "angular_z_max",
)

BOOLEAN_FIELDS = (
    "mission_enabled",
    "rear_camera_enabled",
)

# Which physical camera sits in the front mount, and what cameras.launch.py
# and vision_nav.launch each need to be told about it. "wdr" is the launch
# files' own default (front:=true brio_front:=false, camera_topic defaults to
# /usb_cam/...), so it needs no extra args -- only "brio_front" does, because
# it overrides both the node cameras.launch starts AND the topic
# vision_nav.launch subscribes to; those are two separate launch files with
# no shared state, so both sides must be told every time a run picks it.
FRONT_SOURCES = ("wdr", "brio_front")
BRIO_FRONT_TOPIC = "/brio_front/image_raw/compressed"


def cameras_launch_args(front_source="wdr"):
    """The real robot's USB camera bringup -- the only frame source the panel
    starts.

    ⚠ It deliberately does NOT start Gazebo in simulation, though it used to.
    The settled sim workflow (2026-08-06) is the simulator in its OWN terminal:
    Gazebo's output is extremely noisy and buries the vision-nav startup config
    block, which is the part actually worth reading. The panel offering to
    start it as well was one more way to end up with two simulators, or with
    the config block unreadable, for no benefit. The `simulation` checkbox now
    means exactly one thing -- sim:=true on the vision-nav launch, which picks
    the Gazebo camera topics -- and says nothing about who started the
    simulator.

    front_source picks which camera sits in the front mount ("wdr", the
    launch file default, or "brio_front" -- a spare Brio swapped in when the
    WDR camera is off the robot). "wdr" passes nothing, matching
    cameras.launch's own default so a plain cameras_launch_args() call keeps
    behaving exactly as before.
    """
    if front_source not in FRONT_SOURCES:
        raise ValueError("unknown front_source: %s" % front_source)
    args = [PKG, CAMERAS_LAUNCH]
    if front_source == "brio_front":
        args.append("front:=false")
        args.append("brio_front:=true")
    return args


def mission_launch_args(model_path="", sim=False, front_source="wdr", **fields):
    """roslaunch argv for vision_nav.launch.

    Args:
        model_path: absolute path to the .pt, or blank to use the launch
            file's default (config/exported_best.pt in the package).
        sim: pass sim:=true (Gazebo camera topics) when set.
        front_source: "wdr" (default) or "brio_front" -- must match whatever
            was passed to cameras_launch_args() for this run, since
            vision_nav_node has to subscribe to the topic cameras.launch is
            actually publishing. Ignored when sim=True: the Gazebo topic
            always wins on the real vision_nav.launch camera_topic default,
            and there is no Brio in simulation.
        **fields: any of PASSTHROUGH_FIELDS (blank/None => not passed) and
            BOOLEAN_FIELDS (always passed).

    Raises:
        ValueError: unknown field name, or unknown front_source -- a typo
            would otherwise be dropped silently and the run would quietly
            use the yaml/launch-default value the operator was trying to
            override.
    """
    unknown = set(fields) - set(PASSTHROUGH_FIELDS) - set(BOOLEAN_FIELDS)
    if unknown:
        raise ValueError("unknown launch field(s): %s" % ", ".join(sorted(unknown)))
    if front_source not in FRONT_SOURCES:
        raise ValueError("unknown front_source: %s" % front_source)

    args = [PKG, VISION_NAV_LAUNCH]
    model_path = (model_path or "").strip()
    if model_path:
        args.append("model_path:=%s" % model_path)
    if sim:
        args.append("sim:=true")
    elif front_source == "brio_front":
        args.append("camera_topic:=%s" % BRIO_FRONT_TOPIC)
        args.append("camera_topic_is_compressed:=true")
    for name in BOOLEAN_FIELDS:
        if name in fields:
            args.append("%s:=%s" % (name, "true" if fields[name] else "false"))
    for name in PASSTHROUGH_FIELDS:
        value = fields.get(name)
        value = "" if value is None else str(value).strip()
        if value:
            args.append("%s:=%s" % (name, value))
    return args
