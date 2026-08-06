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


def cameras_launch_args():
    return [PKG, CAMERAS_LAUNCH]


def mission_launch_args(model_path="", sim=False, **fields):
    """roslaunch argv for vision_nav.launch.

    Args:
        model_path: absolute path to the .pt, or blank to use the launch
            file's default (config/exported_best.pt in the package).
        sim: pass sim:=true (Gazebo camera topics) when set.
        **fields: any of PASSTHROUGH_FIELDS (blank/None => not passed) and
            BOOLEAN_FIELDS (always passed).

    Raises:
        ValueError: unknown field name -- a typo would otherwise be dropped
            silently and the run would quietly use the yaml value the
            operator was trying to override.
    """
    unknown = set(fields) - set(PASSTHROUGH_FIELDS) - set(BOOLEAN_FIELDS)
    if unknown:
        raise ValueError("unknown launch field(s): %s" % ", ".join(sorted(unknown)))

    args = [PKG, VISION_NAV_LAUNCH]
    model_path = (model_path or "").strip()
    if model_path:
        args.append("model_path:=%s" % model_path)
    if sim:
        args.append("sim:=true")
    for name in BOOLEAN_FIELDS:
        if name in fields:
            args.append("%s:=%s" % (name, "true" if fields[name] else "false"))
    for name in PASSTHROUGH_FIELDS:
        value = fields.get(name)
        value = "" if value is None else str(value).strip()
        if value:
            args.append("%s:=%s" % (name, value))
    return args
