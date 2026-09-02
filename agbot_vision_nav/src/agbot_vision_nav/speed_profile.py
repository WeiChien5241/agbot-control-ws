"""Scale the speed-coupled tuning knobs when linear_x_cruise changes.

Pure Python, no rospy, so the arithmetic below is unit-testable and lives in
exactly one place instead of in a comment that says "scale proportionally".

WHY THIS EXISTS -- the measurement, so the rule is not taken on faith.

config/params.yaml is tuned at 0.15 m/s. On 2026-09-02 a sim run was made at
0.5 m/s with nothing else changed and the robot drove over every plant.
~/agbot_logs/vision_nav_20260902_173947.csv says why:

    |angular_z| p95 = 0.175 = angular_z_max exactly -- saturated, not busy
    offset_norm swinging -0.46 -> +0.31, RMS 0.238, max 0.477
    12 WATCHDOG_ZERO events; control period 0.431 s; e2e latency 549 ms

Path curvature is what actually keeps a robot inside a row, and it is

    kappa = angular_z / linear_x        [1/m]

An angular_z_max of 0.175 is a 0.86 m turn radius at 0.15 m/s and a 2.86 m
turn radius at 0.5 m/s: the SAME clamp buys 3.3x less turning per metre
driven. Row spacing is 0.75 m and the Jackal is ~0.43 m wide, leaving ~0.16 m
of clearance per side, so the authority to correct simply was not there. The
controller did what it was told; the envelope was wrong.

WHAT SCALES, AND WHY EACH ONE

  angular_z_max        proportional to v  -- holds kappa_max constant, so the
                       tightest correctable path is the same at any speed.
  delta_angular_z_max  proportional to v  -- same argument one derivative up:
                       it holds the curvature SLEW per metre constant.
  mpc_alpha            proportional to v  -- lateral drift is e_dot = v*sin(theta),
                       so the coupling from heading into offset genuinely grows
                       with forward speed. Leaving it fixed makes the MPC model
                       less drift than is happening and under-correct early.

WHAT DELIBERATELY DOES NOT

  mpc_beta             control effectiveness is a heading rate, theta_dot =
                       angular_z, which is independent of forward speed. Scaling
                       it would tell the MPC that going faster makes it turn
                       harder, which is false.
  mpc_dt               NOT a speed constant. It is the REAL control period --
                       a property of the machine (0.04 s on the GPU robot,
                       0.431 s measured on the laptop sim), and the controller
                       already rescales alpha/beta/delta by dt/0.1 internally
                       (controller.py). So mpc_alpha stays quoted at the 0.1 s
                       reference here and there is no double counting. Pass it
                       via control_period when you have measured it.
  every distance-valued detector threshold
                       exit_confirm_distance, min_in_row_distance,
                       headland_clearance and the rest are already
                       speed-invariant BY DESIGN -- that is the whole reason
                       the 2026-07-24 field failure was fixed by moving
                       debounce from frames to metres. Scaling them here would
                       undo it.
  max_data_age_sec     the watchdog. Raising it to stop the WATCHDOG_ZERO
                       events above would buy blind driving at speed (1.0 s is
                       0.5 m at 0.5 m/s) and silence the one signal that was
                       correctly reporting the loop could not keep up. In sim
                       the fix is agbot_bringup/scripts/set_sim_rtf.sh; on the
                       robot it is a faster machine.
"""

# The speed config/params.yaml is tuned at. Every REFERENCE_* value below is
# that file's value at that speed; they are duplicated here on purpose so the
# scaling is checkable without a ROS master, and the tests pin that a request
# for REFERENCE_SPEED returns them unchanged.
REFERENCE_SPEED = 0.15

REFERENCE_ANGULAR_Z_MAX = 0.175
REFERENCE_DELTA_ANGULAR_Z_MAX = 0.2
REFERENCE_MPC_ALPHA = 0.10

# Knobs scaled proportionally to target_speed / REFERENCE_SPEED.
SPEED_SCALED = (
    ("angular_z_max", REFERENCE_ANGULAR_Z_MAX),
    ("delta_angular_z_max", REFERENCE_DELTA_ANGULAR_Z_MAX),
    ("mpc_alpha", REFERENCE_MPC_ALPHA),
)


def scaled_params(target_speed, control_period=None):
    """Return the knobs that must move with linear_x_cruise, and only those.

    target_speed:   desired linear_x_cruise in m/s.
    control_period: measured seconds between completed inferences, if known.
                    Supplied => mpc_dt is included. Omitted => it is not,
                    because inventing one is worse than leaving params.yaml's.

    Returns an ordered dict of {launch arg name: float}. Nothing else in the
    configuration is touched.
    """
    target_speed = float(target_speed)
    if target_speed <= 0.0:
        raise ValueError("target_speed must be positive, got %r" % (target_speed,))

    factor = target_speed / REFERENCE_SPEED

    params = {"linear_x_cruise": target_speed}
    for name, reference in SPEED_SCALED:
        params[name] = reference * factor

    if control_period is not None:
        control_period = float(control_period)
        if control_period <= 0.0:
            raise ValueError(
                "control_period must be positive, got %r" % (control_period,)
            )
        params["mpc_dt"] = control_period

    return params


def curvature_limit(params):
    """kappa_max = angular_z_max / linear_x_cruise, in 1/m.

    The invariant the scaling exists to preserve, and the number to quote when
    arguing about a speed envelope: it is the tightest path the controller is
    allowed to command, and it does not depend on how fast the robot is going.
    """
    return params["angular_z_max"] / params["linear_x_cruise"]


def format_launch_args(params):
    """Render as a roslaunch argument string: 'name:=value name:=value ...'."""
    return " ".join("%s:=%s" % (k, _fmt(v)) for k, v in params.items())


def _fmt(value):
    # 3 decimals is finer than any of these knobs is tuned to, and avoids
    # handing roslaunch a float repr like 0.5830000000000001.
    return ("%.3f" % value).rstrip("0").rstrip(".")
