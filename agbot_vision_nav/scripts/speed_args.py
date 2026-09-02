#!/usr/bin/env python3
"""Print the roslaunch arguments for running vision-nav at a given speed.

No ROS and no rebuild needed -- like analyze_run.py and bag_distance.py, this
is run straight out of the source tree:

    python3 agbot_vision_nav/scripts/speed_args.py 0.5 --control-period 0.13

which is meant to be substituted into the launch line:

    roslaunch agbot_vision_nav vision_nav.launch model_path:=... sim:=true \
      mission_enabled:=true num_rows:=7 rear_camera_enabled:=true \
      $(python3 agbot_vision_nav/scripts/speed_args.py 0.5 --control-period 0.13)

Why a command and not a second params file: config/params.yaml is the source
of truth for tuning and a launch argument is the per-run override. A speed
profile is exactly a per-run override, so it belongs on the command line --
adding a second rosparam file would put two files in charge of the same 44
knobs, which is the failure that made editing params.yaml a no-op before
2026-07-30.

The arithmetic and the reasoning for each knob live in
src/agbot_vision_nav/speed_profile.py.
"""

import argparse
import os
import sys

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
)

from agbot_vision_nav.speed_profile import (  # noqa: E402
    REFERENCE_SPEED,
    curvature_limit,
    format_launch_args,
    scaled_params,
)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "speed",
        type=float,
        help="target linear_x_cruise in m/s (params.yaml is tuned at %.2f)"
        % REFERENCE_SPEED,
    )
    parser.add_argument(
        "--control-period",
        type=float,
        default=None,
        metavar="SECONDS",
        help="MEASURED seconds between completed inferences, which becomes "
        "mpc_dt. Read it off a previous run: the 'control period' is the mean "
        "gap between rows of the metrics CSV, and analyze_run.py's end-to-end "
        "latency is the closely related number. Omitted, mpc_dt is left to "
        "params.yaml -- guessing it is worse than not setting it.",
    )
    parser.add_argument(
        "--explain",
        action="store_true",
        help="also print the reasoning to stderr (stdout stays paste-safe)",
    )
    args = parser.parse_args(argv)

    params = scaled_params(args.speed, control_period=args.control_period)

    if args.explain:
        kappa = curvature_limit(params)
        print(
            "target %.2f m/s  =  %.2fx the %.2f m/s params.yaml is tuned at"
            % (args.speed, args.speed / REFERENCE_SPEED, REFERENCE_SPEED),
            file=sys.stderr,
        )
        print(
            "max path curvature %.3f /m  (tightest radius %.2f m) -- unchanged "
            "by design; that invariance is the point of the scaling"
            % (kappa, 1.0 / kappa),
            file=sys.stderr,
        )
        if args.control_period is None:
            print(
                "mpc_dt NOT set: pass --control-period with a measured value. "
                "Leaving it at 0.1 while the loop really runs at 0.43 s made "
                "the MPC model ~1/4 of the drift actually happening "
                "(sim 2026-09-02).",
                file=sys.stderr,
            )
        else:
            print(
                "mpc_dt %.3f s: at %.2f m/s that is %.3f m of open-loop travel "
                "per decision."
                % (
                    args.control_period,
                    args.speed,
                    args.control_period * args.speed,
                ),
                file=sys.stderr,
            )
            print(
                "Row spacing is 0.75 m and the robot is ~0.43 m wide, so there "
                "is ~0.16 m of clearance per side. If the number above is a "
                "large fraction of that, the loop is too slow for this speed -- "
                "throttle the sim (agbot_bringup/scripts/set_sim_rtf.sh) rather "
                "than raising max_data_age_sec.",
                file=sys.stderr,
            )

    print(format_launch_args(params))
    return 0


if __name__ == "__main__":
    sys.exit(main())
