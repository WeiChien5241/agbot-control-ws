#!/usr/bin/python3
"""How far did the robot drive? Distance (and interventions) from a rosbag.

    python3 scripts/bag_distance.py ~/bags/field_2026-08-05*.bag

ROS1 ONLY -- it imports `rosbag`, so run it on the Noetic machine (or the
robot), not in a ROS2 sandbox. Nothing else about it needs a running roscore:
it reads the file, so it works months after the run.

WHY THIS EXISTS ALONGSIDE analyze_run.py. The run CSV already carries odometry
and gives you the same numbers for any run recorded with the metrics logger
on. This script answers the same question about a bag -- including bags
recorded before the logger existed, bags from runs driven entirely by hand,
and bags where the vision node was not running at all. Record with:

    rosbag record /odometry/filtered /bluetooth_teleop/joy   # + whatever else

TWO DISTANCES, AND WHY THEY DIFFER. Integrating the POSE gives path length;
integrating |twist.linear.x| dt gives wheel-travelled distance. On a clean run
they agree within a couple of percent. The pose sum is biased UP by EKF
jitter, which is why increments below --min-step are dropped as noise; the
twist sum is biased up by wheel slip, which is a real and large effect in wet
soil. Neither is ground truth. Quote the pose sum, report the method, and if
the two disagree by more than ~10% say so rather than picking the flattering
one.
"""

import argparse
import math
import os
import sys

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
)

from agbot_vision_nav.intervention_detector import (  # noqa: E402
    DEFAULT_DEADMAN_BUTTONS,
    InterventionDetector,
)


class BagTally(object):
    """Running totals over one or more bags, in chronological order."""

    def __init__(self, min_step, detector):
        self.min_step = min_step
        self.detector = detector

        self.pose_distance = 0.0
        self.autonomous_distance = 0.0
        self.twist_distance = 0.0
        self.t_first = None
        self.t_last = None
        self.odom_messages = 0
        self.joy_messages = 0
        self._prev_xy = None
        self._prev_twist_t = None
        self.start_xy = None
        self.end_xy = None

    def add_odom(self, t, x, y, linear_x):
        self.odom_messages += 1
        if self.t_first is None:
            self.t_first = t
            self.start_xy = (x, y)
        self.t_last = t
        self.end_xy = (x, y)

        if self._prev_xy is not None:
            step = math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
            # Below min_step is EKF jitter, not motion. Summed over a 20-minute
            # run at 50 Hz, keeping it would add tens of meters of "distance"
            # to a robot that never moved.
            if step >= self.min_step:
                self.pose_distance += step
                if not (self.detector and self.detector.active(t)):
                    self.autonomous_distance += step
                self._prev_xy = (x, y)
        else:
            self._prev_xy = (x, y)

        if linear_x is not None and self._prev_twist_t is not None:
            dt = t - self._prev_twist_t
            # A gap means the bag was split or recording paused; integrating
            # across it would invent motion.
            if 0.0 < dt < 1.0:
                self.twist_distance += abs(linear_x) * dt
        self._prev_twist_t = t

    def add_joy(self, t, buttons, axes):
        self.joy_messages += 1
        if self.detector is not None:
            self.detector.update(t, buttons, axes)

    # -- results ----------------------------------------------------------

    @property
    def duration(self):
        if self.t_first is None or self.t_last is None:
            return None
        return self.t_last - self.t_first

    @property
    def net_displacement(self):
        if self.start_xy is None or self.end_xy is None:
            return None
        return math.hypot(
            self.end_xy[0] - self.start_xy[0], self.end_xy[1] - self.start_xy[1]
        )

    @property
    def interventions(self):
        return self.detector.count if self.detector is not None else 0


def read_bags(paths, tally, odom_topic, joy_topic):
    import rosbag  # ROS1 only; imported late so --help works anywhere

    topics = [odom_topic] + ([joy_topic] if joy_topic else [])
    for path in paths:
        with rosbag.Bag(path, "r") as bag:
            for topic, msg, stamp in bag.read_messages(topics=topics):
                # Header stamp where there is one (it is what the EKF used),
                # bag receipt time otherwise.
                t = stamp.to_sec()
                try:
                    header_t = msg.header.stamp.to_sec()
                    if header_t > 0.0:
                        t = header_t
                except AttributeError:
                    pass
                if topic == odom_topic:
                    pose = msg.pose.pose.position
                    linear_x = None
                    try:
                        linear_x = msg.twist.twist.linear.x
                    except AttributeError:
                        pass
                    tally.add_odom(t, pose.x, pose.y, linear_x)
                else:
                    tally.add_joy(t, msg.buttons, msg.axes)


def bag_start_time(path):
    import rosbag

    with rosbag.Bag(path, "r") as bag:
        return bag.get_start_time()


def report(tally, args):
    print("")
    print("odometry messages : %d (%s)" % (tally.odom_messages, args.odom_topic))
    if tally.odom_messages == 0:
        print("")
        print("  No messages on %s. Check the topic name with:" % args.odom_topic)
        print("      rosbag info <bag>")
        return 1

    duration = tally.duration
    print("duration          : %s"
          % ("%.0f s (%.1f min)" % (duration, duration / 60.0)
             if duration else "-"))
    print("")
    print("distance travelled: %.1f m   (path length from pose, "
          "steps < %.3f m dropped as jitter)"
          % (tally.pose_distance, args.min_step))
    print("  cross-check     : %.1f m   (integrated wheel speed "
          "|twist.linear.x| dt)" % tally.twist_distance)
    if tally.pose_distance > 1.0:
        gap = abs(tally.twist_distance - tally.pose_distance) / tally.pose_distance
        if gap > 0.10:
            print("  ** the two methods disagree by %.0f%% -- suspect wheel slip "
                  "(twist high)" % (100.0 * gap))
            print("     or a jumpy EKF (pose high). Report which one you quote.")
    net = tally.net_displacement
    if net is not None:
        print("  net displacement: %.1f m   (straight line start -> end)" % net)
    if duration:
        print("  average speed   : %.2f m/s" % (tally.pose_distance / duration))

    if args.joy_topic:
        print("")
        print("joy messages      : %d (%s)" % (tally.joy_messages, args.joy_topic))
        print("human interventions: %d   (joystick takeovers, "
              "activity within %.0f s merged)"
              % (tally.interventions, args.gap))
        print("autonomous distance: %.1f m   (teleop stretches subtracted)"
              % tally.autonomous_distance)
        if tally.interventions > 0:
            print("")
            print("  >>> %.1f m per intervention"
                  % (tally.autonomous_distance / tally.interventions))
        else:
            print("")
            print("  >>> 0 interventions over %.1f m (a lower bound, not a mean --"
                  % tally.autonomous_distance)
            print("      pool runs: sum the distances, sum the interventions)")
        if tally.joy_messages == 0:
            print("")
            print("  No joy messages: either nobody touched the pad, or the bag")
            print("  never recorded %s. Only the first is an autonomy result."
                  % args.joy_topic)
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Distance travelled (and interventions) from ROS1 bags.",
        epilog="Run on a ROS1 Noetic machine -- it imports rosbag.",
    )
    parser.add_argument("bag", nargs="+", help="one or more .bag files")
    parser.add_argument("--odom-topic", default="/odometry/filtered")
    parser.add_argument(
        "--joy-topic", default="/bluetooth_teleop/joy",
        help="teleop joystick topic for the intervention count; "
             "'' to skip the autonomy section",
    )
    parser.add_argument(
        "--deadman", default=",".join(str(b) for b in DEFAULT_DEADMAN_BUTTONS),
        help="comma-separated joy button indices that mean 'human driving' "
             "(default 4,5 = L1/R1 on the Jackal pad; '' falls back to stick "
             "deflection)",
    )
    parser.add_argument(
        "--gap", type=float, default=3.0,
        help="seconds of joystick quiet that separate two interventions "
             "(default 3)",
    )
    parser.add_argument(
        "--min-step", type=float, default=0.01,
        help="pose steps below this are dropped as EKF jitter (default 0.01 m)",
    )
    parser.add_argument(
        "--per-bag", action="store_true",
        help="also report each bag separately (default: one pooled total)",
    )
    args = parser.parse_args()

    buttons = [int(b) for b in args.deadman.split(",") if b.strip() != ""]

    def new_detector():
        if not args.joy_topic:
            return None
        return InterventionDetector(
            deadman_buttons=buttons, gap_seconds=args.gap
        )

    # Chronological order matters: the tally integrates across bag boundaries,
    # so a split recording handed over in shell-glob order must still be
    # replayed in the order it was written.
    try:
        paths = sorted(args.bag, key=bag_start_time)
    except Exception as exc:                      # noqa: BLE001
        print("could not read bags: %s" % exc, file=sys.stderr)
        return 1

    if args.per_bag:
        for path in paths:
            print("\n=== %s" % os.path.basename(path))
            one = BagTally(args.min_step, new_detector())
            read_bags([path], one, args.odom_topic, args.joy_topic)
            report(one, args)
        print("\n=== pooled: %d bag(s)" % len(paths))

    tally = BagTally(args.min_step, new_detector())
    read_bags(paths, tally, args.odom_topic, args.joy_topic)
    return report(tally, args)


if __name__ == "__main__":
    sys.exit(main())
