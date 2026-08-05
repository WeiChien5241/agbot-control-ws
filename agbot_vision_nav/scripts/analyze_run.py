#!/usr/bin/python3
"""Turn one or more vision-nav run CSVs into a performance report.

No ROS and no third-party packages -- runs anywhere the CSV can be copied,
including a laptop that has never had catkin on it. Same standalone spirit as
scripts/benchmark_inference.py.

    PYTHONPATH=src python3 scripts/analyze_run.py ~/agbot_logs/vision_nav_*.csv

WHAT THE TRACKING ERROR IS, AND IS NOT. offset_norm is normalized IMAGE space:
0.0 is the image centre column, +-1.0 the image edges. It is what the MPC
minimises, so it is the honest measure of how well the controller held the
row. It is NOT a distance from the row centreline in meters, and it shifts
with camera mount height (the normal in-row corridor at the near scan row
reads ~0.5 on the tall mount and ~0.7 on the low one) -- so never compare a
number from one rig against a number from another. Say which robot and which
mount produced each column.

FOLLOW_ROW is the number to quote. The turn and traverse states are
odometry-driven open loop; their offsets describe the headland, not the
controller, which is why the per-state breakdown exists.
"""

import argparse
import os
import sys

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
)

from agbot_vision_nav.metrics_logger import read_csv, summarize  # noqa: E402


def fmt(value, spec="%.3f"):
    return "-" if value is None else spec % value


def dist_cells(dist, scale=1.0, spec="%.3f"):
    if dist is None:
        return ["-"] * 4
    return [
        fmt(dist["mean"] * scale, spec),
        fmt(dist["p50"] * scale, spec),
        fmt(dist["p95"] * scale, spec),
        fmt(dist["max"] * scale, spec),
    ]


def print_table(headers, rows):
    if not rows:
        return
    widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(str(cell)))
    line = "  ".join(h.ljust(widths[i]) for i, h in enumerate(headers))
    print("  " + line.rstrip())
    print("  " + "  ".join("-" * w for w in widths))
    for row in rows:
        print(
            "  "
            + "  ".join(str(c).ljust(widths[i]) for i, c in enumerate(row)).rstrip()
        )


def report(labels, summaries, max_events):
    multi = len(summaries) > 1
    headers = ["metric"] + labels

    def section(title):
        print("\n" + title)
        print("=" * len(title))

    def col(fn):
        return [fn(s) for s in summaries]

    section("Tracking error  (normalized image space -- NOT meters)")
    rows = []
    for key, label in (
        ("rms", "RMS |offset|"),
        ("mean_abs", "mean |offset|"),
        ("p95_abs", "p95 |offset|"),
        ("max_abs", "max |offset|"),
        ("mean_signed", "signed mean (bias)"),
        ("n", "valid frames"),
    ):
        spec = "%d" if key == "n" else "%.3f"
        rows.append(
            [label]
            + col(lambda s, k=key, sp=spec: fmt(
                s["tracking"][k] if s["tracking"] else None, sp
            ))
        )
    print_table(headers, rows)
    print(
        "\n  RMS is the headline: it is the quantity the MPC's quadratic offset\n"
        "  cost actually minimises. The signed mean is a standing left/right\n"
        "  bias -- a non-zero value with a small RMS means the robot held a\n"
        "  line, just not the centre one (check camera yaw alignment)."
    )

    section("Tracking error by mission state")
    states = sorted({st for s in summaries for st in s["by_state"]})
    rows = []
    for state in states:
        row = [state]
        for s in summaries:
            st = s["by_state"].get(state)
            row.append(
                "-" if st is None
                else "rms %.3f  p95 %.3f  max %.3f  (n=%d)"
                % (st["rms"], st["p95_abs"], st["max_abs"], st["n"])
            )
        rows.append(row)
    print_table(headers, rows)
    print(
        "\n  Quote FOLLOW_ROW. TURN_1/TURN_2/TRAVERSE are odometry open loop --\n"
        "  their offsets measure the headland, not the controller."
    )

    section("Autonomy  (distance per intervention)")

    def aut(s, key):
        return (s.get("autonomy") or {}).get(key)

    rows = [
        ["distance travelled (m)"] + col(lambda s: fmt(s["distance_m"], "%.1f")),
        ["autonomous distance (m)"]
        + col(lambda s: fmt(aut(s, "autonomous_distance_m"), "%.1f")),
        ["human interventions"]
        + col(lambda s: fmt(aut(s, "interventions"), "%d")),
        ["m per intervention (MDBI)"]
        + col(lambda s: (
            ">= %.1f" % aut(s, "autonomous_distance_m")
            if aut(s, "distance_per_intervention_m") is None
            and aut(s, "interventions") == 0
            and aut(s, "autonomous_distance_m") is not None
            else fmt(aut(s, "distance_per_intervention_m"), "%.1f")
        )),
        ["teleop time (s)"] + col(lambda s: fmt(aut(s, "teleop_seconds"), "%.0f")),
        ["duration (s)"] + col(lambda s: fmt(s["duration_s"], "%.0f")),
    ]
    print_table(headers, rows)
    print(
        "\n  Distance per intervention is the headline autonomy number and the\n"
        "  one comparable to the literature -- unlike offset_norm it is in\n"
        "  meters and does not depend on the camera mount. An intervention is\n"
        "  a joystick takeover (deadman held); activity within a few seconds\n"
        "  of the previous counts as the SAME intervention, so one messy\n"
        "  rescue scores 1, not 5. Distance driven under teleop is subtracted\n"
        "  from the autonomous distance. A run with zero interventions has no\n"
        "  mean, only the lower bound shown as '>='; pool several runs (sum\n"
        "  the distances, sum the interventions) before quoting a figure.\n"
        "  Distance comes from /odometry/filtered -- wheel odometry, so it\n"
        "  over-reads where the wheels slip."
    )

    section("Control effort")
    rows = [
        ["|angular_z| mean / p95 (rad/s)"]
        + col(lambda s: "%s / %s" % (
            fmt((s["control"].get("angular_abs") or {}).get("mean")),
            fmt((s["control"].get("angular_abs") or {}).get("p95")),
        )),
        ["step |d angular_z| mean / p95"]
        + col(lambda s: "%s / %s" % (
            fmt((s["control"].get("angular_delta_abs") or {}).get("mean")),
            fmt((s["control"].get("angular_delta_abs") or {}).get("p95")),
        )),
        ["linear_x mean (m/s)"]
        + col(lambda s: fmt((s["control"].get("linear_x") or {}).get("mean"))),
    ]
    print_table(headers, rows)
    print(
        "\n  The step size is the one to watch: a controller can hold a fine RMS\n"
        "  offset while sawing the wheels back and forth to do it."
    )

    section("Perception health")
    rows = [
        ["invalid frames (%)"]
        + col(lambda s: fmt(
            None if s["invalid_fraction"] is None
            else 100.0 * s["invalid_fraction"], "%.1f"
        )),
    ]
    for key, label in (
        ("traversable_fraction", "traversable fraction mean"),
        ("obstacle_fraction", "obstacle fraction mean"),
        ("near_row_width", "near-row corridor width mean"),
    ):
        rows.append(
            [label]
            + col(lambda s, k=key: fmt((s["perception"].get(k) or {}).get("mean")))
        )
    print_table(headers, rows)
    print(
        "\n  Near-row corridor width is mount-specific (~0.5 tall, ~0.7 low).\n"
        "  A high invalid-frame rate makes every number above less trustworthy."
    )

    section("Timing")
    rows = []
    for key, label in (
        ("inference_s", "inference (ms)"),
        ("e2e_latency_s", "end-to-end latency (ms)"),
        ("wait_age_s", "frame wait age (ms)"),
    ):
        rows.append(
            [label]
            + col(lambda s, k=key: "%s / %s / %s" % tuple(
                dist_cells(s["timing"].get(k), 1000.0, "%.0f")[:3]
            ))
        )
    rows.append(
        ["dropped frames (%) [by design]"]
        + col(lambda s: fmt(
            None if s["timing"].get("dropped_fraction") is None
            else 100.0 * s["timing"]["dropped_fraction"], "%.0f"
        ))
    )
    rows.append(["logged frames"] + col(lambda s: "%d" % s["rows"]))
    print_table(headers, rows)
    print("\n  Timing cells are mean / p50 / p95.")

    section("Failure events")
    for label, s in zip(labels, summaries):
        if multi:
            print("\n%s:" % label)
        if not s["events"]:
            print("  none")
            continue
        counts = ", ".join(
            "%s x%d" % (k, v) for k, v in sorted(s["event_counts"].items())
        )
        print("  %s" % counts)
        rows = []
        for row in s["events"][:max_events]:
            rows.append([
                row.get("event") or "-",
                fmt(row.get("t_ros"), "%.1f"),
                row.get("state") or "-",
                fmt(row.get("distance_in_row"), "%.2f"),
                fmt(row.get("offset_norm")),
                fmt(row.get("near_row_width"), "%.2f"),
                "%s/%s" % (
                    fmt(row.get("near_row_edge_l"), "%.2f"),
                    fmt(row.get("near_row_edge_r"), "%.2f"),
                ),
                fmt(row.get("traversable_fraction"), "%.2f"),
                fmt(row.get("obstacle_fraction"), "%.2f"),
            ])
        print()
        print_table(
            ["event", "t", "state", "d_row", "offset", "near w",
             "edges", "trav", "obst"],
            rows,
        )
        if len(s["events"]) > max_events:
            print("  ... %d more (raise --max-events)"
                  % (len(s["events"]) - max_events))
    print(
        "\n  Each event carries the perception state that produced it, so a\n"
        "  failure arrives with its own evidence: trav falling while obst\n"
        "  climbs is a healthy blocker; both at 0.00 is a garbage mask."
    )
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Performance report from vision_nav run CSVs.",
        epilog="offset_norm is normalized image space, NOT meters, and is "
               "mount-dependent -- do not compare across camera heights.",
    )
    parser.add_argument("csv", nargs="+", help="one or more run CSV files")
    parser.add_argument(
        "--max-events", type=int, default=20,
        help="failure-event rows to print per run (default 20)",
    )
    args = parser.parse_args()

    labels, summaries = [], []
    for path in args.csv:
        try:
            rows = read_csv(path)
        except IOError as exc:
            print("skipping %s: %s" % (path, exc), file=sys.stderr)
            continue
        if not rows:
            print("skipping %s: no data rows" % path, file=sys.stderr)
            continue
        labels.append(os.path.basename(path).replace("vision_nav_", "")
                      .replace(".csv", ""))
        summaries.append(summarize(rows))

    if not summaries:
        print("no readable runs", file=sys.stderr)
        return 1

    print("vision-nav run report: %d run(s)" % len(summaries))
    report(labels, summaries, args.max_events)
    return 0


if __name__ == "__main__":
    sys.exit(main())
