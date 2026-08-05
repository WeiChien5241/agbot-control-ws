"""Per-run CSV metrics for the vision-nav pipeline.

Pure Python, no rospy -- the ROS node hands it one row per processed frame and
this module owns the file, the schema, and the statistics. Answers the
questions the console log cannot: how far off the row centre did the robot
actually run, how hard was it steering, and what happened at each failure.

WHY A FILE AND NOT THE CONSOLE. Every quantity here is already computed on
every frame inside the node, but the console only ever sees a throttled prose
summary: the timing line every 5 s, the detector line at 1 Hz while a meter is
moving. On the 2 Hz CPU robot that is one frame in ten; on the GPU robot at
~24 Hz it is roughly one in a hundred and twenty. An RMS or a p95 computed from
a 5%-duty sample is not a measurement, and the same throttling has already made
a partially-firing exit signature look like no signature at all (sim,
2026-07-28). One row per frame, written as the run happens, is the fix.

UNITS, AND THE ONE THING NOT TO CLAIM. ``offset_norm`` is normalized IMAGE
space: 0.0 is the image centre column and +-1.0 the image edges. It is the
right control-loop error signal -- it is what the MPC minimises -- but it is
NOT a distance from the row centreline in meters, and it does not convert to
one without per-mount camera geometry. It also shifts with camera height (the
normal in-row corridor at the near scan row reads ~0.5 on the tall mount and
~0.7 on the low one), so a number from one rig must never be compared against
a number from another. Report it dimensionless, say which mount produced it,
and use Gazebo ground truth if a figure in meters is genuinely needed.
"""

import csv
import math
import os
import threading
import time


# The schema, in one place. The writer emits exactly these columns in exactly
# this order and analyze_run.py reads them back by name, so the two cannot
# drift apart. Appending a column here is safe: log() fills anything a caller
# omits with an empty string, so no call site breaks.
COLUMNS = (
    # -- time -------------------------------------------------------------
    "t_ros",                  # rospy clock (follows sim time under Gazebo)
    "t_wall",                 # monotonic-ish wall clock, for sim-time runs
    "frame_stamp",            # camera header stamp of the frame behind this row
    # -- perception -------------------------------------------------------
    "valid",                  # did the centerline estimate succeed
    "offset_norm",            # THE tracking error (normalized image space)
    "slope_term",
    "traversable_fraction",
    "obstacle_fraction",
    # -- scan rows --------------------------------------------------------
    # The nearest row's numbers get first-class columns because they are the
    # ones that decide an exit and therefore the ones you tune against. The
    # full per-row lists ride along as ';'-joined strings so that changing the
    # LENGTH of scan_row_fractions does not churn the schema.
    "near_row_width",
    "near_row_edge_l",
    "near_row_edge_r",
    "scan_offsets",
    "scan_x_left",
    "scan_x_right",
    # -- control ----------------------------------------------------------
    "linear_x",
    "angular_z",
    "camera",                 # "front" | "rear"
    "teleop",                 # 1 while a human is driving (joystick deadman)
    # -- mission ----------------------------------------------------------
    "state",
    "rows_driven",
    "distance_in_row",
    # -- exit detector ----------------------------------------------------
    "open_distance",
    "blocked_seconds",
    "corridor_rows",
    "wide_rows",
    "open_rows",
    "open_armed",
    "blocked_armed",
    # -- odometry ---------------------------------------------------------
    "odom_x",
    "odom_y",
    "odom_yaw",
    # -- timing -----------------------------------------------------------
    "inference_s",
    "e2e_latency_s",
    "wait_age_s",
    "frames_received",
    "frames_processed",
    # -- failures ---------------------------------------------------------
    "event",
)

# Rows between flushes. A field run ends with Ctrl-C if you are lucky and a
# yanked battery if you are not, so buffering the whole run in memory would
# regularly lose it. At ~24 Hz this costs a flush a second.
_FLUSH_EVERY = 20


def join_list(values):
    """Format a per-scan-row sequence as one ';'-joined CSV cell.

    None entries (a scan row that found no corridor) become empty, so a row
    with a missing corridor is distinguishable from one reading zero.
    """
    if not values:
        return ""
    return ";".join("" if v is None else "%.4f" % float(v) for v in values)


def make_run_path(directory, now=None):
    """Timestamped CSV path inside `directory` (which is ~-expanded)."""
    stamp = time.localtime(now if now is not None else time.time())
    return os.path.join(
        os.path.expanduser(directory),
        "vision_nav_%s.csv" % time.strftime("%Y%m%d_%H%M%S", stamp),
    )


class RunMetricsLogger:
    """Writes one CSV row per processed frame.

    Deliberately unkillable: every public method swallows its own exceptions
    and reports through `errors`. An instrument that can take down the node it
    is measuring is worse than no instrument, and this one runs on the
    inference thread.
    """

    def __init__(self, path):
        self.path = path
        self.rows_written = 0
        self.errors = []          # one-shot messages; the node logs them
        self._lock = threading.Lock()
        self._pending_event = None
        self._handle = None
        self._writer = None
        self._since_flush = 0

        try:
            parent = os.path.dirname(path)
            if parent:
                # exist_ok is Python 3 only; this repo is 3.8+ everywhere.
                os.makedirs(parent, exist_ok=True)
            self._handle = open(path, "w", newline="")
            self._writer = csv.DictWriter(
                self._handle, fieldnames=list(COLUMNS), extrasaction="ignore"
            )
            self._writer.writeheader()
            self._handle.flush()
        except Exception as exc:                      # noqa: BLE001
            self._fail("could not open %s: %s" % (path, exc))
            self._handle = None
            self._writer = None

    # -- writing ----------------------------------------------------------

    def mark_event(self, name):
        """Attach `name` to the NEXT row written, then clear it.

        Events are attached rather than written as rows of their own so that a
        failure arrives together with the perception and detector state that
        produced it -- "BLOCKED at 4.2 m" is not actionable, "BLOCKED at 4.2 m
        with obst=0.31 trav=0.00 near w=-" is. Safe to call from the watchdog
        timer thread.
        """
        try:
            with self._lock:
                if self._pending_event and self._pending_event != name:
                    # Two events before the next frame: keep both rather than
                    # silently dropping one. Rare, but losing a BLOCKED because
                    # a WATCHDOG_ZERO landed in the same gap would be exactly
                    # the wrong failure.
                    self._pending_event = self._pending_event + "|" + name
                else:
                    self._pending_event = name
        except Exception as exc:                      # noqa: BLE001
            self._fail("mark_event failed: %s" % exc)

    def log(self, **fields):
        """Write one row. Missing columns are written empty."""
        if self._writer is None:
            return
        try:
            with self._lock:
                if self._pending_event is not None:
                    fields["event"] = self._pending_event
                    self._pending_event = None
                row = {}
                for name in COLUMNS:
                    value = fields.get(name)
                    row[name] = "" if value is None else _fmt(value)
                self._writer.writerow(row)
                self.rows_written += 1
                self._since_flush += 1
                # Always flush an event row: those are the ones you go looking
                # for after a run that ended badly.
                if self._since_flush >= _FLUSH_EVERY or row["event"]:
                    self._handle.flush()
                    self._since_flush = 0
        except Exception as exc:                      # noqa: BLE001
            self._fail("write failed: %s" % exc)
            self._writer = None                       # stop trying

    def close(self):
        try:
            with self._lock:
                # `closed` guard: a run whose fd already died mid-way would
                # otherwise report a second, redundant error at shutdown and
                # bury the one that actually mattered.
                if self._handle is not None and not self._handle.closed:
                    self._handle.flush()
                    self._handle.close()
                self._handle = None
                self._writer = None
        except Exception as exc:                      # noqa: BLE001
            self._fail("close failed: %s" % exc)

    def _fail(self, message):
        # Deduplicated: a broken file descriptor would otherwise append a
        # message per frame and turn a diagnostics failure into a memory leak.
        if message not in self.errors:
            self.errors.append(message)


def _fmt(value):
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return ""
        return "%.6g" % value
    return str(value)


# -- reading + statistics -------------------------------------------------


def read_csv(path):
    """Parse a run CSV into a list of dicts. Numeric cells become floats."""
    with open(path, "r", newline="") as handle:
        return [_coerce(row) for row in csv.DictReader(handle)]


_TEXT_COLUMNS = frozenset(
    ("state", "camera", "event", "scan_offsets", "scan_x_left", "scan_x_right")
)


def _coerce(row):
    out = {}
    for key, value in row.items():
        if value is None or value == "":
            out[key] = None
        elif key in _TEXT_COLUMNS:
            out[key] = value
        else:
            try:
                out[key] = float(value)
            except (TypeError, ValueError):
                out[key] = value
    return out


def _percentile(sorted_vals, q):
    """Nearest-rank percentile of an already-sorted non-empty list.

    Same definition timing_stats.py uses, so a p95 means one thing across the
    whole package.
    """
    idx = int(round(q / 100.0 * (len(sorted_vals) - 1)))
    return sorted_vals[idx]


def error_stats(values):
    """RMS / mean-abs / p95-abs / max-abs of a tracking-error series.

    RMS is the headline: it is the quantity the MPC's quadratic offset cost
    actually minimises, so it is the honest summary of how well the controller
    did its job. mean-abs is what a reader intuits. p95 and max say how bad the
    worst of it got, which is the half of the story an average hides.
    """
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    abs_vals = sorted(abs(v) for v in vals)
    return {
        "n": len(vals),
        "rms": math.sqrt(sum(v * v for v in vals) / len(vals)),
        "mean_abs": sum(abs_vals) / len(abs_vals),
        "p95_abs": _percentile(abs_vals, 95),
        "max_abs": abs_vals[-1],
        "mean_signed": sum(vals) / len(vals),   # a standing left/right bias
    }


def _dist(values):
    vals = sorted(v for v in values if v is not None)
    if not vals:
        return None
    return {
        "n": len(vals),
        "mean": sum(vals) / len(vals),
        "p50": _percentile(vals, 50),
        "p95": _percentile(vals, 95),
        "max": vals[-1],
    }


def summarize(rows):
    """Whole-run statistics from parsed rows.

    Shared by analyze_run.py and the node's own shutdown summary so that
    "tracking error" is defined once and means the same thing in the console
    log and in the report.
    """
    rows = list(rows)
    summary = {
        "rows": len(rows),
        "tracking": None,
        "by_state": {},
        "invalid_fraction": None,
        "control": {},
        "perception": {},
        "timing": {},
        "events": [],
        "event_counts": {},
        "duration_s": None,
        "distance_m": None,
        "autonomy": None,
    }
    if not rows:
        return summary

    # Steering quality is only meaningful on frames where perception actually
    # produced a centerline AND the robot was following a row. Turn and
    # traverse states are odometry-driven open loop -- averaging their offsets
    # into the tracking error would measure the headland, not the controller.
    def is_valid(row):
        return row.get("valid") in (1.0, 1, True)

    valid_rows = [r for r in rows if is_valid(r)]
    summary["invalid_fraction"] = 1.0 - float(len(valid_rows)) / len(rows)
    summary["tracking"] = error_stats([r.get("offset_norm") for r in valid_rows])

    by_state = {}
    for row in valid_rows:
        by_state.setdefault(row.get("state") or "-", []).append(
            row.get("offset_norm")
        )
    for state, values in by_state.items():
        stats = error_stats(values)
        if stats is not None:
            summary["by_state"][state] = stats

    # Control effort. The rate of change is the interesting one: a controller
    # that holds the line by sawing the wheels back and forth has a fine RMS
    # offset and is still doing something wrong.
    angular = [r.get("angular_z") for r in rows if r.get("angular_z") is not None]
    summary["control"]["angular_abs"] = _dist([abs(v) for v in angular])
    deltas = [abs(angular[i] - angular[i - 1]) for i in range(1, len(angular))]
    summary["control"]["angular_delta_abs"] = _dist(deltas)
    summary["control"]["linear_x"] = _dist(
        [r.get("linear_x") for r in rows]
    )

    summary["perception"]["traversable_fraction"] = _dist(
        [r.get("traversable_fraction") for r in rows]
    )
    summary["perception"]["obstacle_fraction"] = _dist(
        [r.get("obstacle_fraction") for r in rows]
    )
    summary["perception"]["near_row_width"] = _dist(
        [r.get("near_row_width") for r in rows]
    )

    for key in ("inference_s", "e2e_latency_s", "wait_age_s"):
        summary["timing"][key] = _dist([r.get(key) for r in rows])

    # Frame accounting comes from the last row that carries it: the node's
    # counters are cumulative, so the final value is the run total.
    for row in reversed(rows):
        received = row.get("frames_received")
        processed = row.get("frames_processed")
        if received and processed:
            summary["timing"]["frames_received"] = received
            summary["timing"]["frames_processed"] = processed
            summary["timing"]["dropped_fraction"] = 1.0 - processed / received
            break

    for row in rows:
        if row.get("event"):
            summary["events"].append(row)
            for name in str(row["event"]).split("|"):
                summary["event_counts"][name] = (
                    summary["event_counts"].get(name, 0) + 1
                )

    stamps = [r.get("t_ros") for r in rows if r.get("t_ros") is not None]
    if len(stamps) >= 2:
        summary["duration_s"] = stamps[-1] - stamps[0]
    summary["distance_m"] = _odometry_distance(rows)
    summary["autonomy"] = _autonomy(rows, summary)
    return summary


def _is_teleop(row):
    return row.get("teleop") in (1.0, 1, True)


def _odometry_distance(rows, autonomous_only=False):
    """Path length from the odometry columns, or None without odometry.

    With `autonomous_only`, a segment whose either end was driven by a human
    is dropped rather than counted -- distance per intervention has to be
    distance the ROBOT drove, or a long manual repositioning would flatter
    the number it is supposed to indict.
    """
    total = 0.0
    prev = None
    seen = False
    for row in rows:
        x, y = row.get("odom_x"), row.get("odom_y")
        if x is None or y is None:
            continue
        seen = True
        teleop = _is_teleop(row)
        if prev is not None and not (autonomous_only and (teleop or prev[2])):
            total += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y, teleop)
    return total if seen else None


def _autonomy(rows, summary):
    """Interventions, autonomous distance, and distance per intervention.

    The headline autonomy metric in the field-robot literature: meters the
    robot drove itself per time a human had to take over. Reported as None
    when there is no odometry (nothing to divide) and separately flagged when
    there were zero interventions -- a clean run has no MEAN distance between
    interventions, it has a lower bound, and printing "inf" or silently
    dividing by one would misreport both.
    """
    interventions = int(summary["event_counts"].get("INTERVENTION", 0))
    teleop_rows = [r for r in rows if _is_teleop(r)]
    autonomous_m = _odometry_distance(rows, autonomous_only=True)

    teleop_s = None
    stamps = [r.get("t_ros") for r in rows if r.get("t_ros") is not None]
    if stamps and any(r.get("teleop") is not None for r in rows):
        teleop_s = 0.0
        prev = None
        for row in rows:
            t = row.get("t_ros")
            if t is None:
                continue
            if prev is not None and _is_teleop(row) and _is_teleop(prev[1]):
                teleop_s += t - prev[0]
            prev = (t, row)

    out = {
        "interventions": interventions,
        "teleop_frames": len(teleop_rows),
        "teleop_seconds": teleop_s,
        "autonomous_distance_m": autonomous_m,
        "distance_per_intervention_m": None,
    }
    if autonomous_m is not None and interventions > 0:
        out["distance_per_intervention_m"] = autonomous_m / interventions
    return out


def format_summary(summary):
    """Compact multi-line report. Used verbatim in the node's shutdown log."""
    lines = []
    t = summary["tracking"]
    if t is None:
        lines.append("no valid frames recorded")
    else:
        lines.append(
            "tracking error (normalized image space, NOT meters): "
            "rms=%.3f mean|e|=%.3f p95=%.3f max=%.3f bias=%+.3f over %d frames"
            % (t["rms"], t["mean_abs"], t["p95_abs"], t["max_abs"],
               t["mean_signed"], t["n"])
        )
    for state in sorted(summary["by_state"]):
        s = summary["by_state"][state]
        lines.append(
            "  %-16s rms=%.3f p95=%.3f max=%.3f  (n=%d)"
            % (state, s["rms"], s["p95_abs"], s["max_abs"], s["n"])
        )
    if summary["invalid_fraction"] is not None:
        lines.append("invalid frames: %.1f%%" % (100.0 * summary["invalid_fraction"]))
    ang = summary["control"].get("angular_abs")
    delta = summary["control"].get("angular_delta_abs")
    if ang and delta:
        lines.append(
            "control: |angular_z| mean=%.3f p95=%.3f | step mean=%.3f p95=%.3f rad/s"
            % (ang["mean"], ang["p95"], delta["mean"], delta["p95"])
        )
    if summary["distance_m"] is not None:
        lines.append(
            "path: %.1f m over %s"
            % (
                summary["distance_m"],
                "%.0f s" % summary["duration_s"]
                if summary["duration_s"] is not None
                else "?",
            )
        )
    aut = summary.get("autonomy")
    if aut is not None:
        if aut["distance_per_intervention_m"] is not None:
            lines.append(
                "autonomy: %.1f m autonomous / %d intervention(s) = %.1f m per "
                "intervention"
                % (
                    aut["autonomous_distance_m"],
                    aut["interventions"],
                    aut["distance_per_intervention_m"],
                )
            )
        elif aut["interventions"] == 0 and aut["autonomous_distance_m"] is not None:
            lines.append(
                "autonomy: 0 interventions over %.1f m (>= %.1f m per "
                "intervention)"
                % (aut["autonomous_distance_m"], aut["autonomous_distance_m"])
            )
        else:
            lines.append(
                "autonomy: %d intervention(s), no odometry (distance unknown)"
                % aut["interventions"]
            )
    if summary["event_counts"]:
        lines.append(
            "events: "
            + ", ".join(
                "%s x%d" % (k, v) for k, v in sorted(summary["event_counts"].items())
            )
        )
    else:
        lines.append("events: none")
    return "\n".join(lines)
