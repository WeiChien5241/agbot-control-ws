import csv
import math
import os

import pytest

from agbot_vision_nav.metrics_logger import (
    COLUMNS,
    RunMetricsLogger,
    error_stats,
    format_summary,
    join_list,
    make_run_path,
    read_csv,
    summarize,
)


def read_raw(path):
    with open(path, "r", newline="") as handle:
        return list(csv.reader(handle))


# -- the writer -----------------------------------------------------------


def test_header_matches_the_schema(tmp_path):
    path = str(tmp_path / "run.csv")
    logger = RunMetricsLogger(path)
    logger.close()
    assert read_raw(path)[0] == list(COLUMNS)


def test_a_sparse_log_call_still_produces_a_well_formed_row(tmp_path):
    """Callers must be able to omit columns.

    The rear-camera path has no front detector status, and adding a column
    later must not break every existing call site.
    """
    path = str(tmp_path / "run.csv")
    logger = RunMetricsLogger(path)
    logger.log(offset_norm=0.25, state="FOLLOW_ROW")
    logger.close()

    rows = read_raw(path)
    assert len(rows) == 2
    assert len(rows[1]) == len(COLUMNS)
    parsed = read_csv(path)[0]
    assert parsed["offset_norm"] == pytest.approx(0.25)
    assert parsed["state"] == "FOLLOW_ROW"
    assert parsed["odom_x"] is None


def test_event_attaches_to_the_next_row_only(tmp_path):
    path = str(tmp_path / "run.csv")
    logger = RunMetricsLogger(path)
    logger.log(offset_norm=0.0)
    logger.mark_event("BLOCKED")
    logger.log(offset_norm=0.1)
    logger.log(offset_norm=0.2)
    logger.close()

    rows = read_csv(path)
    assert [r["event"] for r in rows] == [None, "BLOCKED", None]


def test_two_events_before_the_next_frame_are_both_kept(tmp_path):
    """Losing a BLOCKED because a WATCHDOG_ZERO landed in the same gap would
    drop exactly the event worth having."""
    path = str(tmp_path / "run.csv")
    logger = RunMetricsLogger(path)
    logger.mark_event("BLOCKED")
    logger.mark_event("WATCHDOG_ZERO")
    logger.log(offset_norm=0.0)
    logger.close()

    assert read_csv(path)[0]["event"] == "BLOCKED|WATCHDOG_ZERO"


def test_booleans_and_nan_are_written_readably(tmp_path):
    path = str(tmp_path / "run.csv")
    logger = RunMetricsLogger(path)
    logger.log(valid=True, blocked_armed=False, offset_norm=float("nan"))
    logger.close()

    row = read_csv(path)[0]
    assert row["valid"] == 1.0
    assert row["blocked_armed"] == 0.0
    assert row["offset_norm"] is None


def test_event_rows_are_flushed_immediately(tmp_path):
    """An event row must survive a run that ends by losing power, which is how
    field runs actually end."""
    path = str(tmp_path / "run.csv")
    logger = RunMetricsLogger(path)
    logger.mark_event("BLOCKED")
    logger.log(offset_norm=0.0)
    # No close(): read what is on disk right now.
    assert any(r["event"] == "BLOCKED" for r in read_csv(path))
    logger.close()


def test_an_unopenable_path_does_not_raise(tmp_path):
    """The instrument must never take down the node it is measuring."""
    blocker = tmp_path / "not-a-dir"
    blocker.write_text("")
    logger = RunMetricsLogger(str(blocker / "sub" / "run.csv"))
    logger.log(offset_norm=0.5)      # must be a no-op, not an exception
    logger.mark_event("BLOCKED")
    logger.close()
    assert logger.errors
    assert logger.rows_written == 0


def test_a_write_failure_stops_quietly_and_does_not_spam_errors(tmp_path):
    path = str(tmp_path / "run.csv")
    logger = RunMetricsLogger(path)
    logger.log(offset_norm=0.1)
    logger._handle.close()           # simulate the fd going away mid-run
    for _ in range(10):
        logger.log(offset_norm=0.2)
    logger.close()
    assert len(logger.errors) == 1
    assert logger.rows_written == 1


def test_make_run_path_is_timestamped_under_the_directory():
    path = make_run_path("~/agbot_logs", now=0.0)
    assert path.startswith(os.path.expanduser("~/agbot_logs"))
    assert os.path.basename(path).startswith("vision_nav_")
    assert path.endswith(".csv")
    assert make_run_path("/tmp/x", now=0.0) != make_run_path("/tmp/x", now=7200.0)


def test_join_list_distinguishes_a_missing_corridor_from_zero():
    assert join_list([0.0, None, 0.5]) == "0.0000;;0.5000"
    assert join_list([]) == ""


# -- statistics -----------------------------------------------------------


def test_error_stats_on_a_known_series():
    stats = error_stats([0.3, -0.4, 0.0])
    assert stats["n"] == 3
    assert stats["rms"] == pytest.approx(math.sqrt((0.09 + 0.16 + 0.0) / 3.0))
    assert stats["mean_abs"] == pytest.approx(0.7 / 3.0)
    assert stats["max_abs"] == pytest.approx(0.4)
    assert stats["mean_signed"] == pytest.approx(-0.1 / 3.0)


def test_error_stats_signed_mean_exposes_a_standing_bias():
    """A robot riding consistently right of centre has a small RMS and a
    non-zero signed mean; only the second says which way."""
    assert error_stats([0.2, 0.2, 0.2])["mean_signed"] == pytest.approx(0.2)
    assert error_stats([0.2, -0.2, 0.2, -0.2])["mean_signed"] == pytest.approx(0.0)


def test_error_stats_of_nothing_is_none():
    assert error_stats([]) is None
    assert error_stats([None, None]) is None


def rows(*specs):
    out = []
    for spec in specs:
        row = {c: None for c in COLUMNS}
        row.update(spec)
        out.append(row)
    return out


def test_summary_excludes_invalid_frames_from_the_tracking_error():
    """An invalid frame has no centerline; its offset is a placeholder, not a
    measurement, and averaging it in would understate the error."""
    s = summarize(
        rows(
            {"valid": 1.0, "offset_norm": 0.2, "state": "FOLLOW_ROW"},
            {"valid": 0.0, "offset_norm": 0.0, "state": "FOLLOW_ROW"},
        )
    )
    assert s["tracking"]["n"] == 1
    assert s["tracking"]["rms"] == pytest.approx(0.2)
    assert s["invalid_fraction"] == pytest.approx(0.5)


def test_summary_groups_by_state():
    """FOLLOW_ROW is the controller's score. TURN_1 is odometry open loop --
    its offsets measure the headland, not the controller, so they must not be
    pooled into one number."""
    s = summarize(
        rows(
            {"valid": 1.0, "offset_norm": 0.1, "state": "FOLLOW_ROW"},
            {"valid": 1.0, "offset_norm": 0.1, "state": "FOLLOW_ROW"},
            {"valid": 1.0, "offset_norm": 0.9, "state": "TURN_1"},
        )
    )
    assert set(s["by_state"]) == {"FOLLOW_ROW", "TURN_1"}
    assert s["by_state"]["FOLLOW_ROW"]["rms"] == pytest.approx(0.1)
    assert s["by_state"]["FOLLOW_ROW"]["n"] == 2
    assert s["by_state"]["TURN_1"]["rms"] == pytest.approx(0.9)


def test_angular_delta_is_measured_across_rows():
    s = summarize(
        rows(
            {"angular_z": 0.0},
            {"angular_z": 0.1},
            {"angular_z": -0.1},
        )
    )
    delta = s["control"]["angular_delta_abs"]
    assert delta["n"] == 2
    assert delta["max"] == pytest.approx(0.2)


def test_summary_counts_events_and_keeps_their_rows():
    s = summarize(
        rows(
            {"event": "BLOCKED", "distance_in_row": 4.2},
            {"event": None},
            {"event": "EXIT_REVOKED|WATCHDOG_ZERO"},
        )
    )
    assert s["event_counts"] == {
        "BLOCKED": 1,
        "EXIT_REVOKED": 1,
        "WATCHDOG_ZERO": 1,
    }
    assert len(s["events"]) == 2
    assert s["events"][0]["distance_in_row"] == pytest.approx(4.2)


def test_summary_measures_path_length_from_odometry():
    s = summarize(
        rows(
            {"odom_x": 0.0, "odom_y": 0.0, "t_ros": 10.0},
            {"odom_x": 3.0, "odom_y": 4.0, "t_ros": 20.0},
        )
    )
    assert s["distance_m"] == pytest.approx(5.0)
    assert s["duration_s"] == pytest.approx(10.0)


def test_summary_without_odometry_reports_no_distance():
    assert summarize(rows({"offset_norm": 0.1}))["distance_m"] is None


def test_summary_of_an_empty_run_is_safe():
    s = summarize([])
    assert s["rows"] == 0
    assert s["tracking"] is None
    assert "no valid frames" in format_summary(s)


def test_format_summary_labels_the_units():
    """The one thing this report must not let a reader assume is meters."""
    s = summarize(rows({"valid": 1.0, "offset_norm": 0.2, "state": "FOLLOW_ROW"}))
    text = format_summary(s)
    assert "NOT meters" in text
    assert "rms=0.200" in text


def test_round_trip_through_a_real_file(tmp_path):
    path = str(tmp_path / "run.csv")
    logger = RunMetricsLogger(path)
    for i in range(5):
        logger.log(
            t_ros=100.0 + i,
            valid=True,
            offset_norm=0.1 * i,
            angular_z=0.05 * i,
            state="FOLLOW_ROW",
            odom_x=float(i),
            odom_y=0.0,
            scan_offsets=join_list([0.1, None, 0.3]),
        )
    logger.close()

    s = summarize(read_csv(path))
    assert s["rows"] == 5
    assert s["tracking"]["n"] == 5
    assert s["distance_m"] == pytest.approx(4.0)
    assert s["duration_s"] == pytest.approx(4.0)
