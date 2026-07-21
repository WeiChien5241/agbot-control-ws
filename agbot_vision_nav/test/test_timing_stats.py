import pytest

from agbot_vision_nav.timing_stats import PipelineTimingStats


def feed_steady(stats, camera_hz=30.0, processed_hz=2.0, inference_s=0.45,
                duration_s=10.0):
    """Simulate a steady pipeline: fast camera, slow inference."""
    n_cam = int(duration_s * camera_hz)
    for i in range(n_cam):
        stats.record_camera_frame(i / camera_hz)
    n_proc = int(duration_s * processed_hz)
    for i in range(n_proc):
        t_pub = i / processed_hz + inference_s
        stats.record_inference(
            inference_s,
            wait_age_s=0.02,
            e2e_latency_s=inference_s + 0.05,
            publish_time=t_pub,
        )


def test_empty_snapshot_is_all_none():
    s = PipelineTimingStats().snapshot()
    assert s["camera_hz"] is None
    assert s["processed_hz"] is None
    assert s["inference_s"] is None
    assert s["wait_age_s"] is None
    assert s["e2e_latency_s"] is None
    assert s["dropped_fraction"] is None


def test_steady_pipeline_rates_and_latency():
    stats = PipelineTimingStats(window=100)
    feed_steady(stats)
    s = stats.snapshot()
    assert s["camera_hz"] == pytest.approx(30.0, rel=0.05)
    assert s["processed_hz"] == pytest.approx(2.0, rel=0.05)
    mean, p50, p95 = s["inference_s"]
    assert mean == pytest.approx(0.45)
    assert p50 == pytest.approx(0.45)
    assert p95 == pytest.approx(0.45)
    assert s["e2e_latency_s"][0] == pytest.approx(0.50)
    # 300 received, 20 processed -> ~93% skipped, which is by design
    assert s["dropped_fraction"] == pytest.approx(1.0 - 20.0 / 300.0)


def test_percentiles_spread():
    stats = PipelineTimingStats(window=100)
    for i in range(100):
        stats.record_inference(0.1 + 0.001 * i)  # 0.100 .. 0.199 s
    mean, p50, p95 = stats.snapshot()["inference_s"]
    assert mean == pytest.approx(0.1495, abs=1e-4)
    assert p50 == pytest.approx(0.150, abs=0.002)
    assert p95 == pytest.approx(0.194, abs=0.002)


def test_window_limits_rate_history_not_totals():
    stats = PipelineTimingStats(window=5)
    for i in range(50):
        stats.record_camera_frame(i * 0.1)
    s = stats.snapshot()
    assert s["frames_received"] == 50           # totals keep counting
    assert s["camera_hz"] == pytest.approx(10.0)  # rate from last 5 only


def test_single_event_gives_no_rate():
    stats = PipelineTimingStats()
    stats.record_camera_frame(1.0)
    stats.record_inference(0.4, publish_time=1.4)
    s = stats.snapshot()
    assert s["camera_hz"] is None
    assert s["processed_hz"] is None
    assert s["inference_s"] is not None


def test_format_summary_and_hud_line():
    stats = PipelineTimingStats(window=100)
    assert stats.hud_line() is None
    feed_steady(stats)
    summary = stats.format_summary()
    assert "cam 30.0 Hz" in summary
    assert "proc 2.0 Hz" in summary
    assert "inf 450 ms" in summary
    assert "e2e 500 ms" in summary
    assert "dropped 93% (by design)" in summary
    hud = stats.hud_line()
    assert "inf=450ms" in hud
    assert "e2e=500ms" in hud
    assert "proc=2.0Hz" in hud


def test_optional_fields_can_be_omitted():
    stats = PipelineTimingStats()
    stats.record_inference(0.3)
    s = stats.snapshot()
    assert s["inference_s"][0] == pytest.approx(0.3)
    assert s["wait_age_s"] is None
    assert s["e2e_latency_s"] is None
    # summary must not crash with partial data
    assert "inf 300 ms" in stats.format_summary()
    assert stats.hud_line() == "inf=300ms"
