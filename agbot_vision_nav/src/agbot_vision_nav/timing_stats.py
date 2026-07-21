"""Rolling timing statistics for the vision pipeline.

Pure Python, no rospy -- the ROS node feeds it timestamps/durations and
periodically logs snapshot()/format_summary(). Answers the questions the
raw pipeline cannot: what rate are camera frames arriving, what rate are we
actually processing, how long does one inference take, and how old is the
image behind each published command (end-to-end latency).

Terminology (all seconds; the caller picks ONE clock per quantity and stays
consistent -- the node uses rospy time for stamps/rates so sim time works,
and a monotonic clock for the inference duration):

  camera Hz     -- arrival rate of frames into the latest-frame slot.
  processed Hz  -- rate of completed inferences (= control rate).
  inference     -- model.predict() wall time.
  wait age      -- how long the picked frame sat in the slot before the
                   inference thread got to it (0 on a fast machine; up to a
                   full inference period on a slow one).
  e2e latency   -- camera header stamp -> cmd_vel publish. The number to
                   quote when asked "how delayed is the image the robot is
                   acting on".
  dropped       -- fraction of received frames never processed. With
                   latest-frame-wins semantics this is EXPECTED to be high
                   on a slow machine (e.g. 93% at 30 Hz camera / 2 Hz
                   inference); it measures skipping, not malfunction.
"""

from collections import deque


def _percentile(sorted_vals, q):
    """Nearest-rank percentile of an already-sorted non-empty list."""
    idx = int(round(q / 100.0 * (len(sorted_vals) - 1)))
    return sorted_vals[idx]


def _rate_hz(times):
    """Average rate from a sequence of event times; None if under 2 events."""
    if len(times) < 2:
        return None
    span = times[-1] - times[0]
    if span <= 0.0:
        return None
    return (len(times) - 1) / span


class PipelineTimingStats:
    def __init__(self, window=50):
        self._camera_times = deque(maxlen=window)
        self._publish_times = deque(maxlen=window)
        self._inference_s = deque(maxlen=window)
        self._wait_age_s = deque(maxlen=window)
        self._e2e_latency_s = deque(maxlen=window)
        self.frames_received = 0
        self.frames_processed = 0

    def record_camera_frame(self, recv_time):
        """One frame stored into a latest-frame slot (front or rear)."""
        self.frames_received += 1
        self._camera_times.append(recv_time)

    def record_inference(
        self, inference_s, wait_age_s=None, e2e_latency_s=None, publish_time=None
    ):
        """One completed inference + command publish."""
        self.frames_processed += 1
        self._inference_s.append(inference_s)
        if wait_age_s is not None:
            self._wait_age_s.append(wait_age_s)
        if e2e_latency_s is not None:
            self._e2e_latency_s.append(e2e_latency_s)
        if publish_time is not None:
            self._publish_times.append(publish_time)

    @staticmethod
    def _dist(samples):
        """(mean, p50, p95) of a sample window, or None if empty."""
        if not samples:
            return None
        vals = sorted(samples)
        return (
            sum(vals) / len(vals),
            _percentile(vals, 50),
            _percentile(vals, 95),
        )

    def snapshot(self):
        """Dict of current stats; distribution entries are (mean, p50, p95)
        tuples in seconds, rates in Hz, or None where no data yet."""
        dropped_fraction = None
        if self.frames_received > 0:
            dropped_fraction = 1.0 - float(self.frames_processed) / self.frames_received
        return {
            "camera_hz": _rate_hz(self._camera_times),
            "processed_hz": _rate_hz(self._publish_times),
            "inference_s": self._dist(self._inference_s),
            "wait_age_s": self._dist(self._wait_age_s),
            "e2e_latency_s": self._dist(self._e2e_latency_s),
            "dropped_fraction": dropped_fraction,
            "frames_received": self.frames_received,
            "frames_processed": self.frames_processed,
        }

    def format_summary(self):
        """One log-friendly line, e.g.
        'cam 29.8 Hz | proc 2.1 Hz | inf 460 ms (p95 510) | e2e 540 ms | dropped 93% (by design)'
        """
        s = self.snapshot()

        def hz(v):
            return "%.1f Hz" % v if v is not None else "n/a"

        def ms(dist):
            if dist is None:
                return "n/a"
            mean, _, p95 = dist
            return "%.0f ms (p95 %.0f)" % (mean * 1000.0, p95 * 1000.0)

        dropped = (
            "%.0f%% (by design)" % (100.0 * s["dropped_fraction"])
            if s["dropped_fraction"] is not None
            else "n/a"
        )
        return "cam %s | proc %s | inf %s | e2e %s | dropped %s" % (
            hz(s["camera_hz"]),
            hz(s["processed_hz"]),
            ms(s["inference_s"]),
            ms(s["e2e_latency_s"]),
            dropped,
        )

    def hud_line(self):
        """Compact overlay line for the debug image, or None if no data."""
        s = self.snapshot()
        if s["inference_s"] is None:
            return None
        parts = ["inf=%.0fms" % (s["inference_s"][0] * 1000.0)]
        if s["e2e_latency_s"] is not None:
            parts.append("e2e=%.0fms" % (s["e2e_latency_s"][0] * 1000.0))
        if s["processed_hz"] is not None:
            parts.append("proc=%.1fHz" % s["processed_hz"])
        return " ".join(parts)
