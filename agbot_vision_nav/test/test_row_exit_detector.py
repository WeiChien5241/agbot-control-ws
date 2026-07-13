import numpy as np

from agbot_vision_nav.centerline_estimator import (
    CLASS_OBSTACLE,
    CLASS_SKY,
    CLASS_TRAVERSABLE,
    estimate_centerline,
)
from agbot_vision_nav.row_exit_detector import (
    EXIT_NONE,
    EXIT_ROW_END_BLOCKED,
    EXIT_ROW_END_OPEN,
    RowExitDetector,
    normalized_corridor_widths,
)

HEIGHT = 100
WIDTH = 200


def make_corridor_mask(half_corridor_width=30, sky_rows=20):
    """Narrow traversable corridor centered in the image (in-row view)."""
    mask = np.full((HEIGHT, WIDTH), CLASS_OBSTACLE, dtype=np.uint8)
    mask[:sky_rows, :] = CLASS_SKY
    cx = WIDTH // 2
    mask[sky_rows:, cx - half_corridor_width : cx + half_corridor_width + 1] = (
        CLASS_TRAVERSABLE
    )
    return mask


def make_open_field_mask(sky_rows=20):
    """Everything below the horizon traversable (out-of-row view)."""
    mask = np.full((HEIGHT, WIDTH), CLASS_TRAVERSABLE, dtype=np.uint8)
    mask[:sky_rows, :] = CLASS_SKY
    return mask


def make_blocked_ahead_mask(sky_rows=20, wall_bottom=95):
    """Wall of crop ahead: image-center column blocked at every scan row,
    but the sides of the lower half are traversable ground."""
    mask = np.full((HEIGHT, WIDTH), CLASS_TRAVERSABLE, dtype=np.uint8)
    mask[:sky_rows, :] = CLASS_SKY
    cx = WIDTH // 2
    # obstacle block covering the center columns through all scan rows
    mask[sky_rows:wall_bottom + 1, cx - 40 : cx + 41] = CLASS_OBSTACLE
    # make sure it covers the nearest scan row (0.92 * 99 = 91) too
    assert wall_bottom >= int(round(0.92 * (HEIGHT - 1)))
    return mask


def feed(detector, mask, distance, n):
    """Feed the same frame n times, return the last signal."""
    result = estimate_centerline(mask)
    signal = EXIT_NONE
    for _ in range(n):
        signal = detector.update(result, WIDTH, distance)
    return signal


def test_normalized_corridor_widths():
    result = estimate_centerline(make_corridor_mask(half_corridor_width=30))
    widths = normalized_corridor_widths(result, WIDTH)
    assert len(widths) == len(result.scan_rows)
    for w in widths:
        assert w is not None
        assert w == (61 / WIDTH)  # 2*30+1 columns


def test_no_exit_in_row():
    det = RowExitDetector()
    assert feed(det, make_corridor_mask(), distance=5.0, n=20) == EXIT_NONE


def test_open_field_triggers_after_debounce():
    det = RowExitDetector(exit_detect_frames=5)
    mask = make_open_field_mask()
    assert feed(det, mask, distance=5.0, n=4) == EXIT_NONE
    assert feed(det, mask, distance=5.0, n=1) == EXIT_ROW_END_OPEN


def make_far_open_mask(near_rows_from=85, half_corridor_width=30, sky_rows=20):
    """Approaching the row end: far scan rows see open field, the nearest
    scan row still sees the last plants' narrow corridor."""
    mask = np.full((HEIGHT, WIDTH), CLASS_TRAVERSABLE, dtype=np.uint8)
    mask[:sky_rows, :] = CLASS_SKY
    cx = WIDTH // 2
    mask[near_rows_from:, : cx - half_corridor_width] = CLASS_OBSTACLE
    mask[near_rows_from:, cx + half_corridor_width + 1 :] = CLASS_OBSTACLE
    return mask


def test_blocked_ahead_triggers_after_debounce():
    det = RowExitDetector(blocked_detect_frames=5)
    mask = make_blocked_ahead_mask()
    result = estimate_centerline(mask)
    # sanity: signature preconditions
    assert all(w is None for w in normalized_corridor_widths(result, WIDTH))
    assert result.traversable_fraction >= 0.15
    assert feed(det, mask, distance=5.0, n=4) == EXIT_NONE
    assert feed(det, mask, distance=5.0, n=1) == EXIT_ROW_END_BLOCKED


def test_unarmed_before_min_distance():
    det = RowExitDetector(min_in_row_distance=2.0)
    mask = make_open_field_mask()
    # open-field view at row entry must NOT trigger
    assert feed(det, mask, distance=0.5, n=50) == EXIT_NONE
    # nor with unknown odometry
    assert feed(det, mask, distance=None, n=50) == EXIT_NONE


def test_debounce_does_not_straddle_arming_boundary():
    det = RowExitDetector(min_in_row_distance=2.0, exit_detect_frames=5)
    mask = make_open_field_mask()
    # 4 frames before arming distance are discarded...
    assert feed(det, mask, distance=1.9, n=4) == EXIT_NONE
    # ...so 4 more after arming still aren't enough; the 5th is
    assert feed(det, mask, distance=2.1, n=4) == EXIT_NONE
    assert feed(det, mask, distance=2.1, n=1) == EXIT_ROW_END_OPEN


def test_blocked_arms_earlier_than_open():
    det = RowExitDetector(
        min_in_row_distance=2.0, blocked_arming_distance=0.3, blocked_detect_frames=5
    )
    blocked = make_blocked_ahead_mask()
    # below the blocked arming distance: nothing
    assert feed(det, blocked, distance=0.2, n=20) == EXIT_NONE
    # between blocked arming and open arming: blocked fires
    assert feed(det, blocked, distance=1.0, n=5) == EXIT_ROW_END_BLOCKED
    # open stays gated by min_in_row_distance in the same window
    det.reset()
    assert feed(det, make_open_field_mask(), distance=1.0, n=20) == EXIT_NONE


def test_blocked_debounce_does_not_straddle_arming_boundary():
    det = RowExitDetector(blocked_arming_distance=0.3, blocked_detect_frames=5)
    blocked = make_blocked_ahead_mask()
    assert feed(det, blocked, distance=0.2, n=4) == EXIT_NONE
    assert feed(det, blocked, distance=0.4, n=4) == EXIT_NONE
    assert feed(det, blocked, distance=0.4, n=1) == EXIT_ROW_END_BLOCKED


def test_far_rows_wide_fires_open_before_near_row_clears():
    # The two farthest scan rows wide + nearest still a narrow corridor
    # must read as an open exit (this is the approach to the row end).
    det = RowExitDetector(exit_open_rows_required=2, exit_detect_frames=5)
    mask = make_far_open_mask()
    assert feed(det, mask, distance=5.0, n=4) == EXIT_NONE
    assert feed(det, mask, distance=5.0, n=1) == EXIT_ROW_END_OPEN
    # ...but requiring all 3 rows keeps the old late behavior
    det = RowExitDetector(exit_open_rows_required=3, exit_detect_frames=5)
    assert feed(det, mask, distance=5.0, n=20) == EXIT_NONE


def test_open_fires_when_only_near_row_wide():
    # Cliff/world-edge regression: beyond the field the mask is garbage, so
    # the far scan rows have NO corridor; only the nearest row (real ground
    # in front of the robot) reads wide. The exit must still fire.
    mask = np.full((HEIGHT, WIDTH), CLASS_OBSTACLE, dtype=np.uint8)
    mask[:20, :] = CLASS_SKY
    mask[86:, :] = CLASS_TRAVERSABLE  # only the nearest scan row (91) is ground
    result = estimate_centerline(mask)
    widths = normalized_corridor_widths(result, WIDTH)
    assert widths[0] is None and widths[1] is None  # far rows invalid
    assert widths[2] == 1.0  # near row full width

    det = RowExitDetector(exit_detect_frames=5)  # defaults: required=1
    assert feed(det, mask, distance=5.0, n=4) == EXIT_NONE
    assert feed(det, mask, distance=5.0, n=1) == EXIT_ROW_END_OPEN
    # and this frame must never count as blocked (near row is valid)
    det = RowExitDetector(blocked_detect_frames=1)
    det.reset()
    assert det.update(result, WIDTH, 5.0) != EXIT_ROW_END_BLOCKED


def test_blocked_uses_its_own_longer_debounce():
    det = RowExitDetector(exit_detect_frames=5, blocked_detect_frames=8)
    mask = make_blocked_ahead_mask()
    assert feed(det, mask, distance=5.0, n=7) == EXIT_NONE
    assert feed(det, mask, distance=5.0, n=1) == EXIT_ROW_END_BLOCKED


def test_far_row_corridor_prevents_blocked():
    # A frame whose far row still has a corridor is not a blocked frame,
    # even if the near rows lose theirs (leaves brushing the camera).
    det = RowExitDetector(blocked_detect_frames=1)
    mask = make_corridor_mask()
    cx = WIDTH // 2
    mask[85:, cx - 40 : cx + 41] = CLASS_OBSTACLE  # near rows blocked
    assert feed(det, mask, distance=5.0, n=10) == EXIT_NONE


def test_interrupted_streak_resets():
    det = RowExitDetector(exit_detect_frames=5)
    open_mask = make_open_field_mask()
    corridor = make_corridor_mask()
    assert feed(det, open_mask, distance=5.0, n=4) == EXIT_NONE
    assert feed(det, corridor, distance=5.0, n=1) == EXIT_NONE  # streak broken
    assert feed(det, open_mask, distance=5.0, n=4) == EXIT_NONE
    assert feed(det, open_mask, distance=5.0, n=1) == EXIT_ROW_END_OPEN
