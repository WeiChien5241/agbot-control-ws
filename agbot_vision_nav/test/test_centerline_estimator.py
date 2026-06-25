import numpy as np
import pytest

from agbot_vision_nav.centerline_estimator import (
    CLASS_OBSTACLE,
    CLASS_SKY,
    CLASS_TRAVERSABLE,
    estimate_centerline,
)

HEIGHT = 100
WIDTH = 200


def make_corridor_mask(center_x_fn, half_corridor_width=30, sky_rows=20):
    """Build a mask with a vertical traversable corridor.

    center_x_fn(row_y) -> corridor center column for that row, so the
    corridor can be straight or tapered across rows.
    """
    mask = np.full((HEIGHT, WIDTH), CLASS_OBSTACLE, dtype=np.uint8)
    mask[:sky_rows, :] = CLASS_SKY
    for y in range(sky_rows, HEIGHT):
        cx = center_x_fn(y)
        x_left = max(0, int(cx - half_corridor_width))
        x_right = min(WIDTH - 1, int(cx + half_corridor_width))
        mask[y, x_left : x_right + 1] = CLASS_TRAVERSABLE
    return mask


def test_centered_corridor_gives_zero_offset():
    mask = make_corridor_mask(lambda y: WIDTH / 2.0)
    result = estimate_centerline(mask)
    assert result.valid
    assert result.offset_norm == pytest.approx(0.0, abs=1e-6)
    assert result.slope_term == pytest.approx(0.0, abs=1e-6)


def test_corridor_shifted_right_gives_positive_offset():
    # shift must stay within half_corridor_width so the image centerline
    # column (cx) is still inside the corridor -- estimate_centerline
    # requires cx itself to be traversable to find a corridor at all,
    # matching a robot that has drifted but not lost the row entirely.
    shift = 15
    mask = make_corridor_mask(lambda y: WIDTH / 2.0 + shift)
    result = estimate_centerline(mask)
    assert result.valid
    expected = shift / (WIDTH / 2.0)
    assert result.offset_norm == pytest.approx(expected, abs=0.02)
    assert result.offset_norm > 0


def test_corridor_shifted_left_gives_negative_offset():
    shift = 15
    mask = make_corridor_mask(lambda y: WIDTH / 2.0 - shift)
    result = estimate_centerline(mask)
    assert result.valid
    expected = -shift / (WIDTH / 2.0)
    assert result.offset_norm == pytest.approx(expected, abs=0.02)
    assert result.offset_norm < 0


def test_tapered_corridor_produces_nonzero_slope_term():
    # Corridor drifts rightward as row_y decreases (i.e. farther from robot),
    # so the farthest scan row should read a larger positive offset than the
    # nearest scan row.
    def center_x(y):
        # y=20 (top, far) -> +60 offset; y=99 (bottom, near) -> +0 offset
        far_to_near = (y - 20) / (HEIGHT - 1 - 20)
        return WIDTH / 2.0 + 60 * (1.0 - far_to_near)

    mask = make_corridor_mask(center_x)
    result = estimate_centerline(mask)
    assert result.valid
    # far row offset > near row offset given the taper direction above
    assert result.slope_term > 0


def test_drift_beyond_corridor_width_is_invalid():
    # Known limitation, by design: if the robot drifts so far that the image
    # centerline column (cx) falls entirely outside the corridor, there is no
    # boundary to scan outward from, so the row contributes no signal. With
    # all 3 default scan rows behaving the same way, the frame is invalid
    # rather than silently reporting offset_norm=0 (which would tell the
    # controller "centered" when the opposite is true).
    shift = 40  # > half_corridor_width=30, so cx is outside the corridor
    mask = make_corridor_mask(lambda y: WIDTH / 2.0 + shift)
    result = estimate_centerline(mask)
    assert not result.valid


def test_all_sky_is_invalid():
    mask = np.full((HEIGHT, WIDTH), CLASS_SKY, dtype=np.uint8)
    result = estimate_centerline(mask)
    assert not result.valid
    assert result.traversable_fraction == 0.0


def test_all_obstacle_is_invalid():
    mask = np.full((HEIGHT, WIDTH), CLASS_OBSTACLE, dtype=np.uint8)
    result = estimate_centerline(mask)
    assert not result.valid


def test_mismatched_lengths_raise():
    mask = make_corridor_mask(lambda y: WIDTH / 2.0)
    with pytest.raises(ValueError):
        estimate_centerline(mask, scan_row_fractions=(0.5, 0.9), scan_row_weights=(1.0,))


def test_low_traversable_fraction_marks_invalid_even_if_center_pixel_ok():
    # A thin sliver of traversable straddling center, but far below the
    # min_traversable_fraction threshold for the lower half of the frame.
    mask = np.full((HEIGHT, WIDTH), CLASS_OBSTACLE, dtype=np.uint8)
    mask[:, WIDTH // 2 - 1 : WIDTH // 2 + 1] = CLASS_TRAVERSABLE
    result = estimate_centerline(mask, min_traversable_fraction=0.10)
    assert not result.valid
