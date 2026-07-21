import numpy as np

from agbot_vision_nav.centerline_estimator import estimate_centerline
from agbot_vision_nav.debug_viz import render_debug_image

HEIGHT = 100
WIDTH = 200


def _make_frame_and_mask():
    frame_bgr = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    mask = np.full((HEIGHT, WIDTH), 2, dtype=np.uint8)  # obstacle
    mask[:20, :] = 0  # sky
    mask[20:, 70:130] = 1  # traversable corridor
    return frame_bgr, mask


def test_render_debug_image_valid_frame_has_correct_shape_and_dtype():
    frame_bgr, mask = _make_frame_and_mask()
    result = estimate_centerline(mask)
    assert result.valid

    debug_img = render_debug_image(frame_bgr, mask, result, linear_x=0.15, angular_z=0.1)
    assert debug_img.shape == frame_bgr.shape
    assert debug_img.dtype == frame_bgr.dtype


def test_render_debug_image_with_timing_line_does_not_crash():
    frame_bgr, mask = _make_frame_and_mask()
    result = estimate_centerline(mask)

    debug_img = render_debug_image(
        frame_bgr, mask, result, linear_x=0.15, angular_z=0.1,
        state_name="FOLLOW_ROW", timing_line="inf=450ms e2e=500ms proc=2.0Hz",
        detector_line="exit: blk 5/8 open 0/5 rows=0 frac=0.12 armed o:Y b:Y",
    )
    assert debug_img.shape == frame_bgr.shape


def test_render_debug_image_invalid_frame_does_not_crash():
    frame_bgr = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)  # all sky -> invalid
    result = estimate_centerline(mask)
    assert not result.valid

    debug_img = render_debug_image(frame_bgr, mask, result)
    assert debug_img.shape == frame_bgr.shape
