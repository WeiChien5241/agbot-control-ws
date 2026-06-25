"""Estimate the lateral offset of a traversable row corridor from image center.

Pure numpy logic, no rospy dependency, so it can be unit-tested with synthetic
mask arrays independent of ROS or the segmentation model.

Adapted from the Agronav paper's per-scanline boundary-midpoint centerline
definition: for each scan row, the centerline point is the midpoint of the
left/right traversable-region boundaries on that row. This is the image-space
analog of the lab's LiDAR-based d_l/d_r centering approach.
"""

from collections import namedtuple

import numpy as np

CLASS_SKY = 0
CLASS_TRAVERSABLE = 1
CLASS_OBSTACLE = 2

ScanRowResult = namedtuple(
    "ScanRowResult", ["row_y", "x_left", "x_right", "x_mid", "offset_norm"]
)

CenterlineResult = namedtuple(
    "CenterlineResult",
    ["offset_norm", "slope_term", "valid", "traversable_fraction", "scan_rows"],
)


def _scan_row_boundaries(row, cx):
    """Scan outward from column cx through a 1D class-index row.

    Returns (x_left, x_right): the bounds of the contiguous traversable run
    straddling cx. Returns (None, None) if cx itself isn't traversable, since
    there is then no corridor to measure on this row aligned with robot heading.
    """
    width = row.shape[0]
    cx = int(np.clip(cx, 0, width - 1))

    if row[cx] != CLASS_TRAVERSABLE:
        return None, None

    x_left = cx
    while x_left - 1 >= 0 and row[x_left - 1] == CLASS_TRAVERSABLE:
        x_left -= 1

    x_right = cx
    while x_right + 1 < width and row[x_right + 1] == CLASS_TRAVERSABLE:
        x_right += 1

    return x_left, x_right


def estimate_centerline(
    mask,
    scan_row_fractions=(0.65, 0.78, 0.92),
    scan_row_weights=(0.2, 0.3, 0.5),
    min_traversable_fraction=0.10,
):
    """Estimate the row corridor's lateral offset from image center.

    Args:
        mask: (H, W) uint8 array of class indices (0=sky, 1=traversable,
            2=obstacle), already resized to match the camera frame resolution.
        scan_row_fractions: fractions of image height (0=top, 1=bottom) at
            which to measure the corridor's left/right boundaries. Ordered
            arbitrarily; the smallest fraction is treated as farthest from
            the robot, the largest as nearest, for slope_term.
        scan_row_weights: weight given to each scan row's offset when
            averaging into offset_norm (same length/order as
            scan_row_fractions; need not sum to 1).
        min_traversable_fraction: minimum fraction of traversable pixels
            required in the lower half of the frame for the result to be
            considered valid (guards against a lost/occluded row).

    Returns:
        CenterlineResult
    """
    if len(scan_row_fractions) != len(scan_row_weights):
        raise ValueError(
            "scan_row_fractions and scan_row_weights must be the same length"
        )

    height, width = mask.shape
    cx = width / 2.0
    half_width = width / 2.0

    weighted_sum = 0.0
    weight_total = 0.0
    scan_rows = []

    for frac, weight in zip(scan_row_fractions, scan_row_weights):
        row_y = int(np.clip(round(frac * (height - 1)), 0, height - 1))
        x_left, x_right = _scan_row_boundaries(mask[row_y, :], cx)

        if x_left is None:
            scan_rows.append(ScanRowResult(row_y, None, None, None, None))
            continue

        x_mid = 0.5 * (x_left + x_right)
        row_offset_norm = (x_mid - cx) / half_width
        scan_rows.append(
            ScanRowResult(row_y, x_left, x_right, x_mid, row_offset_norm)
        )
        weighted_sum += weight * row_offset_norm
        weight_total += weight

    lower_half = mask[height // 2 :, :]
    traversable_fraction = float(np.mean(lower_half == CLASS_TRAVERSABLE))

    valid = weight_total > 0 and traversable_fraction >= min_traversable_fraction

    if weight_total > 0:
        offset_norm = float(np.clip(weighted_sum / weight_total, -1.0, 1.0))
    else:
        offset_norm = 0.0

    # slope_term: offset at the farthest valid scan row minus the nearest valid
    # scan row -- a cheap, single-frame heading proxy computed across rows
    # rather than across time, so it costs nothing extra and needs no history.
    valid_rows = [
        (frac, sr)
        for frac, sr in zip(scan_row_fractions, scan_rows)
        if sr.offset_norm is not None
    ]
    if len(valid_rows) >= 2:
        far_frac, far_row = min(valid_rows, key=lambda item: item[0])
        near_frac, near_row = max(valid_rows, key=lambda item: item[0])
        slope_term = far_row.offset_norm - near_row.offset_norm
    else:
        slope_term = 0.0

    return CenterlineResult(
        offset_norm=offset_norm,
        slope_term=slope_term,
        valid=valid,
        traversable_fraction=traversable_fraction,
        scan_rows=scan_rows,
    )
