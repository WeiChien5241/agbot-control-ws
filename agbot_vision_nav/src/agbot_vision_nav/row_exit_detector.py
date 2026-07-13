"""Detect the end-of-row signature in centerline estimation results.

Pure Python/numpy over CenterlineResult -- no rospy, no extra image
processing. Reuses the per-scan-row boundaries that centerline_estimator
already computes.

Two exit signatures (both observed in the segmentation masks):

1. ROW_END_OPEN: leaving the row, the traversable corridor widens from a
   narrow band (~0.2-0.4 of image width in-row) toward the full image width,
   far scan rows first. We require the `exit_open_rows_required` FARTHEST
   scan rows to be wide -- the far rows go wide well before the nearest one,
   so this fires while the robot is still approaching the last plants
   instead of after its nose is past them (waiting for the nearest row was
   measured to overshoot the row end by 1.5-2.5 m with a low-mounted
   camera).

2. ROW_END_BLOCKED: a wall of crop dead ahead (e.g. exiting toward another
   planted block): every scan row is invalid (image-center column is not
   traversable at any scan height) while the lower half of the frame still
   has plenty of traversable ground to the sides/near the robot.

Guards against false positives:
- Debounce, per signature: open must persist `exit_detect_frames`
  consecutive frames; blocked must persist `blocked_detect_frames` (longer
  by default -- a low-mounted camera sees foliage brush the image center
  for a few frames at row ends, which must not trigger a back-out).
- Arming distance, per signature: the OPEN signature is disabled until the
  robot has travelled `min_in_row_distance` meters (odometry) inside the
  current row, so the open-field view at row entry cannot trigger an exit.
  The BLOCKED signature arms much earlier (`blocked_arming_distance`) so an
  obstacle shortly after row entry is still caught -- it cannot false-fire
  at entry because it requires zero visible corridor at every scan row.
"""

EXIT_NONE = "none"
EXIT_ROW_END_OPEN = "row_end_open"
EXIT_ROW_END_BLOCKED = "row_end_blocked"


def normalized_corridor_widths(centerline_result, image_width):
    """Per-scan-row corridor width as a fraction of image width.

    Returns a list aligned with centerline_result.scan_rows; entries are
    floats in [0, 1] for valid rows and None for rows where no corridor was
    found (image-center column not traversable).
    """
    widths = []
    for sr in centerline_result.scan_rows:
        if sr.x_left is None:
            widths.append(None)
        else:
            widths.append((sr.x_right - sr.x_left + 1) / float(image_width))
    return widths


class RowExitDetector:
    """Debounced end-of-row detection from centerline results."""

    def __init__(
        self,
        exit_width_threshold=0.8,
        exit_detect_frames=5,
        min_in_row_distance=2.0,
        blocked_min_traversable_fraction=0.15,
        blocked_arming_distance=0.3,
        exit_open_rows_required=2,
        blocked_detect_frames=8,
    ):
        self.exit_width_threshold = exit_width_threshold
        self.exit_detect_frames = exit_detect_frames
        self.min_in_row_distance = min_in_row_distance
        self.blocked_min_traversable_fraction = blocked_min_traversable_fraction
        self.blocked_arming_distance = blocked_arming_distance
        self.exit_open_rows_required = exit_open_rows_required
        self.blocked_detect_frames = blocked_detect_frames
        self._consecutive_open = 0
        self._consecutive_blocked = 0

    def reset(self):
        """Clear debounce counters (call when entering a new row)."""
        self._consecutive_open = 0
        self._consecutive_blocked = 0

    def update(self, centerline_result, image_width, distance_in_row):
        """Feed one frame's result; returns EXIT_NONE / EXIT_ROW_END_*.

        Args:
            centerline_result: CenterlineResult from estimate_centerline().
            image_width: mask width in pixels (normalizes corridor widths).
            distance_in_row: odometry distance travelled since entering the
                current row (meters). None (no odometry yet) keeps the
                detector unarmed.
        """
        if distance_in_row is None:
            self.reset()
            return EXIT_NONE

        # Per-signature arming; while a signature is unarmed its debounce
        # counter is held at zero so a streak cannot straddle the boundary.
        open_armed = distance_in_row >= self.min_in_row_distance
        blocked_armed = distance_in_row >= self.blocked_arming_distance
        if not open_armed and not blocked_armed:
            self.reset()
            return EXIT_NONE

        widths = normalized_corridor_widths(centerline_result, image_width)
        valid_widths = [w for w in widths if w is not None]

        # OPEN: the N farthest scan rows (smallest row_y = highest in the
        # image = farthest ground) must each have a corridor spanning at
        # least exit_width_threshold of the image. The far rows widen first
        # on approach, so this fires before the robot reaches the last
        # plants; EXIT_CLEAR then covers the remaining distance.
        by_distance = sorted(
            zip(centerline_result.scan_rows, widths), key=lambda p: p[0].row_y
        )
        n_required = max(1, min(self.exit_open_rows_required, len(by_distance)))
        far_widths = [w for _, w in by_distance[:n_required]]
        open_signature = len(far_widths) == n_required and all(
            w is not None and w >= self.exit_width_threshold for w in far_widths
        )
        blocked_signature = (
            len(valid_widths) == 0
            and centerline_result.traversable_fraction
            >= self.blocked_min_traversable_fraction
        )

        self._consecutive_open = (
            self._consecutive_open + 1 if (open_signature and open_armed) else 0
        )
        self._consecutive_blocked = (
            self._consecutive_blocked + 1
            if (blocked_signature and blocked_armed)
            else 0
        )

        if self._consecutive_open >= self.exit_detect_frames:
            return EXIT_ROW_END_OPEN
        if self._consecutive_blocked >= self.blocked_detect_frames:
            return EXIT_ROW_END_BLOCKED
        return EXIT_NONE
