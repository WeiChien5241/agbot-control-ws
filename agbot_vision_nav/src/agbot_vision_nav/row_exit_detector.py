"""Detect the end-of-row signature in centerline estimation results.

Pure Python/numpy over CenterlineResult -- no rospy, no extra image
processing. Reuses the per-scan-row boundaries that centerline_estimator
already computes.

Two exit signatures (both observed in the segmentation masks):

1. ROW_END_OPEN: leaving the row, the traversable corridor widens from a
   narrow band (~0.2-0.4 of image width in-row) toward the full image width.
   We require at least `exit_open_rows_required` scan rows -- ANY of them --
   to be wide. Counting any rows rather than specific ones keeps the
   signature robust to unreliable segmentation of distant ground: beyond
   the field the mask is often garbage, so the far rows can stay invalid
   forever (requiring the far rows was field-tested and never fired -- the
   robot drove off the world edge). Where the far rows do segment well they
   go wide first and fire early on approach; where they don't, the near row
   going wide still fires.

2. ROW_END_BLOCKED: a wall of crop dead ahead (e.g. exiting toward another
   planted block): every scan row is invalid (image-center column is not
   traversable at any scan height) while the lower half of the frame still
   has plenty of traversable ground to the sides/near the robot.

Guards against false positives:
- Debounce, per signature: open must persist `exit_detect_frames`
  consecutive frames; blocked must accumulate `blocked_detect_frames`
  (longer by default -- a low-mounted camera sees foliage brush the image
  center for a few frames at row ends, which must not trigger a back-out).
  The blocked counter is LEAKY (non-signature frames decrement rather than
  reset it) so isolated noisy frames cannot postpone detection forever.
- Arming distance, per signature: the OPEN signature is disabled until the
  robot has travelled `min_in_row_distance` meters (odometry) inside the
  current row, so the open-field view at row entry cannot trigger an exit.
  The BLOCKED signature arms much earlier (`blocked_arming_distance`) so an
  obstacle shortly after row entry is still caught -- it cannot false-fire
  at entry because it requires zero visible corridor at every scan row.
"""

from collections import namedtuple

EXIT_NONE = "none"
EXIT_ROW_END_OPEN = "row_end_open"
EXIT_ROW_END_BLOCKED = "row_end_blocked"

# Per-update diagnostic snapshot (see RowExitDetector.last_status): shows on
# the debug HUD so "why didn't the exit fire" is answerable from rqt alone.
ExitDetectorStatus = namedtuple(
    "ExitDetectorStatus",
    [
        "distance_in_row",       # meters, or None (unarmed: no odometry)
        "open_armed",
        "blocked_armed",
        "corridor_rows",         # scan rows that found a corridor
        "wide_rows",             # scan rows at/above exit_width_threshold
        "traversable_fraction",
        "open_count",            # debounce counters AFTER this frame
        "blocked_count",
        "open_rows",             # scan rows that are wide AND flank-clear
    ],
)


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


def flank_clear_flags(centerline_result, image_width, edge_margin_frac):
    """Per-scan-row: True if the traversable corridor reaches within
    `edge_margin_frac` of BOTH image borders, i.e. no non-traversable region
    (corn/sky) flanks the corridor on either side.

    This is the "open exit" discriminator: still inside the row, corn borders
    the corridor and it stops short of the image edge; at a true row end the
    corridor runs edge-to-edge. Returns a list aligned with
    centerline_result.scan_rows; None for rows with no corridor.
    """
    margin_px = int(round(edge_margin_frac * (image_width - 1)))
    flags = []
    for sr in centerline_result.scan_rows:
        if sr.x_left is None:
            flags.append(None)
        else:
            flags.append(
                sr.x_left <= margin_px
                and sr.x_right >= (image_width - 1) - margin_px
            )
    return flags


class RowExitDetector:
    """Debounced end-of-row detection from centerline results."""

    def __init__(
        self,
        exit_width_threshold=0.8,
        exit_detect_frames=5,
        min_in_row_distance=2.0,
        blocked_min_traversable_fraction=0.02,
        blocked_arming_distance=0.3,
        exit_open_rows_required=1,
        blocked_detect_frames=8,
        exit_flank_edge_margin=0.05,
    ):
        self.exit_width_threshold = exit_width_threshold
        self.exit_detect_frames = exit_detect_frames
        self.min_in_row_distance = min_in_row_distance
        self.blocked_min_traversable_fraction = blocked_min_traversable_fraction
        self.blocked_arming_distance = blocked_arming_distance
        self.exit_open_rows_required = exit_open_rows_required
        self.blocked_detect_frames = blocked_detect_frames
        # OPEN also requires the corridor to reach within this fraction of the
        # image width of BOTH borders (no corn flanking the corridor); a
        # mid-row side-gap widens the corridor but leaves corn short of the
        # edge, so it must NOT read as an exit. Larger => more permissive;
        # >= 1.0 disables the flank check (width-only, pre-fix behavior).
        self.exit_flank_edge_margin = exit_flank_edge_margin
        self._consecutive_open = 0
        self._consecutive_blocked = 0
        self.last_status = None

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
            self.last_status = ExitDetectorStatus(
                None, False, False, 0, 0,
                centerline_result.traversable_fraction, 0, 0, 0,
            )
            return EXIT_NONE

        # Per-signature arming; while a signature is unarmed its debounce
        # counter is held at zero so a streak cannot straddle the boundary.
        open_armed = distance_in_row >= self.min_in_row_distance
        blocked_armed = distance_in_row >= self.blocked_arming_distance
        if not open_armed and not blocked_armed:
            self.reset()
            self.last_status = ExitDetectorStatus(
                distance_in_row, False, False, 0, 0,
                centerline_result.traversable_fraction, 0, 0, 0,
            )
            return EXIT_NONE

        widths = normalized_corridor_widths(centerline_result, image_width)
        flank_flags = flank_clear_flags(
            centerline_result, image_width, self.exit_flank_edge_margin
        )
        valid_widths = [w for w in widths if w is not None]

        # OPEN: at least exit_open_rows_required scan rows (ANY of them) see a
        # corridor that is BOTH wide (>= exit_width_threshold) AND flank-clear
        # (reaches within exit_flank_edge_margin of both image borders -- no
        # corn beside the corridor). No specific row is required, so unreliable
        # segmentation of distant ground (far rows invalid beyond the field)
        # cannot mask the exit: whichever row sees open field first starts the
        # debounce. The flank-clear term rejects a mid-row side-gap that merely
        # widens the corridor while corn still borders it.
        wide_rows = sum(1 for w in valid_widths if w >= self.exit_width_threshold)
        open_rows = sum(
            1
            for w, fc in zip(widths, flank_flags)
            if w is not None and w >= self.exit_width_threshold and fc
        )
        open_signature = open_rows >= max(1, self.exit_open_rows_required)
        blocked_signature = (
            len(valid_widths) == 0
            and centerline_result.traversable_fraction
            >= self.blocked_min_traversable_fraction
        )

        self._consecutive_open = (
            self._consecutive_open + 1 if (open_signature and open_armed) else 0
        )
        # Leaky debounce for BLOCKED: a non-signature frame decrements
        # instead of resetting, so one flickery frame (segmentation noise on
        # a static close-up view) delays detection by a frame rather than
        # restarting the whole streak -- a strict N-in-a-row requirement was
        # observed to never complete in front of a sim blocker. Sustained
        # noise still drains the counter to zero. OPEN stays strictly
        # consecutive (field-proven).
        if blocked_signature and blocked_armed:
            self._consecutive_blocked += 1
        else:
            self._consecutive_blocked = max(0, self._consecutive_blocked - 1)

        self.last_status = ExitDetectorStatus(
            distance_in_row,
            open_armed,
            blocked_armed,
            len(valid_widths),
            wide_rows,
            centerline_result.traversable_fraction,
            self._consecutive_open,
            self._consecutive_blocked,
            open_rows,
        )

        if self._consecutive_open >= self.exit_detect_frames:
            return EXIT_ROW_END_OPEN
        if self._consecutive_blocked >= self.blocked_detect_frames:
            return EXIT_ROW_END_BLOCKED
        return EXIT_NONE
