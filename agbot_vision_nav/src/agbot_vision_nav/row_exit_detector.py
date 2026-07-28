"""Detect the end-of-row signature in centerline estimation results.

Pure Python/numpy over CenterlineResult -- no rospy, no extra image
processing. Reuses the per-scan-row boundaries that centerline_estimator
already computes.

Two exit signatures (both observed in the segmentation masks):

1. ROW_END_OPEN: leaving the row, the traversable corridor widens from a
   narrow band (~0.2-0.4 of image width in-row) toward the full image width.
   We require at least `exit_open_rows_required` scan rows -- ANY of them --
   to be wide AND flank-clear. Counting any rows rather than specific ones
   keeps the signature robust to unreliable segmentation of distant ground:
   beyond the field the mask is often garbage, so the far rows can stay
   invalid forever (requiring the far rows was field-tested and never fired
   -- the robot drove off the world edge). Where the far rows do segment well
   they go wide first and fire early on approach; where they don't, the near
   row going wide still fires.

2. ROW_END_BLOCKED: a wall of crop dead ahead (e.g. exiting toward another
   planted block): every scan row is invalid (image-center column is not
   traversable at any scan height) while the lower half of the frame still
   has plenty of traversable ground to the sides/near the robot.

Guards against false positives:

- Debounce in PHYSICAL UNITS, per signature -- never in frames. A frame count
  means completely different things on different robots: the field-proven
  "5 frames" is 2.5 s on the 2 Hz CPU robot but 0.2 s on the ~25 Hz GPU
  robot, which is how a mid-row gap came to fire EXIT_CLEAR instantly in the
  field (2026-07-24). The units are chosen per signature:
    * OPEN accumulates METERS TRAVELLED (`exit_confirm_distance`). An exit
      must be confirmed by driving through it; if the robot stalls, not
      firing is the safe direction.
    * BLOCKED accumulates SECONDS (`blocked_confirm_seconds`). A blocked view
      stops the robot (the MPC goes invalid), so a distance-based counter
      would never fill and the back-out would deadlock -- it must be timed by
      the clock.
  Both accumulators are LEAKY and symmetric: a non-signature frame subtracts
  the same increment it would have added, floored at zero. Strict
  consecutiveness must NOT be used with these units -- 0.4 m at 25 Hz is ~65
  consecutive frames, and one flickery frame resetting the streak would turn
  the debounce into a never-fires bug. The leaky form asks for NET sustained
  evidence instead, which is what is actually meant. `exit_detect_min_frames`
  is a floor so no single large-delta frame can fire the exit alone.
- Arming distance, per signature: the OPEN signature is disabled until the
  robot has travelled `min_in_row_distance` meters (odometry) inside the
  current row, so the open-field view at row entry cannot trigger an exit.
  The BLOCKED signature arms much earlier (`blocked_arming_distance`) so an
  obstacle shortly after row entry is still caught -- it cannot false-fire
  at entry because it requires zero visible corridor at every scan row.
"""

import time
from collections import namedtuple

EXIT_NONE = "none"
EXIT_ROW_END_OPEN = "row_end_open"
EXIT_ROW_END_BLOCKED = "row_end_blocked"

# Largest per-frame time step credited to the BLOCKED accumulator. Guards
# against a stalled pipeline (or a paused sim clock jumping forward) banking
# seconds of "evidence" from a single frame.
_MAX_DT = 2.0

# Both accumulators are sums of many small floating-point increments, so an
# exact >= comparison can miss the threshold by ~1e-16 after the "right"
# number of frames. Nothing physical rides on that last nanometre.
_EPS = 1e-9

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
        "open_distance",         # meters of OPEN evidence banked (leaky)
        "blocked_seconds",       # seconds of BLOCKED evidence banked (leaky)
        "open_rows",             # scan rows that are wide AND flank-clear
        "open_streak_start",     # distance_in_row where the streak began
        "near_row_flank_clear",  # nearest scan row flank-clear? (revocation)
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


def flank_clear_flags(
    centerline_result, image_width, edge_margin_frac, min_clear_fraction=0.8
):
    """Per-scan-row: True if no non-traversable region (corn) flanks the
    traversable corridor on either side.

    This is the "open exit" discriminator: still inside the row, corn borders
    the corridor and it stops short of the image edge; at a true row end the
    corridor runs edge-to-edge.

    Preferred test (when centerline_estimator supplied the strip fractions):
    the outer `edge_margin_frac` strip on each side must be at least
    `min_clear_fraction` traversable. Measuring strip OCCUPANCY is immune to a
    single stray misclassified pixel truncating the outward corridor scan --
    the failure mode of the exact-edge-reach test on the weaker low-camera
    masks.

    Fallback (strip fractions absent, e.g. hand-built ScanRowResults): the
    corridor bounds must reach within `edge_margin_frac` of both borders.

    `edge_margin_frac >= 1.0` disables the flank term entirely (width-only
    behavior). Returns a list aligned with centerline_result.scan_rows; None
    for rows with no corridor.
    """
    disabled = edge_margin_frac >= 1.0
    margin_px = int(round(edge_margin_frac * (image_width - 1)))
    flags = []
    for sr in centerline_result.scan_rows:
        if sr.x_left is None:
            flags.append(None)
        elif disabled:
            flags.append(True)
        elif sr.left_clear_frac is not None and sr.right_clear_frac is not None:
            flags.append(
                sr.left_clear_frac >= min_clear_fraction
                and sr.right_clear_frac >= min_clear_fraction
            )
        else:
            flags.append(
                sr.x_left <= margin_px
                and sr.x_right >= (image_width - 1) - margin_px
            )
    return flags


def nearest_row_flank_clear(
    centerline_result, image_width, edge_margin_frac, min_clear_fraction=0.8
):
    """Flank-clear flag of the NEAREST scan row (largest row_y), or None.

    Used to revoke a just-fired open exit. The nearest scan row images the
    ground immediately beside the robot, so it answers "is there still a corn
    wall right next to me?" -- unlike the far rows, which legitimately see the
    corn block across the headland while the robot is genuinely exiting. It is
    also independent of `exit_open_rows_required`, so raising that knob cannot
    break revocation.
    """
    rows = centerline_result.scan_rows
    if not rows:
        return None
    flags = flank_clear_flags(
        centerline_result, image_width, edge_margin_frac, min_clear_fraction
    )
    near_index = max(range(len(rows)), key=lambda i: rows[i].row_y)
    return flags[near_index]


class RowExitDetector:
    """End-of-row detection debounced in meters (open) and seconds (blocked)."""

    def __init__(
        self,
        exit_width_threshold=0.8,
        exit_confirm_distance=0.4,
        min_in_row_distance=2.0,
        blocked_min_traversable_fraction=0.02,
        blocked_arming_distance=0.3,
        exit_open_rows_required=1,
        blocked_confirm_seconds=4.0,
        exit_flank_edge_margin=0.05,
        exit_flank_min_clear_fraction=0.8,
        exit_detect_min_frames=2,
    ):
        self.exit_width_threshold = exit_width_threshold
        # Meters of sustained OPEN evidence before firing. Default 0.4 m
        # reproduces the field-proven CPU-robot behavior (5 frames at 2 Hz and
        # 0.15 m/s = 0.375 m) on a robot of ANY inference rate.
        self.exit_confirm_distance = exit_confirm_distance
        self.min_in_row_distance = min_in_row_distance
        self.blocked_min_traversable_fraction = blocked_min_traversable_fraction
        self.blocked_arming_distance = blocked_arming_distance
        self.exit_open_rows_required = exit_open_rows_required
        # Seconds of sustained BLOCKED evidence before firing. Default 4.0 s
        # is the same field/sim-validated 8 frames at 2 Hz.
        self.blocked_confirm_seconds = blocked_confirm_seconds
        # OPEN also requires no corn flanking the corridor: the outer
        # exit_flank_edge_margin strip on each side must be at least
        # exit_flank_min_clear_fraction traversable. A mid-row side-gap widens
        # the corridor (~0.83 on the low camera) but leaves corn in the strips,
        # so it must NOT read as an exit. Larger margin => more permissive;
        # >= 1.0 disables the flank check (width-only, pre-2026-07-24 behavior).
        self.exit_flank_edge_margin = exit_flank_edge_margin
        self.exit_flank_min_clear_fraction = exit_flank_min_clear_fraction
        # Floor on the number of frames contributing to an open streak, so a
        # single frame carrying a large odometry delta cannot fire the exit.
        self.exit_detect_min_frames = exit_detect_min_frames

        self._open_distance = 0.0
        self._open_streak_start = None
        self._open_frames = 0
        self._blocked_seconds = 0.0
        self._last_distance = None
        self._last_now = None
        self.last_status = None

    def reset(self):
        """Clear debounce accumulators (call when entering a new row)."""
        self._open_distance = 0.0
        self._open_streak_start = None
        self._open_frames = 0
        self._blocked_seconds = 0.0
        self._last_distance = None
        self._last_now = None

    @property
    def open_streak_start(self):
        """distance_in_row at which the current open streak began, or None.

        MissionFSM back-dates the EXIT_CLEAR odometry reference to this point
        so the confirmation distance is not ADDED to headland_clearance: total
        travel is measured from where the exit was first seen, not from where
        it was confirmed.
        """
        return self._open_streak_start

    def update(self, centerline_result, image_width, distance_in_row, now=None):
        """Feed one frame's result; returns EXIT_NONE / EXIT_ROW_END_*.

        Args:
            centerline_result: CenterlineResult from estimate_centerline().
            image_width: mask width in pixels (normalizes corridor widths).
            distance_in_row: odometry distance travelled since entering the
                current row (meters). None (no odometry yet) keeps the
                detector unarmed.
            now: monotonic timestamp in seconds for the BLOCKED accumulator.
                The ROS node passes rospy.get_time() so it follows sim time
                under Gazebo (where wall clock and sim clock diverge at
                RTF < 1). Defaults to time.monotonic().
        """
        if now is None:
            now = time.monotonic()

        if distance_in_row is None:
            self.reset()
            self._last_now = now
            self.last_status = self._status(None, False, False, 0, 0,
                                            centerline_result, 0, None)
            return EXIT_NONE

        # Per-frame increments. Distance deltas are clamped non-negative so a
        # revoked exit (which restores an earlier row-entry reference) cannot
        # feed a negative step into the accumulator; time deltas are capped so
        # a stalled pipeline cannot bank seconds of evidence in one frame.
        d_delta = (
            max(0.0, distance_in_row - self._last_distance)
            if self._last_distance is not None
            else 0.0
        )
        t_delta = (
            min(_MAX_DT, max(0.0, now - self._last_now))
            if self._last_now is not None
            else 0.0
        )
        self._last_distance = distance_in_row
        self._last_now = now

        # Per-signature arming; while a signature is unarmed its accumulator
        # is held at zero so a streak cannot straddle the boundary.
        open_armed = distance_in_row >= self.min_in_row_distance
        blocked_armed = distance_in_row >= self.blocked_arming_distance
        if not open_armed and not blocked_armed:
            self._open_distance = 0.0
            self._open_streak_start = None
            self._open_frames = 0
            self._blocked_seconds = 0.0
            self.last_status = self._status(
                distance_in_row, False, False, 0, 0, centerline_result, 0, None
            )
            return EXIT_NONE

        widths = normalized_corridor_widths(centerline_result, image_width)
        flank_flags = flank_clear_flags(
            centerline_result,
            image_width,
            self.exit_flank_edge_margin,
            self.exit_flank_min_clear_fraction,
        )
        valid_widths = [w for w in widths if w is not None]

        # OPEN: at least exit_open_rows_required scan rows (ANY of them) see a
        # corridor that is BOTH wide (>= exit_width_threshold) AND flank-clear.
        # No specific row is required, so unreliable segmentation of distant
        # ground (far rows invalid beyond the field) cannot mask the exit:
        # whichever row sees open field first starts the streak. The
        # flank-clear term rejects a mid-row side-gap that merely widens the
        # corridor while corn still borders it.
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

        if open_signature and open_armed:
            if self._open_streak_start is None:
                # First frame of a streak: record where it began and credit no
                # distance yet, so _open_distance stays exactly "meters driven
                # since the exit was first seen".
                self._open_streak_start = distance_in_row
                self._open_frames = 1
            else:
                self._open_distance += d_delta
                self._open_frames += 1
        else:
            self._open_distance = max(0.0, self._open_distance - d_delta)
            if self._open_distance <= 0.0:
                self._open_streak_start = None
                self._open_frames = 0

        if blocked_signature and blocked_armed:
            self._blocked_seconds += t_delta
        else:
            self._blocked_seconds = max(0.0, self._blocked_seconds - t_delta)

        near_clear = nearest_row_flank_clear(
            centerline_result,
            image_width,
            self.exit_flank_edge_margin,
            self.exit_flank_min_clear_fraction,
        )
        self.last_status = self._status(
            distance_in_row,
            open_armed,
            blocked_armed,
            len(valid_widths),
            wide_rows,
            centerline_result,
            open_rows,
            near_clear,
        )

        if (
            self._open_distance >= self.exit_confirm_distance - _EPS
            and self._open_frames >= self.exit_detect_min_frames
        ):
            return EXIT_ROW_END_OPEN
        if self._blocked_seconds >= self.blocked_confirm_seconds - _EPS:
            return EXIT_ROW_END_BLOCKED
        return EXIT_NONE

    def _status(self, distance_in_row, open_armed, blocked_armed,
                corridor_rows, wide_rows, centerline_result, open_rows,
                near_clear):
        return ExitDetectorStatus(
            distance_in_row=distance_in_row,
            open_armed=open_armed,
            blocked_armed=blocked_armed,
            corridor_rows=corridor_rows,
            wide_rows=wide_rows,
            traversable_fraction=centerline_result.traversable_fraction,
            open_distance=self._open_distance,
            blocked_seconds=self._blocked_seconds,
            open_rows=open_rows,
            open_streak_start=self._open_streak_start,
            near_row_flank_clear=near_clear,
        )
