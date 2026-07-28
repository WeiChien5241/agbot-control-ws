import pytest
import numpy as np

from agbot_vision_nav.centerline_estimator import (
    CLASS_OBSTACLE,
    CLASS_SKY,
    CLASS_TRAVERSABLE,
    CenterlineResult,
    ScanRowResult,
    estimate_centerline,
)
from agbot_vision_nav.row_exit_detector import (
    EXIT_NONE,
    EXIT_ROW_END_BLOCKED,
    EXIT_ROW_END_OPEN,
    RowExitDetector,
    nearest_row_flank_clear,
    normalized_corridor_widths,
)

HEIGHT = 100
WIDTH = 200

STEP_M = 0.05   # meters travelled between frames
STEP_S = 0.5    # seconds between frames


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


class Drive:
    """Feeds frames to a detector while advancing odometry and the clock.

    The debounce is measured in METERS (open) and SECONDS (blocked), never in
    frames, so a test has to move the robot and the clock rather than count
    frames -- which is the whole point of the change: the same test must pass
    at any frame rate. `.meters()` and `.seconds()` bank exactly the amount
    asked for, accounting for the first frame of a session crediting nothing
    (there is no previous sample to take a delta from).
    """

    def __init__(self, detector, distance=5.0, step_m=STEP_M, step_s=STEP_S):
        self.det = detector
        self.distance = distance
        self.step_m = step_m
        self.step_s = step_s
        self.now = 0.0
        self.last = EXIT_NONE
        self._primed = False

    def feed(self, mask, frames=1, distance=None):
        if distance is not None:
            self.distance = distance
        result = (
            mask
            if isinstance(mask, CenterlineResult)
            else estimate_centerline(mask)
        )
        for _ in range(frames):
            self.last = self.det.update(
                result, WIDTH, self.distance, now=self.now
            )
            if self.distance is not None:
                self.distance += self.step_m
            self.now += self.step_s
            self._primed = True
        return self.last

    def meters(self, mask, meters, **kwargs):
        frames = int(round(meters / self.step_m)) + (0 if self._primed else 1)
        return self.feed(mask, frames=frames, **kwargs)

    def seconds(self, mask, seconds, **kwargs):
        frames = int(round(seconds / self.step_s)) + (0 if self._primed else 1)
        return self.feed(mask, frames=frames, **kwargs)


def parked(detector, distance=5.0):
    """A Drive that does not move -- for BLOCKED, where the MPC has already
    stopped the robot, so only the clock advances."""
    return Drive(detector, distance=distance, step_m=0.0)


def test_normalized_corridor_widths():
    result = estimate_centerline(make_corridor_mask(half_corridor_width=30))
    widths = normalized_corridor_widths(result, WIDTH)
    assert len(widths) == len(result.scan_rows)
    for w in widths:
        assert w is not None
        assert w == (61 / WIDTH)  # 2*30+1 columns


def test_no_exit_in_row():
    det = RowExitDetector()
    assert Drive(det).meters(make_corridor_mask(), 2.0) == EXIT_NONE


def test_open_field_fires_after_confirm_distance():
    det = RowExitDetector(exit_confirm_distance=0.4)
    mask = make_open_field_mask()
    d = Drive(det)
    assert d.meters(mask, 0.30) == EXIT_NONE
    assert d.meters(mask, 0.10) == EXIT_ROW_END_OPEN


def test_open_fires_at_same_distance_at_any_frame_rate():
    """The core regression (field, 2026-07-24). With a frame-counted debounce
    the exit fired after 5 frames = 2.5 s on the 2 Hz robot but 0.2 s on the
    25 Hz robot, so a mid-row gap that the slow robot shrugged off committed
    the fast robot into the corn. Firing must depend on distance travelled,
    not on how fast the pipeline runs."""
    mask = make_open_field_mask()
    steps = (0.075, 0.006)          # 2 Hz and 25 Hz at 0.15 m/s
    fired_at = []
    for step_m in steps:
        det = RowExitDetector(exit_confirm_distance=0.4)
        d = Drive(det, distance=5.0, step_m=step_m)
        while d.feed(mask) == EXIT_NONE:
            assert d.distance < 6.0, "exit never fired"
        fired_at.append(d.distance - step_m - 5.0)   # distance AT the fire
    slow, fast = fired_at
    # Same place to within one sample of the COARSE robot -- all that is left
    # is quantization. Under the old frame-counted rule these were 0.375 m and
    # 0.03 m apart, a 12x difference, which is what drove into the corn.
    assert abs(slow - fast) <= steps[0] + 1e-9
    assert all(0.4 <= f <= 0.4 + steps[0] for f in fired_at)


def test_open_tolerates_a_dropout_frame():
    """The accumulator is leaky, not strictly consecutive. At 25 Hz, 0.4 m is
    ~65 frames; if one flickery frame reset the streak the exit would never
    fire at all."""
    det = RowExitDetector(exit_confirm_distance=0.4)
    d = Drive(det)
    assert d.meters(make_open_field_mask(), 0.35) == EXIT_NONE
    assert d.feed(make_corridor_mask()) == EXIT_NONE      # one bad frame
    assert d.meters(make_open_field_mask(), 0.10) == EXIT_ROW_END_OPEN


def test_open_accumulator_drains_on_sustained_non_open():
    det = RowExitDetector(exit_confirm_distance=0.4)
    d = Drive(det)
    assert d.meters(make_open_field_mask(), 0.30) == EXIT_NONE
    # Draining is deliberately slower than filling (exit_leak_ratio 0.5), so
    # clearing 0.30 m of evidence takes 0.60 m of in-row driving.
    assert d.meters(make_corridor_mask(), 0.40) == EXIT_NONE
    assert det.last_status.open_distance == pytest.approx(0.10)
    assert d.meters(make_corridor_mask(), 0.30) == EXIT_NONE   # drained to 0
    assert det.last_status.open_distance == 0.0
    assert det.open_streak_start is None
    assert d.meters(make_open_field_mask(), 0.30) == EXIT_NONE  # starts over


def test_min_frames_floor_blocks_a_single_jump():
    """One frame carrying a huge odometry delta must not fire the exit alone."""
    det = RowExitDetector(exit_confirm_distance=0.4, exit_detect_min_frames=3)
    mask = make_open_field_mask()
    d = Drive(det, step_m=1.0)
    assert d.feed(mask, frames=2) == EXIT_NONE   # 1.0 m banked, only 2 frames
    assert det.last_status.open_distance >= 0.4
    assert d.feed(mask, frames=1) == EXIT_ROW_END_OPEN


def test_open_streak_start_is_reported_for_back_dating():
    det = RowExitDetector(exit_confirm_distance=0.4)
    d = Drive(det, distance=5.0)
    d.meters(make_open_field_mask(), 0.4)
    # The FSM back-dates the headland leg to here, so the confirmation
    # distance is not added on top of headland_clearance.
    assert abs(det.open_streak_start - 5.0) < 1e-9


def make_far_open_mask(near_rows_from=85, half_corridor_width=30, sky_rows=20):
    """Approaching the row end: far scan rows see open field, the nearest
    scan row still sees the last plants' narrow corridor."""
    mask = np.full((HEIGHT, WIDTH), CLASS_TRAVERSABLE, dtype=np.uint8)
    mask[:sky_rows, :] = CLASS_SKY
    cx = WIDTH // 2
    mask[near_rows_from:, : cx - half_corridor_width] = CLASS_OBSTACLE
    mask[near_rows_from:, cx + half_corridor_width + 1 :] = CLASS_OBSTACLE
    return mask


def test_blocked_ahead_fires_after_confirm_seconds():
    det = RowExitDetector(blocked_confirm_seconds=2.5)
    mask = make_blocked_ahead_mask()
    result = estimate_centerline(mask)
    # sanity: signature preconditions
    assert all(w is None for w in normalized_corridor_widths(result, WIDTH))
    assert result.traversable_fraction >= 0.15
    d = parked(det)
    assert d.seconds(mask, 2.0) == EXIT_NONE
    assert d.seconds(mask, 0.5) == EXIT_ROW_END_BLOCKED


def test_blocked_fires_while_stationary():
    """BLOCKED is timed in seconds, not meters, precisely because a blocked
    view stops the robot: the MPC goes invalid and commands zero. A
    distance-based counter would never fill and the back-out would deadlock."""
    det = RowExitDetector(blocked_confirm_seconds=2.0)
    d = parked(det)   # step_m = 0: the robot never moves
    assert d.seconds(make_blocked_ahead_mask(), 2.0) == EXIT_ROW_END_BLOCKED


def test_unarmed_before_min_distance():
    det = RowExitDetector(min_in_row_distance=2.0)
    mask = make_open_field_mask()
    # open-field view at row entry must NOT trigger
    assert Drive(det, distance=0.5).feed(mask, frames=10) == EXIT_NONE
    # nor with unknown odometry
    d = Drive(det, distance=None)
    assert d.feed(mask, frames=50) == EXIT_NONE


def test_debounce_does_not_straddle_arming_boundary():
    """Open field visible the whole way, driving continuously through the
    arming distance: the 0.5 m seen BEFORE arming must not count toward the
    confirmation, so the exit cannot fire until ~0.4 m past 2.0 m."""
    det = RowExitDetector(min_in_row_distance=2.0, exit_confirm_distance=0.4)
    mask = make_open_field_mask()
    d = Drive(det, distance=1.5)
    # everything before the arming distance is discarded
    assert d.meters(mask, 0.45) == EXIT_NONE
    assert det.last_status.open_distance == 0.0
    # ...and the confirmation only starts accumulating from there
    assert d.meters(mask, 0.25) == EXIT_NONE          # ~2.20 m
    while d.feed(mask) == EXIT_NONE:
        assert d.distance < 2.6, "exit never fired"
    assert 2.3 <= d.distance <= 2.5


def test_blocked_arms_earlier_than_open():
    det = RowExitDetector(
        min_in_row_distance=2.0, blocked_arming_distance=0.3,
        blocked_confirm_seconds=2.0,
    )
    blocked = make_blocked_ahead_mask()
    # below the blocked arming distance: nothing
    assert parked(det, distance=0.2).seconds(blocked, 10.0) == EXIT_NONE
    # between blocked arming and open arming: blocked fires
    det.reset()
    assert parked(det, distance=1.0).seconds(blocked, 2.0) == EXIT_ROW_END_BLOCKED
    # open stays gated by min_in_row_distance in the same window
    det.reset()
    d = Drive(det, distance=1.0, step_m=0.0)
    assert d.feed(make_open_field_mask(), frames=50) == EXIT_NONE


def test_blocked_debounce_does_not_straddle_arming_boundary():
    det = RowExitDetector(
        blocked_arming_distance=0.3, blocked_confirm_seconds=2.0
    )
    blocked = make_blocked_ahead_mask()
    d = parked(det, distance=0.2)
    assert d.seconds(blocked, 1.5) == EXIT_NONE
    d = parked(det, distance=0.4)
    assert d.seconds(blocked, 1.5) == EXIT_NONE
    assert d.seconds(blocked, 0.5) == EXIT_ROW_END_BLOCKED


def test_far_rows_wide_fires_open_before_near_row_clears():
    # The two farthest scan rows wide + nearest still a narrow corridor
    # must read as an open exit (this is the approach to the row end).
    det = RowExitDetector(exit_open_rows_required=2, exit_confirm_distance=0.4)
    mask = make_far_open_mask()
    d = Drive(det)
    assert d.meters(mask, 0.30) == EXIT_NONE
    assert d.meters(mask, 0.10) == EXIT_ROW_END_OPEN
    # ...but requiring all 3 rows keeps the old late behavior
    det = RowExitDetector(exit_open_rows_required=3, exit_confirm_distance=0.4)
    assert Drive(det).meters(mask, 2.0) == EXIT_NONE


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

    det = RowExitDetector(exit_confirm_distance=0.4)  # defaults: required=1
    d = Drive(det)
    assert d.meters(mask, 0.30) == EXIT_NONE
    assert d.meters(mask, 0.10) == EXIT_ROW_END_OPEN
    # and this frame must never count as blocked (near row is valid)
    det = RowExitDetector(blocked_confirm_seconds=0.5)
    assert parked(det).seconds(mask, 5.0) != EXIT_ROW_END_BLOCKED


def test_far_row_corridor_prevents_blocked():
    # A frame whose far row still has a corridor is not a blocked frame,
    # even if the near rows lose theirs (leaves brushing the camera).
    det = RowExitDetector(blocked_confirm_seconds=0.5)
    mask = make_corridor_mask()
    cx = WIDTH // 2
    mask[85:, cx - 40 : cx + 41] = CLASS_OBSTACLE  # near rows blocked
    assert parked(det).seconds(mask, 10.0) == EXIT_NONE


def make_close_blocker_mask(sky_rows=20, ground_cols=8):
    """Stopped nose-up to an obstacle: it fills nearly the whole view, no
    corridor at any scan row, only a sliver of ground at the left edge
    (lower-half traversable fraction ~0.04 -- matches the sim HUD reading
    in front of a large box blocker)."""
    mask = np.full((HEIGHT, WIDTH), CLASS_OBSTACLE, dtype=np.uint8)
    mask[:sky_rows, :] = CLASS_SKY
    mask[HEIGHT // 2 :, :ground_cols] = CLASS_TRAVERSABLE
    return mask


def test_blocked_fires_at_close_range_low_ground_fraction():
    # Regression (sim, 2026-07-21, twice): the robot stopped in front of a
    # mid-row blocker and the back-out never fired. Up close the obstacle
    # dominates the frame -- the sim HUD showed frac=0.04, under both the
    # 0.15 and 0.08 gates. The default gate must accept this view.
    mask = make_close_blocker_mask()
    result = estimate_centerline(mask)
    assert all(w is None for w in normalized_corridor_widths(result, WIDTH))
    assert 0.02 <= result.traversable_fraction < 0.08

    det = RowExitDetector(blocked_confirm_seconds=2.0)  # default gate 0.02
    assert parked(det).seconds(mask, 2.0) == EXIT_ROW_END_BLOCKED


def test_blocked_debounce_is_leaky_not_reset():
    det = RowExitDetector(blocked_confirm_seconds=4.0)
    blocked = make_blocked_ahead_mask()
    corridor = make_corridor_mask()
    # 2.5 s blocked, one noisy corridor frame (4.0 -> 3.5 s of headroom, NOT
    # a full reset), then the remainder fires.
    d = parked(det)
    assert d.seconds(blocked, 2.5) == EXIT_NONE
    assert d.seconds(corridor, 0.5) == EXIT_NONE
    assert d.seconds(blocked, 1.5) == EXIT_NONE
    assert d.seconds(blocked, 0.5) == EXIT_ROW_END_BLOCKED


def test_blocked_leak_drains_on_sustained_noise():
    # Alternating blocked/corridor frames must never accumulate to a fire.
    det = RowExitDetector(blocked_confirm_seconds=4.0)
    blocked = make_blocked_ahead_mask()
    corridor = make_corridor_mask()
    d = parked(det)
    for _ in range(30):
        assert d.feed(blocked) == EXIT_NONE
        assert d.feed(corridor) == EXIT_NONE


def test_last_status_reports_detector_internals():
    det = RowExitDetector()
    assert det.last_status is None

    corridor_result = estimate_centerline(make_corridor_mask())
    det.update(corridor_result, WIDTH, None, now=0.0)
    assert det.last_status.distance_in_row is None

    det.update(corridor_result, WIDTH, 1.0, now=0.5)  # blocked armed, open not
    s = det.last_status
    assert (s.open_armed, s.blocked_armed) == (False, True)
    assert s.corridor_rows == 3 and s.wide_rows == 0
    assert s.open_distance == 0.0 and s.blocked_seconds == 0.0

    blocked_result = estimate_centerline(make_blocked_ahead_mask())
    now = 1.0
    for _ in range(3):
        det.update(blocked_result, WIDTH, 5.0, now=now)
        now += 0.5
    s = det.last_status
    assert abs(s.blocked_seconds - 1.5) < 1e-9
    assert s.corridor_rows == 0
    assert s.open_armed and s.blocked_armed


# ---------------------------------------------- flank-clear (open exit) gate --
def make_wide_but_flanked_mask(margin=16, sky_rows=20):
    """Mid-row side-gap: the corridor is WIDE (>= exit_width_threshold) but corn
    still flanks it on BOTH sides -- it stops `margin` px short of each image
    edge. Must NOT read as an open exit (this is the low-camera false-positive
    the flank check fixes)."""
    mask = np.full((HEIGHT, WIDTH), CLASS_OBSTACLE, dtype=np.uint8)
    mask[:sky_rows, :] = CLASS_SKY
    mask[sky_rows:, margin : WIDTH - margin] = CLASS_TRAVERSABLE
    return mask


def make_left_open_right_corn_mask(right_margin=16, sky_rows=20):
    """Corridor reaches the LEFT image edge but corn borders it on the right."""
    mask = np.full((HEIGHT, WIDTH), CLASS_OBSTACLE, dtype=np.uint8)
    mask[:sky_rows, :] = CLASS_SKY
    mask[sky_rows:, 0 : WIDTH - right_margin] = CLASS_TRAVERSABLE
    return mask


def test_open_blocked_by_flank_corn_mid_row_gap():
    # Core regression (low camera + GPU robot): a near scan row widens to ~0.84
    # at a mid-row gap but corn still flanks the corridor short of the edges.
    # wide_rows sees it, but open_rows must stay 0 so OPEN never fires.
    det = RowExitDetector(exit_confirm_distance=0.4)  # default margin 0.05
    mask = make_wide_but_flanked_mask(margin=16)
    result = estimate_centerline(mask)
    widths = normalized_corridor_widths(result, WIDTH)
    assert all(w is not None and w >= 0.8 for w in widths)  # genuinely wide
    assert Drive(det).meters(mask, 3.0) == EXIT_NONE
    s = det.last_status
    assert s.wide_rows == 3 and s.open_rows == 0


def test_open_fires_when_corridor_reaches_both_edges():
    det = RowExitDetector(exit_confirm_distance=0.4)
    mask = make_open_field_mask()  # traversable edge-to-edge below the sky
    d = Drive(det)
    assert d.meters(mask, 0.30) == EXIT_NONE
    assert d.meters(mask, 0.10) == EXIT_ROW_END_OPEN
    assert det.last_status.open_rows >= 1


def test_one_sided_edge_does_not_fire():
    # Reaches the left edge but corn on the right -> not flank-clear (both
    # sides must be clear). Proves the AND, not just total width.
    det = RowExitDetector(exit_confirm_distance=0.4)
    mask = make_left_open_right_corn_mask(right_margin=16)
    result = estimate_centerline(mask)
    widths = normalized_corridor_widths(result, WIDTH)
    assert all(w is not None and w >= 0.8 for w in widths)  # wide
    assert Drive(det).meters(mask, 3.0) == EXIT_NONE
    assert det.last_status.open_rows == 0


def test_flank_margin_large_restores_width_only_behavior():
    # exit_flank_edge_margin >= 1.0 disables the flank check: a merely-wide
    # (but corn-flanked) row fires again, i.e. pre-fix width-only behavior.
    det = RowExitDetector(
        exit_confirm_distance=0.4, exit_flank_edge_margin=1.0
    )
    mask = make_wide_but_flanked_mask(margin=16)
    d = Drive(det)
    assert d.meters(mask, 0.30) == EXIT_NONE
    assert d.meters(mask, 0.10) == EXIT_ROW_END_OPEN


def test_last_status_reports_open_rows():
    det = RowExitDetector()
    open_result = estimate_centerline(make_open_field_mask())
    det.update(open_result, WIDTH, 5.0, now=0.0)
    assert det.last_status.open_rows >= 1
    flanked_result = estimate_centerline(make_wide_but_flanked_mask(margin=16))
    det.update(flanked_result, WIDTH, 5.0, now=0.5)
    assert det.last_status.open_rows == 0


# ------------------------------------------ strip-occupancy flank tolerance --
def test_stray_edge_pixel_does_not_veto_a_real_exit():
    """The corridor scan stops at the FIRST non-traversable pixel, so a single
    misclassified pixel just inside the image would veto an exact-edge-reach
    test outright -- a real false-negative risk on the low-camera masks.
    Measuring strip OCCUPANCY instead tolerates it."""
    mask = make_open_field_mask()
    mask[:, 12] = CLASS_OBSTACLE          # stray column, outside the 5% strip
    result = estimate_centerline(mask)
    near = result.scan_rows[-1]
    assert near.x_left == 13              # the scan really was truncated
    assert near.left_clear_frac == 1.0    # ...but the strip itself is clear

    det = RowExitDetector(exit_confirm_distance=0.4)
    d = Drive(det)
    assert d.meters(mask, 0.30) == EXIT_NONE
    assert d.meters(mask, 0.10) == EXIT_ROW_END_OPEN


def test_corn_inside_the_strip_still_vetoes():
    """Partial corn in the outer strip must still block the exit -- and this
    case the old exact-edge test let through, since the corridor bound (col 6)
    was within the 5% margin."""
    mask = make_wide_but_flanked_mask(margin=6)
    result = estimate_centerline(mask)
    near = result.scan_rows[-1]
    assert near.x_left == 6               # would pass the old margin test
    assert near.left_clear_frac < 0.8     # but the strip is 60% corn

    det = RowExitDetector(exit_confirm_distance=0.4)
    assert Drive(det).meters(mask, 3.0) == EXIT_NONE
    assert det.last_status.open_rows == 0


def _hand_built_result(x_left, x_right, row_ys=(64, 77, 91)):
    """CenterlineResult without strip fractions (they default to None)."""
    rows = [
        ScanRowResult(y, x_left, x_right, 0.5 * (x_left + x_right), 0.0)
        for y in row_ys
    ]
    return CenterlineResult(0.0, 0.0, True, 0.5, rows)


def test_falls_back_to_edge_reach_without_strip_fractions():
    det = RowExitDetector(exit_confirm_distance=0.4)
    # Reaches both edges -> open, via the fallback path.
    assert Drive(det).meters(_hand_built_result(0, WIDTH - 1), 0.4) == (
        EXIT_ROW_END_OPEN
    )
    # Wide but stops short of both edges -> still in row.
    det = RowExitDetector(exit_confirm_distance=0.4)
    assert Drive(det).meters(
        _hand_built_result(16, WIDTH - 17), 3.0
    ) == EXIT_NONE


def test_nearest_row_flank_clear_picks_the_bottom_row():
    """Revocation reads the NEAREST row only. Here the far rows are open field
    while the near row is corn-flanked -- exactly the geometry of a false exit
    fired mid-row, and the inverse of a genuine exit (where the near row is
    clear and a far row may see the corn block across the headland)."""
    mask = make_open_field_mask()
    mask[85:, :40] = CLASS_OBSTACLE
    mask[85:, -40:] = CLASS_OBSTACLE
    result = estimate_centerline(mask)
    assert nearest_row_flank_clear(result, WIDTH, 0.05, 0.8) is False
    assert nearest_row_flank_clear(
        estimate_centerline(make_open_field_mask()), WIDTH, 0.05, 0.8
    ) is True


# ------------------------------------------------ asymmetric leak (marginal exit) --
def alternating(drive, mask_a, mask_b, meters, duty):
    """Feed frames alternating between two views with the given duty cycle on
    mask_a, driving `meters` in total. Models a marginal exit: the mask is
    good enough to read as open only some of the time."""
    frames = int(round(meters / drive.step_m))
    signal = EXIT_NONE
    acc = 0.0
    for _ in range(frames):
        acc += duty
        if acc >= 1.0:
            acc -= 1.0
            signal = drive.feed(mask_a)
        else:
            signal = drive.feed(mask_b)
        if signal != EXIT_NONE:
            return signal
    return signal


def test_marginal_exit_fires_despite_a_flickering_signature():
    """The 2026-07-28 sim failure. At the row end the model DOES label the
    ground traversable, just imperfectly -- width ~0.8-0.9 with patchy edges --
    so the open signature is true only about half the frames. Under a
    symmetric leak that nets exactly zero and can NEVER fire however far the
    robot drives (the log showed the meter reach 0.13 of 0.40 m and drain back
    to 0.00 while the robot left the world). Filling faster than it drains
    fixes it."""
    det = RowExitDetector(exit_confirm_distance=0.4)   # default leak 0.5
    d = Drive(det)
    assert alternating(
        d, make_open_field_mask(), make_corridor_mask(), meters=3.0, duty=0.5
    ) == EXIT_ROW_END_OPEN


def test_symmetric_leak_reproduces_the_failure():
    """exit_leak_ratio=1.0 restores the old behavior exactly -- and with it,
    the bug: a 50% signature never fires. Kept as the counter-example that
    documents WHY the default is 0.5."""
    det = RowExitDetector(exit_confirm_distance=0.4, exit_leak_ratio=1.0)
    d = Drive(det)
    assert alternating(
        d, make_open_field_mask(), make_corridor_mask(), meters=5.0, duty=0.5
    ) == EXIT_NONE


def test_weak_signature_still_does_not_fire():
    """The leak must not be so forgiving that noise accumulates. A signature
    true only ~20% of the time is noise, not a row end."""
    det = RowExitDetector(exit_confirm_distance=0.4)
    d = Drive(det)
    assert alternating(
        d, make_open_field_mask(), make_corridor_mask(), meters=5.0, duty=0.2
    ) == EXIT_NONE


def test_short_mid_row_gap_still_does_not_fire():
    """The false positive this whole subsystem exists to prevent: a gap in the
    corn reads open for ~0.25 m, then the row closes back up. Even with the
    gentler leak it must not reach 0.40 m."""
    det = RowExitDetector(exit_confirm_distance=0.4)
    d = Drive(det)
    assert d.meters(make_open_field_mask(), 0.25) == EXIT_NONE
    assert d.meters(make_corridor_mask(), 1.0) == EXIT_NONE
    assert det.last_status.open_distance == 0.0


def test_near_row_width_and_edges_reported():
    """The HUD needs to say WHICH threshold is blocking an exit: without these
    numbers 'width just under the bar' and 'edges just under the bar' both
    render as openrows=0."""
    det = RowExitDetector()
    det.update(estimate_centerline(make_open_field_mask()), WIDTH, 5.0, now=0.0)
    s = det.last_status
    assert s.near_row_width == pytest.approx(1.0)
    assert s.near_row_edges == (1.0, 1.0)

    det.update(
        estimate_centerline(make_wide_but_flanked_mask(margin=16)),
        WIDTH, 5.0, now=0.5,
    )
    s = det.last_status
    assert 0.8 <= s.near_row_width < 1.0        # wide enough...
    assert s.near_row_edges == (0.0, 0.0)       # ...but corn in both strips
