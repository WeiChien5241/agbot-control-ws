from agbot_vision_nav.intervention_detector import (
    DEFAULT_DEADMAN_BUTTONS,
    InterventionDetector,
)


def held(buttons=(0, 0, 0, 0, 1, 0)):
    return list(buttons)


def released():
    return [0, 0, 0, 0, 0, 0]


# -- what counts as a human driving ---------------------------------------


def test_deadman_button_is_a_takeover():
    d = InterventionDetector()
    assert d.sample_is_active(held())
    assert not d.sample_is_active(released())


def test_either_deadman_button_counts():
    d = InterventionDetector()
    for index in DEFAULT_DEADMAN_BUTTONS:
        buttons = [0] * 8
        buttons[index] = 1
        assert d.sample_is_active(buttons)


def test_short_button_array_does_not_crash():
    # A different pad reports fewer buttons than the index we look at.
    d = InterventionDetector()
    assert not d.sample_is_active([0, 0])


def test_stick_fallback_when_no_deadman_configured():
    d = InterventionDetector(deadman_buttons=(), axis_deadzone=0.2)
    assert not d.sample_is_active([], axes=[0.1, -0.05])
    assert d.sample_is_active([], axes=[0.1, -0.9])


def test_sticks_alone_do_not_count_when_a_deadman_is_configured():
    # The pad publishes stick noise constantly; only the deadman means intent.
    d = InterventionDetector()
    assert not d.sample_is_active(released(), axes=[1.0, 1.0])


# -- collapsing activity into events --------------------------------------


def test_first_takeover_starts_one_intervention():
    d = InterventionDetector()
    assert d.update(10.0, held()) is True
    assert d.count == 1


def test_released_joystick_never_counts():
    d = InterventionDetector()
    assert d.update(10.0, released()) is False
    assert d.count == 0


def test_one_messy_rescue_is_one_intervention():
    # Nudge, release, re-grip, nudge -- all inside the gap.
    d = InterventionDetector(gap_seconds=3.0)
    d.update(10.0, held())
    for t in (10.1, 11.0, 12.5, 13.0, 15.0):
        assert d.update(t, held()) is False
    assert d.count == 1


def test_a_quiet_gap_starts_a_new_intervention():
    d = InterventionDetector(gap_seconds=3.0)
    d.update(10.0, held())
    assert d.update(20.0, held()) is True
    assert d.count == 2


def test_release_inside_the_gap_does_not_split_the_event():
    d = InterventionDetector(gap_seconds=3.0)
    d.update(10.0, held())
    d.update(11.0, released())          # released samples are ignored entirely
    assert d.update(12.0, held()) is False
    assert d.count == 1


def test_teleop_seconds_accumulates_only_inside_a_window():
    d = InterventionDetector(gap_seconds=3.0)
    d.update(10.0, held())
    d.update(11.0, held())
    d.update(12.0, held())
    assert d.teleop_seconds == 2.0
    d.update(60.0, held())              # new window; the 48 s gap is not teleop
    assert d.teleop_seconds == 2.0
    assert d.count == 2


# -- the teleop flag written to each CSV row -------------------------------


def test_active_holds_briefly_then_expires():
    d = InterventionDetector(hold_seconds=0.5)
    d.update(10.0, held())
    assert d.active(10.2) is True
    assert d.active(10.5) is True
    assert d.active(11.0) is False


def test_active_is_false_before_any_joystick_message():
    assert InterventionDetector().active(10.0) is False
