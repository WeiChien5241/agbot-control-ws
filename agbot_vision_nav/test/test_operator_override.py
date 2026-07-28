from agbot_vision_nav.operator_override import OperatorOverride


def test_inactive_before_any_teleop_message():
    ov = OperatorOverride(hold_off_sec=2.0)
    assert not ov.active(0.0)
    assert not ov.active(1000.0)
    assert ov.seconds_since_notify(5.0) is None


def test_active_immediately_on_the_first_message():
    """The whole point is that the node stands down at once -- an operator
    grabbing the stick is usually preventing a collision."""
    ov = OperatorOverride(hold_off_sec=2.0)
    ov.notify(10.0)
    assert ov.active(10.0)


def test_stays_active_through_the_hold_off_then_releases():
    ov = OperatorOverride(hold_off_sec=2.0)
    ov.notify(10.0)
    assert ov.active(11.9)          # teleop is a stream of discrete messages:
    assert not ov.active(12.0)      # without the hold-off the node would
    assert not ov.active(50.0)      # interleave commands and fight the operator


def test_each_message_extends_the_window():
    ov = OperatorOverride(hold_off_sec=2.0)
    ov.notify(10.0)
    ov.notify(11.5)
    assert ov.active(13.0)          # would have expired from the first message
    assert not ov.active(13.5)


def test_disabled_never_activates():
    ov = OperatorOverride(hold_off_sec=2.0, enabled=False)
    ov.notify(10.0)
    assert not ov.active(10.0)


def test_backwards_clock_does_not_latch_the_override_on():
    """Sim time restarts at zero when Gazebo is relaunched. A naive
    now - last < hold_off would then stay true for the whole run and the node
    would never publish again."""
    ov = OperatorOverride(hold_off_sec=2.0)
    ov.notify(500.0)
    assert not ov.active(0.0)
    assert not ov.active(3.0)


def test_reset_releases_control():
    ov = OperatorOverride(hold_off_sec=2.0)
    ov.notify(10.0)
    assert ov.active(10.5)
    ov.reset()
    assert not ov.active(10.5)


def test_seconds_since_notify_reports_age():
    ov = OperatorOverride(hold_off_sec=2.0)
    ov.notify(10.0)
    assert ov.seconds_since_notify(12.5) == 2.5
