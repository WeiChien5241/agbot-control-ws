"""Operator takeover: let a human on the joystick make the node stand down.

Pure Python, no rospy -- this holds only the timing rule, so it unit-tests
without ROS like the rest of the package's core.

Why this exists at all, given twist_mux already arbitrates: on the Jackal the
node publishes to `cmd_vel` (twist_mux `external`, priority 1) and the
joystick to `bluetooth_teleop/cmd_vel` (priority 9), so the mux SHOULD hand
control to the operator regardless of publish rate. In the field it did not --
a grad student could not override the node while the robot drove into corn,
and killing the node was the only recourse. The failure was rate-dependent
(fine at 2 Hz, broken at ~25 Hz), which is the fingerprint of the mux being
bypassed: two publishers racing on the controller topic, faster one wins.

So this is defence in depth. The mux wiring is fixed separately; this makes
the node yield on its own, which is the only thing that still works when the
arbitration layer is not in the path.

The signal is any message on the teleop output topic. `teleop_twist_joy`
publishes only while the deadman (L1 on the Jackal's PS4 pad) is held, so a
message arriving there means "a human is driving right now" -- including the
case where they hold the deadman with the sticks centred, which is exactly how
an operator says "stop".
"""


class OperatorOverride:
    """Tracks whether a human has control, from teleop message arrivals.

    The caller feeds `notify(now)` on every teleop message and asks
    `active(now)` before publishing. Autonomy resumes `hold_off_sec` after the
    operator stops -- teleop is a stream of discrete messages, so without a
    hold-off the node would interleave its own commands between joystick
    frames and fight the operator.
    """

    def __init__(self, hold_off_sec=2.0, enabled=True):
        self.hold_off_sec = hold_off_sec
        self.enabled = enabled
        self._last_notify = None

    def notify(self, now):
        """Record a teleop message seen at time `now` (seconds)."""
        self._last_notify = now

    def active(self, now):
        """True while the operator is considered to have control."""
        if not self.enabled or self._last_notify is None:
            return False
        # A clock that jumped backwards (sim time restarts at zero when Gazebo
        # is relaunched) leaves a timestamp from the future. Treat the history
        # as invalid rather than comparing against it -- a naive
        # `now - last < hold_off` would latch the override on for the whole
        # run and the node would never publish again.
        if now < self._last_notify:
            self._last_notify = None
            return False
        return (now - self._last_notify) < self.hold_off_sec

    def seconds_since_notify(self, now):
        """Age of the last teleop message, or None if there has never been one."""
        if self._last_notify is None:
            return None
        return max(0.0, now - self._last_notify)

    def reset(self):
        """Forget any operator activity (autonomy resumes immediately)."""
        self._last_notify = None
