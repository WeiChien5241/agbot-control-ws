"""Counts human interventions from the joystick, for the autonomy metric.

The number field papers report is distance per intervention (MDBI, "mean
distance between interventions"): how far the robot drove itself before a
person had to step in. It is only meaningful if "a person stepped in" is
defined mechanically and counted the same way on every run, so this module
owns that definition and nothing else touches it.

THE DEFINITION. An intervention is a JOYSTICK TAKEOVER: the operator holding
the deadman button on the Jackal's `bluetooth_teleop` pad. That is the
mechanism by which a person actually takes the robot away from the
controller -- the twist mux gives teleop priority, so the moment the deadman
goes down our cmd_vel stops reaching the wheels whether we keep publishing or
not. Nothing has to be remembered or pressed specially during a run, which is
the whole point: a metric that depends on someone tagging events in the
field is a metric with missing events.

WHY A GAP, NOT AN EDGE. An operator rescuing the robot does not hold the
deadman down cleanly; they nudge, release, re-grip, nudge again. Counting
button edges would score one rescue as five interventions. Instead a takeover
opens a window, and joystick activity within `gap_seconds` of the previous
activity belongs to the SAME intervention. Only activity after a quiet gap
starts a new one.

Pure Python: the node hands it (time, buttons, axes) triples out of
sensor_msgs/Joy and gets back a bool. No rospy, so it is unit-testable.
"""


# Jackal's bluetooth_teleop deadman buttons on the stock PS4/Xbox mapping:
# L1 (normal speed) and R1 (turbo). Held, not tapped.
DEFAULT_DEADMAN_BUTTONS = (4, 5)


class InterventionDetector(object):
    """Joystick takeovers, collapsed into intervention events.

    `update()` returns True exactly once per intervention -- on the sample
    that opens a new window -- so the caller can mark the event without
    tracking edges itself.
    """

    def __init__(
        self,
        deadman_buttons=DEFAULT_DEADMAN_BUTTONS,
        axis_deadzone=0.15,
        gap_seconds=3.0,
        hold_seconds=0.5,
    ):
        # An empty button list means "this pad has no deadman we can name":
        # fall back to stick deflection past the deadzone. Worse (an idle pad
        # resting off-centre reads as a takeover), which is why the buttons
        # are the default.
        self.deadman_buttons = tuple(int(b) for b in (deadman_buttons or ()))
        self.axis_deadzone = float(axis_deadzone)
        self.gap_seconds = float(gap_seconds)
        # How long one active sample keeps `active()` true. joy drivers
        # publish on change (plus an autorepeat), so a held deadman can go
        # quiet for a fraction of a second without the human letting go.
        self.hold_seconds = float(hold_seconds)

        self.count = 0
        self.teleop_seconds = 0.0
        self._last_active_t = None
        self._current_start_t = None

    # -- the definition ---------------------------------------------------

    def sample_is_active(self, buttons=(), axes=()):
        """Is this one Joy message a human driving?"""
        if self.deadman_buttons:
            for index in self.deadman_buttons:
                if 0 <= index < len(buttons) and buttons[index]:
                    return True
            return False
        for value in axes or ():
            if abs(float(value)) > self.axis_deadzone:
                return True
        return False

    # -- the counter ------------------------------------------------------

    def update(self, t, buttons=(), axes=()):
        """Feed one Joy message. True iff it STARTS a new intervention."""
        if not self.sample_is_active(buttons, axes):
            return False
        t = float(t)
        started = False
        if self._last_active_t is None or (t - self._last_active_t) > self.gap_seconds:
            self.count += 1
            self._current_start_t = t
            started = True
        else:
            # Only the gap between two active samples is teleop time; the
            # quiet stretch that ended the last window is not.
            self.teleop_seconds += max(0.0, t - self._last_active_t)
        self._last_active_t = t
        return started

    def active(self, t):
        """Is a human driving as of time `t`?"""
        if self._last_active_t is None:
            return False
        return (float(t) - self._last_active_t) <= self.hold_seconds

    @property
    def current_intervention_start(self):
        return self._current_start_t
