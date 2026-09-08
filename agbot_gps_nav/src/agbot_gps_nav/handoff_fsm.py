"""Sequencing for a GPS transit followed by a vision-nav row mission.

Pure python -- no rospy. `scripts/mission_supervisor.py` is the ROS shell around
this; every decision about what should be running, and every way the sequence
can fail, is here where it can be tested without a simulator.

    IDLE --start()--> TRANSIT --gps ARRIVED--> ROW_MISSION --done--> FINISHED
                         |                          |
                         +------ timeout/abort -----+--> FAILED

⚠ EXACTLY ONE NODE MAY DRIVE AT A TIME. Both publish /cmd_vel, which twist_mux
takes as ONE input at priority 1 -- it arbitrates by priority, not rate, so two
publishers on it are not arbitrated at all, they interleave. Each tick therefore
reports the FULL desired state of both nodes rather than edge-triggered
"switch now" events: a supervisor that missed one edge would otherwise leave
both enabled and shred the commands. Re-asserting an unchanged state is free.

⚠ AND THE ORDER MATTERS ON THE HANDOFF TICK: disable the outgoing node BEFORE
enabling the incoming one. Doing it the other way round leaves a window, however
short, with two nodes publishing. `HandoffTick.disable_first` names which one
must be switched off first so the caller cannot get it wrong by accident.

⚠ ARRIVING IS NOT THE SAME AS ARRIVING POINTED THE RIGHT WAY, and until
2026-09-07 this module treated them as the same thing: gps_state == ARRIVED was
the only condition on the handoff. GPS pins position, never orientation, so a
robot with a bad yaw estimate arrives at exactly the right PLACE facing
somewhere else -- measured that day at ~72 deg off, which handed vision nav a
view of solid corn and cost the mission at rows=0/3. The follower measures the
course it actually drove over the final on-axis leg and compares it with the
yaw it thinks it has; that residual is passed in here, and a bad one aborts
instead of handing over. Refusing at the row entrance is cheap. Discovering it
one row spacing later is not.
"""

import collections

STATE_IDLE = "IDLE"
STATE_TRANSIT = "TRANSIT"
STATE_ROW_MISSION = "ROW_MISSION"
STATE_FINISHED = "FINISHED"
STATE_FAILED = "FAILED"

# gps_state values this module cares about; anything else is "still working".
GPS_ARRIVED = "ARRIVED"
GPS_FAILED = "FAILED"

HandoffTick = collections.namedtuple(
    "HandoffTick",
    "state gps_enabled vision_enabled send_goal disable_first done failed reason",
)


class HandoffFSM(object):
    """Decides what should be running, and when the sequence has failed."""

    def __init__(self, transit_timeout_sec=300.0, mission_timeout_sec=1800.0,
                 max_arrival_heading_error_deg=12.0):
        # Both are backstops, not schedules. A transit that has not arrived in
        # five minutes is not slow, it is lost -- and a lost robot on open
        # ground keeps going.
        self.transit_timeout_sec = float(transit_timeout_sec)
        self.mission_timeout_sec = float(mission_timeout_sec)
        # Same 12 deg as the heading bootstrap's tolerance, and for the same
        # reason: it is the disagreement between course over ground and the yaw
        # estimate that is small enough to drive a row on.
        self.max_arrival_heading_error_deg = float(max_arrival_heading_error_deg)
        self.state = STATE_IDLE
        self.reason = ""
        self._entered_at = None
        self._goal_sent = False

    # ---- transitions ------------------------------------------------------

    def start(self, now):
        """Begin the sequence: GPS drives, vision nav is held off."""
        self.state = STATE_TRANSIT
        self.reason = ""
        self._entered_at = float(now)
        self._goal_sent = False
        return self._tick(send_goal=True)

    def abort(self, reason):
        """Stop everything. Used for an operator stop or an external fault."""
        self.state = STATE_FAILED
        self.reason = reason
        return self._tick()

    def update(self, now, gps_state=None, vision_done=False,
               arrival_heading_error_deg=None):
        """Advance one tick and return the full desired state of both nodes.

        `arrival_heading_error_deg` is the follower's course-driven-vs-yaw
        residual on the final leg. None means it could not be measured -- which
        is allowed through, loudly, by the caller: the gate fires on evidence of
        being wrong, not on the absence of evidence.
        """
        now = float(now)
        if self.state in (STATE_IDLE, STATE_FINISHED, STATE_FAILED):
            return self._tick()

        elapsed = now - self._entered_at if self._entered_at is not None else 0.0

        if self.state == STATE_TRANSIT:
            if gps_state == GPS_FAILED:
                return self.abort("GPS nav aborted during the transit")
            if elapsed > self.transit_timeout_sec:
                return self.abort("transit timed out after %.0f s" % elapsed)
            if gps_state == GPS_ARRIVED:
                if (arrival_heading_error_deg is not None
                        and abs(float(arrival_heading_error_deg))
                        > self.max_arrival_heading_error_deg):
                    return self.abort(
                        "arrived at the waypoint pointing %+.0f deg off the "
                        "approach bearing (limit %.0f) -- the yaw estimate is "
                        "wrong, so this is the right place facing the wrong "
                        "way. NOT handing over to vision nav."
                        % (float(arrival_heading_error_deg),
                           self.max_arrival_heading_error_deg))
                self.state = STATE_ROW_MISSION
                self._entered_at = now
                return self._tick()
            return self._tick()

        if self.state == STATE_ROW_MISSION:
            if vision_done:
                self.state = STATE_FINISHED
                return self._tick()
            if elapsed > self.mission_timeout_sec:
                return self.abort("row mission timed out after %.0f s" % elapsed)
            return self._tick()

        return self._tick()

    # ---- the desired state of the world ----------------------------------

    def _tick(self, send_goal=False):
        gps = self.state == STATE_TRANSIT
        vision = self.state == STATE_ROW_MISSION
        # On the handoff tick both change; name the one to switch OFF first so
        # there is never a moment with two publishers on /cmd_vel.
        disable_first = None
        if self.state == STATE_ROW_MISSION:
            disable_first = "gps"
        elif self.state == STATE_TRANSIT:
            disable_first = "vision"
        if send_goal:
            self._goal_sent = True
        return HandoffTick(
            state=self.state,
            gps_enabled=gps,
            vision_enabled=vision,
            send_goal=send_goal,
            disable_first=disable_first,
            done=self.state == STATE_FINISHED,
            failed=self.state == STATE_FAILED,
            reason=self.reason,
        )
