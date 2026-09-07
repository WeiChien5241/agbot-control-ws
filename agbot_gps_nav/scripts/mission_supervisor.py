#!/usr/bin/python3
"""Sequences a GPS transit to a row entrance, then a vision-nav row mission.

The second rospy file in this package. Every DECISION lives in
`agbot_gps_nav.handoff_fsm`, which is rospy-free and unit-tested; this is the
plumbing around it.

    rosrun agbot_gps_nav mission_supervisor.py _waypoint:=corridor_0

⚠ EXACTLY ONE NODE MAY DRIVE AT A TIME, and this node is what guarantees it.
Both gps_nav_node and vision_nav_node publish /cmd_vel, which twist_mux takes as
ONE input at priority 1 -- it arbitrates by PRIORITY, not rate, so two
publishers on that topic are not arbitrated at all, they interleave and the
robot stutters between two commanders. Each tick therefore re-asserts the FULL
desired state of both nodes rather than firing one-shot switches, and on a
handoff the outgoing node is disabled BEFORE the incoming one is enabled.

Disabling makes a node go SILENT rather than publish zeros, which is what makes
that safe -- see the ~set_enabled comments in either node.
"""

import math
import os
import sys

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from agbot_gps_nav import geo  # noqa: E402
from agbot_gps_nav.handoff_fsm import (  # noqa: E402
    GPS_ARRIVED, GPS_FAILED, HandoffFSM,
)


class MissionSupervisor(object):

    def __init__(self):
        rospy.init_node("mission_supervisor")

        waypoint_file = rospy.get_param("~waypoint_file")
        wanted = rospy.get_param("~waypoint", "corridor_0")
        with open(waypoint_file) as handle:
            spec = yaml.safe_load(handle)
        self._datum = (spec["datum"][0], spec["datum"][1])
        entry = next((w for w in spec["waypoints"] if w["name"] == wanted), None)
        if entry is None:
            names = ", ".join(w["name"] for w in spec["waypoints"])
            rospy.logfatal("no waypoint '%s' in %s (have: %s)",
                           wanted, waypoint_file, names)
            raise rospy.ROSInitException("unknown waypoint")
        self._goal_xy = geo.latlon_to_enu(entry["lat"], entry["lon"], self._datum)
        self._bearing = math.radians(entry.get("approach_bearing_deg", 0.0))
        self._waypoint_name = wanted

        self._fsm = HandoffFSM(
            transit_timeout_sec=rospy.get_param("~transit_timeout_sec", 300.0),
            mission_timeout_sec=rospy.get_param("~mission_timeout_sec", 1800.0),
        )

        self._gps_state = None
        self._vision_done = False
        self._applied = (None, None)     # last (gps_enabled, vision_enabled)

        self._goal_pub = rospy.Publisher(
            "/gps_nav_node/goal_pose", PoseStamped, queue_size=1, latch=True)
        self._status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        rospy.Subscriber("/gps_nav_node/status", String, self._gps_status_cb, queue_size=1)
        rospy.Subscriber("/vision_nav_node/mission_done", Bool,
                         self._mission_done_cb, queue_size=1)

        self._gps_enable = self._wait_for_service("/gps_nav_node/set_enabled")
        self._vision_enable = self._wait_for_service("/vision_nav_node/set_enabled")

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("---- mission supervisor ----")
        rospy.loginfo("waypoint: %s -> map (%.2f, %.2f), approach bearing %.0f deg",
                      wanted, self._goal_xy[0], self._goal_xy[1],
                      math.degrees(self._bearing))
        rospy.loginfo("timeouts: transit %.0f s, row mission %.0f s",
                      self._fsm.transit_timeout_sec, self._fsm.mission_timeout_sec)
        rospy.loginfo("----------------------------")

        self._apply(self._fsm.start(rospy.get_time()))
        rospy.Timer(rospy.Duration(0.5), self._tick)

    @staticmethod
    def _wait_for_service(name):
        rospy.loginfo("waiting for %s ...", name)
        rospy.wait_for_service(name)
        return rospy.ServiceProxy(name, SetBool)

    # ---- inputs ----------------------------------------------------------

    def _gps_status_cb(self, msg):
        """Reduce the follower's status line to the two words this cares about.

        The status string is written for a person; only ARRIVED and FAILED are
        load-bearing here, and both are matched as whole leading words so a
        future wording change cannot silently start matching something else.
        """
        head = msg.data.split()[0] if msg.data else ""
        if head == GPS_ARRIVED:
            self._gps_state = GPS_ARRIVED
        elif head.startswith(GPS_FAILED):
            self._gps_state = GPS_FAILED
        else:
            self._gps_state = head

    def _mission_done_cb(self, msg):
        self._vision_done = bool(msg.data)

    # ---- the loop --------------------------------------------------------

    def _tick(self, _event):
        tick = self._fsm.update(rospy.get_time(), gps_state=self._gps_state,
                                vision_done=self._vision_done)
        self._apply(tick)

    def _apply(self, tick):
        desired = (tick.gps_enabled, tick.vision_enabled)
        if desired != self._applied:
            rospy.loginfo("%s: gps=%s vision=%s%s", tick.state,
                          "on" if tick.gps_enabled else "off",
                          "on" if tick.vision_enabled else "off",
                          ("  (%s)" % tick.reason) if tick.reason else "")
            # ⚠ Off before on, always. The FSM names which one, so this cannot
            # be got wrong by reordering two lines here.
            if tick.disable_first == "gps":
                self._set(self._gps_enable, "gps", tick.gps_enabled)
                self._set(self._vision_enable, "vision", tick.vision_enabled)
            else:
                self._set(self._vision_enable, "vision", tick.vision_enabled)
                self._set(self._gps_enable, "gps", tick.gps_enabled)
            self._applied = desired

        if tick.send_goal:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x, goal.pose.position.y = self._goal_xy
            goal.pose.orientation.z = math.sin(self._bearing / 2.0)
            goal.pose.orientation.w = math.cos(self._bearing / 2.0)
            self._goal_pub.publish(goal)
            rospy.loginfo("goal sent: %s", self._waypoint_name)

        self._status_pub.publish(String(
            data="%s%s" % (tick.state, (" -- " + tick.reason) if tick.reason else "")))

        if tick.done:
            rospy.loginfo("SEQUENCE COMPLETE: transit + row mission finished.")
            rospy.signal_shutdown("mission complete")
        elif tick.failed:
            rospy.logerr("SEQUENCE FAILED: %s", tick.reason)
            rospy.signal_shutdown("mission failed")

    def _set(self, proxy, name, enabled):
        try:
            proxy(enabled)
        except rospy.ServiceException as exc:
            rospy.logerr("could not %s %s nav: %s",
                         "enable" if enabled else "disable", name, exc)

    def _on_shutdown(self):
        """Leave both nodes stopped, whatever happened.

        A supervisor that exits with one of them still enabled has handed an
        unattended robot to a node with no one watching it.
        """
        for proxy, name in ((self._gps_enable, "gps"), (self._vision_enable, "vision")):
            try:
                proxy(False)
            except Exception:                       # shutting down; best effort
                pass


def main():
    node = MissionSupervisor()
    del node
    rospy.spin()


if __name__ == "__main__":
    main()
