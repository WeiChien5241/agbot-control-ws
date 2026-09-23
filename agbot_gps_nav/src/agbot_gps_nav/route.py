"""Multi-waypoint routes: a list of lat/lon points, walked in order.

Pure python -- no rospy. Two pieces:

  Route        the list the operator edits: append (a mapviz click), undo,
               clear, save/load as YAML. Stored in LAT/LON, never map x/y, so a
               saved route survives a datum change being noticed and fixed --
               only its conversion moves, not the file.
  RouteRunner  walks a list of map-frame points with one WaypointFollower and
               exposes the SAME update() 4-tuple, so gps_nav_node drives a
               route exactly as it drives a single goal.

THE LEGS

  Every point but the last is PASS-THROUGH: the robot does not stop, slow down
  or re-bootstrap its heading there. It counts as reached when the robot is
  within `pass_radius` of it OR has crossed the plane through it perpendicular
  to the incoming leg. ⚠ The plane is not optional: a radius alone is exactly
  the failure GPS_plan.md 3.2 records -- a near miss never enters it, the
  follower keeps chasing the point, and the robot orbits it or drives off.

  The LAST point is an ordinary goal. With an approach bearing it gets the full
  staging -> ALIGN -> on-axis approach, which is what a row entrance needs, and
  ARRIVED latches only there. A bearing on an intermediate point is ignored:
  aligning to it would mean stopping, which is what pass-through is not.

  The heading bootstrap runs on the FIRST leg only. After it the estimate has
  been verified, and the dual EKF keeps correcting yaw for as long as the robot
  moves, which on a route it does not stop doing.
"""

import collections
import math

import yaml

from agbot_gps_nav import geo
from agbot_gps_nav.waypoint_follower import (
    STATE_ARRIVED, STATE_FAILED, STATE_HEADING_INIT, STATE_IDLE,
    WaypointFollower,
)

# `bearing_deg` is ENU degrees, CCW from east -- the same convention as
# `approach_bearing_deg` in the generated waypoints file, so a corridor entry
# can be copied into a route unchanged.
Waypoint = collections.namedtuple("Waypoint", "lat lon bearing_deg name")
Waypoint.__new__.__defaults__ = (None, None)


class Route(object):
    """The operator's editable list of waypoints."""

    def __init__(self, waypoints=None):
        self.waypoints = list(waypoints or [])

    def __len__(self):
        return len(self.waypoints)

    def append(self, lat, lon, bearing_deg=None, name=None):
        if name is None:
            name = "wp%d" % (len(self.waypoints) + 1)
        self.waypoints.append(Waypoint(float(lat), float(lon),
                                       None if bearing_deg is None
                                       else float(bearing_deg), name))

    def undo(self):
        """Drop the last point. Returns it, or None if the route was empty."""
        return self.waypoints.pop() if self.waypoints else None

    def clear(self):
        self.waypoints = []

    def to_map(self, datum):
        """[(x, y, bearing_rad_or_None)] in navsat_transform's map frame."""
        out = []
        for wp in self.waypoints:
            x, y = geo.latlon_to_map(wp.lat, wp.lon, datum)
            out.append((x, y, None if wp.bearing_deg is None
                        else math.radians(wp.bearing_deg)))
        return out

    # ---- persistence -----------------------------------------------------
    #
    # The schema is the generated waypoints file's: a `waypoints:` list of
    # {name, lat, lon, approach_bearing_deg?}. That file loads as a route, and
    # a saved route can be read by anything that reads that file.

    def to_dict(self, datum=None):
        spec = {}
        if datum is not None:
            spec["datum"] = [float(datum[0]), float(datum[1]), 0.0]
        spec["waypoints"] = []
        for wp in self.waypoints:
            entry = {"name": wp.name, "lat": wp.lat, "lon": wp.lon}
            if wp.bearing_deg is not None:
                entry["approach_bearing_deg"] = wp.bearing_deg
            spec["waypoints"].append(entry)
        return spec

    @classmethod
    def from_dict(cls, spec):
        route = cls()
        for i, entry in enumerate((spec or {}).get("waypoints") or []):
            route.append(entry["lat"], entry["lon"],
                         entry.get("approach_bearing_deg"),
                         entry.get("name", "wp%d" % (i + 1)))
        return route

    def save(self, path, datum=None):
        with open(path, "w") as handle:
            handle.write("# GPS route: walked in order, last point is the "
                         "goal.\n# bearing = approach_bearing_deg, ENU degrees "
                         "CCW from east (90 = north).\n")
            yaml.safe_dump(self.to_dict(datum), handle, default_flow_style=False,
                           sort_keys=False)

    @classmethod
    def load(cls, path):
        with open(path) as handle:
            return cls.from_dict(yaml.safe_load(handle))


class RouteRunner(object):
    """Walk map-frame points in order with one WaypointFollower.

    `update(pose, now)` returns (linear_x, angular_z, state, done) exactly like
    WaypointFollower.update(); `done` is True only once the LAST point is
    reached, so a supervisor waiting for ARRIVED waits for the whole route.
    """

    def __init__(self, follower=None, pass_radius=1.0):
        self.follower = follower if follower is not None else WaypointFollower()
        self.pass_radius = float(pass_radius)
        self._points = []
        self._leg = 0
        self._leg_start = None

    # ---- control ---------------------------------------------------------

    def start(self, points):
        """Begin a route. `points` is [(x, y, bearing_rad_or_None)], map frame.

        A one-point route is exactly a single goal, bootstrap and all.
        """
        if not points:
            raise ValueError("a route needs at least one point")
        self._points = [(float(p[0]), float(p[1]),
                         None if len(p) < 3 or p[2] is None else float(p[2]))
                        for p in points]
        self._leg = 0
        self._leg_start = None
        self._set_leg(first=True)

    def stop(self):
        self._points = []
        self._leg = 0
        self._leg_start = None
        self.follower.clear_goal()

    # ---- telemetry -------------------------------------------------------

    @property
    def active(self):
        return bool(self._points) and self.follower.state not in (
            STATE_IDLE, STATE_ARRIVED, STATE_FAILED)

    @property
    def leg(self):
        """1-based index of the point being driven to (0 with no route)."""
        return self._leg + 1 if self._points else 0

    @property
    def leg_count(self):
        return len(self._points)

    @property
    def points(self):
        return list(self._points)

    @property
    def is_last_leg(self):
        return self._leg == len(self._points) - 1

    # ---- the tick --------------------------------------------------------

    def update(self, pose, now=None):
        if not self._points:
            return self.follower.update(pose, now)

        if pose is not None and not self.is_last_leg:
            if self._leg_start is None:
                self._leg_start = (float(pose[0]), float(pose[1]))
            # ⚠ Never advance out of the heading bootstrap: it drives straight
            # in whatever direction the robot faces, can sail past point 1 while
            # doing it, and the next leg is told NOT to bootstrap -- so leaving
            # early would hand the rest of the route an unverified heading.
            if (self.follower.state != STATE_HEADING_INIT
                    and self._passed(pose)):
                self._leg += 1
                self._leg_start = (float(pose[0]), float(pose[1]))
                self._set_leg(first=False)

        linear_x, angular_z, state, done = self.follower.update(pose, now)
        # An intermediate point reached by the follower's own tolerance before
        # the pass test saw it (pass_radius set below goal_tolerance): move on
        # rather than latching ARRIVED in the middle of a route.
        if state == STATE_ARRIVED and not self.is_last_leg:
            self._leg += 1
            self._leg_start = (float(pose[0]), float(pose[1]))
            self._set_leg(first=False)
            linear_x, angular_z, state, done = self.follower.update(pose, now)
        return linear_x, angular_z, state, done

    # ---- internals -------------------------------------------------------

    def _set_leg(self, first):
        x, y, bearing = self._points[self._leg]
        last = self.is_last_leg
        self.follower.set_goal(
            (x, y),
            approach_bearing=bearing if last else None,
            heading_init=first,
            slow_on_arrival=last)

    def _passed(self, pose):
        """Within pass_radius of the current point, or across its plane."""
        x, y, _ = self._points[self._leg]
        px, py = float(pose[0]), float(pose[1])
        if math.hypot(px - x, py - y) <= self.pass_radius:
            return True
        # The plane through the point, perpendicular to the leg as it was
        # INTENDED: from the previous point, or for leg 1 from where the robot
        # started. Past it along that direction means passed, however wide.
        if self._leg > 0:
            ox, oy = self._points[self._leg - 1][:2]
        else:
            ox, oy = self._leg_start
        dx, dy = x - ox, y - oy
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return True
        along = ((px - x) * dx + (py - y) * dy) / length
        return along >= 0.0
