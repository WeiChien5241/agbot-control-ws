"""WGS84 <-> local ENU conversion about a fixed datum.

Pure python: `math` only. No rospy, no pyproj, no geodesy -- none of those are
installed on the Noetic box and none of them are needed at field scale.

WHY NOT UTM. `navsat_transform_node` internally goes through UTM, and that is
fine for it, but a UTM zone boundary or a band letter is one more thing that can
be silently wrong. Over a single field the honest and much simpler model is a
local tangent plane: fix a datum, treat the Earth as locally flat, and scale
degrees to metres with the WGS84 radii of curvature AT THE DATUM LATITUDE.

The two radii are not the same and using one for both axes is the classic bug:

  R_m  meridional (north-south), shrinks the further you are from the equator
  R_n  prime vertical (east-west), and the east axis picks up a cos(lat) on top

At Purdue's latitude R_m and R_n differ by 0.39 %, so using R_n for the north
axis would put a 10 m northward move 3.9 cm out -- twice the RTK accuracy we are
paying for. The distinction is not academic.

ACCURACY, MEASURED against Vincenty's inverse geodesic (see test_geo.py, which
is where these numbers come from -- they are not derived):

  distance from datum      error vs Vincenty
  280 m                    1.5 mm
  1.4 km                   3.7 cm
  7 km                     0.94 m

So: negligible against RTK's ~2 cm anywhere within a few hundred metres of the
datum, COMPARABLE TO RTK ERROR by ~1.4 km, and unusable by ~7 km. A field fits
in the first row with room to spare. If a site ever needs more than ~500 m of
reach, the fix is a nearer datum, not a better projection.

CONVENTION. ENU, matching REP-103 and `robot_localization`'s map frame:
  east  = +x, north = +y, and yaw is measured CCW from east.
That is deliberately NOT the compass convention (CW from north); see
`bearing_to` for the one place the difference bites.
"""

import math

# WGS84 defining constants.
WGS84_A = 6378137.0                 # semi-major axis (m)
WGS84_F = 1.0 / 298.257223563       # flattening
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)  # first eccentricity squared


def radii_of_curvature(lat_deg):
    """Return (R_meridional, R_prime_vertical) in metres at this latitude.

    R_m governs the north axis, R_n (times cos lat) the east axis.
    """
    lat = math.radians(lat_deg)
    sin_lat = math.sin(lat)
    w2 = 1.0 - WGS84_E2 * sin_lat * sin_lat
    w = math.sqrt(w2)
    r_n = WGS84_A / w
    r_m = WGS84_A * (1.0 - WGS84_E2) / (w2 * w)
    return r_m, r_n


def latlon_to_enu(lat_deg, lon_deg, datum):
    """(lat, lon) -> (east, north) in metres relative to `datum`.

    `datum` is (lat_deg, lon_deg); extra elements (altitude, yaw) are ignored so
    a rosparam `datum: [lat, lon, yaw]` list can be passed straight through.
    """
    lat0, lon0 = float(datum[0]), float(datum[1])
    r_m, r_n = radii_of_curvature(lat0)
    north = math.radians(float(lat_deg) - lat0) * r_m
    east = math.radians(float(lon_deg) - lon0) * r_n * math.cos(math.radians(lat0))
    return east, north


def enu_to_latlon(east, north, datum):
    """(east, north) in metres relative to `datum` -> (lat, lon) in degrees.

    Exact inverse of `latlon_to_enu` -- both use the radii at the DATUM
    latitude, not at the point, which is what makes the pair invertible.
    """
    lat0, lon0 = float(datum[0]), float(datum[1])
    r_m, r_n = radii_of_curvature(lat0)
    lat = lat0 + math.degrees(float(north) / r_m)
    lon = lon0 + math.degrees(float(east) / (r_n * math.cos(math.radians(lat0))))
    return lat, lon


def bearing_to(from_xy, to_xy):
    """ENU bearing from one point to another, radians CCW from EAST.

    This is `atan2(dy, dx)`, i.e. the same convention as the robot's yaw, so it
    can be subtracted from yaw directly. It is NOT a compass bearing -- do not
    compare it against a heading quoted in degrees-clockwise-from-north without
    converting.
    """
    return math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0])


def distance(from_xy, to_xy):
    """Planar distance between two ENU points, metres."""
    return math.hypot(to_xy[0] - from_xy[0], to_xy[1] - from_xy[1])


def wrap_angle(a):
    """Wrap an angle to [-pi, pi).

    Every heading error in this package goes through here. Without it a robot
    facing 179 deg with a goal at -179 deg computes a 358 deg error and turns
    the long way round.
    """
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi
