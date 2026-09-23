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

⚠ TWO FRAMES, AND THEY ARE NOT THE SAME NUMBERS (measured 2026-09-22).
`latlon_to_enu` is TRUE ground metres. `navsat_transform`'s map frame -- the
one the EKF, and therefore the robot, lives in -- is NOT: robot_localization
2.7.7 has no `use_local_cartesian`, so it goes through UTM, which scales every
distance by k0 = 0.9996. Against `/fromLL` at this datum:

  distance from datum     latlon_to_enu vs /fromLL
  50 m                    2.0 cm
  200 m                   8.0 cm
  500 m                   20 cm
  1 km                    40 cm

A 0.75 m corridor leaves ~16 cm per side, so at field scale a goal converted
with the tangent plane lands visibly off the row axis. Hence:

  latlon_to_map / map_to_latlon   the navsat_transform map frame. Use these for
                                  ANYTHING the robot drives to or reports.
  latlon_to_enu / enu_to_latlon   the true-metre tangent plane. This is also
                                  exactly hector's GPS plugin model, so it is
                                  the right one for GAZEBO WORLD coordinates
                                  (rows_to_waypoints.py) and nothing else.
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


UTM_K0 = 0.9996


def _utm_zone(lon_deg):
    return int((float(lon_deg) + 180.0) // 6.0) + 1


def latlon_to_utm(lat_deg, lon_deg, zone):
    """(lat, lon) -> (easting, northing) in UTM `zone` (northern hemisphere).

    Snyder's series (USGS PP 1395, eqs. 8-9 to 8-13) -- the same formulation
    as robot_localization's legacy LLtoUTM. Within a zone it agrees with the
    GeographicLib projection navsat_transform now uses to well under a
    millimetre over a field; only DIFFERENCES about a datum are used here, so
    false easting/northing never matter.
    """
    lat = math.radians(float(lat_deg))
    lon0 = math.radians((zone - 1) * 6.0 - 180.0 + 3.0)
    lon = math.radians(float(lon_deg))
    e2 = WGS84_E2
    ep2 = e2 / (1.0 - e2)
    sin_lat, cos_lat, tan_lat = math.sin(lat), math.cos(lat), math.tan(lat)
    n = WGS84_A / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = ep2 * cos_lat * cos_lat
    a = cos_lat * (lon - lon0)
    m = WGS84_A * ((1.0 - e2 / 4.0 - 3.0 * e2 ** 2 / 64.0 - 5.0 * e2 ** 3 / 256.0) * lat
                   - (3.0 * e2 / 8.0 + 3.0 * e2 ** 2 / 32.0 + 45.0 * e2 ** 3 / 1024.0)
                   * math.sin(2.0 * lat)
                   + (15.0 * e2 ** 2 / 256.0 + 45.0 * e2 ** 3 / 1024.0) * math.sin(4.0 * lat)
                   - (35.0 * e2 ** 3 / 3072.0) * math.sin(6.0 * lat))
    easting = UTM_K0 * n * (a + (1.0 - t + c) * a ** 3 / 6.0
                            + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ep2)
                            * a ** 5 / 120.0) + 500000.0
    northing = UTM_K0 * (m + n * tan_lat * (
        a * a / 2.0 + (5.0 - t + 9.0 * c + 4.0 * c * c) * a ** 4 / 24.0
        + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ep2) * a ** 6 / 720.0))
    return easting, northing


def _map_frame(datum):
    """(zone, datum easting/northing, cos/sin of the meridian convergence).

    navsat_transform rotates the UTM grid by the convergence at the datum so
    the map frame's +y is TRUE north (measured: a point 1 km due north of the
    datum comes back with x = 0.000, where a raw UTM grid would give -4 cm).
    The convergence is taken numerically -- the grid direction of a small
    northward step -- so it cannot disagree with the projection in sign.
    """
    lat0, lon0 = float(datum[0]), float(datum[1])
    zone = _utm_zone(lon0)
    e0, n0 = latlon_to_utm(lat0, lon0, zone)
    e1, n1 = latlon_to_utm(lat0 + 1e-4, lon0, zone)
    gamma = math.atan2(e1 - e0, n1 - n0)
    return zone, e0, n0, math.cos(gamma), math.sin(gamma)


def latlon_to_map(lat_deg, lon_deg, datum):
    """(lat, lon) -> (x, y) in navsat_transform's map frame about `datum`.

    Matches `/fromLL` to 0.1 mm out to 2 km (test_geo.py pins it against values
    recorded from the running node). This, not `latlon_to_enu`, is where the
    robot actually goes.
    """
    zone, e0, n0, cg, sg = _map_frame(datum)
    e, n = latlon_to_utm(lat_deg, lon_deg, zone)
    de, dn = e - e0, n - n0
    return cg * de - sg * dn, sg * de + cg * dn


def map_to_latlon(x, y, datum):
    """Inverse of `latlon_to_map`, by fixed-point iteration.

    The map frame is within 0.04 % of the tangent plane, so starting from the
    tangent-plane inverse and correcting by the forward residual converges to
    sub-millimetre in two or three steps. Cheaper to trust than a second
    hand-typed series.
    """
    x, y = float(x), float(y)
    ex, ny = x, y
    lat, lon = enu_to_latlon(ex, ny, datum)
    for _ in range(8):
        fx, fy = latlon_to_map(lat, lon, datum)
        dx, dy = x - fx, y - fy
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            break
        ex, ny = ex + dx, ny + dy
        lat, lon = enu_to_latlon(ex, ny, datum)
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
