"""Tests for the WGS84 <-> ENU tangent-plane conversion.

The load-bearing tests here check the module against an INDEPENDENT formula
rather than against itself. A round-trip test passes even when both directions
share the same wrong scale factor, so it proves invertibility and says nothing
about correctness.

The reference is Vincenty's inverse geodesic: iterative, ellipsoidal, and
structurally unrelated to a tangent plane, so an error in either shows up as
disagreement. A SPHERICAL reference (haversine) was tried first and rejected --
sphere-vs-ellipsoid disagreement is ~0.4 %, which swamps the ~1e-6 we are
actually trying to detect and would have hidden a wrong-radius bug completely.

The measured agreement quoted in geo.py's docstring comes from
`test_accuracy_degrades_with_distance_from_datum` below.
"""

import math

import pytest

from agbot_gps_nav import geo

# Purdue ACRE, approximately. Only the latitude matters to these tests (it sets
# the radii of curvature); the longitude just has to be consistent.
DATUM = (40.4693, -86.9915)

# A high latitude, where R_m and R_n diverge most and cos(lat) is small. If a
# single-radius bug exists, it shows up here first.
ARCTIC = (78.2232, 15.6469)

def vincenty_m(lat1, lon1, lat2, lon2):
    """Vincenty's inverse geodesic distance on the WGS84 ellipsoid, metres.

    An independent reference implementation, kept in the test file on purpose:
    geo.py must never import it, or the cross-check becomes circular.
    """
    a = 6378137.0
    f = 1 / 298.257223563
    b = (1 - f) * a
    u1 = math.atan((1 - f) * math.tan(math.radians(lat1)))
    u2 = math.atan((1 - f) * math.tan(math.radians(lat2)))
    su1, cu1 = math.sin(u1), math.cos(u1)
    su2, cu2 = math.sin(u2), math.cos(u2)
    lon_diff = math.radians(lon2 - lon1)

    lam = lon_diff
    for _ in range(200):
        sin_lam, cos_lam = math.sin(lam), math.cos(lam)
        sin_sigma = math.hypot(cu2 * sin_lam, cu1 * su2 - su1 * cu2 * cos_lam)
        if sin_sigma == 0:
            return 0.0                      # coincident points
        cos_sigma = su1 * su2 + cu1 * cu2 * cos_lam
        sigma = math.atan2(sin_sigma, cos_sigma)
        sin_alpha = cu1 * cu2 * sin_lam / sin_sigma
        cos_sq_alpha = 1 - sin_alpha * sin_alpha
        cos_2sigma_m = (cos_sigma - 2 * su1 * su2 / cos_sq_alpha) if cos_sq_alpha else 0.0
        c = f / 16 * cos_sq_alpha * (4 + f * (4 - 3 * cos_sq_alpha))
        lam_prev = lam
        lam = lon_diff + (1 - c) * f * sin_alpha * (
            sigma + c * sin_sigma * (cos_2sigma_m + c * cos_sigma * (-1 + 2 * cos_2sigma_m ** 2))
        )
        if abs(lam - lam_prev) < 1e-14:
            break
    else:                                   # pragma: no cover - antipodal only
        raise AssertionError("Vincenty did not converge")

    u_sq = cos_sq_alpha * (a * a - b * b) / (b * b)
    big_a = 1 + u_sq / 16384 * (4096 + u_sq * (-768 + u_sq * (320 - 175 * u_sq)))
    big_b = u_sq / 1024 * (256 + u_sq * (-128 + u_sq * (74 - 47 * u_sq)))
    d_sigma = big_b * sin_sigma * (
        cos_2sigma_m
        + big_b / 4 * (
            cos_sigma * (-1 + 2 * cos_2sigma_m ** 2)
            - big_b / 6 * cos_2sigma_m * (-3 + 4 * sin_sigma ** 2) * (-3 + 4 * cos_2sigma_m ** 2)
        )
    )
    return b * big_a * (sigma - d_sigma)


def test_datum_maps_to_the_origin():
    east, north = geo.latlon_to_enu(DATUM[0], DATUM[1], DATUM)
    assert east == pytest.approx(0.0, abs=1e-9)
    assert north == pytest.approx(0.0, abs=1e-9)


def test_north_is_positive_y_and_east_is_positive_x():
    """Sign convention. Getting either of these backwards sends the robot
    exactly the wrong way, and nothing downstream would notice."""
    east, north = geo.latlon_to_enu(DATUM[0] + 0.001, DATUM[1], DATUM)
    assert north > 0 and east == pytest.approx(0.0, abs=1e-9)

    east, north = geo.latlon_to_enu(DATUM[0], DATUM[1] + 0.001, DATUM)
    assert east > 0 and north == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("datum", [DATUM, ARCTIC, (0.0, 0.0), (-33.87, 151.21)])
@pytest.mark.parametrize("dlat,dlon", [
    (0.0, 0.0005), (0.0005, 0.0), (0.0005, 0.0005), (-0.002, 0.001),
])
def test_distance_agrees_with_vincenty_at_field_scale(datum, dlat, dlon):
    """Within ~250 m of the datum -- the whole operating envelope -- the tangent
    plane and the true geodesic agree to 1e-5 relative, i.e. millimetres."""
    lat, lon = datum[0] + dlat, datum[1] + dlon
    east, north = geo.latlon_to_enu(lat, lon, datum)
    ours = math.hypot(east, north)
    theirs = vincenty_m(datum[0], datum[1], lat, lon)
    assert theirs < 300.0, "this case is meant to stay at field scale"
    assert ours == pytest.approx(theirs, rel=1e-5)


def test_cos_lat_on_the_east_axis_is_not_omitted():
    """Dropping cos(lat) is the classic bug and it is invisible at the datum
    (both directions share it) and invisible near the equator. At 78 N it is a
    5x error on a pure-east move, so pin it there specifically."""
    lat, lon = ARCTIC[0], ARCTIC[1] + 0.01
    east, north = geo.latlon_to_enu(lat, lon, ARCTIC)
    assert east == pytest.approx(vincenty_m(ARCTIC[0], ARCTIC[1], lat, lon), rel=1e-5)
    assert abs(north) < 1e-6

    naive_no_cos = math.radians(0.01) * geo.radii_of_curvature(ARCTIC[0])[1]
    assert naive_no_cos > 4.0 * east      # the bug this test exists to catch


def test_accuracy_degrades_with_distance_from_datum():
    """The numbers quoted in geo.py's docstring. They are a real limit, not a
    formality: by ~1.4 km the projection error is already comparable to RTK's
    2 cm, so the answer to a bigger site is a nearer datum."""
    # measured: 1.5 mm / 37 mm / 936 mm. Ceilings sit just above, so a change
    # that quietly degrades the projection fails here rather than in a field.
    for dlat, dlon, ceiling_m in [(0.002, 0.002, 0.003),      # ~280 m
                                  (0.01, -0.01, 0.05),        # ~1.4 km
                                  (0.05, 0.05, 1.2)]:         # ~7 km
        lat, lon = DATUM[0] + dlat, DATUM[1] + dlon
        east, north = geo.latlon_to_enu(lat, lon, DATUM)
        error = abs(math.hypot(east, north) - vincenty_m(DATUM[0], DATUM[1], lat, lon))
        assert error < ceiling_m


def test_east_and_north_use_different_radii():
    """R_m != R_n away from the equator. Using one for both is a real bug that
    a round-trip test cannot see, because it cancels."""
    r_m, r_n = geo.radii_of_curvature(DATUM[0])
    assert r_m != r_n
    assert abs(r_n - r_m) / r_m > 1e-3     # ~0.39 % at this latitude


@pytest.mark.parametrize("datum", [DATUM, ARCTIC, (0.0, 0.0), (-33.87, 151.21)])
@pytest.mark.parametrize("east,north", [
    (0.0, 0.0), (10.0, 0.0), (0.0, 10.0), (-250.0, 400.0), (1000.0, -1000.0),
])
def test_round_trip_is_exact(datum, east, north):
    lat, lon = geo.enu_to_latlon(east, north, datum)
    back_e, back_n = geo.latlon_to_enu(lat, lon, datum)
    assert back_e == pytest.approx(east, abs=1e-6)
    assert back_n == pytest.approx(north, abs=1e-6)


def test_datum_may_carry_extra_elements():
    """rosparam gives the datum as [lat, lon, yaw]; it must pass straight in
    without the caller having to slice it."""
    three = (DATUM[0], DATUM[1], 0.0)
    assert geo.latlon_to_enu(40.47, -86.99, three) == geo.latlon_to_enu(40.47, -86.99, DATUM)


def test_string_datum_from_yaml_is_accepted():
    """A yaml value can arrive as a string; float() it rather than fail at the
    first arithmetic op with an unreadable TypeError."""
    assert geo.latlon_to_enu("40.47", "-86.99", ("40.4693", "-86.9915")) == \
        geo.latlon_to_enu(40.47, -86.99, DATUM)


def test_bearing_is_ccw_from_east_not_clockwise_from_north():
    """The compass convention would give pi/2 for due east and 0 for due north.
    We want the REP-103 one, so that yaw - bearing is a heading error."""
    assert geo.bearing_to((0, 0), (1, 0)) == pytest.approx(0.0)
    assert geo.bearing_to((0, 0), (0, 1)) == pytest.approx(math.pi / 2)
    assert geo.bearing_to((0, 0), (-1, 0)) == pytest.approx(math.pi)
    assert geo.bearing_to((0, 0), (0, -1)) == pytest.approx(-math.pi / 2)


def test_distance_is_symmetric_and_zero_at_a_point():
    assert geo.distance((3.0, 4.0), (3.0, 4.0)) == 0.0
    assert geo.distance((0, 0), (3, 4)) == pytest.approx(5.0)
    assert geo.distance((3, 4), (0, 0)) == pytest.approx(5.0)


@pytest.mark.parametrize("raw,expected", [
    (0.0, 0.0),
    (math.pi / 2, math.pi / 2),
    (3 * math.pi, -math.pi),
    (-3 * math.pi, -math.pi),
    (2 * math.pi + 0.1, 0.1),
])
def test_wrap_angle(raw, expected):
    assert geo.wrap_angle(raw) == pytest.approx(expected)


def test_wrap_angle_takes_the_short_way_across_the_branch_cut():
    """Facing 179 deg with a goal at -179 deg is a 2 deg error, not 358."""
    err = geo.wrap_angle(math.radians(-179) - math.radians(179))
    assert abs(err) == pytest.approx(math.radians(2), abs=1e-9)
