"""Georef: lat/lon <-> a saved map's frame, fitted from paired fixes."""
import math

import numpy as np
import pytest

from agbot_gps_nav.geo import enu_to_latlon
from agbot_nav.georef import (
    Georef, GeorefError, fit_georef, fit_rigid_2d, thin_by_distance,
)

DATUM = (40.4222, -86.9203)          # near the 10/6 RTK test spot


def _lap(n=120):
    """A 10 m x 6 m loop in ENU (a mapping lap)."""
    t = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return np.stack([5.0 * np.cos(t), 3.0 * np.sin(t)], axis=1)


def _to_map(enu, yaw, tx, ty):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([c * enu[:, 0] - s * enu[:, 1] + tx,
                     s * enu[:, 0] + c * enu[:, 1] + ty], axis=1)


def _latlon(enu, datum=DATUM):
    return np.array([enu_to_latlon(e, n, datum) for e, n in enu])


def test_rigid_fit_recovers_exact_transform():
    src = _lap()
    yaw, tx, ty = fit_rigid_2d(src, _to_map(src, 1.2, -7.9, 3.4))
    assert yaw == pytest.approx(1.2, abs=1e-12)
    assert (tx, ty) == (pytest.approx(-7.9), pytest.approx(3.4))


def test_fit_georef_recovers_transform_and_round_trips():
    enu = _lap()
    yaw, tx, ty = math.radians(-91.2), -7.92, -1.30
    g, res, kept = fit_georef(_latlon(enu), _to_map(enu, yaw, tx, ty), datum=DATUM)
    assert g.yaw == pytest.approx(yaw, abs=1e-9)
    assert (g.tx, g.ty) == (pytest.approx(tx, abs=1e-6), pytest.approx(ty, abs=1e-6))
    assert g.rms < 1e-6 and kept.all()
    lat, lon = g.map_to_latlon(*g.latlon_to_map(*_latlon(enu[:1])[0]))
    assert (lat, lon) == (pytest.approx(_latlon(enu[:1])[0][0], abs=1e-10),
                          pytest.approx(_latlon(enu[:1])[0][1], abs=1e-10))


def test_default_datum_is_first_fix_and_still_maps_correctly():
    enu = _lap() + [30.0, -12.0]
    map_xy = _to_map(enu, 0.4, 1.0, 2.0)
    g, _, _ = fit_georef(_latlon(enu), map_xy)          # datum = first fix
    lat, lon = _latlon(enu[5:6])[0]
    # 0.1 mm: two tangent planes 30 m apart differ by micrometres
    assert g.latlon_to_map(lat, lon) == (pytest.approx(map_xy[5][0], abs=1e-4),
                                         pytest.approx(map_xy[5][1], abs=1e-4))


def test_noise_gives_rms_near_noise_and_small_yaw_error():
    rng = np.random.default_rng(0)
    enu = _lap(300)
    map_xy = _to_map(enu, 0.7, 2.0, -1.0) + rng.normal(0, 0.02, enu.shape)
    g, _, _ = fit_georef(_latlon(enu), map_xy, datum=DATUM)
    assert 0.015 < g.rms < 0.035
    assert abs(math.degrees(g.yaw - 0.7)) < 0.2


def test_a_multipath_jump_is_rejected_not_fitted():
    enu = _lap()
    map_xy = _to_map(enu, 0.3, 0.0, 0.0)
    ll = _latlon(enu)
    ll[10] = enu_to_latlon(enu[10, 0] + 2.0, enu[10, 1], DATUM)   # 2 m jump
    g, res, kept = fit_georef(ll, map_xy, datum=DATUM)
    assert not kept[10] and kept.sum() == len(enu) - 1
    assert res[10] == pytest.approx(2.0, abs=0.01)
    assert g.rms < 1e-6 and g.yaw == pytest.approx(0.3, abs=1e-9)


def test_short_baseline_is_refused():
    enu = np.stack([np.linspace(0, 1.0, 20), np.zeros(20)], axis=1)
    with pytest.raises(GeorefError, match="span"):
        fit_georef(_latlon(enu), enu, datum=DATUM)


def test_a_skewed_map_shows_up_in_the_residual():
    """A rigid fit cannot absorb a sheared map; the RMS must say so."""
    enu = _lap()
    sheared = enu.copy()
    sheared[:, 0] += 0.08 * enu[:, 1]                 # 8 % shear
    g, _, _ = fit_georef(_latlon(enu), sheared, datum=DATUM, reject=10.0)
    assert g.rms > 0.08


def test_compass_bearing_conversions():
    g = Georef(DATUM, math.radians(30.0), 0, 0)
    # due east (compass 90) is ENU yaw 0 -> map yaw = +30 deg
    assert math.degrees(g.compass_to_map_yaw(90.0)) == pytest.approx(30.0)
    # due north (compass 0) is ENU yaw 90 -> map 120
    assert math.degrees(g.compass_to_map_yaw(0.0)) == pytest.approx(120.0)
    assert g.map_yaw_to_compass(g.compass_to_map_yaw(217.0)) == pytest.approx(217.0)


def test_dict_round_trip():
    g = Georef(DATUM, 0.5, 1.0, -2.0, rms=0.03, max_err=0.08, n=40, span=12.0)
    h = Georef.from_dict(g.to_dict())
    assert (h.datum, h.yaw, h.tx, h.ty, h.rms, h.n) == (g.datum, 0.5, 1.0, -2.0, 0.03, 40)


def test_thin_by_distance_collapses_a_parked_stretch():
    pts = [(0.0, 0.0)] * 300 + [(0.1 * i, 0.0) for i in range(1, 31)]
    keep = thin_by_distance(pts, 0.25)
    assert keep[0] == 0 and len(keep) == 1 + 10        # 0.3, 0.6, ... 3.0
