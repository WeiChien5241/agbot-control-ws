"""Georeferencing a saved 2D map: the fixed transform between lat/lon and it.

Pure python + numpy. No rospy -- `scripts/fit_georef.py` reads the bag and
hands this module plain numbers.

WHY IT IS NEEDED. A SLAM map's frame is arbitrary: (0, 0) is where odometry
started (the robot BOOTED), +x is whichever way it faced. GPS is lat/lon. The
two agree up to one rigid 2D motion -- a rotation and an offset -- and once that
is known a surveyed row entrance becomes a map pose move_base can drive to, and
an RTK fix becomes an AMCL initial pose.

HOW. During a lap, pair each good RTK fix with where the map says the ANTENNA
was at that instant (map -> base_link from TF, plus the antenna's lever arm on
the robot), project the fixes onto a local east/north plane, and least-squares
fit `map = R(yaw) * enu + t` (2D Kabsch). No scale term: the LiDAR, odometry
and the plane are all true metres, so a scale would only soak up error that
the residual should show.

The plane is `agbot_gps_nav.geo.latlon_to_enu` (true ground metres) about a
datum stored WITH the fit, so a georef file is self-contained and does not
depend on gps_datum.yaml (still a TODO placeholder). ⚠ navsat_transform's map
frame is UTM, 0.9996x true distance; a pose from `agbot_gps_nav`'s EKF is NOT
an ENU point here (4 mm at 10 m, 20 cm at 500 m). Convert lat/lon, never its x/y.

THE RESIDUAL IS THE POINT. A rigid fit cannot bend, so a SLAM map that is
skewed or stretched (sim SLAM is: the parked Jackal slides) shows up as a
large RMS -- report it, do not hide it by adding parameters.
"""

import math

import numpy as np

from agbot_gps_nav.geo import enu_to_latlon, latlon_to_enu, wrap_angle

# A fit over a short baseline turns GPS noise into heading error: 2 cm of
# noise across 3 m is ~0.4 deg, across 0.5 m it is ~2.3 deg. Refuse below this.
MIN_SPAN_M = 3.0


class GeorefError(ValueError):
    """The pairs cannot support a trustworthy fit (too few, too short)."""


def fit_rigid_2d(src, dst):
    """Least-squares (yaw, tx, ty) with dst ~= R(yaw) @ src + t.

    `src`, `dst`: (N, 2) arrays of corresponding points, N >= 2.
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 2:
        raise GeorefError("src and dst must both be (N, 2), got %s and %s"
                          % (src.shape, dst.shape))
    if len(src) < 2:
        raise GeorefError("need at least 2 pairs, got %d" % len(src))
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    a, b = src - cs, dst - cd
    # 2D Kabsch in closed form: the angle that best rotates a onto b.
    s_cos = float(np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    s_sin = float(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]))
    yaw = math.atan2(s_sin, s_cos)
    c, s = math.cos(yaw), math.sin(yaw)
    tx = cd[0] - (c * cs[0] - s * cs[1])
    ty = cd[1] - (s * cs[0] + c * cs[1])
    return yaw, tx, ty


def _apply(yaw, tx, ty, pts):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = np.asarray(pts, dtype=float)
    return np.stack([c * pts[:, 0] - s * pts[:, 1] + tx,
                     s * pts[:, 0] + c * pts[:, 1] + ty], axis=1)


class Georef(object):
    """lat/lon <-> one saved map's frame.

    `yaw`, `tx`, `ty`: map = R(yaw) * enu + t, with enu about `datum`
    (lat_deg, lon_deg). `yaw` is therefore also "map yaw = ENU yaw + yaw".
    """

    def __init__(self, datum, yaw, tx, ty, rms=None, max_err=None, n=None,
                 span=None):
        self.datum = (float(datum[0]), float(datum[1]))
        self.yaw = float(yaw)
        self.tx = float(tx)
        self.ty = float(ty)
        self.rms = rms
        self.max_err = max_err
        self.n = n
        self.span = span

    # -- conversions ------------------------------------------------------
    def latlon_to_map(self, lat, lon):
        e, n = latlon_to_enu(lat, lon, self.datum)
        x, y = _apply(self.yaw, self.tx, self.ty, [[e, n]])[0]
        return float(x), float(y)

    def map_to_latlon(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = float(x) - self.tx, float(y) - self.ty
        e, n = c * dx + s * dy, -s * dx + c * dy
        return enu_to_latlon(e, n, self.datum)

    def compass_to_map_yaw(self, bearing_deg):
        """Compass bearing (deg CW from true north) -> map-frame yaw (rad)."""
        enu_yaw = math.radians(90.0 - float(bearing_deg))
        return wrap_angle(enu_yaw + self.yaw)

    def map_yaw_to_compass(self, yaw):
        enu_yaw = float(yaw) - self.yaw
        return (90.0 - math.degrees(enu_yaw)) % 360.0

    # -- (de)serialisation --------------------------------------------------
    def to_dict(self):
        d = {"datum": [self.datum[0], self.datum[1]],
             "yaw_rad": self.yaw, "x": self.tx, "y": self.ty}
        for key in ("rms", "max_err", "n", "span"):
            v = getattr(self, key)
            if v is not None:
                d[key] = v
        return d

    @classmethod
    def from_dict(cls, d):
        return cls(d["datum"], d["yaw_rad"], d["x"], d["y"], rms=d.get("rms"),
                   max_err=d.get("max_err"), n=d.get("n"), span=d.get("span"))


def thin_by_distance(points, min_step):
    """Indices of `points` keeping one every `min_step` metres of travel.

    A parked minute at 5 Hz is 300 fixes of the same spot; unthinned it would
    outweigh the whole drive and pin the fit's rotation to nothing.
    """
    keep, last = [], None
    for i, p in enumerate(points):
        if last is None or math.hypot(p[0] - last[0], p[1] - last[1]) >= min_step:
            keep.append(i)
            last = p
    return keep


def fit_georef(latlon, map_xy, datum=None, reject=0.5, min_span=MIN_SPAN_M):
    """Fit a Georef from paired fixes and map-frame antenna positions.

    `latlon`: (N, 2) of (lat_deg, lon_deg). `map_xy`: (N, 2) antenna in the map.
    `datum`: plane origin; default the first fix. `reject`: after a first
    fit, drop pairs whose residual exceeds max(reject, 3 * rms) and refit once
    (a GPS multipath jump or a TF hiccup should not tilt the whole map).

    Returns (Georef, residuals, kept_mask), residuals in metres for EVERY pair
    against the final fit.
    """
    latlon = np.asarray(latlon, dtype=float)
    map_xy = np.asarray(map_xy, dtype=float)
    if len(latlon) != len(map_xy):
        raise GeorefError("%d fixes vs %d map points" % (len(latlon), len(map_xy)))
    if len(latlon) < 3:
        raise GeorefError("need at least 3 pairs, got %d" % len(latlon))
    if datum is None:
        datum = (latlon[0, 0], latlon[0, 1])
    enu = np.array([latlon_to_enu(la, lo, datum) for la, lo in latlon])

    span = _span(enu)
    if span < min_span:
        raise GeorefError("the fixes span only %.2f m (need >= %.1f m): drive "
                          "farther, a short baseline turns GPS noise into "
                          "heading error" % (span, min_span))

    kept = np.ones(len(enu), dtype=bool)
    yaw, tx, ty = fit_rigid_2d(enu, map_xy)
    res = np.linalg.norm(_apply(yaw, tx, ty, enu) - map_xy, axis=1)
    rms = float(np.sqrt(np.mean(res ** 2)))
    limit = max(float(reject), 3.0 * rms)
    if np.any(res > limit):
        kept = res <= limit
        if kept.sum() < 3 or _span(enu[kept]) < min_span:
            raise GeorefError("outlier rejection (>%.2f m) left too little to "
                              "fit: the map and GPS disagree badly" % limit)
        yaw, tx, ty = fit_rigid_2d(enu[kept], map_xy[kept])
        res = np.linalg.norm(_apply(yaw, tx, ty, enu) - map_xy, axis=1)

    k = res[kept]
    g = Georef(datum, yaw, tx, ty, rms=float(np.sqrt(np.mean(k ** 2))),
               max_err=float(k.max()), n=int(kept.sum()), span=_span(enu[kept]))
    return g, res, kept


def _span(pts):
    """Largest distance between any two points (O(N^2); N is a few hundred)."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return 0.0
    d = pts[:, None, :] - pts[None, :, :]
    return float(np.sqrt((d ** 2).sum(axis=2)).max())
