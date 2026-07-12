"""
seg_coverage.py — shared segment-coverage check.

Used by generate_zone_route.py (planned-route coverage report) and
strip_covered_segments.py (post-run KMZ stripping) so the two can't drift.

A KMZ segment counts as covered when >= COVER_MIN_FRAC of its LENGTH lies
within COVER_SNAP_M of the track polyline. This approximates Wandrer's own
crediting rule (~90% of a way traveled) and fixes two failure modes of the
old all-vertex check:

  * straight 2-vertex blocks were marked covered when the track merely
    crossed both cross-streets at their ends (false positive) — resampling
    every SAMPLE_STEP_M along the segment adds interior test points;
  * one noisy GPS vertex at 26m failed an otherwise fully-run segment
    (false negative) — the fractional rule tolerates up to 10% misses.

Distances are point-to-polyline (nearest track LEG, not nearest track
vertex), so sparse Valhalla shape points on long straights no longer
inflate distances.
"""

import math

COVER_SNAP_M = 25.0    # lateral tolerance, matches Wandrer's ~25m buffer
COVER_MIN_FRAC = 0.90  # fraction of segment length that must be within snap
SAMPLE_STEP_M = 10.0   # resample interval along each KMZ segment

_M_PER_DEG_LAT = 111320.0


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return R * 2 * math.asin(math.sqrt(a))


class TrackIndex:
    """Grid-indexed track polyline for fast point-to-polyline distance.

    Projects to local equirectangular meters (fine at city scale) and
    buckets track legs into cells so each query touches only nearby legs.
    Build once per track; query per sample point.
    """

    def __init__(self, track_points, cell_m=50.0):
        if len(track_points) < 2:
            raise ValueError("track needs at least 2 points")
        lat0 = sum(p[0] for p in track_points) / len(track_points)
        self._mlat = _M_PER_DEG_LAT
        self._mlon = _M_PER_DEG_LAT * math.cos(math.radians(lat0))
        self._cell = cell_m
        self._xy = [(p[1] * self._mlon, p[0] * self._mlat) for p in track_points]

        self._grid = {}
        for i in range(len(self._xy) - 1):
            ax, ay = self._xy[i]
            bx, by = self._xy[i + 1]
            if ax == bx and ay == by:
                continue
            cx0 = int(min(ax, bx) // cell_m)
            cx1 = int(max(ax, bx) // cell_m)
            cy0 = int(min(ay, by) // cell_m)
            cy1 = int(max(ay, by) // cell_m)
            for cx in range(cx0, cx1 + 1):
                for cy in range(cy0, cy1 + 1):
                    self._grid.setdefault((cx, cy), []).append(i)

    def _project(self, lat, lon):
        return lon * self._mlon, lat * self._mlat

    def min_dist_m(self, lat, lon, cutoff_m):
        """Distance from point to nearest track leg, or None if > cutoff_m."""
        px, py = self._project(lat, lon)
        r = int(cutoff_m // self._cell) + 1
        ccx = int(px // self._cell)
        ccy = int(py // self._cell)
        best2 = cutoff_m * cutoff_m
        found = False
        seen = set()
        for cx in range(ccx - r, ccx + r + 1):
            for cy in range(ccy - r, ccy + r + 1):
                for i in self._grid.get((cx, cy), ()):
                    if i in seen:
                        continue
                    seen.add(i)
                    d2 = self._pt_leg_d2(px, py, i)
                    if d2 <= best2:
                        best2 = d2
                        found = True
        return math.sqrt(best2) if found else None

    def _pt_leg_d2(self, px, py, i):
        ax, ay = self._xy[i]
        bx, by = self._xy[i + 1]
        dx, dy = bx - ax, by - ay
        t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        qx, qy = ax + t * dx, ay + t * dy
        return (px - qx) ** 2 + (py - qy) ** 2


def resample_polyline(pts, step_m=SAMPLE_STEP_M):
    """Evenly spaced (lat, lon) samples along a polyline, endpoints included."""
    leg_len = [haversine_m(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
               for i in range(len(pts) - 1)]
    total = sum(leg_len)
    if total == 0:
        return [tuple(pts[0])]
    n = max(2, int(math.ceil(total / step_m)) + 1)
    targets = [total * k / (n - 1) for k in range(n)]

    samples = []
    ti = 0
    acc = 0.0  # length before current leg
    for t in targets:
        while ti < len(leg_len) - 1 and acc + leg_len[ti] < t:
            acc += leg_len[ti]
            ti += 1
        L = leg_len[ti]
        f = (t - acc) / L if L > 0 else 0.0
        f = min(max(f, 0.0), 1.0)
        a, b = pts[ti], pts[ti + 1]
        samples.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
    return samples


def covered_fraction(seg_pts, track_index, snap_m=COVER_SNAP_M,
                     step_m=SAMPLE_STEP_M):
    """Fraction of the segment's length within snap_m of the track polyline."""
    samples = resample_polyline(seg_pts, step_m)
    hits = 0
    for lat, lon in samples:
        if track_index.min_dist_m(lat, lon, snap_m) is not None:
            hits += 1
    return hits / len(samples)


def segment_covered(seg_pts, track_index, snap_m=COVER_SNAP_M,
                    min_frac=COVER_MIN_FRAC, step_m=SAMPLE_STEP_M):
    """True if >= min_frac of the segment's length is within snap_m of track.

    Early-exits as soon as enough samples have missed that min_frac is
    unreachable — keeps citywide scans (strip script) fast.
    """
    samples = resample_polyline(seg_pts, step_m)
    n = len(samples)
    allowed_misses = int(n * (1.0 - min_frac))
    misses = 0
    for lat, lon in samples:
        if track_index.min_dist_m(lat, lon, snap_m) is None:
            misses += 1
            if misses > allowed_misses:
                return False
    return True
