"""
generate_zone_route.py — Detroit Wandrer Route Generator

PURPOSE
-------
Generates an optimized running loop for a Detroit neighborhood, designed to cover
as many untraveled streets as possible (per Wandrer.earth tracking) in 6–8 miles.

WORKFLOW
--------
1. Parse wandrer-kris.kmz — the user's Wandrer export of all remaining untraveled
   street segments in Detroit.
2. Filter segments to the target zone (by bounding box or radius from a center point).
3. Compute each segment's midpoint; deduplicate nearby midpoints so no two waypoints
   are closer than DEDUP_M meters (avoids redundant visits on parallel streets).
4. Solve the Travelling Salesman Problem (TSP) on those waypoints using Valhalla's
   real walking-distance matrix and a nearest-neighbor + 2-opt algorithm.
   If waypoints exceed 50 (Valhalla's matrix limit), split into spatial clusters
   (CHUNK_K) and optimize each cluster independently, then chain them.
5. Feed the ordered waypoints to Valhalla /route to get the actual walking path
   with turn-by-turn maneuver data.
6. Write the route as both GPX (for Strava upload) and FIT (for Garmin FR965 watch).
   FIT files include CoursePoint turn arrows so the watch shows LEFT/RIGHT prompts.

DEPENDENCIES
------------
  pip install requests fit-tool
  Valhalla public API: https://valhalla1.openstreetmap.de (no key needed)
  wandrer-kris.kmz: downloaded from Wandrer.earth > Export map > KMZ

QUICK START
-----------
  1. Edit the CONFIG block below — set BBOX or ZONE_LAT/ZONE_LON, SLUG, ROUTE_NAME.
  2. Activate the venv: source ~/.venvs/detroit-wandrer/bin/activate
  3. Run: python generate_zone_route.py
  4. Copy the output FIT to the Garmin watch:
       gio copy "Wandrer Routes/route.fit" "mtp://DEVICE/Internal Storage/GARMIN/Courses/route.fit"
"""

import xml.etree.ElementTree as ET
import json
import re
import math
import os
import time
import requests
import zipfile

# ── File paths ────────────────────────────────────────────────────────────────
# All paths use TresoritDrive (encrypted cloud sync). The venv must live outside
# TresoritDrive because its FUSE filesystem does not support symlinks.

DETROIT_WANDRER = os.path.expanduser("~/TresoritDrive/Running/detroit-wandrer")
if not os.path.isdir(DETROIT_WANDRER):
    DETROIT_WANDRER = os.path.expanduser("~/Tresorit/Running/detroit-wandrer")  # Mac sync root differs from Linux
ROUTES_DIR = os.path.expanduser("~/TresoritDrive/Running/Wandrer Routes")
if not os.path.isdir(ROUTES_DIR):
    ROUTES_DIR = os.path.expanduser("~/Tresorit/Running/Wandrer Routes")
SOURCE_KMZ = os.path.join(DETROIT_WANDRER, "wandrer-kris.kmz")

# ── FIT library imports ───────────────────────────────────────────────────────
# fit-tool encodes Garmin FIT binary format. Unit notes (non-obvious):
#   timestamp fields   → Unix milliseconds (library applies Garmin epoch offset)
#   position_lat/long  → decimal degrees  (library converts to semicircles)
#   distance fields    → meters
#   time fields        → seconds
from fit_tool.fit_file_builder import FitFileBuilder
from fit_tool.profile.messages.file_id_message import FileIdMessage
from fit_tool.profile.messages.course_message import CourseMessage
from fit_tool.profile.messages.lap_message import LapMessage
from fit_tool.profile.messages.record_message import RecordMessage
from fit_tool.profile.messages.event_message import EventMessage
from fit_tool.profile.messages.course_point_message import CoursePointMessage
from fit_tool.profile.profile_type import (
    FileType, Manufacturer, CourseCapabilities, CoursePoint, Sport,
    Event, EventType
)

VALHALLA = "https://valhalla1.openstreetmap.de"
OSRM = "https://router.project-osrm.org"

# pyvalhalla in-process backend (used when available — no HTTP overhead, no public API dependency)
# Set VALHALLA_CONFIG env var to your valhalla.json path, or place it at ~/.valhalla/valhalla.json
_VALHALLA_ACTOR = None
try:
    import valhalla as _pyvalhalla
    import json as _json
    _VALHALLA_CONFIG = os.environ.get("VALHALLA_CONFIG", os.path.expanduser("~/.valhalla/valhalla.json"))
    if os.path.exists(_VALHALLA_CONFIG):
        _VALHALLA_ACTOR = _pyvalhalla.Actor(_VALHALLA_CONFIG)
        print("Using local pyvalhalla (in-process)")
except ImportError:
    pass

# ── CONFIG — edit per zone ────────────────────────────────────────────────────
# This is the only block you need to change when generating a new route.
#
# FILTER_MODE: how to select segments from the KMZ
#   "bbox"   — use the BBOX rectangle (preferred for named neighborhoods)
#   "radius" — use a circle of RADIUS_MI miles around (ZONE_LAT, ZONE_LON)
#
# DEDUP_M: minimum distance between waypoints in meters.
#   140m works for most Detroit residential grids (~100-150m block spacing).
#   80-100m for tighter grids. Too small → >50 waypoints (Valhalla cap). Too large
#   → whole parallel streets get merged into one waypoint and are skipped.
#
# MIN_SEG_M: skip segments shorter than this (meters). Use 50 for stub-heavy
#   areas (dead ends, cul-de-sacs) that inflate waypoint count without adding value.
#   0 = keep all segments.
#
# WAYPOINT_MODE: how to derive TSP waypoints from segment geometry.
#   "endpoints" — use both endpoints (intersections) of each segment. More reliable
#                 coverage: routing through endpoint A then B forces Valhalla to
#                 traverse the street between them. Produces more waypoints than
#                 midpoints, so pair with CHUNK_K="auto" for large zones.
#   "midpoints" — legacy: one midpoint per segment. Fine for short straight segments;
#                 can miss coverage on long or curved streets.
#
# CHUNK_K: spatial clustering for zones that produce >50 waypoints after dedup.
#   "auto" = set K automatically so each cluster stays under 45 waypoints.
#   1      = standard single-pass (truncates at 50 if exceeded — use for small zones).
#   2-4    = explicit cluster count (each cluster must be ≤50 for Valhalla matrix).
#
# LOOP: True = route returns to start (required for solo runs — car/transit start).

FILTER_MODE = "bbox"

# radius mode — circle around a center point
ZONE_LAT, ZONE_LON = 42.397, -83.218
RADIUS_MI = 0.50

# bbox mode — explicit bounding box (midpoint filter); set FILTER_MODE = "bbox" to use
# BBOX = (lat_min, lat_max, lon_min, lon_max)
BBOX = (42.430, 42.455, -83.005, -82.950)  # Morningside (7 Mile to 8 Mile)

DEDUP_M = 75
MIN_SEG_M = 30
WAYPOINT_MODE = "both"       # "endpoints", "midpoints", or "both" — both forces full block traversal and covers long-segment endpoints
CHUNK_K = "auto"
SLUG = "morningside"
ROUTE_NAME = "Morningside"
LOOP = True
# When LOOP=False: prefer open paths whose end falls within this straight-line
# distance (miles) of the start. Set to None to disable and just minimize total distance.
NEAR_LOOP_MI = 0.31  # 500m

# ── Boustrophedon routing (ROUTE_STRATEGY = "boustrophedon") ──────────────────
# Groups remaining segments by the street they lie on (E-W segments bucketed by
# latitude ±STREET_BUCKET_M; N-S segments by longitude). Sweeps E-W streets
# south-to-north, alternating W→E / E→W. Each transition between streets is a
# single cross-street block — often itself untraveled — rather than a multi-block
# backtrack through already-covered streets.
#
# ROUTE_STRATEGY:   "2opt" = existing TSP; "boustrophedon" = street-level sweep
# STREET_BUCKET_M:  lat/lon tolerance (meters) for grouping segments onto the same
#                   street. 50m ≈ half a Detroit block — tight enough to separate
#                   parallel streets (~100-150m apart) but loose enough to group
#                   segments that lie on the same street with slight GPS variation.
ROUTE_STRATEGY = "2opt"
STREET_BUCKET_M = 75
# WRITE_COURSE_POINTS: True = embed LEFT/RIGHT turn cues in the FIT file (audible on Garmin).
# False = breadcrumb track only, no audio alerts.
WRITE_COURSE_POINTS = True
# COURSE_POINT_MIN_DIST_M: minimum meters between consecutive course points. Prevents the
# watch from beeping every 20m in dense urban "both"-mode routes. ~75m ≈ one Detroit block.
# Only actual directional turns (LEFT/RIGHT/etc.) are written; STRAIGHT and GENERIC are
# always skipped since they carry no useful guidance.
COURSE_POINT_MIN_DIST_M = 75
# FORCED_WAYPOINTS: list of (lat, lon) pairs injected directly into the waypoint pool
# before dedup and TSP. Use to force coverage of specific segments that the bbox filter
# misses or that Valhalla routes around (e.g. segments near freeway interchanges).
FORCED_WAYPOINTS = [
]
# ──────────────────────────────────────────────────────────────────────────────

OUTPUT_GPX = os.path.join(ROUTES_DIR, f"{SLUG}.gpx")
OUTPUT_FIT = os.path.join(ROUTES_DIR, f"{SLUG}.fit")


# ── KMZ parsing ───────────────────────────────────────────────────────────────
# Wandrer exports a KMZ file (a ZIP containing a .kml XML file). Each untraveled
# street segment is a Placemark. The user's KMZ uses MultiGeometry, so each
# Placemark may contain multiple <LineString> elements — each is extracted as its
# own segment. Coordinates are stored lon,lat (GeoJSON order) and converted to
# (lat, lon) tuples for internal use.

def parse_kmz_linestrings(path):
    with zipfile.ZipFile(path) as z:
        kml_name = [n for n in z.namelist() if n.endswith(".kml")][0]
        kml_text = z.read(kml_name).decode("utf-8")
    linestrings = re.findall(r'<LineString><coordinates>([^<]+)</coordinates></LineString>', kml_text)
    segments = []
    for ls in linestrings:
        pts = []
        for pair in ls.strip().split():
            lon, lat = pair.split(",")[:2]
            pts.append((float(lat), float(lon)))
        if len(pts) >= 2:
            segments.append(pts)
    return segments


# ── Geometry helpers ──────────────────────────────────────────────────────────

def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in meters between two (lat, lon) points."""
    R = 6371000
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return R * 2 * math.asin(math.sqrt(a))


def deduplicate(points, min_dist_m=100):
    """
    Remove waypoints that are too close together.

    Greedily keeps each point only if it is at least min_dist_m away from all
    already-kept points. This prevents the TSP from wasting distance visiting
    two waypoints on the same block or adjacent parallel streets.
    """
    result = []
    for p in points:
        if all(haversine_m(p[0], p[1], q[0], q[1]) > min_dist_m for q in result):
            result.append(p)
    return result


# ── Valhalla matrix + 2-opt TSP ───────────────────────────────────────────────
# The TSP pipeline:
#   1. valhalla_matrix() — fetch real pedestrian walking distances between every
#      pair of waypoints using Valhalla's /sources_to_targets API.
#      HARD LIMIT: Valhalla rejects requests with >50 sources or targets (HTTP 400).
#   2. nearest_neighbor() — greedy tour construction: always go to the closest
#      unvisited waypoint. Fast but suboptimal; used as the starting point for 2-opt.
#   3. two_opt_improve() — iteratively swap pairs of edges when the swap reduces
#      total tour length. Continues until no improving swap exists.
#   4. best_2opt_tour() — runs nearest_neighbor + two_opt_improve from every possible
#      starting waypoint, keeps the shortest result. More starting points = better
#      solution at the cost of O(n³) time (fine for n ≤ 50).

def valhalla_matrix(points):
    """Fetch the n×n pedestrian walking-distance matrix (in miles) from Valhalla."""
    locations = [{"lat": lat, "lon": lon} for lat, lon in points]
    payload = {"sources": locations, "targets": locations, "costing": "pedestrian", "units": "miles"}
    if _VALHALLA_ACTOR is not None:
        data = _json.loads(_VALHALLA_ACTOR.matrix(payload))
    else:
        resp = requests.post(f"{VALHALLA}/sources_to_targets", json=payload, timeout=120)
        resp.raise_for_status()
        data = resp.json()
    n = len(points)
    dist = [[float("inf")] * n for _ in range(n)]
    for i, row in enumerate(data["sources_to_targets"]):
        for j, cell in enumerate(row):
            if cell and cell.get("distance") is not None:
                dist[i][j] = cell["distance"]
    for i in range(n):
        dist[i][i] = 0.0
    return dist


def tour_length(tour, dist, loop):
    """Sum of edge distances in a tour. If loop=True, adds the return edge to start."""
    n = len(tour)
    total = sum(dist[tour[i]][tour[i + 1]] for i in range(n - 1))
    if loop:
        total += dist[tour[-1]][tour[0]]
    return total


def nearest_neighbor(dist, start):
    """Greedy nearest-neighbor tour from a given starting waypoint index."""
    n = len(dist)
    unvisited = set(range(n))
    tour = [start]
    unvisited.remove(start)
    while unvisited:
        cur = tour[-1]
        nxt = min(unvisited, key=lambda j: dist[cur][j])
        tour.append(nxt)
        unvisited.remove(nxt)
    return tour


def two_opt_improve(tour, dist, loop, max_passes=200):
    """
    2-opt local search improvement.

    Repeatedly checks all pairs of edges (i-1,i) and (j, j+1). If reversing the
    segment between i and j shortens the tour, applies the reversal. Stops when
    no improving swap exists (local optimum).

    max_passes bounds the outer loop. Valhalla's pedestrian matrix isn't always
    perfectly symmetric (one-way crossings, slightly different forward/reverse
    costs), so a reversal can occasionally look like an "improvement" in both
    directions, oscillating forever with the original 1e-9 epsilon. The larger
    1e-4 mi (~0.5ft) epsilon ignores noise-level deltas, and max_passes guarantees
    termination even if oscillation still occurs.
    """
    n = len(tour)
    improved = True
    passes = 0
    while improved and passes < max_passes:
        improved = False
        passes += 1
        for i in range(1, n - 1):
            for j in range(i + 1, n):
                if not loop and j == n - 1:
                    continue
                a, b = tour[i - 1], tour[i]
                c, d = tour[j], tour[(j + 1) % n]
                if dist[a][c] + dist[b][d] < dist[a][b] + dist[c][d] - 1e-4:
                    tour[i: j + 1] = tour[i: j + 1][::-1]
                    improved = True
    return tour


def best_2opt_tour(points, dist, loop, near_loop_mi=None):
    """
    Try nearest-neighbor + 2-opt from every starting waypoint; return the best tour.

    Returns a list of waypoint indices (not coordinates). The caller converts to
    coordinates. If loop=True, the caller should append ordered[0] to close the loop.

    When near_loop_mi is set and loop=False, prefers open paths whose end falls
    within near_loop_mi straight-line miles of the start, choosing the shortest
    such path. Falls back to the globally shortest open path if none qualifies.
    """
    n = len(points)
    best_tour, best_len = None, float("inf")
    best_near_tour, best_near_len = None, float("inf")

    for start in range(n):
        t = nearest_neighbor(dist, start)
        t = two_opt_improve(t, dist, loop)
        l = tour_length(t, dist, loop)
        if l < best_len:
            best_len, best_tour = l, t
        if near_loop_mi and not loop:
            sp = points[t[0]]
            ep = points[t[-1]]
            gap_mi = haversine_m(sp[0], sp[1], ep[0], ep[1]) / 1609.34
            if gap_mi <= near_loop_mi and l < best_near_len:
                best_near_len, best_near_tour = l, t

    if near_loop_mi and not loop:
        if best_near_tour is not None:
            sp = points[best_near_tour[0]]
            ep = points[best_near_tour[-1]]
            gap_mi = haversine_m(sp[0], sp[1], ep[0], ep[1]) / 1609.34
            print(f"  Near-loop tour: {best_near_len:.2f} mi, end is {gap_mi:.2f} mi from start")
            return best_near_tour
        else:
            sp = points[best_tour[0]]
            ep = points[best_tour[-1]]
            gap_mi = haversine_m(sp[0], sp[1], ep[0], ep[1]) / 1609.34
            print(f"  Open path: {best_len:.2f} mi — no tour ends within {near_loop_mi} mi of start "
                  f"(best gap: {gap_mi:.2f} mi)")
    else:
        print(f"  Best 2-opt tour: {best_len:.2f} mi across {n} waypoints")
    return best_tour


# ── Spatial chunking for large zones (>50 waypoints) ─────────────────────────
# Valhalla's /sources_to_targets matrix is capped at 50 nodes. For zones with
# more waypoints, chunked_tour() splits them into K spatial clusters using k-means,
# runs the 2-opt TSP independently on each cluster (each must be ≤50 waypoints),
# then chains the cluster sequences together into a single ordered list.
#
# Chaining logic: for each cluster join, choose whether to traverse the next cluster
# forward or reversed based on which direction minimises the connector gap distance.
# Clusters are sorted south→north by centroid latitude before chaining.

def kmeans_cluster(points, k, max_iter=50):
    """
    Spatial k-means clustering.

    Returns (clusters, centroids) where clusters is a list of K point-lists,
    sorted south→north by centroid latitude so chunked_tour chains them in
    geographic order rather than arbitrary order.
    """
    pts = list(points)
    n = len(pts)
    # Initialize centroids evenly spaced by latitude
    sorted_by_lat = sorted(pts)
    centroids = [sorted_by_lat[i * n // k] for i in range(k)]
    clusters = [[] for _ in range(k)]
    for _ in range(max_iter):
        clusters = [[] for _ in range(k)]
        for p in pts:
            best = min(range(k), key=lambda i: haversine_m(p[0], p[1], centroids[i][0], centroids[i][1]))
            clusters[best].append(p)
        new_centroids = []
        for j, cl in enumerate(clusters):
            if cl:
                new_centroids.append((sum(p[0] for p in cl) / len(cl),
                                      sum(p[1] for p in cl) / len(cl)))
            else:
                new_centroids.append(centroids[j])
        if new_centroids == centroids:
            break
        centroids = new_centroids
    # Return clusters sorted S→N by centroid latitude
    paired = sorted(zip(centroids, clusters), key=lambda x: x[0][0])
    return [cl for _, cl in paired], [c for c, _ in paired]


def _best_cluster_chain(cluster_seqs, loop, near_loop_mi=None):
    """Find the visiting order and orientation of cluster_seqs that minimises
    total inter-cluster connector distance.

    Uses real Valhalla pedestrian distances between cluster endpoints (one small
    matrix call, 2N × 2N points) so obstacles like freeways are accounted for.
    Falls back to Haversine if the matrix call fails.

    Tries all N! orderings × 2^N orientation combos — instant for N ≤ 6.

    When near_loop_mi is set and loop=False, prefers chains whose global
    start-to-end straight-line gap is ≤ near_loop_mi, choosing the one with
    minimum connector cost among those. Falls back to global minimum if none qualify.
    """
    from itertools import permutations as _perms
    n = len(cluster_seqs)
    if n == 1:
        seq = list(cluster_seqs[0])
        return seq + ([seq[0]] if loop else [])

    # 2 endpoints per cluster: index 2i = start, 2i+1 = end
    endpoints = []
    for seq in cluster_seqs:
        endpoints.append(seq[0])
        endpoints.append(seq[-1])

    print(f"  Fetching {2*n}×{2*n} inter-cluster distance matrix...")
    try:
        ep_dist = routing_matrix(endpoints)  # miles
        def gap(exit_idx, entry_idx):
            return ep_dist[exit_idx][entry_idx]
    except Exception:
        def gap(exit_idx, entry_idx):
            a, b = endpoints[exit_idx], endpoints[entry_idx]
            return haversine_m(a[0], a[1], b[0], b[1]) / 1609.34

    best_cost, best_flat = float("inf"), None
    best_near_cost, best_near_flat = float("inf"), None

    for perm in _perms(range(n)):
        for dirs in range(1 << n):
            oriented = []
            ep_idx = []  # (entry_ep_idx, exit_ep_idx) for each slot
            for i, ci in enumerate(perm):
                fwd = not (dirs >> i & 1)
                oriented.append(cluster_seqs[ci] if fwd else list(reversed(cluster_seqs[ci])))
                ep_idx.append((2*ci if fwd else 2*ci+1, 2*ci+1 if fwd else 2*ci))

            cost = sum(gap(ep_idx[i][1], ep_idx[i+1][0]) for i in range(n - 1))
            if loop:
                cost += gap(ep_idx[-1][1], ep_idx[0][0])

            flat = [pt for seq in oriented for pt in seq]
            if cost < best_cost:
                best_cost = cost
                best_flat = flat

            if near_loop_mi and not loop:
                sp, ep = flat[0], flat[-1]
                global_gap = haversine_m(sp[0], sp[1], ep[0], ep[1]) / 1609.34
                if global_gap <= near_loop_mi and cost < best_near_cost:
                    best_near_cost = cost
                    best_near_flat = flat

    if near_loop_mi and not loop:
        if best_near_flat is not None:
            sp, ep = best_near_flat[0], best_near_flat[-1]
            actual_gap = haversine_m(sp[0], sp[1], ep[0], ep[1]) / 1609.34
            print(f"  Near-loop cluster chain: {best_near_cost:.2f} mi connector gap, end {actual_gap:.2f} mi from start")
            return best_near_flat
        else:
            sp, ep = best_flat[0], best_flat[-1]
            actual_gap = haversine_m(sp[0], sp[1], ep[0], ep[1]) / 1609.34
            print(f"  Best cluster chain: {best_cost:.2f} mi connector gap — no near-loop within {near_loop_mi} mi (best gap: {actual_gap:.2f} mi)")

    else:
        print(f"  Best cluster chain: {best_cost:.2f} mi connector gap")

    if loop:
        best_flat.append(best_flat[0])
    return best_flat


_VALHALLA_MATRIX_LIMIT = 49  # Valhalla /sources_to_targets max nodes (leave 1 buffer)


def _split_cluster(cl):
    """Recursively split a cluster until all sub-clusters fit within the matrix limit."""
    if len(cl) <= _VALHALLA_MATRIX_LIMIT:
        return [cl]
    sub_k = math.ceil(len(cl) / (_VALHALLA_MATRIX_LIMIT // 2))
    subs, _ = kmeans_cluster(cl, sub_k)
    result = []
    for sub in subs:
        if sub:
            result.extend(_split_cluster(sub))
    return result


def chunked_tour(midpoints, k, loop, near_loop_mi=None):
    """
    Split midpoints into k spatial clusters, optimize each with 2-opt, then
    find the globally best cluster visiting order + orientation.

    Within each cluster: 2-opt TSP using the real Valhalla pedestrian matrix.
    Between clusters: exhaustive search over all N! orderings × 2^N orientations
    using Haversine as a proxy (no extra API calls). This eliminates the fixed
    S→N ordering and greedy fwd/rev chaining that caused systematic overhead.
    """
    clusters, centroids = kmeans_cluster(midpoints, k)
    # Recursively split any cluster that exceeds the Valhalla matrix limit
    safe_clusters = []
    for cl in clusters:
        safe_clusters.extend(_split_cluster(cl))
    clusters = [cl for cl in safe_clusters if cl]
    print(f"  Chunks: {[len(c) for c in clusters]} waypoints")
    cluster_seqs = []
    for i, cl in enumerate(clusters):
        if len(cl) == 0:
            continue
        if len(cl) == 1:
            cluster_seqs.append(cl)
            continue
        print(f"  Chunk {i+1}/{len(clusters)} ({len(cl)} wpts)...")
        local_dist = routing_matrix(cl)
        local_tour = best_2opt_tour(cl, local_dist, loop=False)
        cluster_seqs.append([cl[j] for j in local_tour])
    return _best_cluster_chain(cluster_seqs, loop, near_loop_mi=near_loop_mi)


# ── Valhalla routing with turn-by-turn maneuvers ──────────────────────────────
# Valhalla's /route endpoint accepts up to 50 locations per request. For longer
# ordered waypoint lists, valhalla_route_with_maneuvers() splits into overlapping
# chunks of 50 (each chunk re-uses the last point of the previous chunk as its
# first point, so the path is continuous). It decodes the polyline6-encoded shape
# and collects maneuver data (turn type, location, instruction) for FIT course points.
#
# _VALHALLA_TO_COURSE_POINT maps Valhalla maneuver type integers to Garmin CoursePoint
# enum values (LEFT, RIGHT, SLIGHT_LEFT, etc.). _SKIP_TYPES are straight/continue
# maneuvers that are too frequent to be useful as watch prompts.

_VALHALLA_TO_COURSE_POINT = {
    1: CoursePoint.GENERIC, 2: CoursePoint.GENERIC, 3: CoursePoint.GENERIC,
    4: CoursePoint.GENERIC, 5: CoursePoint.GENERIC, 6: CoursePoint.GENERIC,
    7: CoursePoint.STRAIGHT, 8: CoursePoint.STRAIGHT,
    9: CoursePoint.SLIGHT_RIGHT, 10: CoursePoint.RIGHT, 11: CoursePoint.SHARP_RIGHT,
    12: CoursePoint.U_TURN, 13: CoursePoint.U_TURN,
    14: CoursePoint.SHARP_LEFT, 15: CoursePoint.LEFT, 16: CoursePoint.SLIGHT_LEFT,
    17: CoursePoint.STRAIGHT, 18: CoursePoint.SLIGHT_RIGHT, 19: CoursePoint.SLIGHT_LEFT,
    22: CoursePoint.STRAIGHT, 23: CoursePoint.SLIGHT_RIGHT, 24: CoursePoint.SLIGHT_LEFT,
    26: CoursePoint.RIGHT, 27: CoursePoint.STRAIGHT,
    37: CoursePoint.RIGHT, 38: CoursePoint.LEFT,
}
_SKIP_TYPES = {7, 8, 17, 22}  # straight / continue — too noisy to show on watch


def decode_polyline6(encoded):
    """
    Decode a Google Polyline-encoded string (precision 1e-6) to (lat, lon) pairs.

    Valhalla uses polyline6 (6 decimal places) for route shapes. Each coordinate
    delta is encoded as a variable-length sequence of 5-bit chunks in ASCII.
    """
    coords = []
    index, lat, lon = 0, 0, 0
    while index < len(encoded):
        for is_lon in (False, True):
            shift, result = 0, 0
            while True:
                b = ord(encoded[index]) - 63
                index += 1
                result |= (b & 0x1F) << shift
                shift += 5
                if b < 0x20:
                    break
            delta = ~(result >> 1) if result & 1 else result >> 1
            if is_lon:
                lon += delta
            else:
                lat += delta
        coords.append((lat / 1e6, lon / 1e6))
    return coords


def valhalla_route_with_maneuvers(waypoints):
    """
    Route through all ordered waypoints via Valhalla /route (pedestrian costing).

    Splits the waypoint list into overlapping chunks of 50 to stay within Valhalla's
    per-request location limit. Intermediate waypoints use type "through" so Valhalla
    doesn't stop at them; only the first and last of each chunk are "break" points.

    Returns:
        track_points  — list of (lat, lon) for every point along the route path
        total_miles   — total route distance in miles
        maneuvers     — list of (lat, lon, dist_m, CoursePoint type, instruction str)
                        for each non-trivial turn (used for FIT CoursePoint records)
    """
    track_points = []
    maneuvers = []
    total_miles = 0.0
    cumulative_m = 0.0
    chunk_size = 49  # overlapping chunks prepend 1 overlap pt → max 50 per request

    i = 0
    while i < len(waypoints):
        chunk = waypoints[i: i + chunk_size]
        if i > 0:
            chunk = [waypoints[i - 1]] + chunk

        locations = [{"lat": lat, "lon": lon, "type": "break"} for lat, lon in chunk]

        payload = {"locations": locations, "costing": "pedestrian", "units": "miles"}
        if _VALHALLA_ACTOR is not None:
            data = _json.loads(_VALHALLA_ACTOR.route(payload))
        else:
            resp = requests.post(f"{VALHALLA}/route", json=payload, timeout=60)
            if not resp.ok:
                print(f"  Valhalla error (chunk i={i}, {len(locations)} locs): "
                      f"HTTP {resp.status_code} — {resp.text[:400]}")
            resp.raise_for_status()
            data = resp.json()

        for leg in data["trip"]["legs"]:
            leg_points = decode_polyline6(leg["shape"])
            total_miles += leg["summary"]["length"]

            # Build cumulative distance array within this leg for maneuver positioning
            leg_dists_m = [0.0]
            for k in range(1, len(leg_points)):
                d = haversine_m(leg_points[k-1][0], leg_points[k-1][1], leg_points[k][0], leg_points[k][1])
                leg_dists_m.append(leg_dists_m[-1] + d)

            for maneuver in leg.get("maneuvers", []):
                mtype = maneuver.get("type", 0)
                if mtype in _SKIP_TYPES:
                    continue
                cp_type = _VALHALLA_TO_COURSE_POINT.get(mtype, CoursePoint.GENERIC)
                idx = min(maneuver.get("begin_shape_index", 0), len(leg_points) - 1)
                lat, lon = leg_points[idx]
                dist_in_leg_m = leg_dists_m[idx]
                instruction = maneuver.get("instruction", "")
                maneuvers.append((lat, lon, cumulative_m + dist_in_leg_m, cp_type, instruction))

            cumulative_m += leg["summary"]["length"] * 1609.34
            track_points.extend(leg_points)

        i += chunk_size

    return track_points, total_miles, maneuvers


# ── OSRM fallback (used when Valhalla public API is unavailable) ───────────────

_OSRM_MODIFIER_TO_COURSE_POINT = {
    "left":        CoursePoint.LEFT,
    "sharp left":  CoursePoint.SHARP_LEFT,
    "right":       CoursePoint.RIGHT,
    "sharp right": CoursePoint.SHARP_RIGHT,
    "uturn":       CoursePoint.U_TURN,
    "slight left": CoursePoint.SLIGHT_LEFT,
    "slight right": CoursePoint.SLIGHT_RIGHT,
}
_OSRM_SKIP_TYPES = {"depart", "arrive", "notification"}
_OSRM_SKIP_MODIFIERS = {"straight", "none"}


def osrm_matrix(points):
    """Fetch n×n pedestrian walking-distance matrix (miles) from OSRM /table."""
    coords = ";".join(f"{lon},{lat}" for lat, lon in points)
    n = len(points)
    idxs = ";".join(str(i) for i in range(n))
    resp = requests.get(
        f"{OSRM}/table/v1/foot/{coords}",
        params={"sources": idxs, "destinations": idxs, "annotations": "distance"},
        timeout=120,
    )
    resp.raise_for_status()
    data = resp.json()
    dist = [[float("inf")] * n for _ in range(n)]
    for i, row in enumerate(data["distances"]):
        for j, val in enumerate(row):
            if val is not None:
                dist[i][j] = val / 1609.34
    for i in range(n):
        dist[i][i] = 0.0
    return dist


def osrm_route_with_maneuvers(waypoints):
    """Route through ordered waypoints via OSRM /route (fallback when Valhalla is down)."""
    coords = ";".join(f"{lon},{lat}" for lat, lon in waypoints)
    resp = requests.get(
        f"{OSRM}/route/v1/foot/{coords}",
        params={"steps": "true", "overview": "full", "geometries": "polyline6"},
        timeout=120,
    )
    resp.raise_for_status()
    data = resp.json()
    if data.get("code") != "Ok":
        raise RuntimeError(f"OSRM route error: {data.get('message', data)}")

    route = data["routes"][0]
    total_miles = route["distance"] / 1609.34
    track_points = decode_polyline6(route["geometry"])

    maneuvers = []
    cumulative_m = 0.0
    for leg in route["legs"]:
        for step in leg.get("steps", []):
            mtype = step["maneuver"]["type"]
            modifier = step["maneuver"].get("modifier", "straight")
            step_dist = step["distance"]
            if mtype not in _OSRM_SKIP_TYPES and modifier not in _OSRM_SKIP_MODIFIERS:
                cp_type = _OSRM_MODIFIER_TO_COURSE_POINT.get(modifier, CoursePoint.GENERIC)
                loc = step["maneuver"]["location"]  # [lon, lat]
                instruction = f"{mtype} {modifier} on {step.get('name', '')}".strip()
                maneuvers.append((loc[1], loc[0], cumulative_m, cp_type, instruction))
            cumulative_m += step_dist

    return track_points, total_miles, maneuvers


def haversine_matrix(points):
    """Straight-line distance matrix (miles). Used when routing APIs are down/unreliable."""
    n = len(points)
    dist = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                dist[i][j] = haversine_m(points[i][0], points[i][1],
                                          points[j][0], points[j][1]) / 1609.34
    return dist


def routing_matrix(points):
    """Valhalla distance matrix, falling back to straight-line when Valhalla is unavailable.
    OSRM's pedestrian matrix gives unreliable distances for TSP ordering, so we skip it."""
    try:
        return valhalla_matrix(points)
    except Exception as e:
        print(f"  Valhalla matrix failed ({type(e).__name__}), using straight-line distances for TSP...")
        return haversine_matrix(points)


def routing_route(waypoints):
    """Valhalla turn-by-turn route with automatic OSRM fallback."""
    try:
        return valhalla_route_with_maneuvers(waypoints)
    except Exception as e:
        print(f"  Valhalla route failed ({type(e).__name__}), falling back to OSRM...")
        return osrm_route_with_maneuvers(waypoints)


# ── GPX writer ────────────────────────────────────────────────────────────────
# Writes a minimal GPX 1.1 track file. Strava accepts this for activity upload.
# Note: GPX timestamps must use a past date — Strava rejects future timestamps.
# This writer omits timestamps (track-only, no timed data); for Strava upload use
# the FIT file or add timestamps separately at 10 min/mile pace.

def write_gpx(track_points, name, output_path):
    lines = ['<?xml version="1.0" encoding="UTF-8"?>']
    lines.append('<gpx version="1.1" creator="detroit-wandrer" xmlns="http://www.topografix.com/GPX/1/1">')
    lines.append("  <trk>")
    lines.append(f"    <name>{name}</name>")
    lines.append("    <trkseg>")
    for lat, lon in track_points:
        lines.append(f'      <trkpt lat="{lat:.6f}" lon="{lon:.6f}"/>')
    lines.append("    </trkseg>")
    lines.append("  </trk>")
    lines.append("</gpx>")
    with open(output_path, "w") as f:
        f.write("\n".join(lines))


# ── FIT course writer ─────────────────────────────────────────────────────────
# Writes a Garmin FIT course file for the FR965 watch. FIT courses display on the
# watch as a breadcrumb map with optional turn-by-turn CoursePoint arrows.
#
# Structure required by Garmin:
#   FileIdMessage   — identifies this as a COURSE type file
#   CourseMessage   — course name and sport
#   EventMessage    — timer START
#   RecordMessage*  — one per trackpoint: position + distance + timestamp
#   LapMessage      — single lap covering the whole course
#   EventMessage    — timer STOP
#   CoursePointMessage* — one per turn (LEFT/RIGHT/etc.), positioned by distance
#
# Timestamps are anchored to 2020-01-01 08:00:00 UTC (arbitrary past date) at
# 10 min/mile pace (2.68 m/s). The watch only uses relative timing for display;
# the absolute date does not matter as long as it is in the past.

PACE_MS = 1 / 2.68  # seconds per meter at 10 min/mile


def write_fit(track_points, maneuvers, name, output_path):
    total_dist_m = sum(
        haversine_m(track_points[i][0], track_points[i][1], track_points[i+1][0], track_points[i+1][1])
        for i in range(len(track_points) - 1)
    )
    total_time_s = total_dist_m * PACE_MS
    start_unix_ms = int(time.mktime(time.strptime("2020-01-01 08:00:00", "%Y-%m-%d %H:%M:%S")) * 1000)

    builder = FitFileBuilder(auto_define=True)

    fid = FileIdMessage()
    fid.type = FileType.COURSE
    fid.manufacturer = Manufacturer.DEVELOPMENT.value
    fid.time_created = start_unix_ms
    builder.add(fid)

    course = CourseMessage()
    course.course_name = name[:16]  # Garmin display truncates at 16 chars
    course.sport = Sport.RUNNING
    course.capabilities = CourseCapabilities.NAVIGATION
    builder.add(course)

    ev_start = EventMessage()
    ev_start.event = Event.TIMER
    ev_start.event_type = EventType.START
    ev_start.timestamp = start_unix_ms
    builder.add(ev_start)

    # One RecordMessage per trackpoint: cumulative distance drives the timestamp
    cum_dist_m = 0.0
    prev = track_points[0]
    for i, (lat, lon) in enumerate(track_points):
        if i > 0:
            cum_dist_m += haversine_m(prev[0], prev[1], lat, lon)
        rec = RecordMessage()
        rec.timestamp = start_unix_ms + int(cum_dist_m * PACE_MS * 1000)
        rec.position_lat = lat
        rec.position_long = lon
        rec.distance = cum_dist_m
        builder.add(rec)
        prev = (lat, lon)

    lap = LapMessage()
    lap.timestamp = start_unix_ms + int(total_time_s * 1000)
    lap.start_time = start_unix_ms
    lap.total_elapsed_time = total_time_s
    lap.total_timer_time = total_time_s
    lap.total_distance = total_dist_m
    builder.add(lap)

    ev_stop = EventMessage()
    ev_stop.event = Event.TIMER
    ev_stop.event_type = EventType.STOP_ALL
    ev_stop.timestamp = start_unix_ms + int(total_time_s * 1000)
    builder.add(ev_stop)

    # CoursePoint records must come after all RecordMessages in FIT spec order
    if WRITE_COURSE_POINTS:
        _SKIP_CP = {CoursePoint.STRAIGHT, CoursePoint.GENERIC}
        last_cp_dist = -COURSE_POINT_MIN_DIST_M
        for lat, lon, dist_m, cp_type, instruction in maneuvers:
            if cp_type in _SKIP_CP:
                continue
            if dist_m - last_cp_dist < COURSE_POINT_MIN_DIST_M:
                continue
            cp = CoursePointMessage()
            cp.timestamp = start_unix_ms + int(dist_m * PACE_MS * 1000)
            cp.position_lat = lat
            cp.position_long = lon
            cp.distance = dist_m
            cp.type = cp_type
            cp.name = instruction[:16] if instruction else cp_type.name[:16]
            builder.add(cp)
            last_cp_dist = dist_m

    fit_file = builder.build()
    with open(output_path, "wb") as f:
        f.write(fit_file.to_bytes())


# ── Coverage & neighborhood completions report ───────────────────────────────
# A route is "good" along three independent axes (per session decision 2026-06-24):
# distance (constraint), new-mile coverage ratio (rank, no fixed floor), and
# neighborhood completions (reported separately — a checklist bonus, not folded
# into the ratio). This section measures all three against the ACTUAL routed
# path, not the candidate segment list, because endpoint-mode TSP routing can
# get rerouted onto a parallel street by Valhalla and silently miss a segment.

NEIGHBORHOODS_GEOJSON = os.path.join(DETROIT_WANDRER, "detroit_neighborhoods_wgs84.geojson")
COVER_SNAP_M = 25  # matches the post-run Strava coverage-check threshold


def _point_in_ring(lon, lat, ring):
    n = len(ring)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if ((yi > lat) != (yj > lat)) and (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def _point_in_polygon(lon, lat, polygon):
    if not _point_in_ring(lon, lat, polygon[0]):
        return False
    return not any(_point_in_ring(lon, lat, h) for h in polygon[1:])


def load_neighborhoods():
    with open(NEIGHBORHOODS_GEOJSON) as f:
        feats = json.load(f)["features"]
    nhoods = []
    for feat in feats:
        name = feat["properties"]["nhood_name"]
        geom = feat["geometry"]
        polys = [geom["coordinates"]] if geom["type"] == "Polygon" else geom["coordinates"]
        lats = [pt[1] for poly in polys for ring in poly for pt in ring]
        lons = [pt[0] for poly in polys for ring in poly for pt in ring]
        nhoods.append({"name": name, "polys": polys,
                        "bbox": (min(lats), max(lats), min(lons), max(lons))})
    return nhoods


def find_neighborhood(nhoods, lat, lon):
    for nh in nhoods:
        lat_min, lat_max, lon_min, lon_max = nh["bbox"]
        if lat_min <= lat <= lat_max and lon_min <= lon <= lon_max:
            if any(_point_in_polygon(lon, lat, p) for p in nh["polys"]):
                return nh["name"]
    return None


def boustrophedon_order(segments, row_width_m=150):
    """Order segments for snake/boustrophedon traversal of a parallel-street grid.

    Groups segments into latitude rows (each row_width_m tall), then alternates
    direction left-to-right / right-to-left between rows. Within each row,
    segments are sorted by their midpoint longitude.

    Returns a reordered list of segments. For scattered/non-grid geometry this
    degrades gracefully — segments still get visited, just not in strict snake
    order. The caller should check geometry before applying (e.g. use 2-opt for
    irregular clusters, boustrophedon for dense parallel grids).

    row_width_m: approximate N-S height of each row in meters. 150m ≈ 1 city
    block in Detroit's residential grid.
    """
    if not segments:
        return segments

    lat_degree_m = 111000.0
    row_height_deg = row_width_m / lat_degree_m

    lats = [sum(p[0] for p in s) / len(s) for s in segments]
    lons = [sum(p[1] for p in s) / len(s) for s in segments]

    lat_min = min(lats)
    indexed = sorted(
        zip(lats, lons, segments),
        key=lambda x: x[0]  # south to north
    )

    rows = []
    current_row = []
    row_base = lat_min

    for lat, lon, seg in indexed:
        if lat > row_base + row_height_deg:
            if current_row:
                rows.append(current_row)
            row_base = lat
            current_row = [(lat, lon, seg)]
        else:
            current_row.append((lat, lon, seg))
    if current_row:
        rows.append(current_row)

    ordered = []
    for i, row in enumerate(rows):
        sorted_row = sorted(row, key=lambda x: x[1], reverse=(i % 2 == 1))
        ordered.extend(seg for _, _, seg in sorted_row)

    return ordered


def boustrophedon_waypoints(segments, row_width_m=120, start_south=True, start_west=True):
    """Extract Valhalla waypoints from segments using directed boustrophedon traversal.

    Groups segments into latitude rows (each row_width_m tall), then alternates
    direction per row. Within each row, segments are sorted by midpoint longitude
    and their endpoints emitted in traversal direction.

    start_south / start_west control which corner the snake begins from:
      (True,  True)  = SW corner — S→N rows, first row W→E  (default)
      (True,  False) = SE corner — S→N rows, first row E→W
      (False, True)  = NW corner — N→S rows, first row W→E
      (False, False) = NE corner — N→S rows, first row E→W

    Returns a flat list of (lat, lon) waypoints in snake traversal order.
    """
    if not segments:
        return []

    lat_degree_m = 111000.0
    row_height_deg = row_width_m / lat_degree_m

    def midpoint(s):
        return sum(p[0] for p in s) / len(s), sum(p[1] for p in s) / len(s)

    sorted_segs = sorted(segments, key=lambda s: midpoint(s)[0])
    lat_min = midpoint(sorted_segs[0])[0]

    rows = []
    current_row = []
    row_base = lat_min
    for seg in sorted_segs:
        m_lat = midpoint(seg)[0]
        if m_lat > row_base + row_height_deg:
            if current_row:
                rows.append(current_row)
            row_base = m_lat
            current_row = [seg]
        else:
            current_row.append(seg)
    if current_row:
        rows.append(current_row)

    if not start_south:
        rows = list(reversed(rows))

    waypoints = []
    for i, row in enumerate(rows):
        west_to_east = (i % 2 == 0) == start_west  # XOR: flip direction per row parity
        row_sorted = sorted(row, key=lambda s: midpoint(s)[1], reverse=not west_to_east)
        for seg in row_sorted:
            pts_by_lon = sorted(seg, key=lambda p: p[1])
            waypoints.extend(pts_by_lon if west_to_east else reversed(pts_by_lon))

    # Collapse near-consecutive duplicates (shared intersections between segments)
    deduped = [waypoints[0]] if waypoints else []
    for pt in waypoints[1:]:
        if haversine_m(pt[0], pt[1], deduped[-1][0], deduped[-1][1]) > 5:
            deduped.append(pt)
    return deduped


def boustrophedon_variants(segments, row_width_m=120):
    """Return all 4 starting-corner variants of boustrophedon_waypoints for a chunk.
    Used by chain_chunks to pick the corner that minimizes the inter-chunk gap."""
    return [boustrophedon_waypoints(segments, row_width_m, ss, sw)
            for ss, sw in [(True, True), (True, False), (False, True), (False, False)]]


def segment_orientation(seg):
    """Return 'EW' if segment runs primarily east-west, else 'NS'."""
    lats = [p[0] for p in seg]
    lons = [p[1] for p in seg]
    dlat_m = (max(lats) - min(lats)) * 111000.0
    mean_lat = sum(lats) / len(lats)
    dlon_m = (max(lons) - min(lons)) * 111000.0 * math.cos(math.radians(mean_lat))
    return 'EW' if dlon_m >= dlat_m else 'NS'


def group_segments_by_street(segments, bucket_m=50):
    """Group segments by the street they lie on using lat/lon bucketing.

    E-W segments with midpoints within bucket_m meters of the same latitude →
    same street group. N-S segments bucketed by longitude similarly.

    Returns a list of segment lists, one per street. Within each street, segments
    are sorted along the primary axis (W→E for E-W streets, S→N for N-S streets).
    """
    lat_deg = bucket_m / 111000.0

    ew_streets = {}  # lat_bucket → [seg, ...]
    ns_streets = {}  # lon_bucket → [seg, ...]

    for seg in segments:
        mid_lat = sum(p[0] for p in seg) / len(seg)
        mid_lon = sum(p[1] for p in seg) / len(seg)
        if segment_orientation(seg) == 'EW':
            key = round(mid_lat / lat_deg) * lat_deg
            ew_streets.setdefault(key, []).append(seg)
        else:
            lon_deg = bucket_m / (111000.0 * math.cos(math.radians(mid_lat)))
            key = round(mid_lon / lon_deg) * lon_deg
            ns_streets.setdefault(key, []).append(seg)

    street_groups = []
    for segs in ew_streets.values():
        # Sort W→E along the street (chain_chunks will choose direction)
        street_groups.append(sorted(segs, key=lambda s: sum(p[1] for p in s) / len(s)))
    for segs in ns_streets.values():
        # Sort S→N along the street
        street_groups.append(sorted(segs, key=lambda s: sum(p[0] for p in s) / len(s)))

    return street_groups


def street_seg_waypoints(street_segs, west_to_east=True):
    """Extract waypoints from a street's segments: one entry + one exit per segment.

    Only emits the two endpoints of each segment (not intermediate vertices).
    Valhalla routes naturally along the segment between the endpoints, so
    intermediate vertices aren't needed as break points — they only inflate turn count.
    """
    waypoints = []
    for seg in street_segs:
        if segment_orientation(seg) == 'EW':
            pts = sorted(seg, key=lambda p: p[1])
            if west_to_east:
                waypoints.extend([pts[0], pts[-1]])
            else:
                waypoints.extend([pts[-1], pts[0]])
        else:
            pts = sorted(seg, key=lambda p: p[0])  # always S→N
            waypoints.extend([pts[0], pts[-1]])
    return waypoints


def street_boustrophedon_waypoints(segments, bucket_m=50, loop=False):
    """Order waypoints by grouping segments onto streets, then chaining streets
    with nearest-neighbor + 4-corner optimization (same as chain_chunks).

    Within each street: segments sorted along the street, all vertices emitted
    → Valhalla has no freedom to deviate within the street block.
    Between streets: chain_chunks nearest-neighbor minimizes inter-street gaps
    → the only connectors are single cross-street jumps.
    """
    if not segments:
        return []

    streets = group_segments_by_street(segments, bucket_m)

    # Build 4-corner variants for each street (W→E, E→W, reversed W→E, reversed E→W)
    # equivalent to boustrophedon_variants but at the street level
    street_variants = []
    for street_segs in streets:
        fwd = street_seg_waypoints(street_segs, west_to_east=True)
        rev = street_seg_waypoints(list(reversed(street_segs)), west_to_east=False)
        fwd_r = list(reversed(fwd))
        rev_r = list(reversed(rev))
        street_variants.append([fwd, rev, fwd_r, rev_r])

    if not street_variants:
        return []

    # Nearest-neighbor chaining with 4-corner selection (reuses chain_chunks logic)
    remaining = list(range(len(street_variants)))
    ordered_wps = list(street_variants[0][0])  # start with first street, W→E
    remaining.remove(0)

    while remaining:
        tail = ordered_wps[-1]
        best_i, best_dist, best_wps = None, float("inf"), None
        for i in remaining:
            for var in street_variants[i]:
                if not var:
                    continue
                d = haversine_m(tail[0], tail[1], var[0][0], var[0][1])
                if d < best_dist:
                    best_i, best_dist, best_wps = i, d, var
        ordered_wps.extend(best_wps)
        remaining.remove(best_i)

    # Collapse near-consecutive duplicates
    deduped = [ordered_wps[0]]
    for pt in ordered_wps[1:]:
        if haversine_m(pt[0], pt[1], deduped[-1][0], deduped[-1][1]) > 5:
            deduped.append(pt)

    if loop and deduped:
        deduped.append(deduped[0])

    return deduped


_SEGS_PER_CHUNK = 12  # target segments per chunk; smaller = finer coverage, better efficiency

def auto_chunk_grid(n_segs, bbox):
    """Compute (n_rows, n_cols) for a chunk grid that gives ~_SEGS_PER_CHUNK segments
    per chunk, shaped proportionally to the bbox's aspect ratio."""
    lat_min, lat_max, lon_min, lon_max = bbox
    lat_m = (lat_max - lat_min) * 111000.0
    lon_m = ((lon_max - lon_min) * 111000.0
             * math.cos(math.radians((lat_min + lat_max) / 2)))
    aspect = lon_m / lat_m if lat_m > 0 else 1.0
    n_chunks = max(1, round(n_segs / _SEGS_PER_CHUNK))
    n_cols = max(1, round(math.sqrt(n_chunks * aspect)))
    n_rows = max(1, round(n_chunks / n_cols))
    return n_rows, n_cols


def divide_into_chunks(segments, bbox, n_rows, n_cols):
    """Assign segments to cells of an n_rows × n_cols grid over the bbox.
    Returns list of (row, col, segment_list) for non-empty cells, sorted S→N, W→E."""
    lat_min, lat_max, lon_min, lon_max = bbox
    lat_step = (lat_max - lat_min) / n_rows
    lon_step = (lon_max - lon_min) / n_cols
    grid = {}
    for seg in segments:
        mid_lat = sum(p[0] for p in seg) / len(seg)
        mid_lon = sum(p[1] for p in seg) / len(seg)
        r = min(int((mid_lat - lat_min) / lat_step), n_rows - 1)
        c = min(int((mid_lon - lon_min) / lon_step), n_cols - 1)
        grid.setdefault((r, c), []).append(seg)
    return [(r, c, segs) for (r, c), segs in sorted(grid.items()) if segs]


def chain_chunks(chunk_seg_lists, row_width_m, loop):
    """Order chunks using nearest-neighbor, trying all 4 starting corners per candidate
    to minimize the inter-chunk gap at each step. Returns a flat ordered waypoint list."""
    if not chunk_seg_lists:
        return []

    # Precompute all 4 corner variants for each chunk
    variants = [boustrophedon_variants(segs, row_width_m) for segs in chunk_seg_lists]

    remaining = list(range(len(chunk_seg_lists)))
    # Start with whichever corner of chunk 0 puts the entry point nearest to the centroid
    # of all other chunks (heuristic: just use the default SW corner for the first chunk)
    ordered_wps = list(variants[0][0])
    remaining.remove(0)

    while remaining:
        tail = ordered_wps[-1]
        best_i, best_dist, best_wps = None, float("inf"), None
        for i in remaining:
            for var in variants[i]:
                d = haversine_m(tail[0], tail[1], var[0][0], var[0][1])
                if d < best_dist:
                    best_i, best_dist, best_wps = i, d, var
        ordered_wps.extend(best_wps)
        remaining.remove(best_i)

    if loop:
        ordered_wps.append(ordered_wps[0])
    return ordered_wps


def segment_covered(seg, track_points, threshold_m):
    """A candidate segment counts as actually-run if every vertex of it falls
    within threshold_m of the real routed path (not just its endpoints) —
    catches cases where Valhalla rerouted onto an adjacent parallel street."""
    for vlat, vlon in seg:
        best = min(haversine_m(vlat, vlon, tlat, tlon) for tlat, tlon in track_points)
        if best >= threshold_m:
            return False
    return True


def coverage_and_completions_report(all_segments, near_segments, track_points, total_miles):
    nhoods = load_neighborhoods()

    seg_nhood = {}
    for s in all_segments:
        mid_lat = sum(p[0] for p in s) / len(s)
        mid_lon = sum(p[1] for p in s) / len(s)
        seg_nhood[id(s)] = find_neighborhood(nhoods, mid_lat, mid_lon)

    covered_mi = 0.0
    covered_segs = []
    for s in near_segments:
        if segment_covered(s, track_points, COVER_SNAP_M):
            covered_segs.append(s)
            covered_mi += sum(
                haversine_m(s[i][0], s[i][1], s[i + 1][0], s[i + 1][1])
                for i in range(len(s) - 1)
            ) / 1609.34

    ratio = covered_mi / total_miles if total_miles else 0.0

    nhood_total_remaining = {}
    for s in all_segments:
        nh = seg_nhood[id(s)]
        if nh:
            nhood_total_remaining[nh] = nhood_total_remaining.get(nh, 0) + 1

    nhood_covered_count = {}
    for s in covered_segs:
        nh = seg_nhood[id(s)]
        if nh:
            nhood_covered_count[nh] = nhood_covered_count.get(nh, 0) + 1

    completions = [nh for nh, count in nhood_covered_count.items()
                   if count == nhood_total_remaining.get(nh, -1)]

    print("\n  COVERAGE SUMMARY")
    print(f"  Distance:    {total_miles:.2f} mi")
    print(f"  New miles:   {covered_mi:.2f} mi ({ratio * 100:.1f}% efficiency)")
    print(f"  Completes:   {', '.join(completions) if completions else 'none'}")


# ── Main ──────────────────────────────────────────────────────────────────────
# Execution flow: parse KMZ → filter to zone → deduplicate waypoints → TSP →
# Valhalla route → write GPX + FIT.

# Step 1: Load all untraveled segments from the Wandrer KMZ export
print(f"Parsing {SOURCE_KMZ}...")
all_segments = parse_kmz_linestrings(SOURCE_KMZ)

# Step 2: Filter to the target zone and apply minimum segment length filter
near_segments = []
for s in all_segments:
    mid_lat = sum(p[0] for p in s) / len(s)
    mid_lon = sum(p[1] for p in s) / len(s)
    seg_len = sum(haversine_m(s[i][0], s[i][1], s[i+1][0], s[i+1][1]) for i in range(len(s)-1))
    if MIN_SEG_M and seg_len < MIN_SEG_M:
        continue
    if FILTER_MODE == "bbox":
        lat_min, lat_max, lon_min, lon_max = BBOX
        if lat_min <= mid_lat <= lat_max and lon_min <= mid_lon <= lon_max:
            near_segments.append(s)
    else:
        if haversine_m(ZONE_LAT, ZONE_LON, mid_lat, mid_lon) < RADIUS_MI * 1609.34:
            near_segments.append(s)

near_mi = sum(
    sum(haversine_m(s[i][0], s[i][1], s[i+1][0], s[i+1][1]) for i in range(len(s)-1))
    for s in near_segments
) / 1609.34

min_note = f", ≥{MIN_SEG_M}m segs only" if MIN_SEG_M else ""
if FILTER_MODE == "bbox":
    print(f"  {len(near_segments)} untraveled segments in bbox {BBOX} ({near_mi:.2f} mi){min_note}")
else:
    print(f"  {len(near_segments)} untraveled segments within {RADIUS_MI} mi of ({ZONE_LAT},{ZONE_LON}) ({near_mi:.2f} mi){min_note}")

# Steps 3-4: Build ordered waypoints — boustrophedon or 2-opt depending on strategy.
if ROUTE_STRATEGY == "boustrophedon":
    ew_count = sum(1 for s in near_segments if segment_orientation(s) == 'EW')
    ns_count = len(near_segments) - ew_count
    print(f"Street boustrophedon: {ew_count} E-W segs, {ns_count} N-S segs, "
          f"STREET_BUCKET_M={STREET_BUCKET_M}")
    ordered = street_boustrophedon_waypoints(near_segments, STREET_BUCKET_M, loop=LOOP)
    print(f"  {len(ordered)} waypoints")

else:
    # Original 2-opt TSP path (unchanged)
    if WAYPOINT_MODE == "endpoints":
        raw_points = []
        for s in near_segments:
            raw_points.append(s[0])
            raw_points.append(s[-1])
    elif WAYPOINT_MODE == "both":
        # Endpoints + midpoint per segment — forces full traversal of each block.
        # Midpoint prevents Valhalla from visiting endpoints via cross-streets only;
        # endpoints prevent missed coverage on long segments where midpoint alone
        # doesn't guarantee the full segment is within COVER_SNAP_M.
        raw_points = []
        for s in near_segments:
            raw_points.append(s[0])
            raw_points.append((sum(p[0] for p in s) / len(s), sum(p[1] for p in s) / len(s)))
            raw_points.append(s[-1])
    else:
        raw_points = [(sum(p[0] for p in s) / len(s), sum(p[1] for p in s) / len(s))
                      for s in near_segments]
    for lat, lon in FORCED_WAYPOINTS:
        raw_points.append((lat, lon))
    midpoints = deduplicate(raw_points, min_dist_m=DEDUP_M)
    print(f"Waypoints after dedup ({WAYPOINT_MODE}): {len(midpoints)}")

    chunk_k = CHUNK_K
    if chunk_k == "auto":
        chunk_k = max(1, math.ceil(len(midpoints) / 40))  # 40 not 45: leaves room for uneven K-means splits under 50-node limit
        if chunk_k > 1:
            print(f"Auto CHUNK_K={chunk_k} ({len(midpoints)} waypoints)")

    _near = NEAR_LOOP_MI if not LOOP else None
    if chunk_k > 1 and len(midpoints) > 45:
        print(f"Running chunked 2-opt TSP ({chunk_k} spatial clusters)...")
        ordered = chunked_tour(midpoints, chunk_k, loop=LOOP, near_loop_mi=_near)
    else:
        if len(midpoints) > 50:
            print(f"WARNING: {len(midpoints)} waypoints exceeds Valhalla cap — truncating to 50")
            midpoints = midpoints[:50]
        print("Fetching walking distance matrix...")
        dist = routing_matrix(midpoints)
        print("Running 2-opt TSP (all starting points)...")
        tour = best_2opt_tour(midpoints, dist, loop=LOOP, near_loop_mi=_near)
        ordered = [midpoints[i] for i in tour]
        if LOOP:
            ordered.append(ordered[0])

# Step 5: Get the actual walking route from Valhalla
print("Generating route (with maneuvers)...")
track_points, total_miles, maneuvers = routing_route(ordered)
print(f"Route distance: {total_miles:.2f} miles")
print(f"Turn instructions: {len(maneuvers)}")
if not LOOP and track_points:
    sp, ep = track_points[0], track_points[-1]
    gap_mi = haversine_m(sp[0], sp[1], ep[0], ep[1]) / 1609.34
    print(f"Start→end gap:   {gap_mi:.2f} mi straight-line ({gap_mi * 1609.34:.0f} m)")

# Step 6: Write output files
write_gpx(track_points, ROUTE_NAME, OUTPUT_GPX)
print(f"GPX written: {OUTPUT_GPX} ({len(track_points)} trackpoints)")

write_fit(track_points, maneuvers, ROUTE_NAME, OUTPUT_FIT)
print(f"FIT written: {OUTPUT_FIT}")

# Step 7: Coverage, efficiency, and neighborhood completions
coverage_and_completions_report(all_segments, near_segments, track_points, total_miles)

# Step 8: OSM safety review — check for restricted access and physical barriers
# Queries the Overpass API for two classes of issues within the route bbox:
#   Access restrictions: foot=no, motorway/trunk, private roads
#   Physical barriers:   gates, bollards, construction, unpaved surfaces
# Any feature within BARRIER_SNAP_M of a KMZ segment midpoint is flagged.
# Includes a Google Maps link for each flag so the runner can do a visual check.
print("\nRunning OSM safety review...")

_OVERPASS = "https://maps.mail.ru/osm/tools/overpass/api/interpreter"
_BARRIER_SNAP_M = 50

def _nearest_seg_dist(lat, lon, segs):
    return min(
        haversine_m(lat, lon, sum(p[0] for p in s)/len(s), sum(p[1] for p in s)/len(s))
        for s in segs
    )

def _osm_review(bbox_segs, lat_min, lat_max, lon_min, lon_max):
    bb = f"{lat_min},{lon_min},{lat_max},{lon_max}"
    query = f"""[out:json][timeout:60];
(
  way["foot"="no"]({bb});
  way["foot"="private"]({bb});
  way["access"="no"]["highway"]({bb});
  way["access"="private"]["highway"]({bb});
  way["highway"="motorway"]({bb});
  way["highway"="motorway_link"]({bb});
  way["highway"="trunk"]({bb});
  way["highway"="trunk_link"]({bb});
  node["barrier"]({bb});
  way["barrier"]({bb});
  way["highway"="construction"]({bb});
  way["construction"]({bb});
  way["surface"="unpaved"]["highway"~"residential|service|tertiary"]({bb});
  way["surface"="gravel"]["highway"~"residential|service|tertiary"]({bb});
  way["surface"="dirt"]({bb});
);
out geom tags;"""
    try:
        resp = requests.post(_OVERPASS, data={"data": query}, timeout=90)
        resp.raise_for_status()
        elements = resp.json().get("elements", [])
    except Exception as e:
        print(f"  OSM review unavailable: {e}")
        return

    access_flags, barrier_flags, other_flags = [], [], []
    for el in elements:
        tags = el.get("tags", {})
        if el["type"] == "node":
            lat, lon = el["lat"], el["lon"]
        elif el["type"] == "way":
            geom = el.get("geometry", [])
            if not geom:
                continue
            lat = sum(p["lat"] for p in geom) / len(geom)
            lon = sum(p["lon"] for p in geom) / len(geom)
        else:
            continue

        dist = _nearest_seg_dist(lat, lon, bbox_segs)
        if dist > _BARRIER_SNAP_M:
            continue

        hw      = tags.get("highway", "")
        foot    = tags.get("foot", "")
        access  = tags.get("access", "")
        barrier = tags.get("barrier", "")
        surface = tags.get("surface", "")
        name    = tags.get("name", "(unnamed)")
        gmaps   = f"https://maps.google.com/?q={lat:.6f},{lon:.6f}"

        detail = " | ".join(f"{k}={v}" for k, v in [
            ("highway", hw), ("foot", foot), ("access", access),
            ("barrier", barrier), ("surface", surface),
        ] if v)
        entry = f"  ({lat:.5f},{lon:.5f}) {dist:3.0f}m  {name}  [{detail}]\n    {gmaps}"

        if hw in ("motorway","motorway_link","trunk","trunk_link") or foot in ("no","private") or access in ("no","private"):
            access_flags.append(entry)
        elif barrier or hw == "construction" or "construction" in tags:
            barrier_flags.append(entry)
        elif surface in ("unpaved","gravel","dirt","grass","sand"):
            other_flags.append(entry)

    if not access_flags and not barrier_flags and not other_flags:
        print("  OSM review: no issues found — route looks clean.")
        return

    if access_flags:
        print(f"\n  ACCESS RESTRICTIONS near route ({len(access_flags)}):")
        for f in access_flags:
            print(f)
    if barrier_flags:
        print(f"\n  PHYSICAL BARRIERS near route ({len(barrier_flags)}):")
        for f in barrier_flags:
            print(f)
    if other_flags:
        print(f"\n  SURFACE ISSUES near route ({len(other_flags)}):")
        for f in other_flags:
            print(f)
    print("\n  Verify flagged locations via the Google Maps links above.")

# Determine bbox from the filtered segments used for this route
if near_segments:
    _lats = [p[0] for s in near_segments for p in s]
    _lons = [p[1] for s in near_segments for p in s]
    _osm_review(near_segments, min(_lats), max(_lats), min(_lons), max(_lons))

print("Done.")
