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
import hashlib
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
_OVERPASS = "https://maps.mail.ru/osm/tools/overpass/api/interpreter"

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
BBOX = (42.4045, 42.416, -83.1624, -83.1408)  # Fitzgerald/Marygrove full cluster — 99% of neighborhood's remaining mileage

DEDUP_M = 75
# USE_ADAPTIVE_DEDUP_M: query OSM for the bbox's actual residential street
# spacing and override DEDUP_M with 70% of the tightest parallel-street gap,
# instead of the fixed 75m compromise above. Off by default — it changes which
# waypoints survive dedup (and therefore the TSP itself), so it's opt-in per
# bbox rather than a silent default-behavior change. Turn on for areas where
# 75m is suspected wrong (unusually tight or loose grids); see
# compute_adaptive_dedup_m() below.
USE_ADAPTIVE_DEDUP_M = False
ADAPTIVE_DEDUP_MULT = 0.7
ADAPTIVE_DEDUP_MIN_M = 30   # clamp floor — guards against a bad/sparse OSM read collapsing DEDUP_M to ~0
ADAPTIVE_DEDUP_MAX_M = 150  # clamp ceiling — guards against merging genuinely distinct streets
MIN_SEG_M = 30
WAYPOINT_MODE = "both"       # full coverage — midpoints force traversal of each block
CHUNK_K = 1  # 95 waypoints, well under local pyvalhalla's 200-node cap
SLUG = "fitzgerald-marygrove-0712"
ROUTE_NAME = "Fitzgerald-Marygrove"
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
# PACE_MS: assumed running pace (seconds per meter), used to derive FIT record/course-point
# timestamps and the course-point lead-in distance below. Set from the median of the last 30
# Strava runs (pulled 2026-07-09): 8:12/mi = 3.27 m/s. Previously hardcoded to 10:00/mile
# (2.68 m/s), which lagged noticeably behind actual pace.
PACE_MS = 1 / 3.27
# COURSE_POINT_LEAD_S: seconds of advance warning a turn cue should give before the actual
# corner, converted to a lead-in distance via PACE_MS. Without this, cues fire exactly at the
# corner's coordinates, which at running pace can mean the alert arrives after the turn.
COURSE_POINT_LEAD_S = 6
COURSE_POINT_LEAD_M = COURSE_POINT_LEAD_S / PACE_MS
# FORCED_WAYPOINTS: list of (lat, lon) pairs injected directly into the waypoint pool
# before dedup and TSP. Use to force coverage of specific segments that the bbox filter
# misses or that Valhalla routes around (e.g. segments near freeway interchanges).
FORCED_WAYPOINTS = [
]
# FORCE_START_POINT: (lat, lon) or None. Forces the loop's start/end to a specific
# point (e.g. a parking lot) — added to the waypoint pool like FORCED_WAYPOINTS,
# then the closed tour is rotated so this point is index 0. Rotating a closed loop
# doesn't change total distance (same edges, different cut point), so this is free.
FORCE_START_POINT = None
# USE_HEADING_CONSTRAINTS: True = inject Valhalla heading constraints based on TSP
# approach direction. Experimental — in a grid city, TSP bearings are often diagonal
# and cause expensive detours. Leave False; use USE_CHESS_TSP_PENALTY instead.
USE_HEADING_CONSTRAINTS = False
HEADING_TOLERANCE_DEG = 45
# USE_CHESS_TSP_PENALTY: True = apply directional cost penalty to the TSP distance
# matrix (chess/knight constraint in the TSP layer). For each transition A→B, if the
# straight-line bearing from A to B deviates from B's segment street bearing by more
# than CHESS_THRESHOLD_DEG, multiply the transition cost by CHESS_PENALTY_FACTOR.
# Bidirectional: east and west are equally "aligned" for an E-W segment.
USE_CHESS_TSP_PENALTY = False
CHESS_THRESHOLD_DEG = 60    # perpendicular deviation above this triggers penalty
CHESS_PENALTY_FACTOR = 2.5  # multiply misaligned transition cost by this factor
# USE_SEGMENT_CHAINING: True = discount TSP matrix legs between waypoints that came
# from the SAME KMZ segment. The TSP's real objective is minimizing CONNECTOR
# distance — travel along an untraveled segment is the goal, not a cost. Without
# this, 2-opt happily interleaves waypoints from different segments, forcing
# Valhalla to backtrack through covered streets to finish a segment later
# (Deficiency #2 in the handoff doc). Discounting same-segment legs makes
# nearest-neighbor + 2-opt chain each segment's start→mid→end consecutively.
# SEGMENT_CHAIN_FACTOR: multiplier for same-segment legs. Keep it >0 — at 0 the
# TSP can't tell monotone traversal (start→mid→end) from a backtracking order
# (start→end→mid), since both look free. 0.3 preserves the internal ordering
# while still making chaining much cheaper than interleaving.
# Single-cluster path only (CHUNK_K=1); the chunked/external-API path ignores it.
USE_SEGMENT_CHAINING = True
SEGMENT_CHAIN_FACTOR = 0.3
# USE_COVERAGE_REPAIR: after the initial route, check every near_segment NOT
# covered against the actual track. For ones within REPAIR_MAX_DIST_M, try
# splicing the segment's midpoint into the waypoint order at its nearest edge
# and re-route just that edge (A→mid, mid→B) to measure the added distance.
# Accept if the detour is under REPAIR_DETOUR_MULT × the segment's own length —
# a cheap way to recover missed segments the TSP left just off the route,
# without touching the TSP order itself.
USE_COVERAGE_REPAIR = True
REPAIR_MAX_DIST_M = 250
REPAIR_DETOUR_MULT = 4.0
# REPAIR_MAX_ROUNDS: the repair pass now runs in a bounded loop — each round
# re-measures coverage against the ACTUAL current track, splices in what's
# still cheap, and re-routes; then checks again before deciding whether
# another round helps. One round can leave a segment still under COVER_MIN_FRAC
# if its worst-covered point wasn't the only gap, so a single blind pass isn't
# enough — but rounds are expensive (each is a full re-route), so this caps it.
REPAIR_MAX_ROUNDS = 3
# USE_ACCESS_PREFILTER: before the TSP, query OSM for foot-inaccessible ways
# (foot=no/private, access=no/private, motorway/trunk and their links) in the
# route bbox and DROP any KMZ segment where AT LEAST ACCESS_PREFILTER_MIN_FRAC of
# its own LENGTH lies within ACCESS_PREFILTER_M of a restricted way's polyline
# (resampled check, same technique as covered_fraction() in seg_coverage.py — NOT
# a single-midpoint check). A midpoint-only check was tried first and wrongly
# dropped long legitimate streets whose midpoint merely passed near a freeway ramp
# or a private driveway at one point (confirmed 2026-07-09: 300+ m real segments
# excluded and reported "uncovered" that were never actually in the candidate
# pool). The fractional check only drops a segment when it's genuinely MOSTLY the
# restricted way, so there's no point spending tour distance approaching a freeway
# shoulder or a gated private drive the router can't legally traverse. Dropping
# them also keeps the coverage report honest — it only scores segments the route
# could reach. Every drop is logged with a Google Maps link and its restricted
# fraction (nothing disappears silently), and the query is cached in
# .valhalla_cache/. Barriers and surface issues are NOT drop reasons — a gate may
# be a walk-around park entrance and unpaved is still runnable; those stay as
# post-route warnings in Step 8.
USE_ACCESS_PREFILTER = True
ACCESS_PREFILTER_M = 25
ACCESS_PREFILTER_MIN_FRAC = 0.5
# USE_GEOMETRY_DENSIFY: KMZ segments can have long straight-line stretches
# between vertices (a real one: 250m with zero intermediate points). Coverage
# checks (covered_fraction) resample every 10m ALONG THAT STRAIGHT LINE — if
# the real street curves or the running path sits off the line's assumed
# position even slightly, resampled points can land 25-35m from where a
# runner actually was, scoring a real, fully-run street as a borderline
# partial. Confirmed live: a 273m segment with a 250m ungapped stretch scored
# 90% on an actual GPS run (Rutland St, Joy Community Day 1, Jul 7) purely
# from this straight-line-vs-real-street mismatch over one ~30m stretch.
# Fix: for any KMZ vertex-to-vertex gap over DENSIFY_MAX_GAP_M, find a real
# OSM way that both endpoints snap to within DENSIFY_SNAP_TOL_M, and splice in
# that way's own (denser, curve-following) vertices between them. Only adds
# points — never removes or moves the segment's own start/end — so waypoint
# extraction (which uses start/mid/end) is unaffected; only coverage/repair
# measurements get more accurate. Always on: like the repair pass, this can
# only improve measurement accuracy, never make a route worse.
USE_GEOMETRY_DENSIFY = True
DENSIFY_MAX_GAP_M = 60
DENSIFY_SNAP_TOL_M = 20
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


def seg_bearing(p1, p2):
    """Compass bearing in degrees (0=N, 90=E) from point p1 to point p2."""
    lat1, lon1 = p1[0], p1[1]
    lat2, lon2 = p2[0], p2[1]
    dlon = math.radians(lon2 - lon1)
    lat1r, lat2r = math.radians(lat1), math.radians(lat2)
    x = math.sin(dlon) * math.cos(lat2r)
    y = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


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


def deduplicate_with_meta(points, meta, min_dist_m=100):
    """Like deduplicate(), but also filters a parallel list of metadata values."""
    result_pts = []
    result_meta = []
    for p, m in zip(points, meta):
        if all(haversine_m(p[0], p[1], q[0], q[1]) > min_dist_m for q in result_pts):
            result_pts.append(p)
            result_meta.append(m)
    return result_pts, result_meta


def apply_chess_penalty(midpoints, dist, bearings, threshold_deg=60, penalty_factor=2.5):
    """
    Returns a modified copy of the distance matrix for TSP optimization only.

    For each transition A→B: if the straight-line bearing from A to B deviates
    from B's segment street bearing by more than threshold_deg, multiply dist[A][B]
    by penalty_factor. Bidirectional: east and west count equally aligned for an
    E-W segment (the TSP shouldn't care which end it enters from, only that it
    doesn't approach perpendicular).

    bearings: list parallel to midpoints — each entry is a float (segment bearing
              in degrees) or None for unconstrained waypoints.
    """
    n = len(midpoints)
    mod = [row[:] for row in dist]
    for j in range(n):
        sb = bearings[j]
        if sb is None:
            continue
        for i in range(n):
            if i == j:
                continue
            approach = seg_bearing(midpoints[i], midpoints[j])
            raw_diff = abs(approach - sb) % 360
            raw_diff = min(raw_diff, 360 - raw_diff)  # normalize to [0, 180]
            # Bidirectional: segment can be entered from either end
            bidi_diff = min(raw_diff, 180 - raw_diff)
            if bidi_diff > threshold_deg:
                mod[i][j] = mod[i][j] * penalty_factor
    return mod


def apply_segment_chain_discount(dist, seg_ids, factor):
    """
    Returns a modified copy of the distance matrix for TSP optimization only.

    Legs between two waypoints of the same KMZ segment are multiplied by
    factor (<1). Travel along an untraveled segment is coverage, not cost, so
    the TSP should be optimizing connector distance — this discount encodes
    that. seg_ids is parallel to the matrix; None entries (forced waypoints)
    never match. The tour must still be evaluated/reported against the TRUE
    matrix — a discounted length is meaningless as a distance estimate.
    """
    n = len(dist)
    mod = [row[:] for row in dist]
    for i in range(n):
        if seg_ids[i] is None:
            continue
        for j in range(n):
            if i != j and seg_ids[i] == seg_ids[j]:
                mod[i][j] = mod[i][j] * factor
    return mod


def segment_chaining_stats(tour, seg_ids):
    """
    (chained, split) counts of multi-waypoint segments in a tour.

    A segment is "chained" when all its surviving waypoints sit at consecutive
    tour positions (wraparound counts — the tour is traversed as a cycle when
    LOOP=True, and a wrapped block is still one visit). Split segments are the
    ones Valhalla will have to backtrack for.
    """
    n = len(tour)
    positions = {}
    for pos, wp_idx in enumerate(tour):
        sid = seg_ids[wp_idx]
        if sid is not None:
            positions.setdefault(sid, []).append(pos)
    chained = split = 0
    for pos_list in positions.values():
        k = len(pos_list)
        if k < 2:
            continue
        pos_set = set(pos_list)
        contiguous = any(
            all((start + off) % n in pos_set for off in range(k))
            for start in pos_list
        )
        if contiguous:
            chained += 1
        else:
            split += 1
    return chained, split


# ── Disk cache for Valhalla API calls ────────────────────────────────────────
# RADIUS_MI tuning re-runs the script 5-8 times per route, and CONFIG flag
# experiments (chaining factor, chess penalty, waypoint mode) often re-run with
# the exact same waypoint set. Cache the response keyed on the literal request
# payload so a repeat request skips the network round-trip / rate limit entirely.
_CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".valhalla_cache")


def _cache_path(kind, payload):
    key = hashlib.sha256(f"{kind}:{json.dumps(payload)}".encode()).hexdigest()
    return os.path.join(_CACHE_DIR, f"{key}.json")


def _cache_get(kind, payload):
    path = _cache_path(kind, payload)
    if os.path.exists(path):
        with open(path) as f:
            return json.load(f)
    return None


def _cache_put(kind, payload, data):
    os.makedirs(_CACHE_DIR, exist_ok=True)
    with open(_cache_path(kind, payload), "w") as f:
        json.dump(data, f)


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
    data = _cache_get("matrix", payload)
    if data is None:
        if _VALHALLA_ACTOR is not None:
            data = _json.loads(_VALHALLA_ACTOR.matrix(_json.dumps(payload)))
        else:
            resp = requests.post(f"{VALHALLA}/sources_to_targets", json=payload, timeout=120)
            resp.raise_for_status()
            data = resp.json()
        _cache_put("matrix", payload, data)
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


def two_opt_improve(tour, dist, loop, max_passes=60):
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


def or_opt_improve(tour, dist, loop, max_passes=60):
    """
    Or-opt local search: relocate chains of 1-3 consecutive waypoints to a
    better position elsewhere in the tour (forward or reversed).

    Complements 2-opt, which can only reverse a contiguous span: a waypoint
    stranded between two far-apart neighbors often can't escape via reversal
    because every reversal keeps it adjacent to at least one of them.
    Relocation moves fix exactly that case. Uses the same 1e-4 mi epsilon and
    max_passes termination guards as two_opt_improve (the Valhalla matrix
    isn't perfectly symmetric).

    Tour endpoints stay fixed: segments are taken strictly from the interior,
    and for open paths (loop=False) insertion after the final waypoint is
    disallowed. For loops, inserting after the last element places the chain
    on the closing edge back to the start.
    """
    n = len(tour)
    improved = True
    passes = 0
    while improved and passes < max_passes:
        improved = False
        passes += 1
        for seg_len in (1, 2, 3):
            i = 1
            while i + seg_len <= n - 1:
                p, q = tour[i - 1], tour[i + seg_len]
                s0, s1 = tour[i], tour[i + seg_len - 1]
                gain = dist[p][s0] + dist[s1][q] - dist[p][q]
                if gain <= 1e-4:
                    i += 1
                    continue
                seg = tour[i: i + seg_len]
                rest = tour[:i] + tour[i + seg_len:]
                m = len(rest)
                best_delta, best_j, best_rev = 1e-4, None, False
                for j in range(m if loop else m - 1):
                    c, d = rest[j], rest[(j + 1) % m]
                    base = dist[c][d]
                    for rev in (False, True):
                        a, b = (s1, s0) if rev else (s0, s1)
                        delta = gain - (dist[c][a] + dist[b][d] - base)
                        if delta > best_delta:
                            best_delta, best_j, best_rev = delta, j, rev
                if best_j is not None:
                    placed = seg[::-1] if best_rev else seg
                    tour[:] = rest[: best_j + 1] + placed + rest[best_j + 1:]
                    improved = True
                # Always advance, whether or not a move was accepted. Re-scanning the
                # same i after an accepted move (the old behavior) can spin forever
                # within a single pass if the matrix's asymmetry makes a move and its
                # reverse both look profitable — max_passes can't catch that because
                # `passes` never increments while stuck at a single i. One sweep per
                # pass is sufficient; further passes (bounded by max_passes) pick up
                # any follow-on improvement from a clean position 1 start.
                i += 1
    return tour


def local_search(tour, dist, loop, max_rounds=8):
    """
    Alternate 2-opt and Or-opt until neither finds an improvement.

    max_rounds bounds the outer alternation, same reasoning as max_passes
    inside two_opt_improve/or_opt_improve: Valhalla's matrix isn't always
    perfectly symmetric, so the two move types can occasionally keep
    trading tiny reciprocal "improvements" back and forth near the 1e-4
    epsilon boundary without settling. Confirmed in practice — a 45-waypoint
    Barton-McFarland instance ran for minutes with an unbounded outer loop
    before this cap was added, despite each individual 2-opt/Or-opt pass
    being fast on its own.
    """
    length = tour_length(tour, dist, loop)
    for _ in range(max_rounds):
        tour = two_opt_improve(tour, dist, loop)
        tour = or_opt_improve(tour, dist, loop)
        new_len = tour_length(tour, dist, loop)
        if new_len > length - 1e-4:
            return tour
        length = new_len
    return tour


def boustrophedon_tour(points, row_width_m=150):
    """
    Row-sweep starting tour: bands points into row_width_m latitude bands
    (~1 city block), snakes E-W/W-E between bands. Returns a list of point
    indices — same format as nearest_neighbor(), for use as one more candidate
    initial tour in best_2opt_tour().

    On its own, boustrophedon bloats routes 35-40% from strip-transition
    overhead (see feedback-detroit-wandrer — confirmed on College Park: 7.86mi
    vs 5.13mi with 2-opt). But as a WARM START it's just a different starting
    permutation for 2-opt to refine — in dense E-W grids it can converge to a
    different local optimum than any nearest-neighbor start does. best_2opt_tour
    only keeps it if it wins, so it can't make the result worse.
    """
    n = len(points)
    lat_degree_m = 111000.0
    row_height_deg = row_width_m / lat_degree_m
    order = sorted(range(n), key=lambda i: points[i][0])  # south to north

    rows = []
    current_row = []
    row_base = points[order[0]][0]
    for i in order:
        lat = points[i][0]
        if lat > row_base + row_height_deg:
            rows.append(current_row)
            row_base = lat
            current_row = [i]
        else:
            current_row.append(i)
    rows.append(current_row)

    tour = []
    for r, row in enumerate(rows):
        tour.extend(sorted(row, key=lambda i: points[i][1], reverse=(r % 2 == 1)))
    return tour


def best_2opt_tour(points, dist, loop, near_loop_mi=None):
    """
    Try nearest-neighbor + 2-opt from every starting waypoint, plus one
    boustrophedon warm start; return the best tour.

    Returns a list of waypoint indices (not coordinates). The caller converts to
    coordinates. If loop=True, the caller should append ordered[0] to close the loop.

    When near_loop_mi is set and loop=False, prefers open paths whose end falls
    within near_loop_mi straight-line miles of the start, choosing the shortest
    such path. Falls back to the globally shortest open path if none qualifies.
    """
    n = len(points)
    best_tour, best_len, best_source = None, float("inf"), None
    best_near_tour, best_near_len = None, float("inf")

    candidates = [("nn", nearest_neighbor(dist, start)) for start in range(n)]
    candidates.append(("boustrophedon", boustrophedon_tour(points)))

    for source, t in candidates:
        t = local_search(t, dist, loop)
        l = tour_length(t, dist, loop)
        if l < best_len:
            best_len, best_tour, best_source = l, t, source
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
        print(f"  Best tour (2-opt + Or-opt): {best_len:.2f} mi across {n} waypoints (start: {best_source})")
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


def chunked_tour(midpoints, k, loop, near_loop_mi=None, bearings=None):
    """
    Split midpoints into k spatial clusters, optimize each with 2-opt, then
    find the globally best cluster visiting order + orientation.

    Within each cluster: 2-opt TSP using the real Valhalla pedestrian matrix.
    Between clusters: exhaustive search over all N! orderings × 2^N orientations
    using Haversine as a proxy (no extra API calls). This eliminates the fixed
    S→N ordering and greedy fwd/rev chaining that caused systematic overhead.

    bearings: optional list parallel to midpoints with segment street bearings.
    When provided, applies the chess TSP penalty within each cluster's distance matrix.
    """
    # Build a lookup so we can extract per-cluster bearings after k-means splits the points
    bearing_by_pt = {}
    if bearings is not None:
        for pt, b in zip(midpoints, bearings):
            bearing_by_pt[pt] = b

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
        if bearings is not None:
            cl_bearings = [bearing_by_pt.get(pt) for pt in cl]
            local_dist = apply_chess_penalty(cl, local_dist, cl_bearings,
                                             CHESS_THRESHOLD_DEG, CHESS_PENALTY_FACTOR)
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
# _SKIP_TYPES are Valhalla's own straight/continue maneuver types — too frequent to be
# useful as watch prompts, and reliable as an initial filter regardless of angle.
_SKIP_TYPES = {7, 8, 17, 22}  # straight / continue — too noisy to show on watch

# Turn severity is classified from the actual route geometry (turn_angle_deg below) rather
# than trusting Valhalla's maneuver-type-to-severity mapping directly: Valhalla's pedestrian
# classification doesn't always match the real bearing change at an intersection (e.g. a
# genuine 90 degree corner sometimes comes back as its "slight right" type). ANGLE_WINDOW_M
# controls how far before/after the corner we look when measuring bearing, to smooth over
# GPS/shape-point jitter right at the vertex.
ANGLE_WINDOW_M = 8.0
# Thresholds (degrees of bearing change) below which a "turn" is really just a jog/curve not
# worth announcing, and above which it counts as slight / normal / sharp / u-turn.
ANGLE_MIN_TURN = 20
ANGLE_SLIGHT_MAX = 45
ANGLE_SHARP_MIN = 100
ANGLE_UTURN_MIN = 150


def turn_angle_deg(leg_points, leg_dists_m, idx):
    """Signed bearing change (degrees) at leg_points[idx], measured between a point
    ANGLE_WINDOW_M before and ANGLE_WINDOW_M after it. Positive = turns right, negative =
    turns left. Returns 0.0 if idx is too close to either end of the leg to measure."""
    target = leg_dists_m[idx]
    i = idx
    while i > 0 and target - leg_dists_m[i] < ANGLE_WINDOW_M:
        i -= 1
    j = idx
    while j < len(leg_points) - 1 and leg_dists_m[j] - target < ANGLE_WINDOW_M:
        j += 1
    if i == idx or j == idx:
        return 0.0
    bearing_in = seg_bearing(leg_points[i], leg_points[idx])
    bearing_out = seg_bearing(leg_points[idx], leg_points[j])
    return (bearing_out - bearing_in + 540) % 360 - 180


def classify_turn(angle_deg):
    """Map a signed bearing-change angle to a Garmin CoursePoint type, or None if the
    angle is too small to be worth an audible cue."""
    a = abs(angle_deg)
    if a < ANGLE_MIN_TURN:
        return None
    right = angle_deg > 0
    if a >= ANGLE_UTURN_MIN:
        return CoursePoint.U_TURN
    if a >= ANGLE_SHARP_MIN:
        return CoursePoint.SHARP_RIGHT if right else CoursePoint.SHARP_LEFT
    if a >= ANGLE_SLIGHT_MAX:
        return CoursePoint.RIGHT if right else CoursePoint.LEFT
    return CoursePoint.SLIGHT_RIGHT if right else CoursePoint.SLIGHT_LEFT


def _point_at_distance(points, dists_m, target_m):
    """Linearly interpolate (lat, lon) at target_m along a polyline described by the
    parallel points/dists_m arrays (dists_m non-decreasing, dists_m[0] == 0)."""
    if target_m <= 0:
        return points[0]
    for k in range(1, len(dists_m)):
        if dists_m[k] >= target_m:
            seg_len = dists_m[k] - dists_m[k - 1]
            frac = 0.0 if seg_len == 0 else (target_m - dists_m[k - 1]) / seg_len
            lat = points[k - 1][0] + frac * (points[k][0] - points[k - 1][0])
            lon = points[k - 1][1] + frac * (points[k][1] - points[k - 1][1])
            return (lat, lon)
    return points[-1]


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

        locations = []
        for pt in chunk:
            loc = {"lat": pt[0], "lon": pt[1], "type": "break"}
            if USE_HEADING_CONSTRAINTS and len(pt) > 2 and pt[2] is not None:
                loc["heading"] = int(pt[2]) % 360
                loc["heading_tolerance"] = HEADING_TOLERANCE_DEG
            locations.append(loc)

        payload = {"locations": locations, "costing": "pedestrian", "units": "miles"}
        data = _cache_get("route", payload)
        if data is None:
            if _VALHALLA_ACTOR is not None:
                data = _json.loads(_VALHALLA_ACTOR.route(_json.dumps(payload)))
            else:
                resp = requests.post(f"{VALHALLA}/route", json=payload, timeout=60)
                if not resp.ok:
                    print(f"  Valhalla error (chunk i={i}, {len(locations)} locs): "
                          f"HTTP {resp.status_code} — {resp.text[:400]}")
                resp.raise_for_status()
                data = resp.json()
            _cache_put("route", payload, data)

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
                idx = min(maneuver.get("begin_shape_index", 0), len(leg_points) - 1)
                dist_in_leg_m = leg_dists_m[idx]

                angle = turn_angle_deg(leg_points, leg_dists_m, idx)
                cp_type = classify_turn(angle)
                if cp_type is None:
                    continue  # geometry says this isn't a real turn, regardless of Valhalla's label

                lead_dist_m = max(0.0, dist_in_leg_m - COURSE_POINT_LEAD_M)
                lat, lon = _point_at_distance(leg_points, leg_dists_m, lead_dist_m)
                instruction = maneuver.get("instruction", "")
                maneuvers.append((lat, lon, cumulative_m + lead_dist_m, cp_type, instruction))

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
# Timestamps are anchored to 2020-01-01 08:00:00 UTC (arbitrary past date) at PACE_MS
# (see config section above). The watch only uses relative timing for display; the
# absolute date does not matter as long as it is in the past.


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
    track_dists_m = [0.0]
    for i, (lat, lon) in enumerate(track_points):
        if i > 0:
            cum_dist_m += haversine_m(prev[0], prev[1], lat, lon)
            track_dists_m.append(cum_dist_m)
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

    # CoursePoint records must come after all RecordMessages in FIT spec order.
    # Turn cues and mile markers are collected into one list and written in
    # ascending distance order (not append order) since some devices assume
    # course points arrive in course order.
    if WRITE_COURSE_POINTS:
        _SKIP_CP = {CoursePoint.STRAIGHT, CoursePoint.GENERIC}
        cps = []
        last_cp_dist = -COURSE_POINT_MIN_DIST_M
        for lat, lon, dist_m, cp_type, instruction in maneuvers:
            if cp_type in _SKIP_CP:
                continue
            if dist_m - last_cp_dist < COURSE_POINT_MIN_DIST_M:
                continue
            name = instruction[:16] if instruction else cp_type.name[:16]
            cps.append((dist_m, lat, lon, cp_type, name))
            last_cp_dist = dist_m

        # Mile markers: GENERIC distance callouts, independent of the turn-spacing
        # dedup above. Also, per Garmin forum reports, Garmin Connect Mobile's course
        # import strips directional (LEFT/RIGHT/etc.) course points but leaves
        # non-directional ones intact — so these should survive even if the turn
        # cues above don't, on the same iPhone share-sheet transfer path.
        mile_m = 1609.34
        n_miles = int(total_dist_m // mile_m)
        for m in range(1, n_miles + 1):
            target = m * mile_m
            lat, lon = _point_at_distance(track_points, track_dists_m, target)
            cps.append((target, lat, lon, CoursePoint.GENERIC, f"{m} mi"[:16]))

        for dist_m, lat, lon, cp_type, cp_name in sorted(cps, key=lambda c: c[0]):
            cp = CoursePointMessage()
            cp.timestamp = start_unix_ms + int(dist_m * PACE_MS * 1000)
            cp.position_lat = lat
            cp.position_long = lon
            cp.distance = dist_m
            cp.type = cp_type
            cp.course_point_name = cp_name
            builder.add(cp)

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
# Coverage rule is shared with strip_covered_segments.py via seg_coverage so
# the planned-route report and the post-run KMZ strip can never drift apart.
# A segment counts as covered when >= COVER_MIN_FRAC of its LENGTH is within
# COVER_SNAP_M of the track polyline (approximates Wandrer's crediting rule).
from seg_coverage import (TrackIndex, covered_fraction, resample_polyline,
                          COVER_SNAP_M, COVER_MIN_FRAC, SAMPLE_STEP_M)


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


# ── Adaptive DEDUP_M ──────────────────────────────────────────────────────────
# The fixed DEDUP_M=75m is a citywide compromise. Some grids are tighter or
# looser than that, so measure the ACTUAL parallel-street spacing from OSM
# residential ways in the bbox instead of assuming it.

def estimate_street_spacing_m(bbox, merge_m=30.0, max_skew_m=60.0):
    """
    Query OSM for residential/tertiary/unclassified streets in the bbox.
    highway=service (alleys, driveways, parking aisles) is deliberately
    EXCLUDED — those sit between streets and would halve the measured spacing.

    For each orientation, collapse ways into "street rows" by 1-D band
    clustering: collect the vertex latitudes of EW ways (longitudes of NS
    ways), sort, and merge consecutive values closer than merge_m into one
    band. Each band is one physical street regardless of how many OSM
    way-segments it's split into or how much it drifts; the median gap
    between adjacent band CENTERS is the parallel-street spacing. (Fixed-grid
    bucketing fails here: contiguous occupied buckets make every gap exactly
    one bucket wide.) Ways skewed more than max_skew_m across their own axis
    (diagonals, curves) are skipped — one diagonal would smear across every
    row and merge them all into a single band.

    Returns (spacing_ew_m, spacing_ns_m) — either is None if that orientation
    had fewer than 2 street bands, or the query itself failed.
    """
    lat_min, lat_max, lon_min, lon_max = bbox
    bb = f"{lat_min},{lon_min},{lat_max},{lon_max}"
    query = f"""[out:json][timeout:60];
(
  way["highway"~"^(residential|tertiary|unclassified)$"]({bb});
);
out geom;"""
    data = _cache_get("overpass", {"q": query})
    if data is None:
        try:
            resp = requests.post(_OVERPASS, data={"data": query}, timeout=90)
            resp.raise_for_status()
            data = resp.json()
        except Exception as e:
            print(f"  Street spacing query unavailable ({type(e).__name__}), falling back to fixed DEDUP_M")
            return None, None
        _cache_put("overpass", {"q": query}, data)
    elements = data.get("elements", [])

    mean_lat = (lat_min + lat_max) / 2
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(mean_lat))

    ew_vals, ns_vals = [], []  # cross-axis positions in meters
    for el in elements:
        geom = el.get("geometry") or []
        pts = [(p["lat"], p["lon"]) for p in geom if p]
        if len(pts) < 2:
            continue
        lats = [p[0] for p in pts]
        lons = [p[1] for p in pts]
        if segment_orientation(pts) == 'EW':
            if (max(lats) - min(lats)) * mlat > max_skew_m:
                continue
            ew_vals.extend(lat * mlat for lat in lats)
        else:
            if (max(lons) - min(lons)) * mlon > max_skew_m:
                continue
            ns_vals.extend(lon * mlon for lon in lons)

    def _band_spacing(vals):
        if len(vals) < 2:
            return None
        vals = sorted(vals)
        bands = [[vals[0]]]
        for v in vals[1:]:
            if v - bands[-1][-1] <= merge_m:
                bands[-1].append(v)
            else:
                bands.append([v])
        if len(bands) < 2:
            return None
        centers = [sum(b) / len(b) for b in bands]
        gaps = sorted(centers[i + 1] - centers[i] for i in range(len(centers) - 1))
        return gaps[len(gaps) // 2]

    return _band_spacing(ew_vals), _band_spacing(ns_vals)


def compute_adaptive_dedup_m(bbox, fallback_m, mult=ADAPTIVE_DEDUP_MULT,
                              min_m=ADAPTIVE_DEDUP_MIN_M, max_m=ADAPTIVE_DEDUP_MAX_M):
    """DEDUP_M override from measured street spacing, or fallback_m if unmeasurable."""
    spacing_ew, spacing_ns = estimate_street_spacing_m(bbox)
    spacings = [s for s in (spacing_ew, spacing_ns) if s is not None]
    ew_str = f"{spacing_ew:.0f}m" if spacing_ew is not None else "n/a"
    ns_str = f"{spacing_ns:.0f}m" if spacing_ns is not None else "n/a"
    if not spacings:
        print(f"  Adaptive DEDUP_M: EW={ew_str} NS={ns_str} — not enough OSM "
              f"street data, using fixed {fallback_m}m")
        return fallback_m
    dedup_m = max(min_m, min(max_m, mult * min(spacings)))
    print(f"  Adaptive DEDUP_M: EW={ew_str} NS={ns_str} → {dedup_m:.0f}m "
          f"({mult:.0%} of tightest, clamped [{min_m},{max_m}]) vs fixed {fallback_m}m")
    return dedup_m


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


def coverage_and_completions_report(all_segments, near_segments, track_points, total_miles):
    nhoods = load_neighborhoods()

    seg_nhood = {}
    for s in all_segments:
        mid_lat = sum(p[0] for p in s) / len(s)
        mid_lon = sum(p[1] for p in s) / len(s)
        seg_nhood[id(s)] = find_neighborhood(nhoods, mid_lat, mid_lon)

    covered_mi = 0.0
    covered_segs = []
    partial_segs = []  # 30-90% of length near track: candidates for a repair pass
    tindex = TrackIndex(track_points)
    for s in near_segments:
        frac = covered_fraction(s, tindex)
        if frac >= COVER_MIN_FRAC:
            covered_segs.append(s)
            covered_mi += sum(
                haversine_m(s[i][0], s[i][1], s[i + 1][0], s[i + 1][1])
                for i in range(len(s) - 1)
            ) / 1609.34
        elif frac >= 0.3:
            partial_segs.append((s, frac))

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
    print(f"  Segments:    {len(covered_segs)}/{len(near_segments)} covered"
          f" (>= {COVER_MIN_FRAC:.0%} of length within {COVER_SNAP_M:.0f}m)")
    if partial_segs:
        print(f"  Partial:     {len(partial_segs)} segments 30-90% covered"
              f" — near misses worth a manual look")
    print(f"  Completes:   {', '.join(completions) if completions else 'none'}")


# ── Coverage repair pass ──────────────────────────────────────────────────────
# Segments can end up "not covered" even though their own waypoints went into
# the TSP: at DEDUP_M=75m > COVER_SNAP_M=25m, a segment's waypoint can get
# merged into a nearby DIFFERENT segment's representative point, so the routed
# path passes near the kept point but not within COVER_SNAP_M of this segment's
# own polyline. Splicing the missed segment's own midpoint back in as an extra
# via-point (at its nearest edge in the existing tour) recovers these cheaply,
# without touching the TSP order.

def _point_to_segment_m(px_lat, px_lon, ax_lat, ax_lon, bx_lat, bx_lon):
    """Approx point-to-segment distance in meters (local equirectangular projection)."""
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(px_lat))
    px, py = px_lon * mlon, px_lat * mlat
    ax, ay = ax_lon * mlon, ax_lat * mlat
    bx, by = bx_lon * mlon, bx_lat * mlat
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return haversine_m(px_lat, px_lon, ax_lat, ax_lon)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = min(max(t, 0.0), 1.0)
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy)


def _nearest_edge_index(pt, waypoints):
    """Index i such that (waypoints[i], waypoints[i+1]) is the closest edge to pt."""
    best_i, best_d = 0, float("inf")
    for i in range(len(waypoints) - 1):
        d = _point_to_segment_m(pt[0], pt[1], waypoints[i][0], waypoints[i][1],
                                 waypoints[i + 1][0], waypoints[i + 1][1])
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def fetch_osm_ways_for_bbox(bbox):
    """
    Query OSM for residential/tertiary/unclassified/service ways in bbox,
    cached in .valhalla_cache/. Returns a list of polylines (each a list of
    (lat, lon) tuples). Service included here (unlike estimate_street_spacing_m,
    which deliberately excludes it) — an alley or driveway is still a real
    street a KMZ segment can legitimately need snapping against.

    On any Overpass failure, returns [] — densify_segment falls back to the
    original (un-densified) geometry for every gap, so a network hiccup can
    only lose the accuracy improvement, never break anything.
    """
    lat_min, lat_max, lon_min, lon_max = bbox
    bb = f"{lat_min},{lon_min},{lat_max},{lon_max}"
    query = f"""[out:json][timeout:60];
(
  way["highway"~"^(residential|tertiary|unclassified|service)$"]({bb});
);
out geom;"""
    data = _cache_get("overpass", {"q": query})
    if data is None:
        try:
            resp = requests.post(_OVERPASS, data={"data": query}, timeout=90)
            resp.raise_for_status()
            data = resp.json()
            _cache_put("overpass", {"q": query}, data)
        except Exception as e:
            print(f"  Geometry densify: OSM ways unavailable ({e}) — using original KMZ geometry")
            return []

    ways = []
    for el in data.get("elements", []):
        geom = el.get("geometry", [])
        if len(geom) >= 2:
            ways.append([(p["lat"], p["lon"]) for p in geom])
    return ways


def densify_segment(seg, osm_ways, max_gap_m=DENSIFY_MAX_GAP_M, snap_tol_m=DENSIFY_SNAP_TOL_M):
    """
    Replace long straight-line vertex-to-vertex gaps in `seg` with the
    matching stretch of a real OSM way's own (denser) vertices, when a way
    can be confidently identified — both gap endpoints must independently
    snap within snap_tol_m of the SAME way. Gaps under max_gap_m, and gaps
    where no single way matches both ends, are left as-is (never removes
    accuracy, only sometimes fails to add it).

    seg[0] and seg[-1] are always preserved exactly — this only inserts
    interior points, so waypoint extraction (start/mid/end) is unaffected.
    """
    out = [seg[0]]
    for i in range(len(seg) - 1):
        a, b = seg[i], seg[i + 1]
        gap_m = haversine_m(a[0], a[1], b[0], b[1])
        if gap_m <= max_gap_m:
            out.append(b)
            continue

        best_way = None
        for way in osm_ways:
            da = min(_point_to_segment_m(a[0], a[1], way[j][0], way[j][1], way[j + 1][0], way[j + 1][1])
                     for j in range(len(way) - 1))
            if da > snap_tol_m:
                continue
            db = min(_point_to_segment_m(b[0], b[1], way[j][0], way[j][1], way[j + 1][0], way[j + 1][1])
                      for j in range(len(way) - 1))
            if db > snap_tol_m:
                continue
            best_way = way
            break

        if best_way is None:
            out.append(b)
            continue

        ia = min(range(len(best_way)), key=lambda j: haversine_m(best_way[j][0], best_way[j][1], a[0], a[1]))
        ib = min(range(len(best_way)), key=lambda j: haversine_m(best_way[j][0], best_way[j][1], b[0], b[1]))
        lo, hi = (ia, ib) if ia <= ib else (ib, ia)
        insert = best_way[lo:hi + 1]
        if ia > ib:
            insert = insert[::-1]
        out.extend(insert[1:-1])  # drop both ends: first is near `a` (already in `out`), last is near `b` (added next)
        out.append(b)
    # Mutate in place (not `return out` as a new list): coverage_and_completions_report
    # keys a dict by id(s) across all_segments vs near_segments/covered_segs. Since Step
    # 2 appends the SAME segment objects (not copies) into near_segments, returning a
    # fresh list here would silently break that identity and KeyError downstream.
    seg[:] = out
    return seg


def densify_all_segments(segments, bbox):
    """Apply densify_segment to every segment, using one shared OSM-ways fetch for bbox."""
    osm_ways = fetch_osm_ways_for_bbox(bbox)
    if not osm_ways:
        return segments
    return [densify_segment(s, osm_ways) for s in segments]


def prefilter_inaccessible_segments(segments, bbox, snap_m=ACCESS_PREFILTER_M):
    """
    Drop KMZ segments that coincide with a foot-inaccessible OSM way.

    Queries Overpass (cached) for ways a pedestrian can't legally walk —
    foot=no/private, access=no/private, and motorway/trunk plus their links —
    within `bbox`, then drops any segment whose midpoint is within snap_m of one
    of those ways' polylines. At snap distance the segment isn't merely near the
    restricted way, it IS that way, so routing to it wastes tour distance on a
    shoulder or gated drive the router can't traverse anyway.

    bbox is (lat_min, lat_max, lon_min, lon_max). Returns (kept, dropped) where
    dropped is a list of (segment, reason_string) for logging. On any Overpass
    failure it returns everything kept — a pre-filter must never lose segments to
    a network hiccup.
    """
    lat_min, lat_max, lon_min, lon_max = bbox
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
);
out geom tags;"""
    data = _cache_get("overpass", {"q": query})
    if data is None:
        try:
            resp = requests.post(_OVERPASS, data={"data": query}, timeout=90)
            resp.raise_for_status()
            data = resp.json()
            _cache_put("overpass", {"q": query}, data)
        except Exception as e:
            print(f"  Access pre-filter unavailable ({e}) — keeping all segments")
            return list(segments), []

    restricted = []  # (list-of-(lat,lon) polyline, reason)
    for el in data.get("elements", []):
        if el.get("type") != "way":
            continue
        geom = el.get("geometry", [])
        if len(geom) < 2:
            continue
        tags = el.get("tags", {})
        # foot=yes/designated is an explicit pedestrian override — respect it and
        # don't drop, even on an otherwise access=private way (e.g. a private road
        # with a signed public footway).
        if tags.get("foot") in ("yes", "designated", "permissive"):
            continue
        hw = tags.get("highway", "")
        if hw in ("motorway", "motorway_link", "trunk", "trunk_link"):
            reason = f"highway={hw}"
        elif tags.get("foot") in ("no", "private"):
            reason = f"foot={tags['foot']}"
        elif tags.get("access") in ("no", "private"):
            reason = f"access={tags['access']}"
        else:
            continue
        name = tags.get("name", "(unnamed)")
        poly = [(p["lat"], p["lon"]) for p in geom]
        restricted.append((poly, f"{name} [{reason}]"))

    if not restricted:
        return list(segments), []

    # Length-weighted check, not a single-midpoint check: a segment's midpoint can
    # land within snap_m of a restricted way even when most of the segment's actual
    # length is a normal public street (e.g. a long street that merely passes near a
    # freeway ramp or a private driveway at one point). Resampling the segment's own
    # polyline and requiring a MAJORITY of its length to be near a restricted way
    # (not just the midpoint) avoids nuking long legitimate streets over a localized
    # obstruction — mirrors the covered_fraction()/resample_polyline() technique
    # already used for track coverage in seg_coverage.py.
    kept, dropped = [], []
    for s in segments:
        mid = (sum(p[0] for p in s) / len(s), sum(p[1] for p in s) / len(s))
        samples = resample_polyline(s, SAMPLE_STEP_M)
        near_hits = 0
        best_reason = None
        for lat, lon in samples:
            hit_d, hit_reason = float("inf"), None
            for poly, reason in restricted:
                for i in range(len(poly) - 1):
                    d = _point_to_segment_m(lat, lon,
                                            poly[i][0], poly[i][1],
                                            poly[i + 1][0], poly[i + 1][1])
                    if d < hit_d:
                        hit_d, hit_reason = d, reason
                    if hit_d <= snap_m:
                        break
                if hit_d <= snap_m:
                    break
            if hit_d <= snap_m:
                near_hits += 1
                best_reason = best_reason or hit_reason
        frac = near_hits / len(samples)
        if frac >= ACCESS_PREFILTER_MIN_FRAC:
            dropped.append((s, best_reason, mid, frac))
        else:
            kept.append(s)
    return kept, dropped


def _worst_covered_point(seg, tindex, snap_m=COVER_SNAP_M, step_m=SAMPLE_STEP_M):
    """
    The point on segment `seg`'s own polyline that is farthest from the
    current track — i.e. the actual location of its coverage gap, not a
    geometric guess. Uses the same resample_polyline() grid as
    covered_fraction()/segment_covered(), so "worst point" and "is this
    segment covered" agree about where the segment's samples fall.

    Returns (point, distance_m). distance_m is the point's real distance to
    the nearest track leg, up to a generous search cutoff — comfortably above
    any real repair radius, so it never affects the accept/reject decision.
    Sample points beyond the cutoff are all treated as equally "far" (the
    cutoff value itself) since we only need to rank candidates near the
    repair threshold, not measure genuinely distant misses precisely.

    IMPORTANT: TrackIndex.min_dist_m's search radius (in grid cells) grows
    with the cutoff, so a naive "just search really far" cutoff (e.g. 100km)
    turns each call into a multi-million-cell scan — this blew up to minutes
    of runtime on a 46-segment test. Keep this cutoff in the hundreds-of-
    meters range.
    """
    _FAR_CUTOFF_M = 1000.0
    samples = resample_polyline(seg, step_m)
    worst_pt, worst_d = samples[0], -1.0
    for lat, lon in samples:
        d = tindex.min_dist_m(lat, lon, _FAR_CUTOFF_M)
        d = d if d is not None else _FAR_CUTOFF_M
        if d > worst_d:
            worst_d, worst_pt = d, (lat, lon)
    return worst_pt, worst_d


def coverage_repair_pass(near_segments, ordered, track_points,
                          max_dist_m=REPAIR_MAX_DIST_M, detour_mult=REPAIR_DETOUR_MULT):
    """
    For each near_segment not covered by the current route, find its WORST-
    COVERED point — the spot on its own polyline farthest from the track,
    pinpointing the actual gap rather than assuming it's near the segment's
    geometric center (it often isn't: a segment can be 80%+ covered with the
    gap concentrated at one end, which a centroid via-point would miss
    entirely). If that gap point is within max_dist_m of the track, try
    splicing it into `ordered` at its nearest edge and check the real routed
    detour (A→gap + gap→B) − (A→B). Accept if the detour is under
    detour_mult × the segment's own length.

    Candidates are processed closest-to-track first, and each insertion is
    made against the current (possibly already-repaired) waypoint order, so
    later candidates see earlier accepted insertions. One call only ever
    inserts ONE point per segment — some gaps need more than that to clear
    COVER_MIN_FRAC, which is why the caller loops this over REPAIR_MAX_ROUNDS
    rounds rather than treating a single pass as final.

    Returns (new_ordered, accepted_count, candidate_count, added_mi).
    """
    tindex = TrackIndex(track_points)
    candidates = []
    for s in near_segments:
        frac = covered_fraction(s, tindex)
        if frac >= COVER_MIN_FRAC:
            continue
        gap_pt, gap_d = _worst_covered_point(s, tindex)
        if gap_d <= max_dist_m:
            seg_len_mi = sum(
                haversine_m(s[i][0], s[i][1], s[i + 1][0], s[i + 1][1])
                for i in range(len(s) - 1)
            ) / 1609.34
            candidates.append((gap_d, gap_pt, seg_len_mi))
    candidates.sort(key=lambda c: c[0])

    new_ordered = list(ordered)
    accepted = 0
    added_mi = 0.0
    for _, gap_pt, seg_len_mi in candidates:
        i = _nearest_edge_index(gap_pt, new_ordered)
        a, b = new_ordered[i], new_ordered[i + 1]
        edge_dist = routing_matrix([a, b, gap_pt])  # miles; rows/cols: 0=a, 1=b, 2=gap
        detour_mi = (edge_dist[0][2] + edge_dist[2][1]) - edge_dist[0][1]
        if detour_mi < detour_mult * seg_len_mi:
            new_ordered.insert(i + 1, gap_pt)
            accepted += 1
            added_mi += detour_mi

    return new_ordered, accepted, len(candidates), added_mi


# ── Main ──────────────────────────────────────────────────────────────────────
# Execution flow: parse KMZ → filter to zone → deduplicate waypoints → TSP →
# Valhalla route → write GPX + FIT.

def main():
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

    # Step 1b: densify sparse KMZ geometry against real OSM ways, so coverage
    # checks resample actual street shape instead of a long straight-line guess
    # between distant KMZ vertices.
    if USE_GEOMETRY_DENSIFY and near_segments:
        _dz_lats = [p[0] for s in near_segments for p in s]
        _dz_lons = [p[1] for s in near_segments for p in s]
        _dz_pad = 0.001
        _dz_bbox = (min(_dz_lats) - _dz_pad, max(_dz_lats) + _dz_pad,
                    min(_dz_lons) - _dz_pad, max(_dz_lons) + _dz_pad)
        _before_pts = sum(len(s) for s in near_segments)
        near_segments = densify_all_segments(near_segments, _dz_bbox)
        _after_pts = sum(len(s) for s in near_segments)
        if _after_pts > _before_pts:
            print(f"  Geometry densify: {_after_pts - _before_pts} points added to sparse segments "
                  f"(snapped to real OSM street shape)")

    # Step 2a: access pre-filter — drop segments that coincide with foot-inaccessible
    # OSM ways (freeways, gated/private roads) before they ever enter the TSP pool.
    if USE_ACCESS_PREFILTER and near_segments:
        _pf_lats = [p[0] for s in near_segments for p in s]
        _pf_lons = [p[1] for s in near_segments for p in s]
        _pf_bbox = (min(_pf_lats), max(_pf_lats), min(_pf_lons), max(_pf_lons))
        _kept, _dropped = prefilter_inaccessible_segments(near_segments, _pf_bbox)
        if _dropped:
            _dropped_mi = sum(
                sum(haversine_m(s[i][0], s[i][1], s[i+1][0], s[i+1][1]) for i in range(len(s)-1))
                for s, _r, _m, _d in _dropped
            ) / 1609.34
            print(f"  Access pre-filter: dropped {len(_dropped)} inaccessible segment(s) "
                  f"({_dropped_mi:.2f} mi) before TSP:")
            for _s, _reason, _mid, _frac in _dropped:
                print(f"    {_reason}  {_frac:.0%} of length restricted  https://maps.google.com/?q={_mid[0]:.6f},{_mid[1]:.6f}")
            near_segments = _kept
            near_mi = sum(
                sum(haversine_m(s[i][0], s[i][1], s[i+1][0], s[i+1][1]) for i in range(len(s)-1))
                for s in near_segments
            ) / 1609.34
            print(f"  {len(near_segments)} segments remain ({near_mi:.2f} mi)")
        else:
            print("  Access pre-filter: no inaccessible segments found")

    # Step 2b: adaptive DEDUP_M — measure actual street spacing instead of assuming 75m
    effective_dedup_m = DEDUP_M
    if USE_ADAPTIVE_DEDUP_M:
        if FILTER_MODE == "bbox":
            dedup_bbox = BBOX
        else:
            pad_lat = RADIUS_MI * 1609.34 / 111320.0
            pad_lon = RADIUS_MI * 1609.34 / (111320.0 * math.cos(math.radians(ZONE_LAT)))
            dedup_bbox = (ZONE_LAT - pad_lat, ZONE_LAT + pad_lat, ZONE_LON - pad_lon, ZONE_LON + pad_lon)
        effective_dedup_m = compute_adaptive_dedup_m(dedup_bbox, DEDUP_M)

    # Steps 3-4: Build ordered waypoints — boustrophedon or 2-opt depending on strategy.
    if ROUTE_STRATEGY == "boustrophedon":
        ew_count = sum(1 for s in near_segments if segment_orientation(s) == 'EW')
        ns_count = len(near_segments) - ew_count
        print(f"Street boustrophedon: {ew_count} E-W segs, {ns_count} N-S segs, "
              f"STREET_BUCKET_M={STREET_BUCKET_M}")
        ordered = street_boustrophedon_waypoints(near_segments, STREET_BUCKET_M, loop=LOOP)
        print(f"  {len(ordered)} waypoints")

    else:
        # Original 2-opt TSP path — build raw_points + parallel meta list carrying
        # (bearing, segment id) per waypoint. The bearing feeds the chess penalty;
        # the segment id survives dedup so the TSP can chain same-segment waypoints.
        if WAYPOINT_MODE == "endpoints":
            raw_points = []
            raw_meta = []
            for si, s in enumerate(near_segments):
                b = seg_bearing(s[0], s[-1])
                raw_points.append(s[0])
                raw_meta.append((b, si))
                raw_points.append(s[-1])
                raw_meta.append((b, si))
        elif WAYPOINT_MODE == "both":
            # Endpoints + midpoint per segment — forces full traversal of each block.
            # Midpoint prevents Valhalla from visiting endpoints via cross-streets only;
            # endpoints prevent missed coverage on long segments where midpoint alone
            # doesn't guarantee the full segment is within COVER_SNAP_M.
            raw_points = []
            raw_meta = []
            for si, s in enumerate(near_segments):
                b = seg_bearing(s[0], s[-1])
                mid = (sum(p[0] for p in s) / len(s), sum(p[1] for p in s) / len(s))
                raw_points.append(s[0])
                raw_meta.append((b, si))
                raw_points.append(mid)
                raw_meta.append((b, si))
                raw_points.append(s[-1])
                raw_meta.append((b, si))
        else:
            raw_points = [(sum(p[0] for p in s) / len(s), sum(p[1] for p in s) / len(s))
                          for s in near_segments]
            raw_meta = [(seg_bearing(s[0], s[-1]), si) for si, s in enumerate(near_segments)]
        for lat, lon in FORCED_WAYPOINTS:
            raw_points.append((lat, lon))
            raw_meta.append((None, None))
        if FORCE_START_POINT is not None:
            raw_points.append(FORCE_START_POINT)
            raw_meta.append((None, None))
        midpoints, _meta = deduplicate_with_meta(raw_points, raw_meta, min_dist_m=effective_dedup_m)
        # A tighter adaptive DEDUP_M can push the waypoint count past the TSP node
        # cap, and the single-cluster path handles overflow by TRUNCATING — which
        # silently drops whole blocks. Widen the radius until it fits instead.
        # Fixed-DEDUP_M behavior is untouched.
        if USE_ADAPTIVE_DEDUP_M and CHUNK_K == 1:
            _cap = 200 if _VALHALLA_ACTOR is not None else 50
            while len(midpoints) > _cap and effective_dedup_m < ADAPTIVE_DEDUP_MAX_M:
                effective_dedup_m = min(effective_dedup_m + 15, ADAPTIVE_DEDUP_MAX_M)
                midpoints, _meta = deduplicate_with_meta(raw_points, raw_meta,
                                                         min_dist_m=effective_dedup_m)
                print(f"  Adaptive DEDUP_M escalated to {effective_dedup_m:.0f}m "
                      f"({len(midpoints)} waypoints vs cap {_cap})")
        midpoint_bearings = [m[0] for m in _meta]
        midpoint_seg_ids = [m[1] for m in _meta]
        print(f"Waypoints after dedup ({WAYPOINT_MODE}): {len(midpoints)}")

        chunk_k = CHUNK_K
        if chunk_k == "auto":
            chunk_k = max(1, math.ceil(len(midpoints) / 40))  # 40 not 45: leaves room for uneven K-means splits under 50-node limit
            if chunk_k > 1:
                print(f"Auto CHUNK_K={chunk_k} ({len(midpoints)} waypoints)")

        _near = NEAR_LOOP_MI if not LOOP else None
        if chunk_k > 1 and len(midpoints) > 45:
            print(f"Running chunked 2-opt TSP ({chunk_k} spatial clusters)...")
            ordered = chunked_tour(midpoints, chunk_k, loop=LOOP, near_loop_mi=_near,
                                   bearings=midpoint_bearings if USE_CHESS_TSP_PENALTY else None)
        else:
            _single_cap = 200 if _VALHALLA_ACTOR is not None else 50
            if len(midpoints) > _single_cap:
                print(f"WARNING: {len(midpoints)} waypoints exceeds cap ({_single_cap}) — truncating")
                midpoints = midpoints[:_single_cap]
                midpoint_bearings = midpoint_bearings[:_single_cap]
                midpoint_seg_ids = midpoint_seg_ids[:_single_cap]
            print("Fetching walking distance matrix...")
            dist = routing_matrix(midpoints)
            dist_tsp = dist
            if USE_CHESS_TSP_PENALTY:
                dist_tsp = apply_chess_penalty(midpoints, dist_tsp, midpoint_bearings,
                                               CHESS_THRESHOLD_DEG, CHESS_PENALTY_FACTOR)
                print("Chess TSP penalty applied")
            if USE_SEGMENT_CHAINING:
                dist_tsp = apply_segment_chain_discount(dist_tsp, midpoint_seg_ids,
                                                        SEGMENT_CHAIN_FACTOR)
                print(f"Segment-chain discount applied (factor {SEGMENT_CHAIN_FACTOR})")
            print("Running 2-opt TSP (all starting points)...")
            tour = best_2opt_tour(midpoints, dist_tsp, loop=LOOP, near_loop_mi=_near)
            if dist_tsp is not dist:
                # best_2opt_tour printed the modified-matrix length; show the real one
                print(f"  True tour length: {tour_length(tour, dist, LOOP):.2f} mi")
            chained, split = segment_chaining_stats(tour, midpoint_seg_ids)
            print(f"  Segment chaining: {chained} chained / {split} split"
                  f" (multi-waypoint segments)")
            ordered = [midpoints[i] for i in tour]
            if FORCE_START_POINT is not None:
                flat, flon = FORCE_START_POINT
                idx = min(range(len(ordered)),
                          key=lambda i: haversine_m(ordered[i][0], ordered[i][1], flat, flon))
                snap_m = haversine_m(ordered[idx][0], ordered[idx][1], flat, flon)
                ordered = ordered[idx:] + ordered[:idx]
                print(f"  Rotated loop to start/end at forced point ({flat},{flon}) — "
                      f"nearest waypoint {snap_m:.0f}m away")
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

    # Step 5b: Coverage repair pass — splice in missed segments that are cheap to
    # reach. Runs in bounded rounds: a single pass only inserts one point per
    # segment (its worst-covered spot), which clears some gaps but not all of
    # them — a segment can have more than one weak stretch, or the routed path
    # can shift slightly on re-route and leave a different portion short. Each
    # round re-measures against the ACTUAL new track before trying again, so it
    # keeps closing gaps that a single guess would silently leave behind, instead
    # of requiring a human to notice a specific missed street and hand-patch it.
    if USE_COVERAGE_REPAIR:
        print("Checking for cheap coverage repairs...")
        for _round in range(1, REPAIR_MAX_ROUNDS + 1):
            repaired_ordered, n_accepted, n_candidates, added_mi = coverage_repair_pass(
                near_segments, ordered, track_points)
            if not n_accepted:
                print(f"  Repair round {_round}: 0/{n_candidates} candidates within "
                      f"{REPAIR_MAX_DIST_M:.0f}m accepted — stopping")
                break
            print(f"  Repair round {_round}: {n_accepted}/{n_candidates} candidates within "
                  f"{REPAIR_MAX_DIST_M:.0f}m accepted (+{added_mi:.2f} mi detour) — re-routing")
            ordered = repaired_ordered
            track_points, total_miles, maneuvers = routing_route(ordered)
            print(f"  Route distance after round {_round}: {total_miles:.2f} miles")
        else:
            print(f"  Repair pass: stopped after {REPAIR_MAX_ROUNDS} rounds (still-cheap gaps may remain)")

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


if __name__ == "__main__":
    main()
