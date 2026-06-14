"""
generate_zone_route.py — Wandrer Route Generator

PURPOSE
-------
Generates an optimized running loop for a neighborhood, designed to cover
as many untraveled streets as possible (per Wandrer.earth tracking) in 6–8 miles.

WORKFLOW
--------
1. Parse wandrer.kmz — your Wandrer export of all remaining untraveled
   street segments in your city.
2. Filter segments to the target zone (by bounding box or radius from a center point).
3. Compute each segment's midpoint; deduplicate nearby midpoints so no two waypoints
   are closer than DEDUP_M meters (avoids redundant visits on parallel streets).
4. Solve the Travelling Salesman Problem (TSP) on those waypoints using Valhalla's
   real walking-distance matrix and a nearest-neighbor + 2-opt algorithm.
   If waypoints exceed 50 (Valhalla's matrix limit), split into spatial clusters
   (CHUNK_K) and optimize each cluster independently, then chain them.
5. Feed the ordered waypoints to Valhalla /route to get the actual walking path
   with turn-by-turn maneuver data.
6. Write the route as both GPX (for Strava upload) and FIT (for Garmin watch).
   FIT files include CoursePoint turn arrows so the watch shows LEFT/RIGHT prompts.

DEPENDENCIES
------------
  pip install requests fit-tool
  Valhalla public API: https://valhalla1.openstreetmap.de (no key needed)
  wandrer.kmz: downloaded from Wandrer.earth > Export map > KMZ

QUICK START
-----------
  1. Download your untraveled-streets KMZ from Wandrer.earth and save as wandrer.kmz
     in the same directory as this script.
  2. Edit the CONFIG block below — set BBOX or ZONE_LAT/ZONE_LON, SLUG, ROUTE_NAME.
  3. Create a venv outside any sync folder and install deps:
       python3 -m venv ~/.venvs/wandrer-route-builder
       source ~/.venvs/wandrer-route-builder/bin/activate
       pip install requests fit-tool
  4. Run: python generate_zone_route.py
  5. Copy the output FIT to your Garmin watch:
       Linux:  gio copy routes/route.fit "mtp://DEVICE/Internal Storage/GARMIN/Courses/route.fit"
       macOS:  cp routes/route.fit "/Volumes/GARMIN/GARMIN/Courses/route.fit"
"""

import xml.etree.ElementTree as ET
import re
import math
import os
import time
import requests
import zipfile

# ── File paths ────────────────────────────────────────────────────────────────

_HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE_KMZ = os.path.join(_HERE, "wandrer.kmz")
ROUTES_DIR = os.path.join(_HERE, "routes")

os.makedirs(ROUTES_DIR, exist_ok=True)

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

# ── CONFIG — edit per zone ────────────────────────────────────────────────────
# This is the only block you need to change when generating a new route.
#
# FILTER_MODE: how to select segments from the KMZ
#   "bbox"   — use the BBOX rectangle (preferred for named neighborhoods)
#   "radius" — use a circle of RADIUS_MI miles around (ZONE_LAT, ZONE_LON)
#
# DEDUP_M: minimum distance between waypoints in meters.
#   140m works for most residential grids (~100-150m block spacing).
#   80-100m for tighter grids. Too small → >50 waypoints (Valhalla cap). Too large
#   → whole parallel streets get merged into one waypoint and are skipped.
#
# MIN_SEG_M: skip segments shorter than this (meters). Use 50 for stub-heavy
#   areas (dead ends, cul-de-sacs) that inflate waypoint count without adding value.
#   0 = keep all segments.
#
# CHUNK_K: spatial clustering for large zones that produce >50 waypoints after dedup.
#   1 = standard single-pass (hard cap at 50 waypoints if exceeded)
#   2-3 = split into K clusters, optimize each independently, chain together.
#   Each cluster must have ≤50 points (Valhalla matrix hard limit).
#
# LOOP: True = route returns to start (required for out-and-back car/transit trips).

FILTER_MODE = "bbox"

# radius mode — circle around a center point
ZONE_LAT, ZONE_LON = 42.397, -83.218
RADIUS_MI = 0.50

# bbox mode — explicit bounding box (midpoint filter); set FILTER_MODE = "bbox" to use
# BBOX = (lat_min, lat_max, lon_min, lon_max)
BBOX = (42.416, 42.430, -83.220, -83.199)

DEDUP_M = 140
MIN_SEG_M = 50
CHUNK_K = 1   # spatial chunks: 1=standard (≤50 wpts), 2-3=for dense areas with many segments
SLUG = "my-route"
ROUTE_NAME = "My Route"
LOOP = True
# ──────────────────────────────────────────────────────────────────────────────

OUTPUT_GPX = os.path.join(ROUTES_DIR, f"{SLUG}.gpx")
OUTPUT_FIT = os.path.join(ROUTES_DIR, f"{SLUG}.fit")


# ── KMZ parsing ───────────────────────────────────────────────────────────────
# Wandrer exports a KMZ file (a ZIP containing a .kml XML file). Each untraveled
# street segment is a Placemark. The KMZ uses MultiGeometry, so each Placemark
# may contain multiple <LineString> elements — each is extracted as its own segment.
# Coordinates are stored lon,lat (GeoJSON order) and converted to (lat, lon) tuples.

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


def two_opt_improve(tour, dist, loop):
    """
    2-opt local search improvement.

    Repeatedly checks all pairs of edges (i-1,i) and (j, j+1). If reversing the
    segment between i and j shortens the tour, applies the reversal. Stops when
    no improving swap exists (local optimum).
    """
    n = len(tour)
    improved = True
    while improved:
        improved = False
        for i in range(1, n - 1):
            for j in range(i + 1, n):
                if not loop and j == n - 1:
                    continue
                a, b = tour[i - 1], tour[i]
                c, d = tour[j], tour[(j + 1) % n]
                if dist[a][c] + dist[b][d] < dist[a][b] + dist[c][d] - 1e-9:
                    tour[i: j + 1] = tour[i: j + 1][::-1]
                    improved = True
    return tour


def best_2opt_tour(points, dist, loop):
    """
    Try nearest-neighbor + 2-opt from every starting waypoint; return the best tour.

    Returns a list of waypoint indices (not coordinates). The caller converts to
    coordinates. If loop=True, the caller should append ordered[0] to close the loop.
    """
    n = len(points)
    best_tour, best_len = None, float("inf")
    for start in range(n):
        t = nearest_neighbor(dist, start)
        t = two_opt_improve(t, dist, loop)
        l = tour_length(t, dist, loop)
        if l < best_len:
            best_len, best_tour = l, t
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


def chunked_tour(midpoints, k, loop):
    """
    Split midpoints into k spatial clusters, optimize each with 2-opt, chain together.

    Each cluster is optimized as an open (non-loop) tour. The clusters are then
    chained by always appending the next cluster in whichever direction (forward or
    reversed) puts its nearest endpoint closest to the current route tail. The loop
    close (returning to start) is handled by appending ordered[0] at the end.
    """
    clusters, centroids = kmeans_cluster(midpoints, k)
    print(f"  Chunks: {[len(c) for c in clusters]} waypoints")
    # Optimize each cluster as an open (non-loop) tour
    cluster_seqs = []
    for i, cl in enumerate(clusters):
        if len(cl) == 0:
            continue
        if len(cl) == 1:
            cluster_seqs.append(cl)
            continue
        print(f"  Chunk {i+1}/{k} ({len(cl)} wpts)...")
        local_dist = valhalla_matrix(cl)
        local_tour = best_2opt_tour(cl, local_dist, loop=False)
        cluster_seqs.append([cl[j] for j in local_tour])
    # Chain clusters: for each join, pick the direction (fwd/rev) that minimises connector gap
    ordered = list(cluster_seqs[0])
    for nxt_seq in cluster_seqs[1:]:
        tail = ordered[-1]
        # Compare connecting tail → nxt_seq[0] vs tail → nxt_seq[-1]
        d_fwd = haversine_m(tail[0], tail[1], nxt_seq[0][0], nxt_seq[0][1])
        d_rev = haversine_m(tail[0], tail[1], nxt_seq[-1][0], nxt_seq[-1][1])
        ordered.extend(nxt_seq if d_fwd <= d_rev else list(reversed(nxt_seq)))
    if loop:
        ordered.append(ordered[0])
    return ordered


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
    chunk_size = 50

    i = 0
    while i < len(waypoints):
        chunk = waypoints[i: i + chunk_size]
        if i > 0:
            chunk = [waypoints[i - 1]] + chunk

        locations = [{"lat": lat, "lon": lon, "type": "through"} for lat, lon in chunk]
        locations[0]["type"] = "break"
        locations[-1]["type"] = "break"

        payload = {"locations": locations, "costing": "pedestrian", "units": "miles"}
        resp = requests.post(f"{VALHALLA}/route", json=payload, timeout=60)
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


# ── GPX writer ────────────────────────────────────────────────────────────────
# Writes a minimal GPX 1.1 track file. Strava accepts this for activity upload.

def write_gpx(track_points, name, output_path):
    lines = ['<?xml version="1.0" encoding="UTF-8"?>']
    lines.append('<gpx version="1.1" creator="wandrer-route-builder" xmlns="http://www.topografix.com/GPX/1/1">')
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
# Writes a Garmin FIT course file. FIT courses display on the watch as a breadcrumb
# map with optional turn-by-turn CoursePoint arrows.
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
# 10 min/mile pace (2.68 m/s). The watch only uses relative timing for display.

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
    for lat, lon, dist_m, cp_type, instruction in maneuvers:
        cp = CoursePointMessage()
        cp.timestamp = start_unix_ms + int(dist_m * PACE_MS * 1000)
        cp.position_lat = lat
        cp.position_long = lon
        cp.distance = dist_m
        cp.type = cp_type
        cp.name = instruction[:16] if instruction else cp_type.name[:16]
        builder.add(cp)

    fit_file = builder.build()
    with open(output_path, "wb") as f:
        f.write(fit_file.to_bytes())


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

# Step 3: Compute segment midpoints and deduplicate
# Each segment's midpoint is used as the TSP waypoint for that segment.
# Deduplication merges nearby waypoints (parallel streets, close segments) to stay
# within Valhalla's 50-node matrix cap and avoid redundant routing.
midpoints = [(sum(p[0] for p in s) / len(s), sum(p[1] for p in s) / len(s)) for s in near_segments]
midpoints = deduplicate(midpoints, min_dist_m=DEDUP_M)
if CHUNK_K <= 1 and len(midpoints) > 50:
    midpoints = midpoints[:50]
print(f"Waypoints after dedup: {len(midpoints)}")

# Step 4: Solve TSP to find the optimal visit order
# If CHUNK_K > 1 and waypoints exceed 50, use spatial clustering to bypass
# Valhalla's matrix limit. Otherwise run the standard single-pass 2-opt.
if CHUNK_K > 1 and len(midpoints) > 50:
    print(f"Running chunked 2-opt TSP ({CHUNK_K} spatial clusters)...")
    ordered = chunked_tour(midpoints, CHUNK_K, loop=LOOP)
else:
    print("Fetching walking distance matrix from Valhalla...")
    dist = valhalla_matrix(midpoints)
    print("Running 2-opt TSP (all starting points)...")
    tour = best_2opt_tour(midpoints, dist, loop=LOOP)
    ordered = [midpoints[i] for i in tour]
    if LOOP:
        ordered.append(ordered[0])

# Step 5: Get the actual walking route from Valhalla
print("Generating route via Valhalla /route (with maneuvers)...")
track_points, total_miles, maneuvers = valhalla_route_with_maneuvers(ordered)
print(f"Route distance: {total_miles:.2f} miles")
print(f"Turn instructions: {len(maneuvers)}")

# Step 6: Write output files
write_gpx(track_points, ROUTE_NAME, OUTPUT_GPX)
print(f"GPX written: {OUTPUT_GPX} ({len(track_points)} trackpoints)")

write_fit(track_points, maneuvers, ROUTE_NAME, OUTPUT_FIT)
print(f"FIT written: {OUTPUT_FIT}")
print("Done.")
