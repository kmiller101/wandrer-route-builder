"""
generate_from_point.py — Wandrer Route Generator (Anchored Start)

PURPOSE
-------
Like generate_zone_route.py, but forces the route to start and end at a specific
GPS coordinate — useful when you need the loop anchored to a parking lot, trailhead,
or other fixed location.

The start point is inserted at index 0 of the waypoint list. The 2-opt TSP runs
from that fixed start, and the closed tour is rotated so it begins and ends there.

WORKFLOW
--------
1. Parse wandrer.kmz — your Wandrer export of all remaining untraveled segments.
2. Filter segments to the target zone (bbox or radius).
3. Deduplicate midpoints; insert the forced start point at index 0.
4. Fetch Valhalla walking-distance matrix (≤50 nodes including the start point).
5. Run nearest-neighbor + 2-opt TSP anchored at the start point.
6. Route via Valhalla /route with turn-by-turn maneuvers.
7. Write GPX and FIT files.

DEPENDENCIES
------------
  pip install requests fit-tool
  Valhalla public API: https://valhalla1.openstreetmap.de (no key needed)
  wandrer.kmz: downloaded from Wandrer.earth > Export map > KMZ
"""

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

# ── CONFIG — edit per run ─────────────────────────────────────────────────────
# START_LAT/LON: your fixed start point (parking lot, landmark, etc.)
# FILTER_MODE:   "bbox" (recommended) or "radius"
# BBOX:          (lat_min, lat_max, lon_min, lon_max) — used when FILTER_MODE="bbox"
# RADIUS_MI:     radius around start point — used when FILTER_MODE="radius"
# DEDUP_M:       min distance between waypoints (meters); 140m for typical grids
# MIN_SEG_M:     skip segments shorter than this (meters); 50 filters short stubs

START_LAT, START_LON = 42.3314, -83.0457   # Example: Grand Circus Park — replace with your start point
FILTER_MODE = "bbox"
BBOX = (42.325, 42.340, -83.060, -83.040)  # Example bbox — replace with your target area
RADIUS_MI = 0.44
DEDUP_M = 140
MIN_SEG_M = 50

OUTPUT_GPX = os.path.join(ROUTES_DIR, "my-route.gpx")
OUTPUT_FIT = os.path.join(ROUTES_DIR, "my-route.fit")
ROUTE_NAME = "My Route"        # 16 char limit for Garmin display
LOOP = True
# ─────────────────────────────────────────────────────────────────────────────


# ── KMZ parsing ───────────────────────────────────────────────────────────────
# wandrer.kmz uses MultiGeometry with multiple <LineString> elements per
# Placemark — must use regex extraction, not XML tree traversal.

def parse_kmz(path):
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
    result = []
    for p in points:
        if all(haversine_m(p[0], p[1], q[0], q[1]) > min_dist_m for q in result):
            result.append(p)
    return result


# ── Valhalla matrix + 2-opt TSP ───────────────────────────────────────────────

def valhalla_matrix(points):
    locations = [{"lat": lat, "lon": lon} for lat, lon in points]
    payload = {
        "sources": locations,
        "targets": locations,
        "costing": "pedestrian",
        "units": "miles",
    }
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
    n = len(tour)
    total = sum(dist[tour[i]][tour[i + 1]] for i in range(n - 1))
    if loop:
        total += dist[tour[-1]][tour[0]]
    return total


def nearest_neighbor(dist, start):
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


# ── Valhalla routing with maneuvers ───────────────────────────────────────────

_VALHALLA_TO_COURSE_POINT = {
    1:  CoursePoint.GENERIC, 2: CoursePoint.GENERIC, 3: CoursePoint.GENERIC,
    4:  CoursePoint.GENERIC, 5: CoursePoint.GENERIC, 6: CoursePoint.GENERIC,
    7:  CoursePoint.STRAIGHT, 8: CoursePoint.STRAIGHT,
    9:  CoursePoint.SLIGHT_RIGHT, 10: CoursePoint.RIGHT, 11: CoursePoint.SHARP_RIGHT,
    12: CoursePoint.U_TURN, 13: CoursePoint.U_TURN,
    14: CoursePoint.SHARP_LEFT, 15: CoursePoint.LEFT, 16: CoursePoint.SLIGHT_LEFT,
    17: CoursePoint.STRAIGHT, 18: CoursePoint.SLIGHT_RIGHT, 19: CoursePoint.SLIGHT_LEFT,
    22: CoursePoint.STRAIGHT, 23: CoursePoint.SLIGHT_RIGHT, 24: CoursePoint.SLIGHT_LEFT,
    26: CoursePoint.RIGHT, 27: CoursePoint.STRAIGHT,
    37: CoursePoint.RIGHT, 38: CoursePoint.LEFT,
}
_SKIP_TYPES = {7, 8, 17, 22}


def decode_polyline6(encoded):
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

            leg_dists_m = [0.0]
            for k in range(1, len(leg_points)):
                d = haversine_m(
                    leg_points[k - 1][0], leg_points[k - 1][1],
                    leg_points[k][0],     leg_points[k][1]
                )
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


# ── FIT writer ────────────────────────────────────────────────────────────────

PACE_MS = 1 / 2.68  # ~10 min/mile (seconds per meter)


def write_fit(track_points, maneuvers, name, output_path):
    total_dist_m = sum(
        haversine_m(track_points[i][0], track_points[i][1],
                    track_points[i + 1][0], track_points[i + 1][1])
        for i in range(len(track_points) - 1)
    )
    total_time_s = total_dist_m * PACE_MS
    start_unix_ms = int(time.mktime(
        time.strptime("2020-01-01 08:00:00", "%Y-%m-%d %H:%M:%S")) * 1000)

    builder = FitFileBuilder(auto_define=True)

    fid = FileIdMessage()
    fid.type = FileType.COURSE
    fid.manufacturer = Manufacturer.DEVELOPMENT.value
    fid.time_created = start_unix_ms
    builder.add(fid)

    course = CourseMessage()
    course.course_name = name[:16]
    course.sport = Sport.RUNNING
    course.capabilities = CourseCapabilities.NAVIGATION
    builder.add(course)

    ev_start = EventMessage()
    ev_start.event = Event.TIMER
    ev_start.event_type = EventType.START
    ev_start.timestamp = start_unix_ms
    builder.add(ev_start)

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

print(f"Parsing {SOURCE_KMZ}...")
all_segments = parse_kmz(SOURCE_KMZ)
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
        if haversine_m(START_LAT, START_LON, mid_lat, mid_lon) < RADIUS_MI * 1609.34:
            near_segments.append(s)
near_mi = sum(sum(haversine_m(s[i][0],s[i][1],s[i+1][0],s[i+1][1]) for i in range(len(s)-1)) for s in near_segments) / 1609.34
print(f"  {len(near_segments)} untraveled segments ({near_mi:.2f} mi)")

midpoints = [
    (sum(p[0] for p in s) / len(s), sum(p[1] for p in s) / len(s))
    for s in near_segments
]
midpoints = deduplicate(midpoints, min_dist_m=DEDUP_M)
if len(midpoints) > 49:        # leave room for the forced start point (50 node cap)
    midpoints = midpoints[:49]

# Force the start point into the waypoint set at index 0
midpoints = [(START_LAT, START_LON)] + midpoints
START_IDX = 0
print(f"Waypoints after dedup (incl. forced start point): {len(midpoints)}")

print("Fetching walking distance matrix from Valhalla...")
dist = valhalla_matrix(midpoints)

print("Running 2-opt TSP (anchored at start point)...")
nn = nearest_neighbor(dist, START_IDX)
tour = two_opt_improve(nn, dist, loop=LOOP)
est_len = tour_length(tour, dist, loop=LOOP)
print(f"  2-opt tour estimate: {est_len:.2f} mi across {len(tour)} waypoints")

# Rotate tour so the start point is first
si = tour.index(START_IDX)
tour = tour[si:] + tour[:si]

ordered = [midpoints[i] for i in tour]
if LOOP:
    ordered.append(ordered[0])

print("Generating route via Valhalla /route (with maneuvers)...")
track_points, total_miles, maneuvers = valhalla_route_with_maneuvers(ordered)
print(f"Route distance: {total_miles:.2f} miles")
print(f"Turn instructions: {len(maneuvers)}")

write_gpx(track_points, ROUTE_NAME, OUTPUT_GPX)
print(f"GPX written: {OUTPUT_GPX} ({len(track_points)} trackpoints)")

write_fit(track_points, maneuvers, ROUTE_NAME, OUTPUT_FIT)
print(f"FIT written: {OUTPUT_FIT}")
print("Done.")
