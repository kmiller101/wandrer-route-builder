# Detroit Street Runner Guide

A pipeline for building optimized running routes to cover every untraveled street in Detroit,
using Wandrer.earth KMZ exports, Valhalla routing (OSM-based), and Garmin FIT course files.

---

## Overview

The goal: systematically run every street in the City of Detroit. This pipeline takes your
Wandrer.earth untraveled-streets export, plans an optimized loop route around a target area
(or anchored to a specific start point), generates a FIT course file with turn-by-turn
instructions, and loads it onto your Garmin watch.

---

## Requirements

### Hardware
- Garmin watch that supports course navigation (tested on Forerunner 965)
- Linux or macOS with Python 3.12+

### Accounts
- [Wandrer.earth](https://wandrer.earth) — tracks your coverage, exports untraveled KMZ
- [Strava](https://strava.com) — GPS source for post-run coverage checking
- Strava API app (free) — needed for the post-run workflow

### Python dependencies

```bash
python3 -m venv ~/.venvs/wandrer-route-builder
source ~/.venvs/wandrer-route-builder/bin/activate
pip install requests fit-tool numpy
```

> **Important:** Create the venv *outside* any sync folder (Tresorit, Dropbox, etc.) —
> their FUSE filesystems don't support symlinks, which venv requires.

---

## Step 1: Get your untraveled-streets KMZ

1. Log into [wandrer.earth](https://wandrer.earth)
2. Go to your city/area page → **Export untraveled streets** → download KMZ
3. Save it as `wandrer.kmz` in `wandrer-route-builder/`

This file contains every street segment you haven't run yet, as your **own** current export —
re-download it every few days (or after a batch of runs) so route planning reflects your
latest progress. **Don't use anyone else's exported KMZ** for your own route planning.

> **Format note:** `wandrer.kmz` stores each street as a `<MultiGeometry>` containing
> multiple `<LineString>` elements per `<Placemark>` — different from older neighborhood-extract
> KMZs that used a single flat `<coordinates>` block. The current scripts handle this with a
> regex-based extractor (`parse_kmz_linestrings`) that pulls every `<LineString><coordinates>`
> independently. Don't reuse the old single-coordinates `parse_kmz` on this file — it'll miss
> most segments.

---

## Step 2: Routing engine — connect to OSM

Routing (and the walking-distance matrix used for route optimization) needs real street-network
data. You have two options:

### Option A — Public Valhalla API (default, what the scripts use)
```
https://valhalla1.openstreetmap.de
```
Free, no signup, no key. It's a hosted Valhalla instance already loaded with global OSM data —
your requests are answered against current OSM streets with `pedestrian` costing (real walking
distances, not straight-line approximations). This is what `generate_zone_route.py` and
`generate_from_point.py` use out of the box. No setup required — just don't hammer it
(one matrix + one route call per generated route is plenty).

### Option B — Self-hosted Valhalla with a regional OSM extract (optional/advanced)
If you ever need higher request volume, lower latency, or offline routing, you can run your
own Valhalla server loaded from a downloaded regional extract, e.g. from
[Geofabrik](https://download.geofabrik.de/north-america/us/michigan.html):
```
michigan-latest.osm.pbf   (~300 MB)
```
Then point `VALHALLA` in the scripts at `http://localhost:8002` (or wherever your instance
listens) instead of the public URL. This is **not currently set up** — the project has a
`michigan-260605.osm.pbf` extract sitting in the project dir as a starting point if you want
to go this route, but no script references it yet. Stick with Option A unless you hit a
concrete limitation.

---

## Step 3: Generate a route

Two scripts cover the two ways you'll want to plan a route. Both source segments from
`wandrer.kmz`, run the same 2-opt TSP pipeline, and produce a turn-by-turn FIT course.

### `generate_zone_route.py` — area-based (current default)
Use this when you want to target a neighborhood or dense cluster of untraveled streets and
don't care exactly where the loop starts — the optimizer picks whatever start minimizes
total distance.

Edit the config block at the top:
```python
FILTER_MODE = "radius"   # "radius" or "bbox"

# radius mode — good for open/unknown areas; tune RADIUS_MI to hit target distance
ZONE_LAT, ZONE_LON = 42.4149, -82.9654
RADIUS_MI = 0.48

# bbox mode — use for known neighborhoods where you want ALL remaining segments,
# not just those within a circle. Prevents silently dropping edge streets.
# BBOX = (lat_min, lat_max, lon_min, lon_max)
BBOX = (42.416, 42.430, -83.220, -83.199)

DEDUP_M = 140                             # waypoint dedup spacing (50-node Valhalla cap)
SLUG = "outer-drive-hayes"                # output filename stem
ROUTE_NAME = "Outer Drive-Hayes"          # max 16 chars — shows on the watch
LOOP = True
```

**When to use bbox vs. radius:**
- **bbox** — use when you know the neighborhood boundary (e.g. College Park, Grandmont-Rosedale). Captures every remaining segment in the box; nothing silently excluded by radius. If DEDUP still gives >50 waypoints, tighten DEDUP_M.
- **radius** — use for large open zones or when exploring a cluster without a defined boundary. Tune RADIUS_MI to control density.

Then run:
```bash
source ~/.venvs/wandrer-route-builder/bin/activate
python3 generate_zone_route.py
```

**Picking ZONE_LAT/LON:** find dense untraveled clusters by binning segment midpoints into a
grid and summing nearby mileage per cell — or just use Nominatim to look up a neighborhood
name and use its centroid. Always sanity-check the resulting route's actual streets against
the intended neighborhood (see "Verifying route labels" below) — it's easy to grab the wrong
coordinates and end up generating a route somewhere else entirely.

### Additional pipeline stages (opt-in flags in the CONFIG block)

Since the initial release, `generate_zone_route.py` grew several extra stages beyond the
core parse → dedup → 2-opt → route loop above. All are configured in the same CONFIG block;
most default to `True`/on since they only improve accuracy or route quality, never make a
route worse:

- **Or-opt local search** (always on, no flag) — after 2-opt converges, relocates chains of
  1–3 consecutive waypoints to a better position in the tour. Runs inside `local_search()`,
  alternating with 2-opt until neither improves. Pure gain on top of 2-opt; adds negligible
  runtime.
- **`USE_SEGMENT_CHAINING` / `SEGMENT_CHAIN_FACTOR`** — discounts TSP matrix legs between
  waypoints that came from the *same* KMZ segment, so the tour visits a segment's
  start→mid→end consecutively instead of interleaving them with other segments (which forces
  Valhalla to backtrack through already-covered streets). Keep the factor > 0 — at 0 the TSP
  can't distinguish monotone traversal from backtracking.
- **`USE_COVERAGE_REPAIR` / `REPAIR_MAX_DIST_M` / `REPAIR_DETOUR_MULT` / `REPAIR_MAX_ROUNDS`**
  — after the initial route, checks every candidate segment that scored below
  `COVER_MIN_FRAC` (see `seg_coverage.py`) against the actual routed track. If splicing the
  segment's worst-covered point into the tour costs less than `REPAIR_DETOUR_MULT`× the
  segment's own length, it's accepted and the route re-generated. Runs in a bounded loop
  (`REPAIR_MAX_ROUNDS`) since one pass doesn't always clear every gap.
- **`USE_ACCESS_PREFILTER` / `ACCESS_PREFILTER_M` / `ACCESS_PREFILTER_MIN_FRAC`** — before
  the TSP, queries Overpass for foot-inaccessible ways (private roads, `foot=no`,
  motorway/trunk links) in the bbox and drops any KMZ segment where at least
  `ACCESS_PREFILTER_MIN_FRAC` of its own length lies within `ACCESS_PREFILTER_M` of one.
  Uses a fractional, resampled check (not just the segment's midpoint) so a long legitimate
  street isn't dropped just because one end brushes a private driveway. Keeps the coverage
  report honest — it only scores segments the route could actually reach.
- **`USE_GEOMETRY_DENSIFY` / `DENSIFY_MAX_GAP_M` / `DENSIFY_SNAP_TOL_M`** — KMZ segments can
  have long straight-line gaps between vertices. For any gap over `DENSIFY_MAX_GAP_M`, looks
  for a real OSM way whose geometry both endpoints snap to within `DENSIFY_SNAP_TOL_M`, and
  splices in that way's own denser vertices. Only improves coverage-measurement accuracy —
  waypoint extraction (start/mid/end) is unaffected.
- **`USE_ADAPTIVE_DEDUP_M`** (off by default) — instead of the fixed `DEDUP_M`, queries
  Overpass for the bbox's actual parallel-street spacing and sets dedup to 70% of the
  tightest gap. Opt-in because it changes which waypoints survive dedup, and therefore the
  TSP itself — turn on if `DEDUP_M` seems to be merging or splitting streets incorrectly for
  a given area's grid spacing.
- **`FORCE_START_POINT`** (default `None`) — forces the loop's start/end to a specific
  `(lat, lon)`, e.g. a parking lot. Added to the waypoint pool like `FORCED_WAYPOINTS`, then
  the closed tour is rotated to begin there — free in distance *only if the point is already
  near the route*. **Reset this to `None` before generating a route in a different area** —
  a stale value left over from a previous zone will silently force a large detour out to it
  and back, inflating the route with no error or warning.
- **Valhalla request caching** — `valhalla_matrix()` and each maneuver-routing call are
  memoized to `.valhalla_cache/`, keyed on a hash of the exact request payload. Repeat runs
  with the same waypoint set (common while tuning `RADIUS_MI` or A/B testing a flag) skip the
  routing engine entirely. Never expires automatically; delete the directory by hand if your
  OSM data changes.

Coverage-related stages (`seg_coverage.py`) share one rule between this script and any
post-run stripping script you build: a segment counts as covered when ≥90% of its length is
within 25m of the track polyline (resampled every 10m, so straight 2-vertex blocks and curved
streets are both measured accurately).

### `generate_from_point.py` — anchored to a specific start point
Use this when you need the loop to start/end at an exact GPS coordinate (e.g., where you
park, or a specific landmark). It forces that point into the waypoint set, runs the 2-opt
TSP anchored there, then rotates the closed loop so the tour begins and ends at your point.

Edit the config block:
```python
START_LAT, START_LON = 42.2871442, -83.1484198
RADIUS_MI = 0.44
DEDUP_M = 200
ROUTE_NAME = "Oakwood Heights"
```

### What the pipeline does (both scripts)
1. Parse `wandrer.kmz` — extract every `<LineString>` as its own segment
2. Filter to segments within `RADIUS_MI` of the zone center (or start point)
3. Take segment midpoints, deduplicate at `DEDUP_M` spacing, cap at 50 waypoints
   (Valhalla `/sources_to_targets` matrix hard limit is 50×50 nodes)
4. Fetch a real NxN walking-distance matrix via Valhalla `/sources_to_targets`
5. Run **2-opt TSP** — nearest-neighbor init from every starting point, 2-opt improve each,
   keep the shortest tour (anchored at the forced start point for `generate_from_point.py`)
6. Route the tour in order via Valhalla `/route`, extracting turn maneuvers
7. Write a `.gpx` trackfile and a `.fit` course file with `course_point` turn-by-turn records

### Why 2-opt TSP?
Row-sweep (boustrophedon) ordering causes ~35–40% route bloat from strip transitions —
confirmed: College Park went from 7.86 mi (boustrophedon) to 5.13–5.72 mi (2-opt). 2-opt with
a *real* walking-distance matrix (not Valhalla's `/optimized_route`, which uses straight-line
approximations) consistently produces noticeably tighter loops.

### Tuning RADIUS_MI to hit a target distance
**This relationship is non-monotonic and jumpy** — small radius changes can cross a
dedup-cluster threshold that adds or removes a whole group of waypoints, swinging the final
route by 2–3 miles. Confirmed example (Cornerstone Village NE): 0.52 mi → 5.15 mi route,
0.53 mi → 6.39 mi, 0.535 mi → 7.86 mi.

- Iterate in small steps (~0.02–0.05 mi), expect 4–8 generation runs to dial in a target
- Sparse/industrial areas (e.g. Delray near the riverfront) have a much higher
  actual-vs-2-opt-estimate ratio (~2x, vs ~1.2–1.4x for dense residential grids) because of
  detours around limited pedestrian infrastructure — use a noticeably *smaller* radius there
  to land on the same target distance

### Verifying route labels match their actual location
Before transferring a generated route, spot-check that its streets are actually in the
neighborhood you intended — it's easy to mix up coordinates between zones. Reverse-geocode
a few sampled track points:

```python
import requests, re

def load_track(gpx_path):
    pts = re.findall(r'lat="([\-0-9.]+)" lon="([\-0-9.]+)"', open(gpx_path).read())
    return [(float(a), float(b)) for a, b in pts]

pts = load_track("Wandrer Routes/your-route.gpx")
for idx in [0, len(pts)//4, len(pts)//2, 3*len(pts)//4]:
    lat, lon = pts[idx]
    r = requests.get("https://nominatim.openstreetmap.org/reverse",
                     params={"lat": lat, "lon": lon, "format": "json", "zoom": 16, "addressdetails": 1},
                     headers={"User-Agent": "wandrer-route-builder/1.0"}).json()
    addr = r.get("address", {})
    print(addr.get("neighbourhood") or addr.get("suburb"), "|", addr.get("road"))
```
If the neighborhoods don't match what you intended, the zone center coordinates were wrong —
relabel and regenerate rather than leaving a mislabeled course on the watch.

---

## Step 4: Transfer to your Garmin

Plug in the watch via USB. On newer Garmin models, you may need to select
**USB Mass Storage** mode from the watch menu (swipe down on the watch face).

**Linux: use `gio copy`, not `cp`** — Garmin mounts via MTP through GVFS, and `cp` returns
"Operation not supported":

```bash
gio copy "Wandrer Routes/outer-drive-hayes.fit" \
  "mtp://091e_50db_0000cd6e6c48/Internal Storage/GARMIN/Courses/outer-drive-hayes.fit"
```

Find your device ID with `gio mount -l | grep mtp`. If it reports "location is not mounted,"
the watch may not have finished mounting yet — wait a moment after connecting (it doesn't
always appear immediately even when "connected"), or reconnect and retry.

**macOS:** the FR965 mounts as a standard USB drive — regular `cp` works:
```bash
cp "Wandrer Routes/outer-drive-hayes.fit" "/Volumes/GARMIN/GARMIN/Courses/"
```

On the watch: **Navigation → Courses** to find and start the route.

### Course label
Make sure `course.course_name` (NOT `course.name`) is set — only `course_name` populates the
field that displays on the Garmin watch:
```python
course.course_name = name[:16]   # 16-char limit
```

### Turn-by-turn on FR965
The FIT course includes `course_point` records (LEFT, RIGHT, SHARP_LEFT, etc.) derived from
Valhalla's maneuver data, mapped via `_VALHALLA_TO_COURSE_POINT`. The watch shows a turn arrow
and street name at each decision point — without these, the FR965 only displays a static
breadcrumb trail with no directional guidance (confirmed cause of getting lost on an early run).

---

## Step 5: Post-run workflow

After each run, pull your GPS track from Strava and check which untraveled segments it covered.

### Strava API setup
1. Create an app at [strava.com/settings/api](https://strava.com/settings/api)
2. Store credentials in `.env`: `STRAVA_CLIENT_ID`, `STRAVA_CLIENT_SECRET`,
   `STRAVA_ACCESS_TOKEN`, `STRAVA_REFRESH_TOKEN`, `STRAVA_TOKEN_EXPIRES`
3. Tokens expire every 6 hours — refresh before each use:
   ```bash
   curl -X POST https://www.strava.com/oauth/token \
     -d client_id=YOUR_ID -d client_secret=YOUR_SECRET \
     -d grant_type=refresh_token -d refresh_token=YOUR_REFRESH_TOKEN
   ```

### Coverage check
Pull the activity's `latlng` stream, then for each untraveled segment check whether every
point along it falls within `COVER_THRESH_M` (~25m) of the GPS track using point-to-segment
distance. Segments that pass are effectively covered — though the authoritative source of
truth is always Wandrer's own next sync of your data, not a local approximation.

> Tip: Strava's activity-list endpoint also returns `map.summary_polyline` for every
> activity (Google polyline encoding, precision 5) — useful for bulk cross-referencing many
> runs against the KMZ without burning per-activity stream-API quota.

---

## Tips

- **Target 6–8 miles per route.** Long enough to make real progress, short enough to follow
  as a breadcrumb trail and recover from if you get turned around.
- **All routes are loops** — they always start and end at the same point. A-to-B routes
  require deadheading back to your car and aren't worth generating.
- **Re-sync `wandrer.kmz` regularly.** Stale exports overstate remaining mileage in
  areas you've already run and will misdirect zone-cluster planning.
- **Valhalla is free but rate-limited.** A 50-node matrix call takes a few seconds; the
  full 2-opt TSP across all starting points takes 1–5 minutes. One generation run per route
  edit is plenty — don't loop-call the API while tuning `RADIUS_MI`.

---

## File layout

```
wandrer-route-builder/
  generate_zone_route.py      # area-based route generation (use this first)
  generate_from_point.py      # anchored to a specific GPS start point
  seg_coverage.py             # shared segment-coverage rule (used by generate_zone_route.py)
  wandrer.kmz                 # YOUR current untraveled-streets export — re-sync regularly (gitignored)
  .env                        # Strava API credentials (gitignored)
  .env.example                # credential template
  requirements.txt
  .valhalla_cache/            # memoized Valhalla matrix/route responses (gitignored, safe to delete)

routes/                       # generated output (gitignored)
  my-route.fit                # Garmin course file (load this onto watch)
  my-route.gpx                # GPX backup
  ...
```

---

## Script reference

| Function | What it does |
|---|---|
| `parse_kmz_linestrings` | Extract every `<LineString>` from a `wandrer-*.kmz` MultiGeometry export |
| `deduplicate` | Remove midpoints closer than `min_dist_m` meters |
| `valhalla_matrix` | Fetch NxN walking-distance matrix (max 50 nodes), memoized to `.valhalla_cache/` |
| `best_2opt_tour` | Try all NN starting points, 2-opt + Or-opt improve each, return shortest |
| `or_opt_improve` / `local_search` | Relocate 1–3-waypoint chains to a better tour position; alternates with 2-opt until neither improves |
| `apply_segment_chain_discount` | Discount TSP legs between waypoints from the same KMZ segment so the tour visits them consecutively |
| `estimate_street_spacing_m` / `compute_adaptive_dedup_m` | Measure real parallel-street spacing from Overpass, derive an adaptive `DEDUP_M` |
| `prefilter_inaccessible_segments` | Drop KMZ segments that are mostly on foot-inaccessible OSM ways before the TSP even sees them |
| `densify_segment` / `densify_all_segments` | Splice in real OSM vertices across long straight-line KMZ gaps for more accurate coverage measurement |
| `coverage_repair_pass` | Post-route: splice in cheap detours to recover segments that scored just under the coverage threshold |
| `coverage_and_completions_report` | Score the routed track against candidate segments and report which neighborhoods it fully completes |
| `valhalla_route_with_maneuvers` | Route waypoints in order, extract turn instructions |
| `write_gpx` / `write_fit` | Write GPX trackfile / FIT course (with `course_point` turn records) |

`seg_coverage.py` (shared module): `TrackIndex` (grid-indexed track polyline for fast
point-to-polyline distance), `covered_fraction` / `segment_covered` (the ≥90%-of-length-within-25m
coverage rule used by both the repair pass and any post-run stripping script you build).
