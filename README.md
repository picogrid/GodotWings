# GodotWings

| <img src="assets/short_godot.gif" height="400"> | <img src="assets/crash.gif" height="400"> | <img src="assets/yolo.gif" height="400"> |
|:--:|:--:|:--:|


A cross-platform flight simulator built in Godot 4 that acts as the physics +
render backend for [ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)

An alternative to Gazebo / AirSim, which can be heavy to setup when needing a simple SITL + Computer Vision setup.

It ships as a Godot addon. Simply add it to your project, drop a few nodes and you're ready to fly.


## Running

### Godot side

Install [Godot 4.2+](https://godotengine.org/download), create / open a project and import GodotWings as an addon (addons/godotswings). 
- GWAircraft and GWMulticopter nodes provide drag-and-drop, fully setup aircrats with dynamics, SITL endpoint, camera streaming (rtsp...) and gimbal. All easy to setup in the inspector
- GWView camera allows you to setup an in-Godot camera when not using the streaming camera attached to the drone 
- GWWind provides very basic wind / turbulence 
Examples/Main.tscn provides the most basic example. 
- GWFloatingOrigin provides Floating origin (origin rebasing) for large worlds. Drop in level and point to your main drone.

default mavlink endpoint: udp:127.0.0.1:14550
and video stream: 127.0.0.1:5600

<img src="assets/example_setup.png" height="400"> 

### ArduPilot side


```bash
docker compose up --build                    # ArduPlane for GWAircraft
NUM_VEHICLES=2 docker compose up # For multiple vehicle, see "#swarm"
VEHICLE=ArduCopter docker compose up --build  # ArduCopter for GWMulticopter
```

**Start Godot before the container** — ArduPilot's JSON backend blocks waiting for physics and emits no MAVLink until Godot is replying. If the container starts
first, `docker compose restart ardupilot-sitl` once Godot is running.

You can then use your ground control software of choice (tested with QGroundcontrol) to fly the drone and view the live video.



## Manual / direct control (no SITL)

You don't need ArduPilot to fly. Every vehicle has a `control_source` property:

- **SITL** (default) — driven by ArduPilot over the UDP lockstep bridge.
- **Manual** — driven directly by keyboard / joypad / USB RC controller.

Set `control_source = Manual` on a `GWAircraft` / `GWMulticopter` (or drop a
`GWManualInput` node under any vehicle body) and run Godot on its own — no Docker,
no autopilot. A `GWManualInput` is added automatically when none is present.

Default controls (RC "mode 2"; all rebindable in *Project Settings → Input Map*,
the `gw_*` actions):

| Input | Keyboard | Joypad |
|---|---|---|
| Roll (aileron) | ← / → | right stick X |
| Pitch (elevator) | ↑ / ↓ | right stick Y |
| Yaw (rudder) | A / D | left stick X |
| Throttle | W / S | left stick Y |
| Reset / un-crash | R | — |

Throttle is **sticky**: it ramps up/down while you hold the key/stick and holds
where you leave it (set `throttle_ramp`). If a control responds backwards for your
airframe, flip the matching `invert_*` flag.

Manual mode is **raw and unstabilised** — sticks map straight to channels 1–4 in
the AETR layout. A fixed-wing flies this directly (it's a real RC "manual" mode:
surfaces deflect, no auto-level). A **multirotor receives channels 1–4 as raw
per-motor servos**, exactly as it would from SITL, so it is *not* hand-flyable as-is
— layer your own mixer / flight-mode sim on top of `GWManualInput` if you want
stabilised quad control. Aux channels (5–16) rest at neutral, so `GWChannelSwitch`
still works against a manual source.

See `examples/Manual.tscn` for a runnable fixed-wing setup.



## Camera & gimbal

`GWCamera` (`sensors/Camera.gd`) renders an off-screen viewport sharing the main
world, grabs frames on a timer to a local TCP server, and lets ffmpeg pull them
and emit H.264 (RTP — QGroundControl's native format — / MPEG-TS / RTSP). A
parallel UDP socket emits one JSON packet per frame (`frame_id`, `sim_time` on the
SITL clock, `pos_ned`, attitude quaternion, fov, mount basis) so a CV process can
correlate video to ground-truth pose. The video path is independent of the SITL
bridge — it only reads pose and never blocks the physics loop.

Add a `GWCamera` under a vehicle (or tick `enable_camera`). Its transform is the
mount; identity looks out the nose, −90° about X looks straight down. Needs
`ffmpeg` on `PATH` (or set `ffmpeg_path`); `launch_ffmpeg = false` runs your own
encoder against the raw-frame TCP server. Reference CV client:

```bash
pip install opencv-python pymavlink numpy
python3 tools/gw_camera_client.py --video user://godotwings_cam.sdp --mavlink udp:127.0.0.1:14550 --show
```

The camera can also act as an ArduPilot **servo gimbal** that follows any mount
mode (MAVLink angle/rate, ROI/GPS, Home, SysID, RC) commanded from the GCS — no
MAVLink parsing in Godot. Configure the mount as a servo gimbal and ArduPilot
resolves the active mode into pitch/yaw/roll servo PWM that arrives over the SITL
link; `GWCamera` reads those channels. Tick `gimbal_enabled`, set the channels to
match your `SERVOn_FUNCTION`, and set the angle ranges to match `MNT1_*_MIN/MAX`.
`docker/sitl-defaults.parm` includes a ready servo-mount block:

```
MNT1_TYPE        1     # servo gimbal
MNT1_DEFLT_MODE  2     # MAVLink targeting
SERVO9_FUNCTION  7     # mount pitch -> ch 9
SERVO10_FUNCTION 6     # mount yaw   -> ch 10
SERVO11_FUNCTION 8     # mount roll  -> ch 11
```

## Ground PTZ (fixed pan/tilt/zoom camera)

`GWGroundPTZ` (`sensors/GroundPTZ.gd`) is a ground-emplaced pan/tilt/zoom camera
— a tripod/mast vantage, not attached to a vehicle. It reuses `GWCamera`
verbatim for rendering + RTSP publish (auto-adding one as a child, configured
from its own exports, on its own path/port) and only points that camera's
mount and adjusts its `fov` for zoom. It does **not** speak any camera control
protocol itself — that translation lives in an external process; Godot only
renders, streams RTSP, and speaks a dumb JSON control protocol over TCP that
drives it.

Drop a `GWGroundPTZ` node in the scene at the tripod/mast position (its own
`position`, same as any other node) and set:

| Export | Meaning |
|---|---|
| `reference_heading_deg` | Compass heading `pan = 0` points at. |
| `base_fov` | `Camera3D.fov` at `zoom = 1` (widest); applied as `fov = base_fov / zoom`. |
| `tilt_min_deg` / `tilt_max_deg` | Tilt clamp, degrees (positive = up, negative = down). |
| `zoom_max` | Zoom is clamped to `[1, zoom_max]`. |
| `max_pan_rate_deg` / `max_tilt_rate_deg` / `max_zoom_rate` | Rate at speed = 100 for `continuous` moves. |
| `default_continuous_timeout` | Used when a `continuous` request omits `"timeout"`. |
| `protocol`, `resolution`, `fps`, `video_host`, `video_port`, `rtsp_url`, `ffmpeg_path`, `launch_ffmpeg`, `bitrate_kbps`, `raw_tcp_port` | Passed straight through to the child `GWCamera` — same meaning as the vehicle camera's own exports; give `rtsp_url` its own path (e.g. `.../groundptz`) so it doesn't collide with a vehicle's stream. |
| `control_host` / `control_port` | Bind address for the JSON control socket (default `0.0.0.0:8770`). |
| `metadata_enabled`, `metadata_host`, `metadata_port` | Off by default. When on, emits one JSON telemetry packet/frame over UDP — pose + geodetic position, for `gw_klv_muxer.py` (below) or your own tooling. |
| `latitude`, `longitude`, `altitude_m` | The mount's true WGS84 position — purely informational (doesn't affect rendering), but required for the telemetry to be geolocatable. |

To pre-configure the camera yourself (custom encoder settings, etc.), add a
`GWCamera` child under the `GWGroundPTZ` by hand — it'll be reused as-is
instead of auto-created.

### JSON control protocol

One TCP connection, newline-delimited JSON: one request object per line, one
reply object back per request (`continuous`/`stop` aside, this is otherwise
stateless). Angles are degrees; `pan` wraps to `[-180, 180]` (it's a full-
rotation yaw), `tilt`/`zoom` hard-clamp to their configured ranges. Every
successful reply echoes the resulting pose so the caller can read state
straight off any command. Errors reply `{"ok":false,"error":"..."}`.

```
{"cmd":"status"}
    -> {"ok":true,"pan":P,"tilt":T,"zoom":Z}

{"cmd":"absolute","pan":P,"tilt":T,"zoom":Z}       // any field optional; missing = unchanged
    -> {"ok":true,"pan":P,"tilt":T,"zoom":Z}

{"cmd":"relative","rpan":dP,"rtilt":dT,"rzoom":dZ} // deltas, any optional
    -> {"ok":true,"pan":P,"tilt":T,"zoom":Z}

{"cmd":"continuous","pan_speed":sx,"tilt_speed":sy,"zoom_speed":sz,"timeout":secs}
    // speeds in [-100,100] (% of max rate); moves each physics tick until
    // "stop" or timeout (default from `default_continuous_timeout`, 5s).
    -> {"ok":true}

{"cmd":"stop"}
    // halts any continuous motion.
    -> {"ok":true,"pan":P,"tilt":T,"zoom":Z}
```

See `examples/GroundPTZ.tscn` for a minimal scene.

### STANAG 4609 / MISB ST 0601 KLV metadata

`tools/gw_klv_muxer.py` is a companion process (not part of the addon) that
turns either camera's raw video + telemetry into a proper STANAG 4609 stream —
H.264 video and MISB ST 0601 KLV metadata as two elementary streams in one
MPEG-TS, the standard FMV wire format. It's what you'd point `launch_ffmpeg =
false` at instead of ffmpeg when you need KLV: ffmpeg's CLI can mux H.264 fine
but can't cleanly interleave the variable-length, packet-boundary-sensitive
KLV track (see `misb_st0601.py`'s docstring for why), so this uses GStreamer
instead — `mpegtsmux` has native KLV support (`meta/x-klv` caps → the
`stream_type 0x06` + `KLVA` registration descriptor MISB readers expect).

```bash
pip install pygobject   # or: brew install gstreamer pygobject3 (macOS)
python3 tools/gw_klv_muxer.py --video-port 5568 --metadata-port 5611 \
    --width 1280 --height 720 --fps 30 --out-port 5700
```

By default it publishes plain UDP MPEG-TS. Pass `--rtsp-url` instead to push
to an RTSP server (e.g. the same MediaMTX `GWCamera`'s own `rtsp_url` already
targets, on its own path) — the H.264+KLV mux is reused completely unchanged;
only the transport at the very end swaps from `udpsink` to `rtspclientsink`
(RTP/MP2T, payload type 33 — the KLV stays inside the MPEG-TS exactly as
`mpegtsmux` built it, since RTSP has no KLV media type of its own to describe
it separately):

```bash
python3 tools/gw_klv_muxer.py --video-port 5568 --metadata-port 5611 \
    --width 1280 --height 720 --fps 30 --rtsp-url rtsp://127.0.0.1:8554/groundptz_klv
```

Two things worth knowing if a downstream tool complains about the stream:

- **`h264parse config-interval=-1`** repeats SPS/PPS before every IDR frame
  (not on a fixed timer) — needed for anything joining mid-stream (an RTSP
  server ingesting the push, a client connecting after start) to decode from
  its first keyframe. The "non-existing PPS" warning right when something
  *first* joins mid-GOP is still normal and expected either way — every
  decoder has to wait for the next IDR when joining mid-stream, no setting
  changes that; it should clear up as soon as that next IDR arrives.
- **`mpegtsmux latency` (`--mux-latency-ms`, default 200)** — the KLV appsrc
  has near-zero latency; the video branch has to actually run through
  `x264enc` first. Without this, `mpegtsmux` can commit to its very first
  PAT/PMT before the video branch has produced anything, so that first table
  declares only the KLV track — and readers that read the PMT once at start
  (MediaMTX's UDP source among them) latch onto that and reject all
  subsequent video as an "undeclared track", even though a corrected PMT
  arrives ~100ms later. Verified directly: `0` (disabled) reliably reproduces
  exactly that failure; `200` reliably fixes it. Raise it if your video branch
  is slower to start (bigger resolution/bitrate, slower machine).

It auto-detects which camera's telemetry arrived — `GWCamera`'s (vehicle-
relative pose + `pos_ned`/quaternion, converted to lat/lon via the same flat-
tangent-plane math as `GWGeoReference.ned_to_geodetic()`; pass `--home-lat` /
`--home-lon` / `--home-alt` to match) or `GWGroundPTZ`'s (already absolute
lat/lon + pan/tilt/zoom) — and maps it onto the ST 0601 tags each schema can
actually populate: platform heading/pitch/roll, sensor lat/lon/altitude
(correctly accounting for the camera's mount offset *and* any gimbal rotation
— not just the vehicle's CG position and attitude), sensor relative az/el,
horizontal/vertical FOV, and a frame-center + 4-corner ground footprint. Point
any FMV/KLV client (or `gst-launch-1.0 udpsrc port=5700 ! tsdemux ! ...`) at
the resulting `udp://host:5700`.

The frame-center/corner tags (23-25, 82-89) come from a **flat-ground-plane**
ray intersection — the boresight and 4 FOV-corner rays intersected with a
horizontal plane at `--ground-alt` (default 0). This is a simplifying
approximation, **not** a real terrain raycast: Godot has the actual terrain
mesh, but this external process doesn't, so it can't ray-cast against it.
Over genuinely flat ground it's exact; over hills it's off by however much the
terrain deviates from that plane. If the camera is looking at or above the
horizon, no footprint can exist — those tags are simply omitted for that
frame rather than emitting nonsense.

With every field this module can populate, the packet is small enough for
BER's single-byte ("short-form") length encoding — except with all 9
frame-center/corner tags added, which pushes it past 127 bytes into
multi-byte ("long-form") length. That's valid KLV, but some simpler KLV
parsers only handle short-form and will error loading tags on a long-form
packet. If your downstream tool complains, pass `--no-footprint` to drop
those 9 tags and check whether that's the cause.

**Testing it end to end**, with no real STANAG4609 client on hand: set
`launch_ffmpeg = false` on the camera (Godot then only hosts the raw-frame
socket instead of spawning its own ffmpeg), run `gw_klv_muxer.py` against it
as above, then in parallel:

```bash
ffplay udp://127.0.0.1:5700            # see the actual video
ffprobe udp://127.0.0.1:5700           # confirm it lists a video AND a data stream
python3 tools/gw_klv_dump.py --port 5700   # decode + print every KLV packet live
```

`gw_klv_dump.py` scans the stream for genuine, checksum-valid KLV units and
prints each one's decoded fields as they arrive — the quickest way to watch
lat/lon/pan/tilt update in real time as you drive the camera over its JSON
control socket, and hard confirmation the metadata survived the mux intact
(a corrupted or misframed packet simply fails its checksum and gets skipped).

## Real-world terrain import

`tools/gw_terrain_import.py` bakes a real place — real satellite imagery +
real elevation — into a terrain tile `GWImportedTerrain` can load, from
entirely open, no-registration data: Sentinel-2 L2A true-color imagery and
Copernicus DEM GLO-30 elevation, both pulled from public AWS Open Data buckets
via Element84's STAC API. It's an offline bake step — Godot never touches the
network for this; the tool downloads/reprojects once, up front.

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install rasterio numpy pillow requests
python3 tools/gw_terrain_import.py --lat 60.221088825593135 --lon 25.018208331290666 \
    --size-km 4 --out terrain_helsinki
```

It searches for the least-cloudy Sentinel-2 scene over the AOI, mosaics
whichever Copernicus DEM tiles cover it (handles an AOI straddling a tile
boundary), and reprojects both onto a local grid in an azimuthal-equidistant
(AEQD) projection centered exactly on `--lat`/`--lon` — the geospatially
rigorous version of the same flat-tangent-plane approximation
`GWCoordConvert`/`GWGeoReference` already use elsewhere in this project.
Writes `<out>_heightmap.png`, `<out>_texture.png`, and `<out>.json` (elevation
range, source scene IDs/dates — for provenance — and everything
`GWImportedTerrain` needs), then prints a `HOME_LOCATION` line for you to
paste into `docker-compose.yml`/`.env`.

Drop a `GWImportedTerrain` node (`addons/godotwings/world/ImportedTerrain.gd`)
and point `heightmap_path` / `texture_path` / `metadata_path` at those three
files — it builds a heightmapped, textured, collision-enabled mesh at
`_ready()` (same procedural-build pattern as `examples/Terrain.gd`, just
driven by real data instead of noise). Leave the node's own transform at
identity: the tile is centered on its own origin, and since a `GWVehicleBody`
spawns at NED (0,0) by default, **the AOI's center IS the takeoff point** with
no extra glue — just make sure `HOME_LOCATION` (or `GWGeoReference.home_lat`/
`home_lon`) actually matches `--lat`/`--lon`, so ArduPilot's own GPS origin
agrees with the terrain under it.

One thing worth knowing: Godot's `Image` has no true 16-bit-per-channel
format — loading a 16-bit grayscale PNG silently truncates to 256 levels
(verified directly, not assumed). So the heightmap packs each 16-bit sample
across the R (high byte) and G (low byte) channels of an ordinary RGB8 PNG
instead, which Godot *does* load at exact, full 8-bit-per-channel precision —
`GWImportedTerrain._decode_height16()` unpacks it back losslessly.

`examples/ImportedTerrain.tscn` is a ready-to-run example — a real baked tile
over Helsinki (`examples/terrain/`, matching this project's own default
`HOME_LOCATION`) with a manually-flyable `GWAircraft` (`terrain_following =
true`) that spawns resting right on the real surface.

## Real Cesium 3D Tiles (buildings/terrain meshes)

Real Cesium 3D Tiles content — e.g.
[Google Photorealistic 3D Tiles](https://cesium.com/platform/cesium-ion/content/google-3d-tiles/)
via Cesium ion — for real building/terrain meshes instead of a flat baked
texture. This is a **separate system from `gw_terrain_import.py` above** —
different data (real 3D meshes vs. a heightmapped imagery bake), different
license/caching rules, don't conflate the two.

**Requires a [Cesium ion](https://cesium.com/) account + access token**
(free tier has a usage quota) — nothing here can provision one for you. Set
it as an environment variable, never a file: `export CESIUM_ION_TOKEN=...`.
It must never be committed or hardcoded.

### Live streaming (`GWTiles3DStreamer`) — the real path

`GWTiles3DStreamer` (`addons/godotwings/world/Tiles3DStreamer.gd`) fetches
real tile content around the vehicle's current position **every session**
and holds it in memory only — nothing is ever written to disk. This is the
only path here that actually satisfies Cesium ion's and Google's terms: an
earlier bounded-prefetch design (download once, cache to disk, fly with zero
network access — see below) turned out to violate them outright, pulled
live from the actual current terms, not assumed:

- Cesium ion ToS §2.2.2 (`cesium.com/legal/terms-of-service/`): "You may not
  copy, store, or redistribute any portion of Cesium Data Output in, or for
  use in, **an offline environment**." The only caching exception is generic
  client/proxy caching "that caches other internet traffic too" — ordinary
  HTTP caching during *live* use, not a deliberate prefetch-then-run-offline
  design.
- Google Maps Content terms (`cesium.com/legal/terms-for-google/`): "You
  will not... download Google Maps tiles **or Street View tiles for storage
  or rehosting**."

Drop a `GWTiles3DStreamer` node anywhere in the scene (it auto-finds the
first `GWVehicleBody`, same pattern as `GWFloatingOrigin`) and set
`home_lat`/`home_lon`/`home_alt` to match the vehicle's actual home position
(e.g. `HOME_LOCATION`), plus `asset_id` (`2275207` = Google Photorealistic 3D
Tiles). It resolves the ion asset, then streams tiles within
`streaming_radius_km` of the vehicle's *live* position on two independent
cadences:

- every `poll_interval_s` (default 5s) it re-evaluates what should be loaded
  around wherever the vehicle currently is, adding whatever's newly in range
  and evicting whatever fell out — tiles stream in/out progressively as you
  fly, not as one big chunk;
- separately, once the vehicle drifts past `reanchor_distance_m`, the
  *render frame's* anchor shifts and every already-loaded tile repositions
  instantly from its stored real-world transform (no re-fetch) — this only
  bounds the flat-tangent approximation error, it doesn't drive streaming.

A background thread does all the fetching; results cross back to the main
thread via a queue it drains every frame (mirrors `GWSITLBridge`'s pattern
exactly) so none of this ever blocks the physics loop.

**Keep `streaming_radius_km` small (a couple km)** — coverage over a large
operating area comes from flying and reanchoring repeatedly, not from
setting one big radius upfront. Every fetch in a pass is sequential on the
one background thread, so a large radius means a large multiple of tiles to
walk before the first one ever appears: 10km never finished a single pass in
90 seconds against the real Google asset, while 1km finished in ~4s.

**Attribution is required, not optional**: `GWTiles3DStreamer.attributions`
is populated once the asset resolves — Cesium ion's and the content
provider's terms require displaying these wherever the content is shown to
anyone besides you. Render them in your own UI.

### Offline bounded prefetch (`tools/gw_3dtiles_prefetch.py` + `GWTiles3DLoader`) — local debugging fixture ONLY

Kept in the repo as a convenience for debugging the traversal/transform math
against a small area **without a real ion token** (point `--tileset-url` at
a public, unauthenticated sample, e.g.
[Cesium's own sample tilesets](https://github.com/CesiumGS/3d-tiles-samples))
— **not a compliant way to use real Cesium ion / Google data**, per the ToS
excerpts above; that's exactly why `GWTiles3DStreamer` exists. Do not point
this at a real ion token for anything beyond a momentary local check.

```bash
pip install requests numpy
python3 tools/gw_3dtiles_prefetch.py \
    --tileset-url https://raw.githubusercontent.com/CesiumGS/3d-tiles-samples/main/1.0/TilesetWithDiscreteLOD/tileset.json \
    --lat 40.04253061142592 --lon -75.61209430782448 --radius-km 1 --detail-m 1
```

Output always lands under `.cache/3dtiles/` (gitignored — never commit tile
content or a manifest); `--purge-stale-hours 24` deletes cache directories
older than that; `GWTiles3DLoader.max_cache_age_hours` (default 24h) refuses
to load a stale manifest at runtime; attribution is captured into the
manifest's `content_attributions` and exposed as `GWTiles3DLoader.attributions`
the same way. All of that reduces how far this can drift from "momentary
local debugging" — it does not make the prefetch-and-store architecture
itself compliant for real use.

Drop a `GWTiles3DLoader` node and point `manifest_path` at the
`manifest.json` the tool wrote — same "generate from an offline tool's
output, no network at runtime" pattern as `GWImportedTerrain`, and likewise
centered so the AOI's center sits at the node's own origin, composing with
the rest of the scene the same way.

## Wind, collision & crash

`GWWind` — drop one in the world and every vehicle auto-finds it. Mean wind
(`wind_speed` + `wind_from_deg`, METAR-style bearing it blows from), optional
altitude shear (power law), and optional turbulence (band-limited sum-of-sinusoids
gusts — cheap, deterministic, good for disturbance-rejection testing). The wind
shifts the airspeed the aero sees and is reported to ArduPilot's windvane.

Collision is layered on by querying Godot's physics (the vehicle is not a
`RigidBody3D`, so the validated FDM is untouched):

- `terrain_following` — a downward raycast (`ground_collision_mask`) gives the
  ground height + normal, so the gear/roll-out/crash logic follows 3D terrain.
- `obstacle_mask` — the hull is shapecast against these layers each frame; any
  contact is a crash.
- `aircraft_layer` — set the same non-zero layer on every vehicle for air-to-air
  collision (both crash on overlap).
- `crash_mode` — **Ragdoll** hands the wreck to the physics engine for a real
  tumble (read back into the SITL state, then recovered once it settles);
  **Simple** runs a scripted decelerate-and-settle.

Vehicles emit `took_off` / `landed` / `crashed` / `recovered` /
`controls_received(channels)`. Channels 1–4 are flight controls; 5–16 are free —
map an aux switch to a servo output and a `GWChannelSwitch` turns it into Godot
signals (drop payload, lights, gear…). Read any channel with `control_norm(ch)` /
`control_pwm(ch)`.

## Swarm

Run several vehicles, each ArduPilot instance paired with its own Godot vehicle on
ArduPilot's per-instance convention: vehicle `i` ↔ JSON physics on `9002 + 10·i`.
Add one `GWAircraft` per vehicle with `sitl_instance = 0, 1, 2…` (give each a
distinct `spawn_north`/`spawn_east`), and on the SITL side:

```bash
NUM_VEHICLES=4 docker compose up --build
```

This launches `-I 0..3` and sends MAVLink to `14550, 14560, …` (one comm link per
vehicle). With `NUM_VEHICLES > 1` the instances run headless.

## License

MIT. The stylized sky in `examples/World.tscn` uses GDQuest's
[godot-4-stylized-sky](https://github.com/gdquest-demos/godot-4-stylized-sky)
shader (MIT procedural resources only — no CC-BY-NC-SA art); see
[examples/sky/CREDITS.md](examples/sky/CREDITS.md). Terrain baked by
`tools/gw_terrain_import.py` contains modified Copernicus Sentinel data and
Copernicus DEM data (ESA/EU, free and open under the Copernicus data policy);
attribute accordingly if you redistribute a baked tile.
