## Live Cesium 3D Tiles client (e.g. Google Photorealistic 3D Tiles via
## Cesium ion) -- fetches real tile content around the vehicle's current
## position EVERY SESSION and holds it in memory only. This exists because
## the offline "prefetch once, cache to disk, fly with zero network access"
## design (tools/gw_3dtiles_prefetch.py + GWTiles3DLoader, kept in the repo
## as a local debugging fixture only) turned out to violate the actual
## Cesium ion and Google Maps Content Terms of Service -- pulled live from
## cesium.com/legal/terms-of-service/ (S2.2.2: no storage "in, or for use
## in, an offline environment") and cesium.com/legal/terms-for-google/ ("You
## will not... download Google Maps tiles or Street View tiles for storage
## or rehosting"). This node never writes anything to disk.
##
## Mirrors GWSITLBridge's threading pattern exactly: a dedicated background
## Thread owns all HTTP I/O (Godot's low-level HTTPClient, blocking calls
## are fine inside a thread), guarded by a Mutex, woken by a Semaphore
## whenever the main thread decides the vehicle has moved far enough to
## re-evaluate the area of interest. The thread never touches the scene
## tree; results cross back via a mutex-guarded queue the main thread polls
## in _process() (GWCamera/GWGroundPTZ's non-blocking-poll idiom, since
## there's no lockstep handshake to drive here the way SITL has).
##
## Tile placement (parsing the glb, wrapping rather than overwriting its own
## transform, applying Google's axis correction) is shared with
## GWTiles3DLoader via GWTiles3DContent -- same hard-won correctness fixes,
## one place to keep them fixed.
##
## @tool ON PURPOSE, for a reason that has nothing to do with editing tiles
## in the editor: a non-@tool script attached to a node that ISN'T RUNNING
## (i.e. you're just editing the scene, not playing it) gets backed by a
## lightweight "placeholder" instance that never runs any of the script's
## actual code -- including set_ion_token's custom getter. Verified live:
## pasting a token into that field with the script NOT @tool got the RAW
## VALUE serialized straight into the saved .tscn in plaintext (the exact
## leak set_ion_token exists to prevent), because the placeholder just
## stores whatever you typed with no getter ever intercepting it. Being
## @tool makes a real script instance back the node even while editing, so
## the getter actually runs. _ready() explicitly no-ops under
## Engine.is_editor_hint() so none of the network/thread activity below
## ever starts just from having the scene open.
@tool
class_name GWTiles3DStreamer
extends Node3D

const ION_ENDPOINT := "https://api.cesium.com/v1/assets/%d/endpoint"

## Real-world position of this node's own origin (NED 0,0,0) -- same
## convention as GWImportedTerrain/docker-compose's HOME_LOCATION. The
## vehicle's native NED state is converted through this to get its live
## geodetic position.
@export var home_lat: float = 0.0
@export var home_lon: float = 0.0
@export var home_alt: float = 0.0

## Cesium ion asset id (e.g. 2275207 = Google Photorealistic 3D Tiles).
@export var asset_id: int = 2275207
## Env var to read the Cesium ion access token from at runtime. The token
## itself is never an export/resource -- it must not end up in a saved
## scene or in git.
@export var ion_token_env: String = "CESIUM_ION_TOKEN"

## Inspector convenience for when launching Godot from the editor GUI (which
## doesn't inherit a terminal's `export CESIUM_ION_TOKEN=...`) is less
## practical than relaunching from a shell. Paste a token here and it's
## applied via OS.set_environment(ion_token_env, ...) for THIS run only.
## Two independent layers keep this out of the saved scene, after the first
## alone turned out not to be enough in practice (a real token got
## serialized into a .tscn in plaintext before this existed):
##   1. the getter always returns "" (so the field visibly clears itself
##      right back), which only works because this script is @tool -- a
##      NON-tool script's properties are backed by a placeholder instance
##      while merely editing (not running) a scene, which never runs any
##      script code at all and just stores the raw typed value, bypassing
##      any getter entirely;
##   2. _get_property_list() below declares it PROPERTY_USAGE_EDITOR only,
##      deliberately omitting PROPERTY_USAGE_STORAGE, so Godot's serializer
##      is told never to persist it regardless of whatever value it holds.
## NOT @export -- @export forces default usage flags (including storage)
## that (2) needs to override; declaring it as a plain var and adding it
## via _get_property_list() is the only way to control that.
## Restarting Godot loses it again, same as if you'd never exported it in a
## shell -- prefer the real env var where practical.
var set_ion_token: String = "":
	set(v):
		if v == "":
			return
		if ion_token_env == "":
			push_error("GWTiles3DStreamer: ion_token_env is empty -- set it (default \"CESIUM_ION_TOKEN\") before pasting a token here.")
			return
		OS.set_environment(ion_token_env, v)
	get:
		return ""


func _get_property_list() -> Array:
	return [{
		"name": "set_ion_token",
		"type": TYPE_STRING,
		"usage": PROPERTY_USAGE_EDITOR,  # deliberately no PROPERTY_USAGE_STORAGE
	}]

## The GWVehicleBody to track. Empty = auto-find the first one in the scene
## (same pattern as GWFloatingOrigin).
@export var vehicle_path: NodePath

## Keep this SMALL (a few km) -- the design is small-radius-plus-continuous-
## reanchoring, not one big upfront radius. Every fetch in a pass is
## sequential on the one background thread, so a large radius means a large
## multiple of tiles to walk before the first one ever appears -- found
## live: 10km never finished a pass in 90s (likely tens of thousands of
## tiles for this asset's density), while 1km finished in ~4s. Coverage over
## a big operating area comes from flying and reanchoring repeatedly, not
## from raising this.
@export var streaming_radius_km: float = 2.0
@export var detail_m: float = 30.0
## Simple two-level LOD: an optional wider, coarser ring loaded alongside
## the near ring above. Low-detail content streams in over a LARGER area,
## and the near ring's higher-detail content overlays it within its own
## smaller radius -- tiles inside streaming_radius_km are skipped for this
## far ring entirely (the near ring already covers them at better detail),
## so the two rings never overlap/z-fight. Set far_radius_km <=
## streaming_radius_km to disable this (the far ring becomes a no-op).
@export var far_radius_km: float = 5.0
@export var far_detail_m: float = 100.0
## Re-anchor the RENDER FRAME once the vehicle drifts this far from the
## current anchor (reposition already-loaded tiles from their stored ECEF
## transform, no re-fetch). Kept small (a few km) on purpose -- it bounds
## the flat-tangent/AEQD approximation error in the traversal/pruning math,
## not tied to GWGeoReference's unrelated 5000m default (a different
## system, for a different purpose -- see the class doc comment). This is
## now INDEPENDENT of how often tiles stream in/out -- see poll_interval_s.
@export var reanchor_distance_m: float = 2000.0
## How often (seconds) to re-evaluate what should be loaded around the
## vehicle's CURRENT position, independent of reanchor_distance_m --
## without this, tiles only ever changed at the rare, large reanchor jumps,
## which looked like a big chunk swap rather than tiles streaming in/out
## progressively as you fly. Keep this comfortably longer than a pass
## actually takes (a few seconds for a 1-2km radius): polling faster than a
## pass can complete means passes keep getting superseded before finishing
## and nothing ever streams in at all (harmless -- the generation check
## below still prevents incorrect evictions -- just wasteful).
@export var poll_interval_s: float = 5.0
## Safety cap: never instantiate more than this many tiles at once,
## regardless of how many the traversal finds within streaming_radius_km.
@export var max_tiles_loaded: int = 300
## Instantiate at most this many newly-arrived tiles per frame, so a burst
## of arrivals (e.g. right after a reanchor) doesn't stutter a frame.
@export var tiles_per_frame_budget: int = 4
## Keep a loaded tile until it is this many times streaming_radius_km from the
## vehicle's CURRENT position, rather than dropping it the moment a pass's area
## of interest no longer covers it.
##
## Without a margin, every applied pass prunes back to a circle around wherever
## the vehicle was when that pass STARTED -- tens of seconds ago. Flying a
## circle, that wipes most of what is loaded each time and reloads it moments
## later: measured on a 2.5km circle, 174 tiles down to 47, then 137 down to
## 34. The ground visibly blinks out and refills. 1.0 reproduces that; the
## default keeps anything still plausibly in view.
@export var evict_margin: float = 2.5
## Centre the area of interest this many seconds AHEAD of the vehicle along its
## own velocity, instead of on where it is right now.
##
## A pass takes tens of seconds against a real asset and places nothing until
## it finishes, so an area of interest centred on the current position always
## delivers content for where the vehicle already was -- at 30 m/s, a 20s pass
## lands 600m behind. Leading the area by roughly one pass duration means the
## tiles arrive about where the vehicle will be when they do. 0 restores
## centring on the current position.
@export var lookahead_s: float = 20.0

## Attribution HTML snippets from the ion endpoint response, populated once
## the background thread resolves the asset. Cesium ion's and the content
## provider's terms REQUIRE these be displayed wherever the content itself
## is shown to anyone besides you -- render them in your own UI.
var attributions: Array = []

var _vehicle: GWVehicleBody
var _has_anchor := false
var _anchor_lat := 0.0
var _anchor_lon := 0.0
var _anchor_alt := 0.0
var _content_axis_correction := ""
var _time_since_poll := 1e9  ## huge, so the very first _process() call polls immediately

## id (stripped content URL) -> {"wrapper": Node3D, "ecef_transform": Transform3D}
var _loaded_tiles: Dictionary = {}
var _incoming_tiles: Array = []  ## FIFO of tiles the thread found but haven't been instantiated yet

var _thread: Thread
var _mutex: Mutex
var _wake_sem: Semaphore
var _running := false

# --- shared state (guard with _mutex) ---------------------------------------
var _target_lat := 0.0
var _target_lon := 0.0
var _target_alt := 0.0
var _target_radius_km := 0.0
var _target_far_radius_km := 0.0
var _target_far_detail_m := 0.0
var _target_generation := 0
var _known_ids := PackedStringArray()
var _pending_results: Array = []  ## [{"new_tiles": [...], "evict_ids": PackedStringArray, "generation": int}]
var _pending_info: Array = []     ## [{"error": String} | {"attributions": Array}]

## Main-thread-only: bumped every _reanchor(). A pass's results are tagged
## with whatever generation was current when the thread started that pass --
## if a NEWER reanchor has since superseded it (the pass took longer than
## the time between two reanchors, e.g. flying fast with a small
## reanchor_distance_m), its evictions are stale and must be discarded
## rather than applied: found live, applying them anyway could evict tiles
## that are still genuinely in range of the CURRENT anchor just because an
## outdated pass's AOI didn't happen to include them, and once nothing new
## ever catches up, everything vanishes and never comes back. New tiles
## from a stale pass are still real content, just placed at a position
## recomputed fresh against the CURRENT anchor rather than trusting the
## stale one baked in at fetch time.
var _anchor_generation := 0


func _ready() -> void:
	if Engine.is_editor_hint():
		return  # @tool exists only so set_ion_token's getter runs while editing (see class doc comment) -- no network/thread activity here
	_vehicle = _resolve_vehicle()
	if _vehicle == null:
		push_warning("GWTiles3DStreamer: no GWVehicleBody found to track.")
		return
	_mutex = Mutex.new()
	_wake_sem = Semaphore.new()
	_running = true
	_thread = Thread.new()
	_thread.start(_stream_loop)


func _exit_tree() -> void:
	if _mutex == null:
		return  # _ready() bailed early (no vehicle found) -- nothing was started
	_mutex.lock()
	_running = false
	_mutex.unlock()
	_wake_sem.post()  # unblock the thread if it's parked waiting for work
	if _thread and _thread.is_started():
		_thread.wait_to_finish()


func _resolve_vehicle() -> GWVehicleBody:
	if not vehicle_path.is_empty():
		return get_node_or_null(vehicle_path) as GWVehicleBody
	return _find_flight_body(_scene_root())


## Same pattern as GWFloatingOrigin._scene_root()/_find_flight_body().
func _scene_root() -> Node:
	var n: Node = self
	var top := get_tree().root if get_tree() else null
	while n.get_parent() != null and n.get_parent() != top:
		n = n.get_parent()
	return n


func _find_flight_body(n: Node) -> GWVehicleBody:
	if n is GWVehicleBody:
		return n
	for c in n.get_children():
		var f := _find_flight_body(c)
		if f != null:
			return f
	return null


func _process(delta: float) -> void:
	if _vehicle == null:
		return

	var pos_ned: Vector3 = _vehicle._pos_ned  # established convention: GWCamera/GWGeoReference already reach into this
	# Stream for where the vehicle is heading, not where it is. The lead is
	# capped at the streaming radius: beyond that the area of interest would no
	# longer cover the vehicle itself, which is the one place ground is
	# definitely needed.
	if lookahead_s > 0.0:
		var lead: Vector3 = _vehicle._vel_ned * lookahead_s
		var max_lead := streaming_radius_km * 1000.0
		if lead.length() > max_lead:
			lead = lead.normalized() * max_lead
		pos_ned += lead
	var geo: Array = GWGeodeticConvert.ned_to_geodetic(pos_ned, home_lat, home_lon, home_alt)
	var lat: float = geo[0]
	var lon: float = geo[1]
	var alt: float = geo[2]

	if not _has_anchor:
		_reanchor(lat, lon, alt)  # also does the very first poll
	else:
		var dist := GWGeodeticConvert.geodetic_to_ned(lat, lon, alt, _anchor_lat, _anchor_lon, _anchor_alt).length()
		if dist > reanchor_distance_m:
			_reanchor(lat, lon, alt)

	# Independent of reanchoring: without this, what's loaded only ever
	# changed at the rare, large reanchor jumps, which looked like a big
	# chunk swap rather than tiles streaming in/out progressively as you
	# fly. Each poll's delta is small (however far the vehicle moved in
	# poll_interval_s), so only the tiles actually at the edge of the AOI
	# change between one poll and the next.
	_time_since_poll += delta
	if _time_since_poll >= poll_interval_s:
		_time_since_poll = 0.0
		_poll(lat, lon, alt)

	_drain_results()


## Wakes the background thread to re-evaluate the area of interest around
## (lat, lon, alt) -- called periodically from _process() (see
## poll_interval_s) and once immediately whenever _reanchor() runs, so a
## fresh anchor gets evaluated right away rather than waiting out the next
## poll interval.
func _poll(lat: float, lon: float, alt: float) -> void:
	_mutex.lock()
	_target_lat = lat
	_target_lon = lon
	_target_alt = alt
	_target_radius_km = streaming_radius_km
	_target_far_radius_km = far_radius_km
	_target_far_detail_m = far_detail_m
	_target_generation = _anchor_generation
	_known_ids = PackedStringArray(_loaded_tiles.keys())
	_mutex.unlock()
	_wake_sem.post()
	_time_since_poll = 0.0


## Recompute every already-loaded tile's Godot position fresh from its
## stored raw ECEF transform, relative to the new anchor -- no re-fetch
## needed. This avoids the curvature drift a naive "shift by a flat delta"
## would accumulate over a long flight (unlike GWFloatingOrigin's rebase,
## which is a flat shift for a DIFFERENT purpose -- keeping render-space
## numbers small -- and is fine as a flat shift for that). This no longer
## drives tile streaming itself -- see poll_interval_s/_poll() -- it just
## keeps the render frame's numbers bounded and repositions what's already
## loaded; it still triggers one immediate poll so a fresh anchor doesn't
## sit idle until the next scheduled one.
##
## Crucially, this node's OWN position is moved too -- not just its
## children. Every wrapper's position is "offset from the anchor," so
## unless this node ALSO moves to where the anchor now sits (relative to
## home, in the SAME frame GWVehicleBody's own render position uses), every
## reanchor recomputes all children relative to a new reference point while
## the parent stays put: found live as periodic large jumps in the ground
## every time the anchor moved, roughly reanchor_distance_m in size. Setting
## it here keeps tile positions consistent with the vehicle's own render
## position (ned_to_world(pos_ned), always relative to home) regardless of
## how many times this has reanchored, and is exactly what makes this node
## behave correctly as a GWFloatingOrigin shift_node too, if one is present.
func _reanchor(lat: float, lon: float, alt: float) -> void:
	# Bumped here and ONLY here: the eviction guard in _drain_results uses this
	# to spot a pass whose AOI was computed against a superseded anchor. It used
	# to be bumped in _poll() instead, which runs every poll_interval_s -- so
	# for any pass slower than one poll (they take tens of seconds against a
	# real asset, polls default to 5s) the guard never matched and evictions
	# were silently dropped every time. Tiles then accumulated until
	# max_tiles_loaded, after which newly fetched tiles could not be placed at
	# all: flying on, the ground ahead stayed empty while stale tiles behind
	# were kept forever. Measured on a 2.5km circle: 46 tiles rising
	# monotonically to exactly 300, with zero evictions applied.
	_anchor_generation += 1
	_has_anchor = true
	_anchor_lat = lat
	_anchor_lon = lon
	_anchor_alt = alt

	var anchor_ned: Vector3 = GWGeodeticConvert.geodetic_to_ned(lat, lon, alt, home_lat, home_lon, home_alt)
	position = GWCoordConvert.ned_to_world(anchor_ned)

	for id in _loaded_tiles:
		var entry: Dictionary = _loaded_tiles[id]
		var wrapper_transform: Transform3D = GWTiles3DTraversal.local_transform_to_godot(
				entry["ecef_transform"], lat, lon, alt)
		if _content_axis_correction == "google_yup_ecef":
			wrapper_transform.basis = wrapper_transform.basis * GWTiles3DContent.GOOGLE_YUP_ECEF_CORRECTION
		entry["wrapper"].transform = wrapper_transform

	_poll(lat, lon, alt)


func _drain_results() -> void:
	_mutex.lock()
	var batches := _pending_results
	_pending_results = []
	var infos := _pending_info
	_pending_info = []
	_mutex.unlock()

	for info in infos:
		if info.has("error"):
			push_error(info["error"])
		if info.has("attributions"):
			attributions = info["attributions"]
			for html in attributions:
				push_warning("GWTiles3DStreamer: attribution required by the content provider's terms -- render this in your own UI: %s" % html)
		if info.has("content_axis_correction"):
			_content_axis_correction = info["content_axis_correction"]
		if info.has("stats"):
			var s: Dictionary = info["stats"]
			var aoi: Dictionary = info["aoi"]
			print("GWTiles3DStreamer: pass at (%.6f, %.6f) r=%.2fkm far_r=%.2fkm -- visited=%d pruned=%d excluded=%d downloaded=%d http_errors=%d" %
					[aoi["lat"], aoi["lon"], aoi["radius_km"], aoi.get("far_radius_km", 0.0), s["visited"], s["pruned"], s.get("excluded", 0), s["downloaded"], s["http_errors"]])
			if s["visited"] == 1 and s["pruned"] == 1:
				push_warning("GWTiles3DStreamer: the tileset root itself was pruned -- home_lat/home_lon likely doesn't overlap " +
						"real content for this asset (e.g. open ocean), or streaming_radius_km is too small.")
			if s["http_errors"] > 0 and s["downloaded"] == 0:
				push_warning("GWTiles3DStreamer: every request this pass failed (http_errors=%d) -- check network access " %
						s["http_errors"] + "and that the ion token/asset are actually valid (a resolve failure would have " +
						"already errored separately; this means the ion asset resolved but tileset/content requests are failing).")

	var got_new_tiles := false
	for batch in batches:
		# A pass started under an OLDER anchor than the current one (a newer
		# reanchor superseded it before it finished) has evictions based on
		# an outdated area -- applying them could remove tiles that are
		# still genuinely in range of the CURRENT anchor just because that
		# stale pass's AOI didn't happen to cover them. Found live: without
		# this, reanchoring while a pass was still in flight made everything
		# vanish and never come back, because no later pass ever "caught up"
		# to re-add what the stale one wrongly evicted. New tiles are still
		# real content either way -- just placed at a position recomputed
		# fresh against the CURRENT anchor (see _place_new_tile) rather than
		# trusting whatever anchor was active when they were fetched.
		if int(batch["generation"]) == _anchor_generation:
			# The pass proposes what its own area of interest no longer covers;
			# whether a tile actually goes is decided here against the
			# vehicle's live position, which has moved on since.
			var keep_radius := streaming_radius_km * 1000.0 * maxf(evict_margin, 1.0)
			var vehicle_pos := GWCoordConvert.ned_to_world(_vehicle._pos_ned) if _vehicle != null else Vector3.ZERO
			for id in batch["evict_ids"]:
				if not _loaded_tiles.has(id):
					continue
				# Deliberately NOT the wrapper's own position: for Google tiles
				# that is ~Earth-center-relative and identical for every tile
				# in a pass (see approx_position's doc comment), so using it
				# put every tile ~18000m away and evicted the lot.
				var e: Vector3 = _loaded_tiles[id].get("approx_ecef", Vector3.ZERO)
				if e == Vector3.ZERO:
					continue  # unknown position: keep it rather than guess
				var tile_pos: Vector3 = position + GWGeodeticConvert.ecef_xyz_to_godot_position(
						e.x, e.y, e.z, _anchor_lat, _anchor_lon, _anchor_alt)
				if tile_pos.distance_to(vehicle_pos) > keep_radius:
					_evict_tile(id)
		for tile in batch["new_tiles"]:
			if not _loaded_tiles.has(tile["id"]):
				_incoming_tiles.append(tile)
				got_new_tiles = true

	# Nearest-first, not traversal-order: a tileset's own tree order has no
	# relation to distance from the anchor, so without this a tile 1km away
	# could sit ahead of the queue while the one right under the vehicle is
	# still waiting its turn under a small per-frame budget -- found live:
	# the first tiles to actually appear were 800-1100m from the aircraft
	# while it sat only ~60m from the camera.
	if got_new_tiles:
		_incoming_tiles.sort_custom(func(a, b): return a["approx_position"].length_squared() < b["approx_position"].length_squared())

	var budget := tiles_per_frame_budget
	while budget > 0 and _incoming_tiles.size() > 0 and _loaded_tiles.size() < max_tiles_loaded:
		_place_new_tile(_incoming_tiles.pop_front())
		budget -= 1


func _place_new_tile(tile: Dictionary) -> void:
	# Recomputed fresh against the CURRENT anchor, not the (possibly several
	# reanchors stale by now) wrapper_transform computed back when this tile
	# was fetched -- same reasoning as the generation check above, just for
	# position instead of eviction. ecef_transform is absolute and anchor-
	# independent, so this is always correct regardless of how long the tile
	# sat queued.
	var wrapper_transform: Transform3D = GWTiles3DTraversal.local_transform_to_godot(
			tile["ecef_transform"], _anchor_lat, _anchor_lon, _anchor_alt)
	var wrapper := GWTiles3DContent.place_tile(self, tile["bytes"], wrapper_transform,
			_content_axis_correction, String(tile["id"]).get_file())
	if wrapper == null:
		push_warning("GWTiles3DStreamer: failed to parse/place tile %s" % tile["id"])
		return
	_loaded_tiles[tile["id"]] = {
		"wrapper": wrapper,
		"ecef_transform": tile["ecef_transform"],
		"approx_ecef": tile.get("approx_ecef", Vector3.ZERO),
	}


func _evict_tile(id: String) -> void:
	if _loaded_tiles.has(id):
		_loaded_tiles[id]["wrapper"].queue_free()
		_loaded_tiles.erase(id)


# -----------------------------------------------------------------------------
# Thread: HTTP I/O and traversal only. Never touch the scene tree from here.
# -----------------------------------------------------------------------------

var _auth_params: Array = []       ## thread-local only
var _tileset_root_url := ""        ## thread-local only
var _is_google := false            ## thread-local only


func _stream_loop() -> void:
	var ion_token := OS.get_environment(ion_token_env)
	if ion_token == "":
		_post_info({"error": "GWTiles3DStreamer: %s is not set -- export it before running (never commit a token)." % ion_token_env})
		return

	var resolved := _resolve_ion_asset(ion_token)
	if not resolved["ok"]:
		_post_info({"error": "GWTiles3DStreamer: " + String(resolved["error"])})
		return
	_tileset_root_url = resolved["tileset_url"]
	_is_google = resolved["is_google"]
	if String(resolved["auth_query"]) != "":
		_auth_params = _ensure_param(_auth_params, resolved["auth_query"])
	_post_info({"attributions": resolved["attributions"],
			"content_axis_correction": "google_yup_ecef" if _is_google else ""})

	while true:
		_wake_sem.wait()
		_mutex.lock()
		var running := _running
		var lat := _target_lat
		var lon := _target_lon
		var alt := _target_alt
		var radius_km := _target_radius_km
		var far_radius_km_snapshot := _target_far_radius_km
		var far_detail_m_snapshot := _target_far_detail_m
		var generation := _target_generation
		var known_ids := PackedStringArray(_known_ids)
		_mutex.unlock()
		if not running:
			break

		var aoi_center: Vector3 = GWGeodeticConvert.geodetic_to_ecef(lat, lon, alt)
		var found_ids := {}
		var new_tiles := []
		var stats := {"visited": 0, "pruned": 0, "downloaded": 0, "http_errors": 0, "excluded": 0}

		# Near ring: full detail, no exclusion -- unchanged behavior.
		var aoi_bbox: Array = GWTiles3DTraversal.bbox_from_center(lat, lon, radius_km)
		var visited_near := {}
		_walk_tileset(_tileset_root_url, Transform3D.IDENTITY, aoi_bbox, aoi_center, radius_km * 1000.0,
				lat, lon, alt, detail_m, 0.0, known_ids, found_ids, new_tiles, visited_near, stats)

		# Far ring: simple two-level LOD -- a wider, coarser pass that skips
		# whatever the near ring above already covers at better detail (see
		# far_radius_km's doc comment). A fresh `visited` set is needed since
		# this walks the SAME tileset tree again at a different detail_m
		# threshold, so tiles the near pass pruned as "too coarse to bother"
		# may legitimately have "content" here instead.
		if far_radius_km_snapshot > radius_km:
			var aoi_bbox_far: Array = GWTiles3DTraversal.bbox_from_center(lat, lon, far_radius_km_snapshot)
			var visited_far := {}
			_walk_tileset(_tileset_root_url, Transform3D.IDENTITY, aoi_bbox_far, aoi_center, far_radius_km_snapshot * 1000.0,
					lat, lon, alt, far_detail_m_snapshot, radius_km * 1000.0, known_ids, found_ids, new_tiles, visited_far, stats)

		var evict_ids := PackedStringArray()
		for id in known_ids:
			if not found_ids.has(id):
				evict_ids.append(id)

		# Always report a summary, even an all-zero one -- a silent pass looks
		# identical to a hung/broken one otherwise, which cost real
		# troubleshooting time before this existed.
		_post_info({"stats": stats, "aoi": {"lat": lat, "lon": lon, "radius_km": radius_km, "far_radius_km": far_radius_km_snapshot}})

		if new_tiles.size() > 0 or evict_ids.size() > 0:
			_mutex.lock()
			_pending_results.append({"new_tiles": new_tiles, "evict_ids": evict_ids, "generation": generation})
			_mutex.unlock()


func _post_info(info: Dictionary) -> void:
	_mutex.lock()
	_pending_info.append(info)
	_mutex.unlock()


## Returns {"ok": bool, "tileset_url": String, "auth_query": String,
## "attributions": Array, "is_google": bool} -- same two response shapes
## verified live against a real ion account this session (see
## gw_3dtiles_prefetch.py's resolve_ion_asset for the full rationale):
## a normal ion-hosted asset {"url":..., "accessToken":...}, or an
## externally-proxied one (Google) {"options":{"url":"...?key=..."}} with no
## separate token -- the query string on that url must be re-applied to
## every subsequent request.
func _resolve_ion_asset(ion_token: String) -> Dictionary:
	var url := (ION_ENDPOINT % asset_id) + "?access_token=" + ion_token
	var resp := _http_get(url)
	if not resp["ok"]:
		return {"ok": false, "error": "ion asset resolve failed (%s)" % resp.get("error", "status %d" % int(resp.get("status", -1)))}
	var parsed = JSON.parse_string((resp["body"] as PackedByteArray).get_string_from_utf8())
	if not (parsed is Dictionary):
		return {"ok": false, "error": "ion endpoint response was not valid JSON"}
	var endpoint: Dictionary = parsed
	var tileset_url = endpoint.get("url", null)
	if tileset_url == null and endpoint.has("options"):
		tileset_url = (endpoint["options"] as Dictionary).get("url", null)
	if tileset_url == null:
		return {"ok": false, "error": "ion endpoint response has no tileset url"}
	var attributions := []
	for a in endpoint.get("attributions", []):
		if (a as Dictionary).has("html"):
			attributions.append(a["html"])
	var tile_token = endpoint.get("accessToken", null)
	var auth_query := ""
	if tile_token != null:
		auth_query = "access_token=%s" % tile_token
	elif String(tileset_url).find("?") != -1:
		auth_query = String(tileset_url).split("?", true, 1)[1]
	return {"ok": true, "tileset_url": tileset_url, "auth_query": auth_query, "attributions": attributions,
			"is_google": String(tileset_url).find("tile.googleapis.com") != -1}


func _ensure_param(params: Array, param: String) -> Array:
	var name: String = param.split("=", true, 1)[0]
	var result := []
	for p in params:
		if String(p).split("=", true, 1)[0] != name:
			result.append(p)
	result.append(param)
	return result


## allow_session=false excludes the "session" param specifically -- see
## _walk_tileset's use of this: the top-level tileset root endpoint (unlike
## every dataset-specific path under it) rejects a session param outright
## (verified live: HTTP 400 "Unknown name 'session': Cannot bind query
## parameter"), which only bites once a session has actually been captured
## -- so the root endpoint fetches fine on the very first poll (nothing
## captured yet) and then 400s on every poll after that, since it keeps
## getting re-fetched (once per poll) with an ever-more-stale session
## param attached. This is what "everything gets evicted and never streams
## back in past the first poll" turned out to be.
func _apply_auth(url: String, allow_session: bool = true) -> String:
	var result := url
	for param in _auth_params:
		var name: String = String(param).split("=", true, 1)[0]
		if name == "session" and not allow_session:
			continue
		if result.find(name + "=") == -1:
			result += ("&" if result.find("?") != -1 else "?") + String(param)
	return result


## Google mints a fresh `session` token per tileset.json fetch, embedded in
## some (not all) of that response's own content URIs -- must be captured
## and reused for every subsequent request tied to that fetch (verified
## live: a content/nested-tileset URI lacking its own `session=` 400s with
## just the ion `key=`; reusing the session captured from a sibling URI in
## the same response succeeds). Returns "" if none found.
func _find_session_token(tile: Dictionary) -> String:
	var contents = tile.get("contents", null)
	if contents == null and tile.has("content"):
		contents = [tile["content"]]
	if contents != null:
		for c in contents:
			var uri: String = (c as Dictionary).get("uri", "")
			var q_idx := uri.find("?")
			if q_idx != -1:
				for pair in uri.substr(q_idx + 1).split("&"):
					var kv := pair.split("=", true, 1)
					if kv.size() == 2 and kv[0] == "session":
						return kv[1]
	for child in tile.get("children", []):
		var found := _find_session_token(child)
		if found != "":
			return found
	return ""


func _walk_tileset(tileset_url: String, base_transform: Transform3D, aoi_bbox: Array, aoi_center: Vector3,
		aoi_radius: float, anchor_lat: float, anchor_lon: float, anchor_alt: float,
		detail_m_threshold: float, exclude_radius_m: float,
		known_ids: PackedStringArray, found_ids: Dictionary, new_tiles: Array, visited: Dictionary, stats: Dictionary) -> void:
	if visited.has(tileset_url):
		return  # avoid infinite loops on a malformed/self-referential tileset
	visited[tileset_url] = true
	# See _apply_auth's doc comment: the top-level root endpoint specifically
	# must never carry a session param, even once one has been captured from
	# an earlier poll's traversal.
	var is_root := tileset_url.split("?", true, 1)[0] == _tileset_root_url.split("?", true, 1)[0]
	var resp := _http_get(_apply_auth(tileset_url, not is_root))
	if not resp["ok"]:
		stats["http_errors"] += 1
		return  # network hiccup -- skip this branch this pass, next wake retries
	var parsed = JSON.parse_string((resp["body"] as PackedByteArray).get_string_from_utf8())
	if not (parsed is Dictionary) or not (parsed as Dictionary).has("root"):
		stats["http_errors"] += 1
		return
	var doc: Dictionary = parsed
	var root: Dictionary = doc["root"]
	var session_token := _find_session_token(root)
	if session_token != "":
		_auth_params = _ensure_param(_auth_params, "session=%s" % session_token)
	_walk_tile(root, base_transform, tileset_url, aoi_bbox, aoi_center, aoi_radius,
			anchor_lat, anchor_lon, anchor_alt, detail_m_threshold, exclude_radius_m, known_ids, found_ids, new_tiles, visited, stats)


func _walk_tile(tile: Dictionary, parent_transform: Transform3D, base_url: String, aoi_bbox: Array,
		aoi_center: Vector3, aoi_radius: float, anchor_lat: float, anchor_lon: float, anchor_alt: float,
		detail_m_threshold: float, exclude_radius_m: float,
		known_ids: PackedStringArray, found_ids: Dictionary, new_tiles: Array, visited: Dictionary, stats: Dictionary) -> void:
	stats["visited"] += 1
	var result := GWTiles3DTraversal.evaluate_tile(tile, parent_transform, detail_m_threshold, aoi_bbox, aoi_center, aoi_radius)
	match result["action"]:
		"content":
			var bv: Dictionary = tile.get("boundingVolume", {})
			for content in result["contents"]:
				_handle_content(content, result["transform"], bv, base_url, aoi_bbox, aoi_center, aoi_radius,
						anchor_lat, anchor_lon, anchor_alt, detail_m_threshold, exclude_radius_m, known_ids, found_ids, new_tiles, visited, stats)
		"recurse":
			for entry in result["children"]:
				_walk_tile(entry["tile"], entry["transform"], base_url, aoi_bbox, aoi_center, aoi_radius,
						anchor_lat, anchor_lon, anchor_alt, detail_m_threshold, exclude_radius_m, known_ids, found_ids, new_tiles, visited, stats)
		"prune":
			stats["pruned"] += 1
		# "none": nothing to do


func _handle_content(content: Dictionary, transform: Transform3D, bv: Dictionary, base_url: String,
		aoi_bbox: Array, aoi_center: Vector3, aoi_radius: float,
		anchor_lat: float, anchor_lon: float, anchor_alt: float,
		detail_m_threshold: float, exclude_radius_m: float,
		known_ids: PackedStringArray, found_ids: Dictionary, new_tiles: Array, visited: Dictionary, stats: Dictionary) -> void:
	var uri: String = content.get("uri", content.get("url", ""))
	if uri == "":
		return
	var content_url := _resolve_relative_url(base_url, uri)
	var stripped := content_url.split("?", true, 1)[0]

	if stripped.ends_with(".json"):
		# External tileset reference -- recurse into it with the transform
		# accumulated so far as its new base. Same AOI (it's anchor-derived,
		# not tileset-derived).
		_walk_tileset(content_url, transform, aoi_bbox, aoi_center, aoi_radius,
				anchor_lat, anchor_lon, anchor_alt, detail_m_threshold, exclude_radius_m, known_ids, found_ids, new_tiles, visited, stats)
		return

	var approx_ecef_for_exclusion := _approx_tile_center_ecef(bv, transform)
	if exclude_radius_m > 0.0 and (approx_ecef_for_exclusion - aoi_center).length() < exclude_radius_m:
		# The far/coarse ring's tile falls within the near ring's radius,
		# which already covers it at better detail -- skip it here so the
		# two rings never place overlapping/z-fighting content for the same
		# real-world area.
		stats["excluded"] += 1
		return

	# A stable spatial hash of the tile's real-world position, NOT the
	# content URL -- found live: Google mints a fresh, unique opaque path
	# segment (not just a query token) for the SAME real-world tile on every
	# separate tileset.json fetch, apparently tied to its ephemeral session.
	# Keying identity on that URL meant every poll saw "all new" content and
	# kept re-adding duplicates of tiles it already had (three identical
	# stationary polls produced 3x the tile count instead of the same one).
	# The bounding-volume-derived position is real, stable geographic data
	# and costs nothing extra to compute (already needed for approx_position;
	# reused from the exclusion check above rather than recomputed).
	var approx_ecef := approx_ecef_for_exclusion
	var stable_id := _stable_tile_id(approx_ecef)

	if found_ids.has(stable_id):
		return  # already handled this pass (a tile can be reachable via multiple paths)
	found_ids[stable_id] = true
	if known_ids.has(stable_id):
		stats["downloaded"] += 1  # already loaded, but still "in range" -- counts toward a non-zero pass
		return  # already loaded by the main thread -- nothing to fetch

	var resp := _http_get(_apply_auth(content_url))
	if not resp["ok"]:
		stats["http_errors"] += 1
		return
	var data: PackedByteArray = GWTiles3DTraversal.unwrap_b3dm(resp["body"])
	if data.size() < 4 or data.slice(0, 4).get_string_from_ascii() != "glTF":
		return  # not a glb/b3dm we handle (e.g. .pnts/.i3dm) -- v1 targets mesh content only

	stats["downloaded"] += 1
	new_tiles.append({
		"id": stable_id,
		"bytes": data,
		"wrapper_transform": GWTiles3DTraversal.local_transform_to_godot(transform, anchor_lat, anchor_lon, anchor_alt),
		"ecef_transform": transform,
		"content_axis_correction": "google_yup_ecef" if _is_google else "",
		# Rough placement-priority position, NOT the real final position (that
		# only exists after parsing the content, see place_tile) -- from the
		# tile's OWN boundingVolume, which is real ECEF-ish data available
		# before downloading anything. Needed because wrapper_transform alone
		# is useless for this: for Google tiles it's identical (~Earth-center-
		# relative) for every tile in a pass, since the real per-tile offset
		# lives entirely inside each tile's own content, not the tileset
		# chain -- found live, sorting by wrapper_transform.origin was a
		# silent no-op (every tile tied on the same key).
		"approx_position": GWGeodeticConvert.ecef_xyz_to_godot_position(
				approx_ecef.x, approx_ecef.y, approx_ecef.z, anchor_lat, anchor_lon, anchor_alt),
		# The same centre, anchor-independent: approx_position above is relative
		# to whatever anchor was current at fetch time, so it goes stale on a
		# reanchor. Keeping the raw ECEF lets a tile's real position be
		# recomputed against the CURRENT anchor whenever it is needed.
		"approx_ecef": approx_ecef,
	})


## Best-effort tile center from its boundingVolume, in world ECEF -- see
## approx_position's doc comment above for why this (not wrapper_transform)
## is the right thing to sort placement priority by.
func _approx_tile_center_ecef(bv: Dictionary, transform: Transform3D) -> Vector3:
	if bv.has("box"):
		var box: Array = bv["box"]
		return transform * Vector3(box[0], box[1], box[2])
	if bv.has("sphere"):
		var sph: Array = bv["sphere"]
		return transform * Vector3(sph[0], sph[1], sph[2])
	if bv.has("region"):
		var region: Array = bv["region"]
		var mid_lon := rad_to_deg((float(region[0]) + float(region[2])) * 0.5)
		var mid_lat := rad_to_deg((float(region[1]) + float(region[3])) * 0.5)
		var mid_alt := (float(region[4]) + float(region[5])) * 0.5
		return GWGeodeticConvert.geodetic_to_ecef(mid_lat, mid_lon, mid_alt)  # region is already geodetic/global, not local to transform
	return transform.origin  # unknown volume type -- fall back to whatever we have


## A stable identity for "this real-world tile" from its approximate ECEF
## center, coarse enough (10m grid) to be stable across separate fetches of
## the same tile (its bounding volume center should match to well under
## that between fetches) while still distinguishing genuinely different
## nearby tiles at this asset's typical tile spacing.
static func _stable_tile_id(ecef_center: Vector3) -> String:
	const GRID := 10.0
	return "%d,%d,%d" % [roundi(ecef_center.x / GRID), roundi(ecef_center.y / GRID), roundi(ecef_center.z / GRID)]


## uri may be a full URL, an absolute path ("/v1/..."), or a path relative
## to base's own directory (e.g. a Cesium sample's "dragon_low.b3dm"). No
## proper URI-join is exposed to GDScript, so this handles exactly the
## shapes seen in real 3D Tiles data.
static func _resolve_relative_url(base: String, uri: String) -> String:
	if uri.begins_with("http://") or uri.begins_with("https://"):
		return uri
	var scheme_end := base.find("://") + 3
	var host_end := base.find("/", scheme_end)
	if host_end == -1:
		host_end = base.length()
	var origin := base.substr(0, host_end)
	if uri.begins_with("/"):
		return origin + uri
	var base_no_query := base.split("?", true, 1)[0]
	var last_slash := base_no_query.rfind("/")
	var base_dir := base_no_query.substr(0, last_slash + 1) if last_slash != -1 else origin + "/"
	return base_dir + uri


var _http_clients: Dictionary = {}  ## thread-local only: "host:port" -> HTTPClient, kept alive across requests


## Blocking GET via Godot's low-level HTTPClient -- fine to call from a
## background thread (mirrors GWSITLBridge's blocking-socket-in-a-thread
## pattern). Reuses one HTTPClient (and its TLS session) per host across the
## whole traversal instead of reconnecting for every request -- found live:
## without this, a single pass against the real Google tileset (~100+
## requests) made a fresh TLS handshake every time and never finished within
## 90 seconds; the equivalent Python tool did the same traversal in ~4s
## using a persistent requests.Session(). Returns {"ok": bool, "status": int,
## "body": PackedByteArray}.
func _http_get(url: String, _retried: bool = false) -> Dictionary:
	var parsed := _parse_url(url)
	if parsed.is_empty():
		return {"ok": false, "error": "could not parse URL: %s" % url}
	var key: String = "%s:%d" % [parsed["host"], parsed["port"]]

	var client: HTTPClient = _http_clients.get(key)
	if client == null:
		client = HTTPClient.new()
		var err := client.connect_to_host(parsed["host"], parsed["port"], TLSOptions.client())
		if err != OK:
			return {"ok": false, "error": "connect_to_host failed: %d" % err}
		while client.get_status() == HTTPClient.STATUS_CONNECTING or client.get_status() == HTTPClient.STATUS_RESOLVING:
			client.poll()
			OS.delay_msec(1)
		if client.get_status() != HTTPClient.STATUS_CONNECTED:
			return {"ok": false, "error": "connection failed, status %d" % client.get_status()}
		_http_clients[key] = client

	var err := client.request(HTTPClient.METHOD_GET, parsed["path"], PackedStringArray(["User-Agent: GodotWings-Tiles3DStreamer"]))
	if err != OK:
		_http_clients.erase(key)
		if not _retried:
			return _http_get(url, true)  # a reused connection can go stale server-side -- retry once, fresh
		return {"ok": false, "error": "request() failed: %d" % err}
	while client.get_status() == HTTPClient.STATUS_REQUESTING:
		client.poll()
		OS.delay_msec(1)

	var status := client.get_status()
	if status != HTTPClient.STATUS_BODY and status != HTTPClient.STATUS_CONNECTED:
		_http_clients.erase(key)
		if not _retried:
			return _http_get(url, true)
		return {"ok": false, "error": "bad response status %d" % status}
	var code := client.get_response_code()
	var body := PackedByteArray()
	while client.get_status() == HTTPClient.STATUS_BODY:
		client.poll()
		var chunk := client.read_response_body_chunk()
		if chunk.size() == 0:
			OS.delay_msec(1)
		else:
			body.append_array(chunk)
	return {"ok": code >= 200 and code < 300, "status": code, "body": body}


## https URLs only -- every real endpoint this node talks to uses https.
static func _parse_url(url: String) -> Dictionary:
	if not url.begins_with("https://"):
		return {}
	var rest := url.substr(8)
	var slash_idx := rest.find("/")
	var host_port := rest if slash_idx == -1 else rest.substr(0, slash_idx)
	var path := "/" if slash_idx == -1 else rest.substr(slash_idx)
	var host := host_port
	var port := 443
	var colon_idx := host_port.find(":")
	if colon_idx != -1:
		host = host_port.substr(0, colon_idx)
		port = int(host_port.substr(colon_idx + 1))
	return {"host": host, "port": port, "path": path}
