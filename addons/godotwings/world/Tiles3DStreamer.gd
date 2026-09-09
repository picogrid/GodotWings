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

## The GWVehicleBody to track. Empty auto-finds the first one in the scene.
@export var vehicle_path: NodePath

## Cameras drive visible-detail selection. streaming_radius_km is a bounded,
## coarse safety area around the vehicle for collision and placement even when
## it is outside every camera. far_radius_km bounds all visible coverage.
@export var streaming_radius_km: float = 1.0
@export var far_radius_km: float = 40.0
@export var maximum_screen_space_error: float = 4.0
@export var reanchor_distance_m: float = 2000.0
@export var poll_interval_s: float = 0.25
@export var max_tiles_loaded: int = 1536
@export var tiles_per_frame_budget: int = 4
@export var lookahead_s: float = 20.0

const MAX_PENDING_RESULTS := 256
const MAX_INCOMING_TILES := 256
const MAX_TILESET_DOCUMENTS := 4096

## Attribution HTML snippets from the ion endpoint response.
var attributions: Array = []

var _vehicle: GWVehicleBody
var _explicit_cameras := false
var _camera_refs: Array[WeakRef] = []
var _warned_orthographic := false
var _has_anchor := false
var _anchor_lat := 0.0
var _anchor_lon := 0.0
var _anchor_alt := 0.0
var _content_axis_correction := ""
var _time_since_poll := 1e9
var _anchor_generation := 0
var _view_revision := 0
var _accepted_view_revision := -1
var _accepted_plan_serial := -1

## id -> wrapper and immutable placement metadata. Hidden wrappers are staged
## REPLACE children or retained fallbacks, never simultaneously active opaque
## coverage with their replacement descendants.
var _loaded_tiles: Dictionary = {}
var _wrapper_entries: Dictionary = {}
var _incoming_tiles: Array = []
var _replacement_groups: Dictionary = {}
var _parent_groups: Dictionary = {}
var _current_desired_ids: Dictionary = {}
var _accepted_fine_desired_ids: Dictionary = {}
var _protected_tile_ids: Dictionary = {}

var _thread: Thread
var _mutex: Mutex
var _wake_sem: Semaphore
var _running := false
var _wake_posted := false

# Shared state guarded by _mutex.
var _target_lat := 0.0
var _target_lon := 0.0
var _target_alt := 0.0
var _target_anchor_lat := 0.0
var _target_anchor_lon := 0.0
var _target_anchor_alt := 0.0
var _target_local_center := Vector3.ZERO
var _target_radius_km := 0.0
var _target_far_radius_km := 0.0
var _target_maximum_sse := 0.0
var _target_generation := 0
var _target_view_revision := 0
var _target_change_revision := 0
var _target_views: Array = []
var _known_ids := PackedStringArray()
var _pending_results: Array = []
var _pending_plans: Dictionary = {}
var _pending_info: Array = []

# Thread-local session state. Documents and payloads never leave memory.
var _auth_params: Array = []
var _tileset_root_url := ""
var _is_google := false
var _document_cache: Dictionary = {}
var _selection_budget_exhausted := false


## Configure every render camera that must participate in selection. Weak
## references keep cameras safe across SubViewport teardown. An empty array
## restores standalone active-root-viewport camera discovery.
func set_cameras(cameras: Array[Camera3D]) -> void:
	_explicit_cameras = not cameras.is_empty()
	_camera_refs.clear()
	for camera in cameras:
		if is_instance_valid(camera):
			_camera_refs.append(weakref(camera))
	_time_since_poll = 1e9


func _ready() -> void:
	if Engine.is_editor_hint():
		return
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
		return
	_mutex.lock()
	_running = false
	_mutex.unlock()
	_wake_sem.post()
	if _thread and _thread.is_started():
		_thread.wait_to_finish()


func _resolve_vehicle() -> GWVehicleBody:
	if not vehicle_path.is_empty():
		return get_node_or_null(vehicle_path) as GWVehicleBody
	return _find_flight_body(_scene_root())


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
	var pos_ned: Vector3 = _vehicle._pos_ned
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
		_reanchor(lat, lon, alt)
	else:
		var dist := GWGeodeticConvert.geodetic_to_ned(lat, lon, alt, _anchor_lat, _anchor_lon, _anchor_alt).length()
		if dist > reanchor_distance_m:
			_reanchor(lat, lon, alt)
	_time_since_poll += delta
	if _time_since_poll >= poll_interval_s:
		_poll(lat, lon, alt)
	_drain_results()


func _camera_snapshots() -> Array:
	var cameras: Array[Camera3D] = []
	if _explicit_cameras:
		var retained: Array[WeakRef] = []
		for ref in _camera_refs:
			var camera := ref.get_ref() as Camera3D
			if is_instance_valid(camera):
				cameras.append(camera)
				retained.append(ref)
		_camera_refs = retained
	else:
		var viewport := get_viewport()
		var camera := viewport.get_camera_3d() if viewport != null else null
		if is_instance_valid(camera):
			cameras.append(camera)
	var views := []
	var inverse_basis := global_transform.basis.inverse()
	for camera in cameras:
		if not camera.is_inside_tree() or camera.get_viewport() == null:
			continue
		if camera.projection != Camera3D.PROJECTION_PERSPECTIVE:
			if not _warned_orthographic:
				push_warning("GWTiles3DStreamer: orthographic cameras do not participate in terrain LOD selection.")
				_warned_orthographic = true
			continue
		var size := camera.get_viewport().get_visible_rect().size
		if size.y <= 0.0 or size.x <= 0.0:
			continue
		var aspect := size.x / size.y
		var tan_x: float
		var tan_y: float
		if camera.keep_aspect == Camera3D.KEEP_HEIGHT:
			tan_y = tan(deg_to_rad(camera.fov) * 0.5)
			tan_x = tan_y * aspect
		else:
			tan_x = tan(deg_to_rad(camera.fov) * 0.5)
			tan_y = tan_x / aspect
		var basis := camera.global_transform.basis
		views.append({
			"position": to_local(camera.global_position),
			"forward": (inverse_basis * -basis.z).normalized(),
			"right": (inverse_basis * basis.x).normalized(),
			"up": (inverse_basis * basis.y).normalized(),
			"tan_half_fov_x": tan_x,
			"tan_half_fov_y": tan_y,
			"viewport_height": float(size.y),
			"near": camera.near,
			"far": camera.far,
		})
	return views


## Coalesce targets without cancelling an in-flight traversal. Completed plans
## remain useful; payload downloading yields to a changed view after a small
## batch, rather than delaying a camera switch behind the entire old cut.
func _poll(lat: float, lon: float, alt: float) -> void:
	_view_revision += 1
	var views := _camera_snapshots()
	_mutex.lock()
	if lat != _target_lat or lon != _target_lon or alt != _target_alt \
			or _anchor_generation != _target_generation or views != _target_views \
			or streaming_radius_km != _target_radius_km or far_radius_km != _target_far_radius_km \
			or maximum_screen_space_error != _target_maximum_sse:
		_target_change_revision = _view_revision
	_target_lat = lat
	_target_lon = lon
	_target_alt = alt
	_target_anchor_lat = _anchor_lat
	_target_anchor_lon = _anchor_lon
	_target_anchor_alt = _anchor_alt
	var target_ned := GWGeodeticConvert.geodetic_to_ned(lat, lon, alt, _anchor_lat, _anchor_lon, _anchor_alt)
	_target_local_center = GWCoordConvert.ned_to_world(target_ned)
	_target_radius_km = streaming_radius_km
	_target_far_radius_km = far_radius_km
	_target_maximum_sse = maximum_screen_space_error
	_target_generation = _anchor_generation
	_target_view_revision = _view_revision
	_target_views = views
	var known := PackedStringArray(_loaded_tiles.keys())
	for tile in _incoming_tiles:
		known.append(String(tile["id"]))
	for tile in _pending_results:
		known.append(String(tile["id"]))
	_known_ids = known
	if not _wake_posted:
		_wake_posted = true
		_wake_sem.post()
	_mutex.unlock()
	_time_since_poll = 0.0


func _reanchor(lat: float, lon: float, alt: float) -> void:
	_anchor_generation += 1
	_has_anchor = true
	_anchor_lat = lat
	_anchor_lon = lon
	_anchor_alt = alt
	var anchor_ned := GWGeodeticConvert.geodetic_to_ned(lat, lon, alt, home_lat, home_lon, home_alt)
	position = GWCoordConvert.ned_to_world(anchor_ned)
	for id in _loaded_tiles:
		var entry: Dictionary = _loaded_tiles[id]
		var xform := GWTiles3DTraversal.local_transform_to_godot(entry["ecef_transform"], lat, lon, alt)
		if _content_axis_correction == "google_yup_ecef":
			xform.basis = xform.basis * GWTiles3DContent.GOOGLE_YUP_ECEF_CORRECTION
		entry["wrapper"].transform = xform
		entry["bounds"] = GWTiles3DTraversal.bounding_sphere(
				entry.get("bounding_volume", {}), entry["ecef_transform"], lat, lon, alt)
	_poll(lat, lon, alt)


func _drain_results() -> void:
	var room := maxi(MAX_INCOMING_TILES - _incoming_tiles.size(), 0)
	_mutex.lock()
	var results := []
	while room > 0 and not _pending_results.is_empty():
		results.append(_pending_results.pop_front())
		room -= 1
	var plans := _pending_plans.values()
	_pending_plans.clear()
	var infos := _pending_info
	_pending_info = []
	_mutex.unlock()
	for info in infos:
		if info.has("error"):
			push_error(info["error"])
		elif info.has("attributions"):
			attributions = info["attributions"]
			_content_axis_correction = info.get("content_axis_correction", "")

	# Control never waits behind payloads. A completed traversal is useful even
	# when a newer camera snapshot was posted while its network requests ran.
	plans.sort_custom(func(a: Dictionary, b: Dictionary) -> bool:
		return _plan_serial(a) < _plan_serial(b))
	for plan in plans:
		var serial := _plan_serial(plan)
		if serial > _accepted_plan_serial:
			_accepted_plan_serial = serial
			_accepted_view_revision = int(plan["view_revision"])
			_apply_selection(plan)
	for event in results:
		if not _loaded_tiles.has(event["id"]) and not _incoming_has(String(event["id"])):
			_incoming_tiles.append(event)

	var budget := tiles_per_frame_budget
	while budget > 0 and not _incoming_tiles.is_empty():
		var tile: Dictionary = _incoming_tiles.pop_front()
		var id := String(tile["id"])
		if _loaded_tiles.has(id):
			continue
		if _accepted_view_revision >= 0 and not _tile_needed(id):
			continue
		if not _make_room_for_tile():
			_incoming_tiles.push_front(tile)
			break
		_place_new_tile(tile)
		budget -= 1
	_try_all_promotions()
	_evict_safe_cached_tiles()


func _incoming_has(id: String) -> bool:
	for tile in _incoming_tiles:
		if String(tile["id"]) == id:
			return true
	return false


func _tile_needed(id: String) -> bool:
	return _current_desired_ids.has(id) or _protected_tile_ids.has(id)


func _rebuild_protected_tile_ids() -> void:
	_protected_tile_ids.clear()
	for group_id in _replacement_groups:
		var group: Dictionary = _replacement_groups[group_id]
		if group.get("promoted", false) \
				or int(group.get("selected_revision", -1)) == _accepted_view_revision:
			for id in group["parent_ids"]:
				_protected_tile_ids[String(id)] = true
			for id in group["required_ids"]:
				_protected_tile_ids[String(id)] = true


func _make_room_for_tile() -> bool:
	if _loaded_tiles.size() < max_tiles_loaded:
		return true
	for id in _loaded_tiles.keys():
		if not _tile_needed(String(id)):
			_evict_tile(String(id))
			return true
	return false


func _place_new_tile(tile: Dictionary) -> void:
	var xform := GWTiles3DTraversal.local_transform_to_godot(tile["ecef_transform"], _anchor_lat, _anchor_lon, _anchor_alt)
	var wrapper := GWTiles3DContent.place_tile(self, tile["bytes"], xform, _content_axis_correction, String(tile["id"]).validate_node_name())
	if wrapper == null:
		push_warning("GWTiles3DStreamer: failed to parse/place tile %s" % tile["id"])
		return
	_accept_placed_tile(tile, wrapper)


## Kept separate from GLB parsing so transition behavior is deterministic in
## synthetic tests and a malformed payload cannot mutate fallback state.
func _accept_placed_tile(tile: Dictionary, wrapper: Node3D) -> void:
	var pending_parent := String(tile.get("pending_parent", ""))
	var bounding_volume: Dictionary = tile.get("bounding_volume", {}).duplicate(true)
	var bounds: Dictionary = tile.get("bounds", {"center": Vector3.ZERO, "radius": INF}).duplicate(true)
	if not bounding_volume.is_empty():
		bounds = GWTiles3DTraversal.bounding_sphere(
				bounding_volume, tile["ecef_transform"], _anchor_lat, _anchor_lon, _anchor_alt)
	wrapper.visible = pending_parent == ""
	var entry := {
		"wrapper": wrapper,
		"ecef_transform": tile["ecef_transform"],
		"bounds": bounds,
		"bounding_volume": bounding_volume,
		"pending_parent": pending_parent,
	}
	_loaded_tiles[tile["id"]] = entry
	_wrapper_entries[wrapper.get_instance_id()] = entry


func _register_transition(event: Dictionary) -> void:
	var parent_ids := PackedStringArray(event.get("parent_ids", []))
	if parent_ids.is_empty():
		return
	var group_id := String(parent_ids[0])
	var required_ids := PackedStringArray(event.get("required_ids", []))
	if required_ids.is_empty():
		return
	for required_id in required_ids:
		if String(required_id) in parent_ids:
			return
	if _replacement_groups.has(group_id):
		var existing: Dictionary = _replacement_groups[group_id]
		# Parent/child relations come from immutable tileset metadata. Never
		# reset a promoted cut merely because another view reports it again.
		if existing["parent_ids"] == parent_ids and existing["required_ids"] == required_ids:
			existing["selected_revision"] = _accepted_view_revision
		return
	_replacement_groups[group_id] = {
		"parent_ids": parent_ids,
		"required_ids": required_ids,
		"promoted": false,
		"selected_revision": _accepted_view_revision,
	}
	for parent_id in parent_ids:
		_parent_groups[String(parent_id)] = group_id


func _try_all_promotions() -> void:
	for group_id in _replacement_groups.keys():
		var group: Dictionary = _replacement_groups[group_id]
		if int(group.get("selected_revision", -1)) == _accepted_view_revision:
			_try_promote(String(group_id))


func _try_promote(group_id: String) -> bool:
	if not _replacement_groups.has(group_id):
		return false
	var group: Dictionary = _replacement_groups[group_id]
	var was_active := false
	for parent_id in group["parent_ids"]:
		var id := String(parent_id)
		if not _loaded_tiles.has(id):
			return false
		was_active = was_active or _loaded_tiles[id]["wrapper"].visible
	for child_id in group["required_ids"]:
		if not _coverage_ready(String(child_id)):
			return false
	group["promoted"] = true
	if was_active:
		for parent_id in group["parent_ids"]:
			_loaded_tiles[String(parent_id)]["wrapper"].visible = false
		for child_id in group["required_ids"]:
			_show_coverage(String(child_id))
	return true


func _show_coverage(id: String) -> void:
	if not _loaded_tiles.has(id):
		return
	var group_id := String(_parent_groups.get(id, ""))
	if group_id != "" and _replacement_groups.get(group_id, {}).get("promoted", false):
		for parent_id in _replacement_groups[group_id]["parent_ids"]:
			if _loaded_tiles.has(String(parent_id)):
				_loaded_tiles[String(parent_id)]["wrapper"].visible = false
		for child_id in _replacement_groups[group_id]["required_ids"]:
			_show_coverage(String(child_id))
	else:
		_loaded_tiles[id]["wrapper"].visible = true


func _hide_coverage(id: String) -> void:
	if _loaded_tiles.has(id):
		_loaded_tiles[id]["wrapper"].visible = false
	var group_id := String(_parent_groups.get(id, ""))
	if group_id != "" and _replacement_groups.has(group_id):
		for child_id in _replacement_groups[group_id]["required_ids"]:
			_hide_coverage(String(child_id))


func _coverage_ready(id: String) -> bool:
	if not _loaded_tiles.has(id):
		return false
	var group_id := String(_parent_groups.get(id, ""))
	if group_id != "" and _replacement_groups.get(group_id, {}).get("promoted", false):
		for child_id in _replacement_groups[group_id]["required_ids"]:
			if not _coverage_ready(String(child_id)):
				return false
	return true


func _has_active_coverage(id: String) -> bool:
	if not _loaded_tiles.has(id):
		return false
	if _loaded_tiles[id]["wrapper"].visible:
		return true
	var group_id := String(_parent_groups.get(id, ""))
	if group_id != "" and _replacement_groups.get(group_id, {}).get("promoted", false):
		for child_id in _replacement_groups[group_id]["required_ids"]:
			if not _has_active_coverage(String(child_id)):
				return false
		return true
	return false


func _apply_selection(event: Dictionary) -> void:
	if int(event.get("phase", 1)) == 0:
		# A new view can acquire coarse coverage while its detail is planned,
		# without coarsening or evicting the already visible replacement cut.
		_current_desired_ids.clear()
		_current_desired_ids.merge(_accepted_fine_desired_ids, true)
		for id in event["desired_ids"]:
			_current_desired_ids[String(id)] = true
		for group in _replacement_groups.values():
			if int(group.get("selected_revision", -1)) >= 0:
				group["selected_revision"] = _accepted_view_revision
		_try_all_promotions()
		_rebuild_protected_tile_ids()
		return
	_accepted_fine_desired_ids.clear()
	for id in event["desired_ids"]:
		_accepted_fine_desired_ids[String(id)] = true
	_current_desired_ids.clear()
	_current_desired_ids.merge(_accepted_fine_desired_ids, true)
	var selected_groups := {}
	for transition in event.get("transitions", []):
		var parent_ids := PackedStringArray(transition.get("parent_ids", []))
		if parent_ids.is_empty():
			continue
		var group_id := String(parent_ids[0])
		selected_groups[group_id] = true
		_register_transition(transition)

	# Zoom-out is atomic too: restore every item of the coarse parent before
	# hiding descendants. If a parent payload is absent, retain the old cut.
	for group_id in _replacement_groups.keys():
		var group: Dictionary = _replacement_groups[group_id]
		if selected_groups.has(group_id):
			group["selected_revision"] = _accepted_view_revision
			continue
		group["selected_revision"] = -1
		var parents_ready := true
		for parent_id in group["parent_ids"]:
			if not _loaded_tiles.has(String(parent_id)):
				parents_ready = false
				break
		if not parents_ready:
			if not group["promoted"]:
				for parent_id in group["parent_ids"]:
					_parent_groups.erase(String(parent_id))
				_replacement_groups.erase(group_id)
			continue
		var was_active := false
		for parent_id in group["parent_ids"]:
			was_active = was_active or _has_active_coverage(String(parent_id))
		group["promoted"] = false
		if was_active:
			for parent_id in group["parent_ids"]:
				_show_coverage(String(parent_id))
		for child_id in group["required_ids"]:
			_hide_coverage(String(child_id))

	_try_all_promotions()
	_rebuild_protected_tile_ids()


func _evict_safe_cached_tiles() -> void:
	for id in _loaded_tiles.keys():
		if not _tile_needed(String(id)):
			_evict_tile(String(id))


func _evict_tile(id: String) -> void:
	if not _loaded_tiles.has(id):
		return
	var wrapper: Node3D = _loaded_tiles[id]["wrapper"]
	_wrapper_entries.erase(wrapper.get_instance_id())
	wrapper.queue_free()
	_loaded_tiles.erase(id)
	var group_id := String(_parent_groups.get(id, ""))
	if group_id != "" and _replacement_groups.has(group_id):
		var group: Dictionary = _replacement_groups[group_id]
		for parent_id in group["parent_ids"]:
			_parent_groups.erase(String(parent_id))
		_replacement_groups.erase(group_id)


## Public collision filter. Unknown bounds conservatively return true. The
## wrapper's visibility remains the authoritative active-coverage signal.
func is_tile_near(tile: Node3D, world_position: Vector3, radius_m: float) -> bool:
	if not is_instance_valid(tile):
		return false
	var entry = _wrapper_entries.get(tile.get_instance_id())
	if entry == null:
		return true
	var bounds: Dictionary = entry.get("bounds", {"center": Vector3.ZERO, "radius": INF})
	if not is_finite(float(bounds.get("radius", INF))):
		return true
	var center_world := global_transform * (bounds["center"] as Vector3)
	return center_world.distance_to(world_position) <= radius_m + float(bounds["radius"])


# -----------------------------------------------------------------------------
# Thread: HTTP I/O and traversal only. Never touches Nodes or the scene tree.
# -----------------------------------------------------------------------------

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
	_post_info({"attributions": resolved["attributions"], "content_axis_correction": "google_yup_ecef" if _is_google else ""})
	var completed_revision := -1
	while _thread_running():
		_wake_sem.wait()
		if not _thread_running():
			break
		while _thread_running():
			_mutex.lock()
			var snapshot := {
				"lat": _target_lat, "lon": _target_lon, "alt": _target_alt,
				"anchor_lat": _target_anchor_lat,
				"anchor_lon": _target_anchor_lon,
				"anchor_alt": _target_anchor_alt,
				"local_center": _target_local_center,
				"local_radius": _target_radius_km * 1000.0,
				"far_radius": _target_far_radius_km * 1000.0,
				"maximum_sse": _target_maximum_sse,
				"generation": _target_generation,
				"view_revision": _target_view_revision,
				"views": _target_views.duplicate(true),
				"known_ids": PackedStringArray(_known_ids),
			}
			_wake_posted = false
			_mutex.unlock()
			if int(snapshot["view_revision"]) == completed_revision:
				break
			_run_selection(snapshot)
			completed_revision = int(snapshot["view_revision"])
			_mutex.lock()
			var newer := _target_view_revision != completed_revision
			_mutex.unlock()
			if not newer:
				break


func _thread_running() -> bool:
	_mutex.lock()
	var result := _running
	_mutex.unlock()
	return result


func _post_result(event: Dictionary) -> bool:
	_mutex.lock()
	if _pending_results.size() >= MAX_PENDING_RESULTS:
		_mutex.unlock()
		return false
	_pending_results.append(event)
	_mutex.unlock()
	return true


func _post_plan(plan: Dictionary) -> void:
	_mutex.lock()
	_pending_plans[_plan_serial(plan)] = plan
	while _pending_plans.size() > 4:
		var revisions := _pending_plans.keys()
		revisions.sort()
		_pending_plans.erase(revisions[0])
	_mutex.unlock()


static func _plan_serial(plan: Dictionary) -> int:
	return int(plan["view_revision"]) * 2 + int(plan.get("phase", 0))


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
## _walk_tileset_live's use of this: the top-level tileset root endpoint (unlike
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


func _run_selection(snapshot: Dictionary) -> void:
	_selection_budget_exhausted = false
	var desired := {}
	var active_documents := {}
	var transitions := []
	# Admit the complete coarse frontier before spending slots on detail.
	# Planning never queues GLB payloads, so a full payload queue cannot block
	# the control decision that releases obsolete residency.
	var coarse := _walk_tileset_live(_tileset_root_url, Transform3D.IDENTITY, snapshot,
			desired, active_documents, "", max_tiles_loaded, false, false, transitions)
	if not coarse["ok"] or not coarse["complete"] or not _thread_running():
		return
	var sent := {}
	for id in snapshot["known_ids"]:
		sent[String(id)] = true
	_publish_selection(snapshot, desired, transitions, 0)
	_download_selection(desired, snapshot, sent)
	active_documents.clear()
	var detailed := _walk_tileset_live(_tileset_root_url, Transform3D.IDENTITY, snapshot,
			desired, active_documents, "", max_tiles_loaded, true, false, transitions)
	if detailed["ok"] and detailed["complete"] and _thread_running():
		_publish_selection(snapshot, desired, transitions, 1)
		_download_selection(desired, snapshot, sent)


func _publish_selection(snapshot: Dictionary, desired: Dictionary,
		transitions: Array, phase: int) -> void:
	_post_plan({
		"kind": "plan", "phase": phase,
		"desired_ids": PackedStringArray(desired.keys()),
		"transitions": transitions.duplicate(true),
		"generation": snapshot["generation"],
		"view_revision": snapshot["view_revision"],
	})


func _download_selection(desired: Dictionary, snapshot: Dictionary, sent: Dictionary) -> void:
	var attempted := 0
	for id in desired:
		if sent.has(id):
			continue
		# Yield at backpressure; the admitted plan is already independently
		# available to the main thread. A later snapshot resumes missing data.
		_mutex.lock()
		var full := _pending_results.size() >= MAX_PENDING_RESULTS
		var superseded := _target_change_revision > int(snapshot["view_revision"])
		_mutex.unlock()
		if full or (superseded and attempted >= 4) or not _thread_running():
			return
		var request: Dictionary = desired[id]
		if _fetch_content(request["url"], id, request["transform"],
				request["bounding_volume"], request["bounds"],
				request["pending_parent"], snapshot):
			sent[id] = true
		attempted += 1


func _walk_tileset_live(tileset_url: String, base_transform: Transform3D, snapshot: Dictionary,
		desired: Dictionary, active_documents: Dictionary, pending_parent: String, budget: int,
		refine_enabled: bool, force_coverage: bool, transitions: Array) -> Dictionary:
	var document_uri := _canonical_content_uri(tileset_url)
	var instance_key := document_uri + "@" + _transform_key(base_transform)
	if active_documents.has(instance_key):
		return {"ok": false, "complete": false, "coverage": PackedStringArray()}
	active_documents[instance_key] = true
	var is_root := document_uri == _canonical_content_uri(_tileset_root_url)
	# Each document can mint its own session. Cache by the effective request,
	# and restore the parent's context after descending into an external set.
	var request_url := _apply_auth(tileset_url, not is_root)
	var cache_key := request_url.sha256_text()
	var doc = _document_cache.get(cache_key)
	if doc == null:
		var resp := _http_get(request_url)
		if not resp["ok"]:
			if int(resp.get("status", 0)) in [400, 401, 403]:
				_document_cache.clear()
			active_documents.erase(instance_key)
			return {"ok": false, "complete": false, "coverage": PackedStringArray()}
		doc = JSON.parse_string((resp["body"] as PackedByteArray).get_string_from_utf8())
		if not (doc is Dictionary) or not doc.has("root"):
			active_documents.erase(instance_key)
			return {"ok": false, "complete": false, "coverage": PackedStringArray()}
	var parent_auth := _auth_params
	var token := _find_session_token(doc["root"])
	if token != "":
		_auth_params = _ensure_param(_auth_params, "session=%s" % token)
	if not _document_cache.has(cache_key):
		if _document_cache.size() >= MAX_TILESET_DOCUMENTS:
			_document_cache.erase(_document_cache.keys()[0])
		_document_cache[cache_key] = doc
	var result := _walk_tile_live(doc["root"], base_transform, tileset_url, snapshot, desired,
			active_documents, pending_parent, budget, "REPLACE", refine_enabled,
			force_coverage, transitions)
	_auth_params = parent_auth
	active_documents.erase(instance_key)
	return result


func _walk_tile_live(tile: Dictionary, parent_transform: Transform3D, base_url: String,
		snapshot: Dictionary, desired: Dictionary, active_documents: Dictionary,
		pending_parent: String, budget: int, inherited_refine: String,
		refine_enabled: bool, force_coverage: bool, transitions: Array) -> Dictionary:
	if not _thread_running():
		return {"ok": false, "complete": false, "coverage": PackedStringArray()}
	var transform := parent_transform
	if tile.has("transform"):
		transform = parent_transform * GWTiles3DTraversal.parse_gltf_transform(tile["transform"])
	var bounds := GWTiles3DTraversal.bounding_sphere(tile.get("boundingVolume", {}), transform,
			snapshot["anchor_lat"], snapshot["anchor_lon"], snapshot["anchor_alt"])
	var local_distance := maxf((bounds["center"] as Vector3).distance_to(
			snapshot["local_center"]) - float(bounds["radius"]), 0.0)
	var local_needed := local_distance <= float(snapshot["local_radius"])
	var visible := false
	for view in snapshot["views"]:
		var camera_distance := maxf((bounds["center"] as Vector3).distance_to(
				view["position"]) - float(bounds["radius"]), 0.0)
		if camera_distance <= float(snapshot["far_radius"]) \
				and GWTiles3DTraversal.bounds_visible(bounds, view):
			visible = true
			break
	if not force_coverage and not local_needed and not visible:
		return {"ok": true, "complete": true, "coverage": PackedStringArray()}

	var refine := String(tile.get("refine", inherited_refine)).to_upper()
	if refine != "ADD":
		refine = "REPLACE"
	var contents := _tile_contents(tile)
	var own_ids := PackedStringArray()
	for index in contents.size():
		var content: Dictionary = contents[index]
		var uri := String(content.get("uri", content.get("url", "")))
		if uri == "":
			continue
		var content_url := _resolve_relative_url(base_url, uri)
		if _canonical_content_uri(content_url).split("?", true, 1)[0].ends_with(".json"):
			var nested := _walk_tileset_live(content_url, transform, snapshot, desired,
					active_documents, pending_parent, budget, refine_enabled,
					force_coverage, transitions)
			if not nested["ok"] or not nested["complete"]:
				return nested
			own_ids.append_array(nested["coverage"])
			continue
		var id := _stable_tile_id(content_url, transform, index)
		if not desired.has(id) and desired.size() >= budget:
			_selection_budget_exhausted = true
			return {"ok": true, "complete": false, "coverage": own_ids}
		desired[id] = {
			"url": _apply_auth(content_url), "transform": transform,
			"bounding_volume": tile.get("boundingVolume", {}),
			"bounds": bounds, "pending_parent": pending_parent,
		}
		own_ids.append(id)

	var children: Array = tile.get("children", [])
	var sse := GWTiles3DTraversal.screen_space_error(
			float(tile.get("geometricError", 0.0)), bounds, snapshot["views"])
	var should_refine := refine_enabled and not _selection_budget_exhausted and visible \
			and sse > float(snapshot["maximum_sse"]) and not children.is_empty()
	if not should_refine:
		if not own_ids.is_empty():
			return {"ok": true, "complete": true, "coverage": own_ids}
		if children.is_empty():
			return {"ok": true, "complete": true, "coverage": PackedStringArray()}
		# A contentless intermediary is not geographical coverage; descend to
		# the first real frontier even when this branch is only a coarse sibling.

	var group_id := String(own_ids[0]) if not own_ids.is_empty() else pending_parent
	var child_pending := group_id if refine == "REPLACE" and group_id != "" else pending_parent
	var child_force := force_coverage or (refine == "REPLACE" and not own_ids.is_empty())
	var child_coverage := PackedStringArray()
	var child_start := desired.size()
	var transition_start := transitions.size()
	# First obtain every sibling's coarse coverage. Only then may any visible
	# sibling consume the remaining budget with deeper refinement.
	for child in children:
		var result := _walk_tile_live(child, transform, base_url, snapshot, desired,
				active_documents, child_pending, budget, refine, false,
				child_force, transitions)
		if not result["ok"] or not result["complete"]:
			if not own_ids.is_empty():
				_rollback_refinement(desired, child_start, transitions, transition_start)
				return {"ok": true, "complete": true, "coverage": own_ids}
			return {"ok": result["ok"], "complete": false, "coverage": child_coverage}
		child_coverage.append_array(result["coverage"])
	if should_refine:
		# Spend a bounded detail budget on the largest projected errors first.
		var order := range(children.size())
		var priorities := PackedFloat64Array()
		priorities.resize(children.size())
		for index in order:
			var child: Dictionary = children[index]
			var child_transform := transform
			if child.has("transform"):
				child_transform = transform * GWTiles3DTraversal.parse_gltf_transform(child["transform"])
			var child_bounds := GWTiles3DTraversal.bounding_sphere(
					child.get("boundingVolume", {}), child_transform,
					snapshot["anchor_lat"], snapshot["anchor_lon"], snapshot["anchor_alt"])
			priorities[index] = GWTiles3DTraversal.screen_space_error(
					float(child.get("geometricError", 0.0)), child_bounds, snapshot["views"])
		order.sort_custom(func(a: int, b: int) -> bool: return priorities[a] > priorities[b])
		for index in order:
			if _selection_budget_exhausted:
				break
			var child: Dictionary = children[index]
			var refinement_start := desired.size()
			var refinement_transitions := transitions.size()
			var result := _walk_tile_live(child, transform, base_url, snapshot, desired,
					active_documents, child_pending, budget, refine, true,
					child_force, transitions)
			if not result["ok"] or not result["complete"]:
				_rollback_refinement(desired, refinement_start, transitions, refinement_transitions)

	if refine == "REPLACE" and not own_ids.is_empty() and not child_coverage.is_empty():
		transitions.append({
			"parent_ids": own_ids,
			"required_ids": child_coverage,
		})
		# Ancestors replace this tile as one geographical unit, not with its
		# currently visible descendants. This keeps relations view-independent.
		return {"ok": true, "complete": true, "coverage": own_ids}
	if refine == "ADD":
		own_ids.append_array(child_coverage)
		return {"ok": true, "complete": true, "coverage": own_ids}
	return {"ok": true, "complete": true, "coverage": child_coverage}


static func _rollback_refinement(desired: Dictionary, keep_count: int,
		transitions: Array, keep_transitions: int) -> void:
	var ids := desired.keys()
	for index in range(keep_count, ids.size()):
		desired.erase(ids[index])
	transitions.resize(keep_transitions)


func _tile_contents(tile: Dictionary) -> Array:
	if tile.has("contents") and tile["contents"] is Array:
		return tile["contents"]
	if tile.has("content") and tile["content"] is Dictionary:
		return [tile["content"]]
	return []


func _fetch_content(content_url: String, id: String, transform: Transform3D,
		bounding_volume: Dictionary, bounds: Dictionary, pending_parent: String,
		snapshot: Dictionary) -> bool:
	if not _thread_running():
		return false
	var resp := _http_get(_apply_auth(content_url))
	if not resp["ok"]:
		if int(resp.get("status", 0)) in [400, 401, 403]:
			_document_cache.clear()
		return false
	var data := GWTiles3DTraversal.unwrap_b3dm(resp["body"])
	if data.size() < 4 or data.slice(0, 4).get_string_from_ascii() != "glTF":
		return false
	return _post_result({
		"kind": "tile", "id": id, "bytes": data, "ecef_transform": transform,
		"bounds": bounds.duplicate(true),
		"bounding_volume": bounding_volume.duplicate(true),
		"pending_parent": pending_parent,
		"generation": snapshot["generation"], "view_revision": snapshot["view_revision"],
	})


## Identity is anchor-independent and includes the canonical source content
## URI plus its complete tileset-instance transform. Ephemeral credentials are
## stripped before the URI enters scene state, logs, node names, or tests.
static func _stable_tile_id(content_uri: String, transform: Transform3D,
		content_index: int = 0) -> String:
	var identity := _canonical_content_uri(content_uri) + "|" \
			+ var_to_bytes(transform).hex_encode() + "|" + str(content_index)
	return "tile:" + identity.sha256_text()


static func _transform_key(transform: Transform3D) -> String:
	return var_to_bytes(transform).hex_encode()


static func _canonical_content_uri(uri: String) -> String:
	var pieces := uri.split("?", true, 1)
	if pieces.size() == 1:
		return uri
	var safe_params := PackedStringArray()
	for pair in String(pieces[1]).split("&"):
		var name := String(pair).split("=", true, 1)[0].to_lower()
		if name not in ["access_token", "key", "session", "token"]:
			safe_params.append(pair)
	return String(pieces[0]) + (("?" + "&".join(safe_params)) if not safe_params.is_empty() else "")


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
		return {"ok": false, "error": "could not parse content URL"}
	var key: String = "%s:%d" % [parsed["host"], parsed["port"]]

	var client: HTTPClient = _http_clients.get(key)
	if client == null:
		client = HTTPClient.new()
		var err := client.connect_to_host(parsed["host"], parsed["port"], TLSOptions.client())
		if err != OK:
			return {"ok": false, "error": "connect_to_host failed: %d" % err}
		while (client.get_status() == HTTPClient.STATUS_CONNECTING or client.get_status() == HTTPClient.STATUS_RESOLVING) and _thread_running():
			client.poll()
			OS.delay_msec(1)
		if not _thread_running():
			return {"ok": false, "error": "shutdown"}
		if client.get_status() != HTTPClient.STATUS_CONNECTED:
			return {"ok": false, "error": "connection failed, status %d" % client.get_status()}
		_http_clients[key] = client

	var err := client.request(HTTPClient.METHOD_GET, parsed["path"], PackedStringArray(["User-Agent: GodotWings-Tiles3DStreamer"]))
	if err != OK:
		_http_clients.erase(key)
		if not _retried:
			return _http_get(url, true)  # a reused connection can go stale server-side -- retry once, fresh
		return {"ok": false, "error": "request() failed: %d" % err}
	while client.get_status() == HTTPClient.STATUS_REQUESTING and _thread_running():
		client.poll()
		OS.delay_msec(1)
	if not _thread_running():
		return {"ok": false, "error": "shutdown"}

	var status := client.get_status()
	if status != HTTPClient.STATUS_BODY and status != HTTPClient.STATUS_CONNECTED:
		_http_clients.erase(key)
		if not _retried:
			return _http_get(url, true)
		return {"ok": false, "error": "bad response status %d" % status}
	var code := client.get_response_code()
	var body := PackedByteArray()
	while client.get_status() == HTTPClient.STATUS_BODY and _thread_running():
		client.poll()
		var chunk := client.read_response_body_chunk()
		if chunk.size() == 0:
			OS.delay_msec(1)
		else:
			body.append_array(chunk)
	if not _thread_running():
		return {"ok": false, "error": "shutdown"}
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
