class_name GWGeoReference
extends Node

## Anchors a Cesium 3D Tiles globe to GodotWings' local flight frame, re-anchoring
## periodically so render precision and terrain LOD stay centred on the aircraft.
##
## GodotWings integrates in a flat NED tangent plane at the SITL home; 3D Tiles live
## on the WGS84 ellipsoid behind a CesiumGeoreference origin. This node:
##   1. converts the aircraft's NED-from-home to geodetic (flat tangent approx),
##   2. drives the CesiumGeoreference origin so terrain renders under the aircraft,
##   3. re-anchors once the aircraft drifts `reanchor_distance` from the scene origin
##      — moving the georef origin onto the aircraft AND rebasing the aircraft's
##      `render_origin` by the same amount, so render coords stay small and the
##      ellipsoid tangent stays near the aircraft.
##
## It OWNS `render_origin`, so do NOT also use GWFloatingOrigin. Set `home_*` to match
## the SITL HOME_LOCATION. Limitation: the FDM stays a flat plane fixed at home, so
## attitude-vs-true-gravity drifts over very long range — fine for local test flights.

const R_EARTH := 6378137.0  # WGS84 semi-major axis (m)

## Geodetic home — MUST match docker HOME_LOCATION (lat, lon, alt).
@export var home_lat: float = -35.363261
@export var home_lon: float = 149.165230
@export var home_alt: float = 584.0
## Vehicle to track. Empty = auto-find the first GWVehicleBody in the scene.
@export var aircraft_path: NodePath
## CesiumGeoreference node to drive. Empty = auto-find one in the scene.
@export var georeference_path: NodePath
## Node3D that PARENTS the tangent-space content (aircraft, view camera, catapult).
## GodotWings renders in a Y-up NED tangent frame, but the Cesium engine frame is
## rotated ECEF — so this node's transform is set to the rotation that maps the
## tangent frame onto the globe at the anchor point. Empty = no anchor (only
## correct if the engine frame already happens to be the tangent frame).
@export var anchor_path: NodePath
## Re-anchor once the aircraft is this far (m) from the current scene origin.
@export var reanchor_distance: float = 5000.0
## Drive the tileset LOD/streaming from the view camera each frame. The addon
## normally needs a CesiumDynamicCamera for this; we drive it from the active camera
## so GWViewCamera stays the single camera. Without this, no tiles ever load.
@export var drive_tile_streaming: bool = true
## Camera whose view drives streaming. Empty = the active (current) Camera3D.
@export var streaming_camera_path: NodePath
## Give every Camera3D in the scene the globe-scale near/far on startup. In
## CartographicOrigin mode the default far (~4000 m) clips the terrain and horizon,
## and a tiny near with a huge far degenerates the light-culler frustum. Disable
## only if you manage camera planes yourself.
@export var apply_camera_planes: bool = true
## Near plane (m) for globe rendering. Cesium's own cameras use 9.
@export var camera_near: float = 9.0
## Far plane (m) for globe rendering (Cesium CartographicOrigin default).
@export var camera_far: float = 35358652.0

var _aircraft: GWVehicleBody
var _geo: Node
var _tilesets: Array = []   # Cesium3DTileset nodes the camera must update each frame
var _stream_cam: Camera3D
var _anchor: Node3D         # parents the tangent content; we drive its transform
var _view_cam: GWViewCamera # view camera we feed the globe frame basis to
var _frame_basis := Basis.IDENTITY  # tangent->engine rotation at the current anchor
var _anchor_ned := Vector3.ZERO  # current georef origin, in NED metres from home


func _ready() -> void:
	_aircraft = get_node_or_null(aircraft_path) as GWVehicleBody if not aircraft_path.is_empty() \
			else _find_vehicle(_scene_root())
	_geo = get_node_or_null(georeference_path) if not georeference_path.is_empty() \
			else _find_named_or_class(_scene_root(), "CesiumGeoreference")
	if _aircraft == null:
		push_warning("GWGeoReference: no GWVehicleBody found to anchor.")
		set_physics_process(false)
		return
	if _geo == null:
		push_warning("GWGeoReference: no CesiumGeoreference node found — set georeference_path.")
	_anchor = get_node_or_null(anchor_path) as Node3D
	_view_cam = _find_view_camera(_scene_root())
	_warn_if_floating_origin()
	# Defer the first anchor/streaming setup until every node (the held aircraft, its
	# onboard camera) has run its own _ready — otherwise our _sync_node would clobber
	# the vehicle's captured launch pose before it is captured.
	set_physics_process(false)
	call_deferred("_initialize")


func _initialize() -> void:
	_reanchor(Vector3.ZERO)  # start anchored at home
	if drive_tile_streaming and _geo != null:
		_collect_tilesets(_geo)
		_stream_cam = get_node_or_null(streaming_camera_path) as Camera3D
	if apply_camera_planes:
		_apply_camera_planes(_scene_root())
	set_physics_process(true)


func _physics_process(_delta: float) -> void:
	if not _aircraft.is_inside_tree():
		return
	# Re-anchor when the rendered aircraft has wandered too far from the origin.
	if _aircraft.global_position.length() > reanchor_distance:
		_reanchor(_aircraft._pos_ned)


## Drive each tileset's LOD from the camera every render frame (what a
## CesiumDynamicCamera would otherwise do). Without this the tileset never streams.
func _process(_delta: float) -> void:
	if _tilesets.is_empty():
		return
	var cam := _stream_cam
	if cam == null and is_inside_tree():
		cam = get_viewport().get_camera_3d()
	if cam == null or not cam.is_inside_tree():
		return
	if _geo == null or not _geo.is_inside_tree() or not _geo.has_method("get_tx_engine_to_ecef"):
		return
	# The tileset wants the camera pose in ECEF, not engine space — same transform
	# AbstractCesiumCamera uses. Without the engine->ECEF multiply, LOD is computed
	# at the wrong place and tiles never stream correctly.
	var to_ecef: Transform3D = _geo.call("get_tx_engine_to_ecef")
	var xf := to_ecef * cam.global_transform
	for t in _tilesets:
		if is_instance_valid(t):
			t.call("update_tileset", xf)
	# Keep the view camera's globe frame current (covers a camera created after us).
	if cam is GWViewCamera:
		(cam as GWViewCamera).frame_basis = _frame_basis


## Give every Camera3D in the scene the globe-scale near/far so terrain and the
## horizon don't clip and the light-culler frustum stays well-conditioned.
func _apply_camera_planes(n: Node) -> void:
	if n is Camera3D:
		(n as Camera3D).near = camera_near
		(n as Camera3D).far = camera_far
	for c in n.get_children():
		_apply_camera_planes(c)


## Collect Cesium3DTileset descendants (any node exposing update_tileset).
func _collect_tilesets(n: Node) -> void:
	if n.has_method("update_tileset"):
		_tilesets.append(n)
	for c in n.get_children():
		_collect_tilesets(c)


## Move the georef origin (and the aircraft's render rebase) onto `anchor_ned` (NED
## metres from home), keeping the aircraft near the scene origin.
func _reanchor(anchor_ned: Vector3) -> void:
	_anchor_ned = anchor_ned
	_aircraft.render_origin = GWCoordConvert.ned_to_world(anchor_ned)
	var geo := ned_to_geodetic(anchor_ned)
	_apply_cesium_origin(geo.x, geo.y, geo.z)
	_update_globe_frame(geo)   # orient the tangent frame onto the globe before re-render
	_aircraft._sync_node()     # re-render now, no one-frame lag


## Compute the rotation M that maps GodotWings' Y-up NED tangent frame (the space
## ned_to_world produces) onto the Cesium engine frame at the anchor, and apply it
## to the anchor node (which parents the aircraft) and the view camera. The Cesium
## engine frame is rotated ECEF, so without M the aircraft renders buried/tilted.
##   M = (ECEF->engine rotation) * (East/Up/South basis at the anchor, in ECEF)
func _update_globe_frame(geo_deg: Vector3) -> void:
	if _geo == null or not _geo.is_inside_tree() \
			or not _geo.has_method("get_tx_ecef_to_engine") or not _geo.has_method("eus_at_ecef"):
		return
	var ecef: Vector3 = _geo.call("lat_lon_alt_rad_to_ecef",
			Vector3(deg_to_rad(geo_deg.x), deg_to_rad(geo_deg.y), geo_deg.z))
	var eus: Basis = _geo.call("eus_at_ecef", ecef)
	var ecef_to_engine: Transform3D = _geo.call("get_tx_ecef_to_engine")
	_frame_basis = (ecef_to_engine.basis * eus).orthonormalized()
	if _anchor != null:
		_anchor.transform = Transform3D(_frame_basis, Vector3.ZERO)
	if _view_cam != null:
		_view_cam.frame_basis = _frame_basis


## Flat-tangent NED-from-home -> geodetic Vector3(lat_deg, lon_deg, alt_m). Good for
## the re-anchor cadence; Cesium computes the exact ellipsoid from the geodetic origin.
func ned_to_geodetic(n: Vector3) -> Vector3:
	var lat := home_lat + rad_to_deg(n.x / R_EARTH)
	var lon := home_lon + rad_to_deg(n.y / (R_EARTH * cos(deg_to_rad(home_lat))))
	var alt := home_alt - n.z
	return Vector3(lat, lon, alt)


## Drive the CesiumGeoreference (3D-Tiles-For-Godot) cartographic origin. Uses the
## addon's set_latitude/set_longitude/set_altitude API and forces CartographicOrigin
## mode so lat/lon/alt anchor the globe at our home/anchor point.
func _apply_cesium_origin(lat: float, lon: float, alt: float) -> void:
	if _geo == null:
		return
	if not _geo.has_method("set_latitude"):
		push_warning("GWGeoReference: node at georeference_path is not a CesiumGeoreference.")
		return
	if _geo.has_method("set_origin_type"):
		_geo.call("set_origin_type", 0)  # 0 = CartographicOrigin (lat/lon/alt drive it)
	_geo.call("set_latitude", lat)
	_geo.call("set_longitude", lon)
	_geo.call("set_altitude", alt)


# --- scene helpers -----------------------------------------------------------
func _scene_root() -> Node:
	var n: Node = self
	var top := get_tree().root if get_tree() else null
	while n.get_parent() != null and n.get_parent() != top:
		n = n.get_parent()
	return n


func _find_vehicle(n: Node) -> GWVehicleBody:
	if n is GWVehicleBody:
		return n
	for c in n.get_children():
		var f := _find_vehicle(c)
		if f != null:
			return f
	return null


func _find_view_camera(n: Node) -> GWViewCamera:
	if n is GWViewCamera:
		return n
	for c in n.get_children():
		var f := _find_view_camera(c)
		if f != null:
			return f
	return null


func _find_named_or_class(n: Node, cname: String) -> Node:
	if n.name == cname or n.is_class(cname):
		return n
	for c in n.get_children():
		var f := _find_named_or_class(c, cname)
		if f != null:
			return f
	return null


func _warn_if_floating_origin() -> void:
	var fo := _find_floating_origin(_scene_root())
	if fo != null:
		push_warning("GWGeoReference: a GWFloatingOrigin is also present — both rebase " +
				"render_origin and will fight. Disabling the GWFloatingOrigin.")
		fo.set_physics_process(false)


func _find_floating_origin(n: Node) -> GWFloatingOrigin:
	if n is GWFloatingOrigin:
		return n
	for c in n.get_children():
		var f := _find_floating_origin(c)
		if f != null:
			return f
	return null
