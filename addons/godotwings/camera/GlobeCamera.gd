@icon("res://addons/godotwings/sensors/camera_icon.svg")
class_name GWGlobeCamera
extends AbstractCesiumCamera

## Chase/orbit spectator camera for the Cesium globe. It extends the addon's
## AbstractCesiumCamera, so it inherits the PROVEN tile streaming/LOD path
## (_update_tilesets) and the globe-aligned atmosphere, instead of GWViewCamera's
## flat-world assumptions. Use it INSTEAD of GWViewCamera in globe scenes:
## set `globe_node`, `tilesets`, point `target_path` at the aircraft, and turn on
## `render_atmosphere` for a sky that actually matches the horizon.
##
## The Cesium engine frame is rotated ECEF (local up is NOT world +Y), so this
## camera derives "up" from the planet centre each frame and keeps the horizon
## level — the same reason GWViewCamera needs a frame basis on the globe.

enum Mode { CHASE, ORBIT }

@export var target_path: NodePath
@export var mode := Mode.CHASE

@export_group("Rig")
## Distance from the target (m), adjustable with the mouse wheel.
@export var distance := 18.0
@export var min_distance := 4.0
@export var max_distance := 4000.0
## Elevation above the local horizon (deg).
@export var pitch_deg := 18.0
@export var chase_smoothing := 5.0

@export_group("Input")
@export var enable_input := true
@export var cycle_key := KEY_C
@export var orbit_button := MOUSE_BUTTON_RIGHT
@export var orbit_sensitivity := 0.006
@export var zoom_step := 2.0

# RADII (WGS84 semi-major) is inherited from AbstractCesiumCamera.

var _target: Node3D
var _yaw := 0.0
var _pitch := 0.0
var _orbiting := false


func _ready() -> void:
	if globe_node == null:
		push_warning("GWGlobeCamera: globe_node not set — assign the CesiumGeoreference.")
	else:
		super._ready()  # near/far for the origin type + atmosphere load
	_pitch = deg_to_rad(pitch_deg)
	_target = _resolve_target()
	current = true


func _resolve_target() -> Node3D:
	if not target_path.is_empty():
		return get_node_or_null(target_path) as Node3D
	var root := get_tree().current_scene if get_tree() else null
	return _find_flight_body(root) if root else null


func _find_flight_body(n: Node) -> Node:
	if n is GWVehicleBody:
		return n
	for c in n.get_children():
		var f := _find_flight_body(c)
		if f != null:
			return f
	return null


func _unhandled_input(event: InputEvent) -> void:
	if not enable_input:
		return
	if event is InputEventKey and event.pressed and not event.echo and event.keycode == cycle_key:
		mode = (mode + 1) % Mode.size()
	elif event is InputEventMouseButton:
		if event.button_index == orbit_button:
			_orbiting = event.pressed
		elif event.button_index == MOUSE_BUTTON_WHEEL_UP:
			distance = clampf(distance - zoom_step, min_distance, max_distance)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			distance = clampf(distance + zoom_step, min_distance, max_distance)
	elif event is InputEventMouseMotion and _orbiting:
		_yaw -= event.relative.x * orbit_sensitivity
		_pitch = clampf(_pitch - event.relative.y * orbit_sensitivity, deg_to_rad(-5.0), deg_to_rad(85.0))


func _process(delta: float) -> void:
	if _target == null:
		_target = _resolve_target()
		return
	if globe_node == null:
		return
	_follow(delta)
	super._process(delta)  # update tile LOD/streaming from the new camera pose


## Position the camera behind/above the target relative to the LOCAL horizon. "Up"
## is the direction from the planet centre; "behind" is the target's heading
## projected onto the local tangent plane (so the camera trails the nose).
func _follow(delta: float) -> void:
	var up := _local_up()
	var t := _target.global_position
	# Heading reference projected onto the tangent plane.
	var ref := -_target.global_basis.z
	var fwd := ref - up * ref.dot(up)
	if fwd.length() < 0.01:
		fwd = -global_basis.z - up * (-global_basis.z).dot(up)
	fwd = fwd.normalized().rotated(up, _yaw)
	var offset := (-fwd * cos(_pitch) + up * sin(_pitch)) * distance
	var desired := t + offset
	if mode == Mode.CHASE:
		global_position = global_position.lerp(desired, clampf(chase_smoothing * delta, 0.0, 1.0))
	else:
		global_position = desired
	look_at(t, up)
	# Altitude above the origin tangent (origin sits on the surface) for the atmosphere.
	last_hit_distance = maxf(1.0, global_position.dot(up))


## True local up in ENGINE space. get_global_center_position() returns (0,0,0) in
## CartographicOrigin (not the planet centre), so derive up from the georeference's
## ECEF origin: the radial-out there, mapped back into engine space.
func _local_up() -> Vector3:
	var ecef0 := Vector3(globe_node.ecefX, globe_node.ecefY, globe_node.ecefZ)
	if ecef0.length() < 1.0:
		return Vector3.UP
	var to_engine: Transform3D = globe_node.get_tx_ecef_to_engine()
	return (to_engine.basis * ecef0.normalized()).normalized()
