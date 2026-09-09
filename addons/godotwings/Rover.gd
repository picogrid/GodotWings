@tool
@icon("res://addons/godotwings/sensors/camera_icon.svg")
## Drop-in ground vehicle: the ArduRover counterpart of GWAircraft / GWMulticopter.
## One node that self-assembles a working SITL rover — wheeled dynamics
## (GWRoverConfig) + UDP bridge + visual model + optional gimbal camera. It IS a
## GWRoverBody, so all the dynamics exports are here too. Run the SITL side with
## VEHICLE=Rover.
##
## MODEL STANDARD — a glTF (or any scene) the sim can read the vehicle from.
## Name the nodes and the rover learns its wheel count, positions, radii and
## hull from the model, and animates the wheels (steer, spin, suspension):
##
##   RoverModel                 root (any name)
##   ├── Hull                   mesh(es) of the body — prefix "Hull"; its bounding
##   │                          box becomes the collision hull + inertia box
##   ├── CG                     optional empty node: centre of mass. Without it the
##   │                          CG is the hull box centre (else the model origin)
##   ├── Wheel_FL               one node per wheel — prefix "Wheel" (any suffix,
##   ├── Wheel_FR               any count ≥ 3, may sit inside a group node). Node
##   ├── Wheel_RL               origin = hub centre. The wheel mesh (the node
##   └── Wheel_RR               itself or its children) is a disc: its thinnest
##                              axis is the axle, half its diameter the radius.
##
## Authoring frame: standard glTF (+Y up, −Z forward) or Onshape/CAD Z-up via
## `model_orientation`. The wheels furthest forward are the steered axle, the
## furthest back the rear axle (see GWRoverConfig steer_front / steer_rear /
## drive_layout). `geometry_from_model = false` keeps the model purely visual
## and uses the config's wheelbase/track/radius instead.
class_name GWRover
extends GWRoverBody

## glTF authoring frames we know how to rotate into Godot's -Z-forward / +Y-up.
enum ModelOrientation {
	GLTF_Y_UP,     ## Standard glTF / Godot: nose -Z, up +Y. No rotation.
	ONSHAPE_Z_UP,  ## Onshape/CAD export: Z up, nose +Y. Rotate -90 deg about X.
}

@export_group("Visual model")
## Leave empty for a placeholder built from the config geometry (box hull + 4
## cylinder wheels). Drop your own glTF here, named per the model standard above.
@export var model_scene: PackedScene:
	set(v):
		model_scene = v
		_refresh_model_preview()
@export var model_orientation: ModelOrientation = ModelOrientation.GLTF_Y_UP:
	set(v):
		model_orientation = v
		_refresh_model_preview()
## Extra yaw (deg) about the up axis — flip to 180 if the model renders tail-first.
@export var model_yaw_offset_deg: float = 0.0:
	set(v):
		model_yaw_offset_deg = v
		_refresh_model_preview()
## Uniform scale applied to the model. Also scales the geometry read from it.
@export var model_scale: float = 1.0:
	set(v):
		model_scale = v
		_refresh_model_preview()
## Read wheels (count, positions, radii), hull box and CG from the model's nodes
## per the standard above. Off = the config's geometry drives the physics and
## the model is only drawn (its wheel nodes are still animated if present).
@export var geometry_from_model: bool = true

@export_group("SITL")
## Vehicle index for multi-vehicle setups: bridge listens on 9002 + 10*instance.
@export var sitl_instance: int = 0

@export_group("Streaming camera")
@export var enable_camera: bool = false
@export var camera_protocol: GWCamera.Protocol = GWCamera.Protocol.RTP_H264
@export var camera_resolution: Vector2i = Vector2i(1280, 720)
@export var camera_fps: float = 30.0
## Camera mount relative to the body (CG). Default: on top, looking forward.
@export var camera_mount: Transform3D = Transform3D(Basis(), Vector3(0, 0.2, -0.4))
## Off hands the raw-frame TCP server to your own pipeline instead of ffmpeg —
## e.g. `tools/gw_klv_muxer.py` for STANAG4609/KLV FMV output (see the README's
## "STANAG 4609 / MISB ST 0601 KLV metadata" section). `camera_protocol` /
## `rtsp_url` etc. are then unused; only `resolution`/`fps` still apply.
@export var camera_launch_ffmpeg: bool = true
## Make the camera an ArduPilot servo gimbal (reads mount servo PWM). See GWCamera.
@export var camera_gimbal: bool = false:
	set(v):
		camera_gimbal = v
		notify_property_list_changed()
## SITL channels (1-16) carrying mount pitch/yaw/roll servo PWM; 0 = axis fixed.
@export_range(0, 16) var gimbal_pitch_channel: int = 0
@export_range(0, 16) var gimbal_yaw_channel: int = 0
@export_range(0, 16) var gimbal_roll_channel: int = 0
## Angle (deg) each PWM range maps to (x at min PWM, y at max); match MNT1_*_MIN/MAX.
@export var gimbal_pitch_range_deg: Vector2 = Vector2(-90.0, 0.0)
@export var gimbal_yaw_range_deg: Vector2 = Vector2(-180.0, 180.0)
@export var gimbal_roll_range_deg: Vector2 = Vector2(-30.0, 30.0)
## Servo PWM endpoints the angle ranges span (SERVOn_MIN..MAX).
@export var gimbal_pwm_min: int = 1000
@export var gimbal_pwm_max: int = 2000

## Set when the model supplied the geometry (for tooling / tests).
var geometry_source := "config"


func _ready() -> void:
	if Engine.is_editor_hint():
		set_physics_process(false)
		_instance_model()   # visual preview only; bridge/camera are runtime-only
		return
	if control_source == ControlSource.MANUAL:
		_ensure_manual_input()  # ←/→ steer, ↑/↓ throttle (manual_drive); no SITL
	else:
		_ensure_bridge(sitl_instance)
	_instance_model()
	_bind_model_geometry()
	if enable_camera:
		_ensure_camera(_camera_opts())
	super._ready()


func _physics_process(delta: float) -> void:
	super._physics_process(delta)
	_animate_wheels()


# --- Visual model ----------------------------------------------------------------

func _instance_model() -> void:
	if get_node_or_null("Model") != null:
		return
	if config == null and ResourceLoader.exists(DEFAULT_CONFIG_PATH):
		config = load(DEFAULT_CONFIG_PATH)
	var wrapper := Node3D.new()
	wrapper.name = "Model"
	var basis := Basis()
	if model_scene != null:
		basis = _orientation_basis(model_orientation)
	if not is_zero_approx(model_yaw_offset_deg):
		basis = Basis(Vector3.UP, deg_to_rad(model_yaw_offset_deg)) * basis
	if not is_equal_approx(model_scale, 1.0):
		basis = basis.scaled(Vector3.ONE * model_scale)
	wrapper.transform = Transform3D(basis, Vector3.ZERO)
	if model_scene != null:
		wrapper.add_child(model_scene.instantiate())
	elif config != null:
		wrapper.add_child(build_placeholder_model(config))
	add_child(wrapper)


## Rebuild the visual preview in the editor when a model export changes.
func _refresh_model_preview() -> void:
	if not Engine.is_editor_hint() or not is_inside_tree():
		return
	var existing := get_node_or_null("Model")
	if existing != null:
		existing.free()
	_instance_model()


func _orientation_basis(orient: ModelOrientation) -> Basis:
	match orient:
		ModelOrientation.ONSHAPE_Z_UP:
			return Basis.from_euler(Vector3(-PI / 2.0, 0.0, 0.0))
		_:
			return Basis()


## Placeholder rover following the model standard, sized from the config: a box
## hull + 4 cylinder wheels + a CG marker. Because it obeys the standard, the same
## scan that reads a user glTF reads this — one code path.
static func build_placeholder_model(cfg: GWRoverConfig) -> Node3D:
	var root := Node3D.new()
	root.name = "RoverModel"
	var dark := StandardMaterial3D.new()
	dark.albedo_color = Color(0.22, 0.24, 0.27)
	var tyre := StandardMaterial3D.new()
	tyre.albedo_color = Color(0.08, 0.08, 0.09)
	var accent := StandardMaterial3D.new()
	accent.albedo_color = Color(0.85, 0.45, 0.1)

	var hull := MeshInstance3D.new()
	hull.name = "Hull"
	var box := BoxMesh.new()
	box.size = Vector3(cfg.body_size.y, cfg.body_size.z, cfg.body_size.x)   # render: W, H, L
	hull.mesh = box
	hull.material_override = dark
	hull.position = Vector3(0.0, cfg.body_center_height - cfg.cg_height, 0.0)
	root.add_child(hull)

	# A nose marker so the driving direction is obvious (not part of the hull box).
	var nose := MeshInstance3D.new()
	nose.name = "Nose"
	var nbox := BoxMesh.new()
	nbox.size = Vector3(cfg.body_size.y * 0.6, cfg.body_size.z * 0.5, cfg.body_size.x * 0.15)
	nose.mesh = nbox
	nose.material_override = accent
	nose.position = hull.position + Vector3(0.0, cfg.body_size.z * 0.5, -cfg.body_size.x * 0.42)
	root.add_child(nose)

	var cg := Node3D.new()
	cg.name = "CG"
	root.add_child(cg)

	for w in GWRoverBody.build_wheels_from_config(cfg):
		var wheel := MeshInstance3D.new()
		wheel.name = "Wheel_" + w.name
		var cyl := CylinderMesh.new()
		cyl.top_radius = w.radius
		cyl.bottom_radius = w.radius
		cyl.height = w.width
		wheel.mesh = cyl
		wheel.material_override = tyre
		# Cylinder axis is local +Y; lay it along the vehicle's X (the axle).
		wheel.transform = Transform3D(Basis(Vector3.BACK, PI / 2.0), _frd_to_render(w.hub_rest))
		root.add_child(wheel)
	return root


static func _frd_to_render(v: Vector3) -> Vector3:
	return Vector3(v.y, -v.z, -v.x)   # right→+X, down→-Y, fwd→-Z


static func _render_to_frd(v: Vector3) -> Vector3:
	return Vector3(-v.z, v.x, -v.y)


# --- Model standard: read geometry, bind wheel nodes -----------------------------

## Scan the instanced model for Hull / CG / Wheel* nodes. With
## geometry_from_model, the wheel layout, hull box and CG drive the physics
## (and the model is shifted so its CG sits on the body origin). Either way the
## wheel nodes are bound for animation when they match the physics wheels.
func _bind_model_geometry() -> void:
	var wrapper := get_node_or_null("Model") as Node3D
	if wrapper == null:
		return
	var scan := scan_model(wrapper)
	var found: Array = scan["wheels"]
	if geometry_from_model:
		if found.size() >= 3:
			var cg: Vector3 = scan["cg"]
			wrapper.position -= cg   # CG onto the body origin
			for w in found:
				w.hub_rest -= _render_to_frd(cg)
			wheels.assign(found)
			if scan["hull_size"] != Vector3.ZERO:
				body_size_override = scan["hull_size"]
			geometry_source = "model"
			return
		if model_scene != null:
			push_warning("GWRover: model has %d 'Wheel*' nodes (need ≥ 3); using config geometry." % found.size())
	# Config geometry drives the physics; still animate matching wheel nodes.
	if wheels.is_empty():
		wheels = build_wheels_from_config(config)
	for w in wheels:
		for f in found:
			if f.hub_rest.distance_to(w.hub_rest) < 0.05 or f.name.to_lower().ends_with(w.name.to_lower()):
				w.node = f.node; w.node_rest = f.node_rest; w.axle_local = f.axle_local
				w.up_in_parent = f.up_in_parent; w.parent_scale = f.parent_scale
				break


## Read a model per the standard. Returns {wheels: Array[GWRoverWheel] (hub_rest in
## body FRD, before any CG shift), hull_size: Vector3 (L, W, H; zero if no Hull),
## cg: Vector3 (render frame, relative to the body origin)}.
static func scan_model(wrapper: Node3D) -> Dictionary:
	var wheels_out: Array = []
	var hull_aabb := AABB()
	var hull_found := false
	var cg_node: Node3D = null
	var stack: Array = [wrapper]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		var lname := n.name.to_lower()
		if n is Node3D and n != wrapper:
			if lname.begins_with("wheel") and not _has_wheel_descendant(n):
				var w := _wheel_from_node(n, wrapper)
				if w != null:
					wheels_out.append(w)
				continue   # a wheel's children are its own meshes
			if lname.begins_with("hull"):
				var xf := _to_body(n, wrapper)
				var ab := _subtree_aabb(n)
				if ab.size != Vector3.ZERO:
					var world_ab := xf * ab
					hull_aabb = world_ab if not hull_found else hull_aabb.merge(world_ab)
					hull_found = true
			if lname == "cg" or lname == "centerofmass" or lname == "centre_of_mass" or lname == "center_of_mass":
				cg_node = n
		for c in n.get_children():
			stack.append(c)
	var cg := Vector3.ZERO
	if cg_node != null:
		cg = _to_body(cg_node, wrapper).origin
	elif hull_found:
		cg = hull_aabb.get_center()
	var hull_size := Vector3.ZERO
	if hull_found:
		hull_size = Vector3(hull_aabb.size.z, hull_aabb.size.x, hull_aabb.size.y)   # L, W, H
	return {"wheels": wheels_out, "hull_size": hull_size, "cg": cg}


## True if any node below `n` is itself named like a wheel — then `n` is a group
## (e.g. "Wheels"), not a wheel.
static func _has_wheel_descendant(n: Node) -> bool:
	for c in n.get_children():
		if c.name.to_lower().begins_with("wheel") or _has_wheel_descendant(c):
			return true
	return false


## Transform from `n`'s local frame to the body frame (through the Model wrapper).
static func _to_body(n: Node3D, wrapper: Node3D) -> Transform3D:
	var xf := Transform3D.IDENTITY
	var cur: Node = n
	while cur != null and cur != wrapper:
		if cur is Node3D:
			xf = (cur as Node3D).transform * xf
		cur = cur.get_parent()
	return wrapper.transform * xf


## Merged AABB of every mesh in `n`'s subtree, in `n`'s local frame.
static func _subtree_aabb(n: Node3D) -> AABB:
	var out := AABB()
	var found := false
	var stack: Array = [[n, Transform3D.IDENTITY]]
	while not stack.is_empty():
		var item: Array = stack.pop_back()
		var node: Node = item[0]
		var xf: Transform3D = item[1]
		if node is MeshInstance3D and (node as MeshInstance3D).mesh != null:
			var ab: AABB = xf * (node as MeshInstance3D).mesh.get_aabb()
			out = ab if not found else out.merge(ab)
			found = true
		for c in node.get_children():
			if c is Node3D:
				stack.append([c, xf * (c as Node3D).transform])
	return out if found else AABB()


## A GWRoverWheel from a wheel node: hub at the node origin, axle = the thinnest
## axis of its mesh bounding box (pointing to the vehicle's right), radius = half
## the largest extent across it.
static func _wheel_from_node(n: Node3D, wrapper: Node3D) -> GWRoverWheel:
	var ab := _subtree_aabb(n)
	if ab.size == Vector3.ZERO:
		push_warning("GWRover: wheel node '%s' has no mesh; skipped." % n.name)
		return null
	var to_body := _to_body(n, wrapper)
	var scale := to_body.basis.get_scale()
	var ext := ab.size * scale   # extents in metres per local axis
	var axle_idx := 0
	if ext.y < ext[axle_idx]: axle_idx = 1
	if ext.z < ext[axle_idx]: axle_idx = 2
	var others: Array = [0, 1, 2]
	others.erase(axle_idx)
	var w := GWRoverWheel.new()
	w.name = n.name
	w.radius = 0.5 * maxf(ext[others[0]], ext[others[1]])
	w.width = ext[axle_idx]
	var axle := Vector3.ZERO
	axle[axle_idx] = 1.0
	if (to_body.basis * axle).dot(Vector3.RIGHT) < 0.0:
		axle = -axle   # point to the vehicle's right so spin sign is consistent
	w.axle_local = axle
	w.hub_rest = _render_to_frd(to_body.origin)
	w.node = n
	w.node_rest = n.transform
	var parent_to_body := to_body * n.transform.affine_inverse()
	w.up_in_parent = (parent_to_body.basis.inverse() * Vector3.UP).normalized()
	w.parent_scale = maxf(parent_to_body.basis.get_scale().y, 1e-4)
	return w


## Pose each bound wheel node from its physics state: steer about the vehicle's
## up axis, roll about its axle, and drop/lift with the suspension.
func _animate_wheels() -> void:
	for w in wheels:
		if w.node == null or not is_instance_valid(w.node):
			continue
		var basis := Basis(w.up_in_parent, -w.steer_angle) * w.node_rest.basis \
				* Basis(w.axle_local, -w.spin_angle)
		var origin := w.node_rest.origin - w.up_in_parent * (w.droop / w.parent_scale)
		w.node.transform = Transform3D(basis, origin)


# --- Camera ----------------------------------------------------------------------

## Build the camera/gimbal options dict for GWVehicleBody._ensure_camera.
func _camera_opts() -> Dictionary:
	return {
		"protocol": camera_protocol, "resolution": camera_resolution, "fps": camera_fps,
		"mount": camera_mount, "instance": sitl_instance, "gimbal": camera_gimbal,
		"launch_ffmpeg": camera_launch_ffmpeg,
		"gimbal_pitch_channel": gimbal_pitch_channel, "gimbal_yaw_channel": gimbal_yaw_channel,
		"gimbal_roll_channel": gimbal_roll_channel, "gimbal_pitch_range_deg": gimbal_pitch_range_deg,
		"gimbal_yaw_range_deg": gimbal_yaw_range_deg, "gimbal_roll_range_deg": gimbal_roll_range_deg,
		"gimbal_pwm_min": gimbal_pwm_min, "gimbal_pwm_max": gimbal_pwm_max,
	}


# Gimbal exports are only meaningful when camera_gimbal is on.
const _GIMBAL_PROPS := [
	"gimbal_pitch_channel", "gimbal_yaw_channel", "gimbal_roll_channel",
	"gimbal_pitch_range_deg", "gimbal_yaw_range_deg", "gimbal_roll_range_deg",
	"gimbal_pwm_min", "gimbal_pwm_max",
]


func _validate_property(property: Dictionary) -> void:
	if not camera_gimbal and property.name in _GIMBAL_PROPS:
		property.usage &= ~PROPERTY_USAGE_EDITOR
