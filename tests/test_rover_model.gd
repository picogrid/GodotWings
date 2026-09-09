extends SceneTree

# GWRover facade: reads a vehicle from a model that follows the naming standard
# (Hull / CG / Wheel*) — wheel count, hub positions, radii, hull box, CG — and
# animates the wheel nodes; falls back to the config's placeholder geometry when
# no model is given; and holds still on a slope in the real physics world.

var _ok := true


func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _initialize() -> void:
	await _test_placeholder()
	await _test_model_standard()
	await _test_slope_hold()
	await _test_buried_by_streamed_terrain()
	print("test_rover_model: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)


func _flat_ground() -> StaticBody3D:
	var ground := StaticBody3D.new()
	ground.collision_layer = 1
	var cs := CollisionShape3D.new()
	cs.shape = WorldBoundaryShape3D.new()
	ground.add_child(cs)
	return ground


# --- no model: the placeholder obeys the standard, so it round-trips the config --
func _test_placeholder() -> void:
	print("[placeholder]")
	var r := GWRover.new()
	r.control_source = GWVehicleBody.ControlSource.MANUAL
	get_root().add_child(r)
	await physics_frame
	var cfg := r.config
	_check(r.geometry_source == "model", "placeholder geometry read through the model scan")
	_check(r.wheels.size() == 4, "4 wheels")
	var fl: GWRoverWheel = null
	for w in r.wheels:
		if w.name.ends_with("FL"):
			fl = w
	_check(fl != null, "wheel named Wheel_FL found")
	if fl != null:
		_check(absf(fl.radius - cfg.wheel_radius) < 1e-3, "radius from mesh = config (%.3f)" % fl.radius)
		_check(absf(fl.width - cfg.wheel_width) < 1e-3, "width from mesh = config (%.3f)" % fl.width)
		var want := Vector3(cfg.wheelbase * 0.5, -cfg.track * 0.5, cfg.cg_height - cfg.wheel_radius)
		_check(fl.hub_rest.distance_to(want) < 1e-3, "FL hub matches config geometry %s" % str(fl.hub_rest))
		_check(fl.node != null and fl.steers, "FL bound to its node and steers")
	var bs := r.body_size_override
	_check(bs.distance_to(cfg.body_size) < 1e-3, "hull box from Hull mesh = config body_size %s" % str(bs))
	_check(absf(r.spawn_altitude - cfg.cg_height) < 1e-3, "spawn height = CG height (%.2f)" % r.spawn_altitude)
	var model := r.get_node("Model") as Node3D
	_check(model.position.is_zero_approx(), "CG marker at origin: model not shifted")
	# Animation: steer + spin + droop move the wheel node as expected.
	if fl != null:
		var rest := fl.node_rest
		fl.steer_angle = 0.3
		fl.spin_angle = 1.0
		fl.droop = 0.05
		r._animate_wheels()
		var xf: Transform3D = fl.node.transform
		_check(absf((xf.origin - rest.origin).y + 0.05) < 1e-4, "droop lowers the wheel node")
		var axle_world := xf.basis * fl.axle_local
		var expect_axle := Basis(Vector3.UP, -0.3) * (rest.basis * fl.axle_local)
		_check(axle_world.distance_to(expect_axle) < 1e-4, "steer rotates the axle about vehicle up (toward the right)")
		_check(expect_axle.z > 0.0, "right steer: the axle's right end swings back (nose right)")
	r.queue_free()
	await physics_frame


# --- a hand-built scene in the standard, as a glTF import would produce ---------
func _test_model_standard() -> void:
	print("[model standard]")
	var root := Node3D.new()
	root.name = "SixWheeler"
	var hull := MeshInstance3D.new()
	hull.name = "Hull"
	var box := BoxMesh.new()
	box.size = Vector3(0.5, 0.2, 1.0)   # W, H, L (render)
	hull.mesh = box
	hull.position = Vector3(0.0, 0.3, 0.0)
	root.add_child(hull)
	var cg := Node3D.new()
	cg.name = "CG"
	cg.position = Vector3(0.0, 0.25, 0.0)
	root.add_child(cg)
	var group := Node3D.new()
	group.name = "Wheels"
	group.position = Vector3(0.0, 0.12, 0.0)   # hubs 0.12 up: wheels touch y=0
	root.add_child(group)
	for side in [-1.0, 1.0]:
		for i in 3:
			var wheel := MeshInstance3D.new()
			wheel.name = "Wheel_%s%d" % ["L" if side < 0 else "R", i]
			var cyl := CylinderMesh.new()
			cyl.top_radius = 0.12
			cyl.bottom_radius = 0.12
			cyl.height = 0.06
			wheel.mesh = cyl
			# Axle along X: the cylinder's Y axis rotated 90 deg about Z.
			wheel.transform = Transform3D(Basis(Vector3.BACK, PI / 2.0), Vector3(0.3 * side, 0.0, -0.4 + 0.4 * i))
			group.add_child(wheel)
	var scene := PackedScene.new()
	for n in [hull, cg, group]:
		n.owner = root
	for c in group.get_children():
		c.owner = root
	_check(scene.pack(root) == OK, "packed the test model")
	root.free()

	var r := GWRover.new()
	r.model_scene = scene
	r.control_source = GWVehicleBody.ControlSource.MANUAL
	r.config = load("res://addons/godotwings/aircraft/Rover4WD.tres").duplicate()
	r.config.drive_layout = GWRoverConfig.DriveLayout.REAR
	get_root().add_child(r)
	await physics_frame
	_check(r.geometry_source == "model", "geometry taken from the model")
	_check(r.wheels.size() == 6, "6 wheels found (%d)" % r.wheels.size())
	var model := r.get_node("Model") as Node3D
	_check(absf(model.position.y + 0.25) < 1e-4, "model shifted so CG sits on the body origin (%.3f)" % model.position.y)
	var front_left: GWRoverWheel = null
	var mid_right: GWRoverWheel = null
	var rear_left: GWRoverWheel = null
	for w in r.wheels:
		match w.name:
			"Wheel_L0": front_left = w
			"Wheel_R1": mid_right = w
			"Wheel_L2": rear_left = w
	_check(front_left != null and mid_right != null and rear_left != null, "wheels named per node")
	if front_left != null and mid_right != null and rear_left != null:
		_check(front_left.hub_rest.distance_to(Vector3(0.4, -0.3, 0.13)) < 1e-3,
				"front-left hub in FRD relative to CG %s" % str(front_left.hub_rest))
		_check(absf(front_left.radius - 0.12) < 1e-3 and absf(front_left.width - 0.06) < 1e-3,
				"radius/width from the disc mesh (%.3f / %.3f)" % [front_left.radius, front_left.width])
		_check(front_left.steers and not mid_right.steers and not rear_left.steers, "only the front axle steers")
		_check(rear_left.driven and not front_left.driven and not mid_right.driven, "REAR layout drives only the rear axle")
		_check(front_left.rear == false and rear_left.rear == true, "rear axle flagged")
		_check(absf(front_left.static_load + mid_right.static_load + rear_left.static_load - r.config.mass * GWVehicleBody.G * 0.5) < 0.5,
				"static load split across the six wheels")
	_check(r.body_size_override.distance_to(Vector3(1.0, 0.5, 0.2)) < 1e-3,
			"hull box (L, W, H) from Hull mesh %s" % str(r.body_size_override))
	_check(absf(r.spawn_altitude - 0.25) < 1e-3, "spawn height = CG above the wheel bottoms (%.3f)" % r.spawn_altitude)
	# Drive it in the tree for a moment: physics runs, wheel nodes animate.
	var pwm := PackedInt32Array()
	pwm.resize(16)
	pwm.fill(1500)
	pwm[1] = 2000
	r._update_controls(pwm)
	var manual := r.get_node("GWManualInput") as GWManualInput
	manual.enabled = false   # drive the FDM ourselves; keep the source quiet
	for _i in 100:
		r._step(0.02)
	r._animate_wheels()
	_check(r.ground_speed() > 0.5, "six-wheeler drives (%.2f m/s)" % r.ground_speed())
	if rear_left != null:
		_check(absf(rear_left.spin_angle) > 1.0, "wheel node spun (%.1f rad)" % rear_left.spin_angle)
		var axle_now: Vector3 = rear_left.node.transform.basis * rear_left.axle_local
		var axle_rest: Vector3 = rear_left.node_rest.basis * rear_left.axle_local
		_check(axle_now.distance_to(axle_rest) < 1e-4, "spin keeps the axle fixed (rolls about it)")
	r.queue_free()
	await physics_frame


# --- real physics world: a 12 deg slope, terrain following via wheel rays ----------
func _test_slope_hold() -> void:
	print("[slope hold]")
	var slope := StaticBody3D.new()
	slope.collision_layer = 1
	var cs := CollisionShape3D.new()
	cs.shape = WorldBoundaryShape3D.new()
	slope.add_child(cs)
	# Tilt the plane about the east axis: uphill toward north (-Z in Godot).
	slope.transform = Transform3D(Basis(Vector3.RIGHT, deg_to_rad(12.0)), Vector3.ZERO)
	get_root().add_child(slope)
	await physics_frame
	await physics_frame

	var r := GWRover.new()
	r.control_source = GWVehicleBody.ControlSource.MANUAL
	r.terrain_following = true
	r.ground_collision_mask = 1
	r.crash_mode = GWVehicleBody.CrashMode.SIMPLE
	get_root().add_child(r)
	await physics_frame
	(r.get_node("GWManualInput") as GWManualInput).enabled = false
	var pwm := PackedInt32Array()
	pwm.resize(16)
	pwm.fill(1500)
	r._update_controls(pwm)
	for _i in 250:
		r._step(0.02)
	var att := GWCoordConvert.dcm_to_ned_attitude(r._dcm)
	var contacts := 0
	for w in r.wheels:
		if w.in_contact:
			contacts += 1
	_check(contacts == 4, "all wheels find the tilted ground (%d)" % contacts)
	_check(absf(att[1] - deg_to_rad(12.0)) < deg_to_rad(2.0), "body pitches to the slope (%.1f deg)" % rad_to_deg(att[1]))
	_check(r._vel_ned.length() < 0.05, "neutral brake holds it on a 12 deg slope (v=%.3f m/s)" % r._vel_ned.length())
	_check(not r._crashed, "not crashed")
	var start := r._pos_ned
	pwm[1] = 2000   # throttle up the hill
	r._update_controls(pwm)
	for _i in 150:
		r._step(0.02)
	_check(r._pos_ned.x - start.x > 1.0 and r._pos_ned.z < start.z - 0.2,
			"climbs the slope under power (dN=%.1f m, dUp=%.2f m)" % [r._pos_ned.x - start.x, start.z - r._pos_ned.z])
	r.queue_free(); slope.queue_free()
	await physics_frame


# --- terrain streams in ABOVE a parked rover (tile arriving after spawn) -----------
func _test_buried_by_streamed_terrain() -> void:
	print("[buried by streamed terrain]")
	var ground := _flat_ground()
	get_root().add_child(ground)
	await physics_frame
	await physics_frame
	var r := GWRover.new()
	r.control_source = GWVehicleBody.ControlSource.MANUAL
	r.terrain_following = true
	r.ground_collision_mask = 1
	r.crash_mode = GWVehicleBody.CrashMode.SIMPLE
	get_root().add_child(r)
	await physics_frame
	(r.get_node("GWManualInput") as GWManualInput).enabled = false
	var pwm := PackedInt32Array()
	pwm.resize(16)
	pwm.fill(1500)
	r._update_controls(pwm)
	for _i in 50:
		r._step(0.02)
	var before := -r._pos_ned.z
	# A "tile" surface 6 m above the flat plane appears over the vehicle.
	var tile := StaticBody3D.new()
	tile.collision_layer = 1
	var cs := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(40.0, 0.5, 40.0)
	cs.shape = box
	tile.add_child(cs)
	tile.position = Vector3(0.0, 6.0, 0.0)
	get_root().add_child(tile)
	await physics_frame
	await physics_frame
	for _i in 100:
		r._step(0.02)
	var after := -r._pos_ned.z
	_check(after > before + 5.0 and absf(after - (6.25 + r.spawn_altitude)) < 0.1,
			"lifted onto the surface that appeared above it (%.2f -> %.2f m)" % [before, after])
	var contacts := 0
	for w in r.wheels:
		if w.in_contact:
			contacts += 1
	_check(contacts == 4 and not r._crashed, "all wheels on the new surface, no crash")
	r.queue_free(); tile.queue_free(); ground.queue_free()
	await physics_frame
