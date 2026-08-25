## Shared SITL vehicle body — the ArduPilot JSON lockstep driver plus all the
## vehicle-agnostic machinery: fixed-step sub-stepping, terrain/ground probing,
## obstacle + air-to-air collision, crash/ragdoll + recovery, wind, floating-origin
## rendering, and IMU + state reporting.
##
## Concrete vehicles extend this and implement the dynamics hooks:
##   _setup_vehicle()  — build models from config; return false if misconfigured
##   _reset_dynamics() — reset actuator/motor lag on spawn/reset
##   _integrate(h)     — advance one non-crashed sub-step (forces, moments, ground)
##   _vehicle_mass()   — mass (kg) for the ragdoll proxy
##   _default_hull_size() — collision-hull size when `hull_size` is 0
## See GWFlightBody (fixed-wing) and GWMultirotorBody (quad).
##
## Dynamics integrate natively in NED / body-FRD (the frames ArduPilot's JSON
## protocol uses), so the state packet needs no conversion; Godot's transform is
## updated for rendering only, via GWCoordConvert.
class_name GWVehicleBody
extends Node3D

## Emitted each SITL frame with all 16 control channels normalised to 0..1
## (PWM 1000..2000 µs). Channels 1-4 drive the vehicle; 5-16 are free to react to
## (e.g. an RC/aux switch). See GWChannelSwitch for a drop-in.
signal controls_received(channels: PackedFloat32Array)
## Lifecycle events so a host game/tool can react without polling.
signal took_off
signal landed
signal crashed
signal recovered

## How a confirmed crash plays out (selectable per vehicle).
enum CrashMode {
	SIMPLE,   ## scripted decelerate-and-settle: deterministic, no physics body
	RAGDOLL,  ## hand the wreck to the physics engine for a real tumble
}

## Where the FDM's control commands come from.
enum ControlSource {
	SITL,    ## ArduPilot JSON lockstep over UDP (GWSITLBridge) — the default
	MANUAL,  ## direct keyboard / joypad / RC input (GWManualInput), no autopilot
}

const G := 9.81
# Ground-contact classification: above any of these at touchdown = a crash.
const CRASH_SINK_RATE := 3.0   ## m/s descent
const CRASH_BANK := 0.6109     ## roll at contact (~35 deg)
const CRASH_PITCH := 0.5236    ## pitch at contact (~30 deg)
const CRASH_STOP_TAU := 0.4    ## time constant (s) for the wreck to decelerate to rest
const CRASH_RECOVER_DELAY := 2.0 ## seconds at rest before auto-recovering
const RAGDOLL_MAX_TIME := 10.0   ## hard cap: recover even if the wreck never fully rests
const CRASH_SLOPE := 0.5236    ## terrain steeper than this at contact (~30 deg) = a crash
const GROUND_PROBE_UP := 1000.0   ## terrain ray starts this far above the vehicle (m)
const GROUND_PROBE_DOWN := 8000.0 ## ...and reaches this far below (m)

## Command source: SITL (ArduPilot lockstep) or MANUAL (direct keyboard/joypad/RC).
## In MANUAL the SITL bridge is bypassed and a GWManualInput drives the FDM in real
## time; if none is found at `manual_input_path` one is auto-created.
@export var control_source: ControlSource = ControlSource.SITL
@export var bridge_path: NodePath = ^"../GWSITLBridge"
## GWManualInput node used when `control_source` is MANUAL.
@export var manual_input_path: NodePath = ^"GWManualInput"
## Spawn altitude above the ground (m); also the resting height on the gear.
@export var spawn_altitude: float = 0.5
## Spawn position North/East (m) and initial heading (rad NED: 0=N, +toward E).
@export var spawn_north: float = 0.0
@export var spawn_east: float = 0.0
@export var spawn_heading: float = 0.0
## Fixed internal integration step (s). The FDM sub-steps each frame down to this
## size, so flight is consistent at any host physics rate (no 50 Hz requirement).
@export var max_substep: float = 0.004
## After a crash, once the wreck settles, re-level it on the ground so the
## autopilot's estimator can realign and you can re-arm. Disable to stay wrecked.
@export var recover_after_crash: bool = true
## On recovery, respawn at the home/spawn point rather than where the wreck settled
## (so a crash on a steep hillside doesn't leave it embedded). False = in place.
@export var respawn_at_home: bool = true
## Freeze the vehicle at its editor placement (position + attitude, e.g. sitting on a
## catapult rail pitched up) until launch() is called — see examples/Catapult.gd. The
## FDM is suspended and the static pose is reported to ArduPilot; controls are ignored
## until release. Overrides spawn_north/east/heading while held.
@export var hold_until_launch: bool = false

@export_group("Ground / terrain")
## Raycast downward against world colliders so the gear/roll-out/crash logic
## follows 3D terrain instead of a flat sea-level plane. Off = flat ground at
## `ground_level` (no physics queries).
@export var terrain_following: bool = false
## Physics layers the ground probe tests. Put terrain on its own layer so the
## probe ignores obstacles and other vehicles.
@export_flags_3d_physics var ground_collision_mask: int = 1
## Flat-ground MSL used when terrain_following is off, or when the probe misses.
@export var ground_level: float = 0.0

@export_group("Atmosphere")
## Wind source. Empty = auto-find the first GWWind in the scene, so dropping one
## GWWind in the world drives every vehicle. No GWWind = still air.
@export var wind_path: NodePath

@export_group("Crash & collision")
## Physics layers to test the hull against each frame (static obstacles, scenery).
## 0 = no obstacle collision. Hitting one is always a crash. For vehicle-vs-vehicle
## use `aircraft_layer` instead.
@export_flags_3d_physics var obstacle_mask: int = 0
## Air-to-air: the physics layer this hull occupies AND tests against. Set the SAME
## non-zero layer on every vehicle and they collide with each other. 0 = off. Use a
## layer not shared with terrain/obstacles.
@export_flags_3d_physics var aircraft_layer: int = 0
## How a confirmed crash (hard ground impact or obstacle hit) is played out:
## RAGDOLL hands the wreck to the physics engine; SIMPLE runs a scripted
## decelerate-and-settle. Both recover once the wreck settles (recover_after_crash).
@export var crash_mode: CrashMode = CrashMode.RAGDOLL
## Collision-hull half-extents (m, render frame). Zero = auto-size per vehicle.
@export var hull_size: Vector3 = Vector3.ZERO
## Bounciness of the ragdoll wreck against the ground/obstacles (0..1).
@export_range(0.0, 1.0) var ragdoll_bounce: float = 0.2

var _bridge: GWSITLBridge
var _source                           # active command source (bridge or manual input)
var _atmos: GWAtmosphereModel
var _cmd := {}                        # latest command dict from the source

# --- 6-DOF state (NED / FRD) -------------------------------------------------
var _pos_ned := Vector3.ZERO   # position, NED, m
var _vel_ned := Vector3.ZERO   # linear velocity, NED, m/s
var _dcm := Basis()            # body->NED DCM (columns: fwd, right, down)
var _omega := Vector3.ZERO     # body angular velocity (p, q, r) FRD, rad/s
var _sim_time := 0.0
var _airspeed := 0.0
var _on_ground := true
var _crashed := false
var _crash_settle := 0.0  # seconds the wreck has been at rest (for auto-recovery)
var _held := false                 # frozen on a launcher until launch() (hold_until_launch)
var _hold_pos_ned := Vector3.ZERO  # editor-placement pose captured for the held spawn
var _hold_dcm := Basis()
var _accel_body := Vector3(0, 0, -G)  # IMU specific force, FRD, m/s^2
var _ground_down := 0.0               # terrain surface under vehicle, NED down (m)
var _ground_normal := Vector3(0, 0, -1)  # terrain surface normal, NED (up = -z)
var _hull_shape: BoxShape3D           # collision hull (obstacle test + ragdoll)
var _hull_body: StaticBody3D          # passive hull presence on aircraft_layer; else null
var _wind: GWWind                     # resolved wind source (null = still air)
var _ragdoll: RigidBody3D             # physics-simulated wreck while ragdolling (else null)
var _ragdolling := false
var _ragdoll_settle := 0.0            # seconds the wreck has been near rest
var _ragdoll_time := 0.0              # total seconds spent ragdolling (max-time fallback)
var _prev_vel_ned := Vector3.ZERO     # for IMU specific force during ragdoll
var _controls := PackedFloat32Array() # 16 channels, normalised 0..1
var _controls_pwm := PackedInt32Array()  # 16 channels, raw microseconds

## Render-only offset subtracted from the visual position (set by GWFloatingOrigin
## for large worlds). Does NOT affect the true NED state reported to ArduPilot.
var render_origin := Vector3.ZERO


# --- Dynamics hooks (override in concrete vehicles) --------------------------
## Build dynamics models from the vehicle's config. Return false if misconfigured
## (physics is then disabled).
func _setup_vehicle() -> bool:
	return true

## Reset actuator/motor lag state (called on spawn and SITL reset).
func _reset_dynamics() -> void:
	pass

## Advance one non-crashed sub-step (size `h`, s): update actuators from `_cmd`,
## compute forces/moments, integrate _pos_ned/_vel_ned/_dcm/_omega, and handle the
## ground / takeoff / touchdown for this vehicle.
func _integrate(_h: float) -> void:
	pass

## Mass (kg) used for the ragdoll proxy.
func _vehicle_mass() -> float:
	return 1.0

## Collision-hull full size (m, render frame) when `hull_size` is left at 0.
func _default_hull_size() -> Vector3:
	return Vector3(1.0, 0.3, 1.0)


func _ready() -> void:
	if not _setup_vehicle():
		set_physics_process(false)
		return
	_atmos = GWAtmosphereModel.new()
	_hull_shape = BoxShape3D.new()
	_hull_shape.size = (hull_size * 2.0) if hull_size != Vector3.ZERO else _default_hull_size()
	_resolve_source()
	if aircraft_layer != 0 and is_inside_tree():
		_create_hull_body()
	_wind = _resolve_wind()
	# Capture the editor placement so a held vehicle spawns on the catapult, not at 0,0.
	if hold_until_launch and is_inside_tree():
		_hold_pos_ned = GWCoordConvert.world_to_ned(global_position)
		_hold_dcm = GWCoordConvert.render_basis_to_dcm(global_transform.basis)
	_reset_state()


## Resolve the active command source for `control_source`. SITL uses the
## GWSITLBridge at `bridge_path`; MANUAL uses the GWManualInput at
## `manual_input_path`, auto-creating one if absent so a bare body still flies.
func _resolve_source() -> void:
	if not is_inside_tree():
		return  # not-in-tree instance (e.g. unit tests): source set up by the test
	if control_source == ControlSource.MANUAL:
		var manual := get_node_or_null(manual_input_path)
		if manual == null:
			manual = GWManualInput.new()
			manual.name = "GWManualInput"
			add_child(manual)
			manual_input_path = ^"GWManualInput"
		_source = manual
		return
	_bridge = get_node_or_null(bridge_path) as GWSITLBridge
	if _bridge == null:
		push_warning("GWVehicleBody: no GWSITLBridge at %s — running idle (no SITL)." % [bridge_path])
	_source = _bridge


func _reset_state() -> void:
	if hold_until_launch:
		_pos_ned = _hold_pos_ned
		_dcm = _hold_dcm
		_held = true
		_on_ground = false  # it's on the launcher, not the runway
	else:
		_pos_ned = Vector3(spawn_north, spawn_east, -spawn_altitude)
		_dcm = GWCoordConvert.attitude_to_dcm(0.0, 0.0, spawn_heading)
		# Rest on the terrain under the spawn point (spawn_altitude is clearance).
		_update_ground_sample()
		_pos_ned.z = _ground_down - spawn_altitude
		_on_ground = true
	_vel_ned = Vector3.ZERO
	_omega = Vector3.ZERO
	_sim_time = 0.0
	_airspeed = 0.0
	_crashed = false
	_crash_settle = 0.0
	_exit_ragdoll()  # discard any wreck proxy from a previous crash
	_accel_body = Vector3(0, 0, -G) # parked: 1 g up -> -z reads -g
	_reset_dynamics()
	_sync_node()


func _physics_process(delta: float) -> void:
	if _source == null or not _source.has_command():
		return # SITL: advance only when ArduPilot has sent PWM. MANUAL: every tick.
	_cmd = _source.take_command()
	_update_controls(_cmd["pwm"])
	if _cmd["reset"]:
		_reset_state()
	if _ragdolling:
		_step_ragdoll(delta)  # FDM is suspended; read the wreck back from physics
	else:
		_step(delta)
	_source.post_state(_build_state())


## Store the latest PWM channels (raw + normalised) and notify listeners.
func _update_controls(pwm: PackedInt32Array) -> void:
	_controls_pwm = pwm
	if _controls.size() != pwm.size():
		_controls.resize(pwm.size())
	for i in pwm.size():
		_controls[i] = clampf((pwm[i] - 1000.0) / 1000.0, 0.0, 1.0)
	controls_received.emit(_controls)


## Channel value normalised to 0..1 (1-based, ch 1..16). 0 if not yet received.
func control_norm(channel: int) -> float:
	var i := channel - 1
	return _controls[i] if i >= 0 and i < _controls.size() else 0.0


## Raw channel PWM in microseconds (1-based). 0 if not yet received.
func control_pwm(channel: int) -> int:
	var i := channel - 1
	return _controls_pwm[i] if i >= 0 and i < _controls_pwm.size() else 0


## Launch the vehicle forward at `speed` (m/s) along its current nose direction — a
## catapult / bungee / hand-throw. Marks it airborne and emits took_off. No-op while
## crashed. The velocity is applied instantly, so ArduPilot's IMU sees a brief
## acceleration spike (a real catapult is only a few g — keep `speed` sane).
func launch(speed: float) -> void:
	if _crashed:
		return
	var was_locked := _on_ground or _held
	_held = false
	_on_ground = false
	var fwd := (_dcm * Vector3(1.0, 0.0, 0.0)).normalized()  # body nose, NED
	_vel_ned = fwd * speed
	_airspeed = speed
	if was_locked:
		took_off.emit()


## Advance the simulation by `dt` using fixed internal sub-steps, so behaviour is
## identical regardless of the host physics tick rate — total advanced time stays
## `dt`, only the integration granularity is fixed.
func _step(dt: float) -> void:
	_update_ground_sample()  # one terrain probe per frame; substeps share it
	var n := maxi(1, ceili(dt / maxf(max_substep, 0.0005)))  # guard a 0/garbage value
	var h := dt / n
	var v_before := _vel_ned
	for _i in n:
		_substep(h)
	# IMU specific force = kinematic accel - gravity, over the whole frame.
	var a_kin := (_vel_ned - v_before) / dt
	_accel_body = _dcm.transposed() * (a_kin - Vector3(0, 0, G)) # gravity down=+z
	_sync_node()
	# Hull vs. obstacles / other vehicles: any contact is a crash (not while held —
	# the hull overlaps the catapult it's sitting on).
	if not _crashed and not _ragdolling and not _held and _check_obstacle():
		_enter_crash()


## One fixed-size sub-step: crashed decay, else the vehicle's own dynamics.
func _substep(h: float) -> void:
	if _ragdolling:
		return  # FDM suspended; the physics-driven wreck takes over
	_sim_time += h  # time advances even when held, so SITL lockstep keeps ticking
	if _held:
		return  # frozen on the launcher; pose unchanged until launch()
	if _crashed:
		_step_crashed(h)
	else:
		_integrate(h)


## Crashed: no propulsion/aero. Velocity and body rates decay to rest while the
## true attitude/gyro/accel are reported, so ArduPilot sees a consistent impact.
## Auto-recovers once settled (recover_after_crash), or on a SITL reset.
func _step_crashed(dt: float) -> void:
	var decay := clampf(dt / CRASH_STOP_TAU, 0.0, 1.0)
	_vel_ned = _vel_ned.lerp(Vector3.ZERO, decay)
	_pos_ned += _vel_ned * dt
	if _agl() < spawn_altitude:
		_pos_ned.z = _ground_down - spawn_altitude
		if _vel_ned.z > 0.0:
			_vel_ned.z = 0.0
	_omega = _omega.lerp(Vector3.ZERO, decay)
	var ang := _omega.length() * dt
	if ang > 1e-9:
		_dcm = (_dcm * Basis(_omega.normalized(), ang)).orthonormalized()
	_airspeed = (_vel_ned - _wind_ned()).length()

	if recover_after_crash:
		if _vel_ned.length() < 0.3 and _omega.length() < 0.3:
			_crash_settle += dt
			if _crash_settle >= CRASH_RECOVER_DELAY:
				_recover()
		else:
			_crash_settle = 0.0


## Push integrated pose onto the scene node (visuals + cameras follow this).
func _sync_node() -> void:
	transform = Transform3D(GWCoordConvert.dcm_to_render_basis(_dcm),
			GWCoordConvert.ned_to_world(_pos_ned) - render_origin)


## Sample the terrain surface under the vehicle (height as NED-down + normal) via a
## downward ray, caching it for the step's substeps. Falls back to a flat plane at
## `ground_level` when disabled, treeless (unit tests), or when the ray misses.
func _update_ground_sample() -> void:
	_ground_normal = Vector3(0, 0, -1)
	if not terrain_following or not is_inside_tree():
		_ground_down = -ground_level
		return
	var space := get_world_3d().direct_space_state
	if space == null:
		_ground_down = -ground_level
		return
	var r := GWCoordConvert.ned_to_world(_pos_ned) - render_origin
	var params := PhysicsRayQueryParameters3D.create(
			r + Vector3(0, GROUND_PROBE_UP, 0),
			r - Vector3(0, GROUND_PROBE_DOWN, 0),
			ground_collision_mask)
	var hit := space.intersect_ray(params)
	if hit.is_empty():
		_ground_down = -ground_level
		return
	# Un-shift the render-space hit by render_origin to recover the true altitude.
	_ground_down = -(hit.position.y + render_origin.y)
	_ground_normal = GWCoordConvert.world_to_ned(hit.normal).normalized()


## Height of the CG above the terrain surface (m, positive = airborne).
func _agl() -> float:
	return _ground_down - _pos_ned.z


## True if the hull currently overlaps an obstacle- or aircraft-layer collider
## (excluding our own hull body).
func _check_obstacle() -> bool:
	var mask := obstacle_mask | aircraft_layer
	if mask == 0 or _hull_shape == null or not is_inside_tree():
		return false
	var space := get_world_3d().direct_space_state
	if space == null:
		return false
	var params := PhysicsShapeQueryParameters3D.new()
	params.shape = _hull_shape
	params.transform = global_transform
	params.collision_mask = mask
	params.margin = 0.05
	if _hull_body != null:
		params.exclude = [_hull_body.get_rid()]  # don't count our own hull
	return not space.intersect_shape(params, 1).is_empty()


## Air velocity (NED, m/s) at the vehicle from the wind source — zero (still air)
## when no GWWind is present.
func _wind_ned() -> Vector3:
	return _wind.sample(_pos_ned, _sim_time) if _wind != null else Vector3.ZERO


## Resolve the wind source: an explicit `wind_path`, else the first GWWind in this
## node's scene (so one dropped-in GWWind drives every vehicle).
func _resolve_wind() -> GWWind:
	if not wind_path.is_empty():
		return get_node_or_null(wind_path) as GWWind
	if not is_inside_tree():
		return null
	return _find_wind(_wind_scene_root())


func _wind_scene_root() -> Node:
	var n: Node = self
	var top := get_tree().root if get_tree() else null
	while n.get_parent() != null and n.get_parent() != top:
		n = n.get_parent()
	return n


func _find_wind(n: Node) -> GWWind:
	if n is GWWind:
		return n
	for c in n.get_children():
		var f := _find_wind(c)
		if f != null:
			return f
	return null


## Present a passive hull collider on `aircraft_layer` so other vehicles' obstacle
## shapecast can detect this one. It rides the body via this node's transform;
## nothing collide-responds to it (the FDM owns the crash) — it only needs to be
## detectable.
func _create_hull_body() -> void:
	var body := StaticBody3D.new()
	body.name = "Hull"
	body.collision_layer = aircraft_layer
	body.collision_mask = 0
	var cs := CollisionShape3D.new()
	cs.shape = _hull_shape
	body.add_child(cs)
	add_child(body)
	_hull_body = body


## A crash has been confirmed: ragdoll the wreck or run the scripted settle, per
## crash_mode.
func _enter_crash() -> void:
	crashed.emit()
	if crash_mode == CrashMode.RAGDOLL and is_inside_tree():
		_enter_ragdoll()
	else:
		_crashed = true
		_crash_settle = 0.0


## Spawn a RigidBody3D wreck proxy carrying the current velocity/spin, let physics
## tumble it against terrain/obstacles, and drive the visible model from it. The
## FDM is suspended until the wreck settles (see _step_ragdoll).
func _enter_ragdoll() -> void:
	if _ragdoll != null:
		return
	_crashed = true
	_ragdolling = true
	_on_ground = false
	var host := get_parent()
	if host == null:
		_crashed = true; _ragdolling = false; return  # can't host a sibling body
	var rb := RigidBody3D.new()
	rb.mass = _vehicle_mass()
	rb.continuous_cd = true
	var cs := CollisionShape3D.new()
	cs.shape = _hull_shape
	rb.add_child(cs)
	var phys := PhysicsMaterial.new()
	phys.bounce = ragdoll_bounce
	phys.friction = 0.8
	rb.physics_material_override = phys
	rb.collision_layer = 1 << 10  # own layer so the ground/obstacle probes ignore it
	rb.collision_mask = ground_collision_mask | obstacle_mask
	# Bleed energy so the wreck comes to rest quickly instead of skittering.
	rb.linear_damp = 0.4
	rb.angular_damp = 1.0
	host.add_child(rb)
	rb.global_transform = global_transform
	rb.linear_velocity = GWCoordConvert.ned_to_world(_vel_ned)
	rb.angular_velocity = GWCoordConvert.ned_to_world(_dcm * _omega)
	_ragdoll = rb
	_ragdoll_settle = 0.0
	_ragdoll_time = 0.0
	_prev_vel_ned = _vel_ned


## While ragdolling: read the physics-simulated wreck back into the NED state (so
## ArduPilot sees a consistent tumbling crash), drive the model, and recover the
## FDM to a level on-ground state once the wreck comes to rest.
func _step_ragdoll(dt: float) -> void:
	_sim_time += dt
	if _ragdoll == null or not is_instance_valid(_ragdoll):
		_exit_ragdoll(); _on_ground = true; return
	_update_ground_sample()
	var xf := _ragdoll.global_transform
	_pos_ned = GWCoordConvert.world_to_ned(xf.origin + render_origin)
	_dcm = GWCoordConvert.render_basis_to_dcm(xf.basis)
	_prev_vel_ned = _vel_ned
	_vel_ned = GWCoordConvert.world_to_ned(_ragdoll.linear_velocity)
	_omega = _dcm.transposed() * GWCoordConvert.world_to_ned(_ragdoll.angular_velocity)
	_airspeed = (_vel_ned - _wind_ned()).length()
	var a_kin := (_vel_ned - _prev_vel_ned) / dt
	_accel_body = _dcm.transposed() * (a_kin - Vector3(0, 0, G))
	_sync_node()

	if not recover_after_crash:
		return
	_ragdoll_time += dt
	# "At rest" is robust to contact micro-jitter: trust the engine's sleep signal,
	# or generous near-ground velocity thresholds. A hard time cap guarantees
	# recovery even if the wreck never fully settles (e.g. on a slope).
	var at_rest := _ragdoll.sleeping or (_vel_ned.length() < 1.0 \
			and _ragdoll.angular_velocity.length() < 1.0 and _agl() < spawn_altitude * 3.0)
	if at_rest:
		_ragdoll_settle += dt
	else:
		_ragdoll_settle = 0.0
	if _ragdoll_settle >= CRASH_RECOVER_DELAY or _ragdoll_time > RAGDOLL_MAX_TIME:
		_recover()


## Return to a flyable, level on-ground state after a crash settles. Respawns at
## the home/spawn point by default; otherwise re-levels in place keeping heading.
func _recover() -> void:
	_exit_ragdoll()
	_crashed = false
	_crash_settle = 0.0
	_vel_ned = Vector3.ZERO
	_omega = Vector3.ZERO
	var yaw := spawn_heading
	if respawn_at_home:
		_pos_ned.x = spawn_north
		_pos_ned.y = spawn_east
	else:
		yaw = GWCoordConvert.dcm_to_ned_attitude(_dcm)[2]
	_dcm = GWCoordConvert.attitude_to_dcm(0.0, 0.0, yaw)
	_on_ground = true
	_update_ground_sample()
	_pos_ned.z = _ground_down - spawn_altitude
	recovered.emit()


## Tear down the wreck proxy (on recovery or reset).
func _exit_ragdoll() -> void:
	_ragdolling = false
	_ragdoll_settle = 0.0
	_ragdoll_time = 0.0
	if _ragdoll != null and is_instance_valid(_ragdoll):
		_ragdoll.queue_free()
	_ragdoll = null


# --- Facade helpers (used by GWAircraft / GWMulticopter to self-assemble) -----

## Find or create a GWSITLBridge child listening on the per-instance port
## (9002 + 10*instance) and point bridge_path at it.
func _ensure_bridge(instance: int) -> void:
	var port := GWSITLBridge.LISTEN_PORT + 10 * instance
	for child in get_children():
		if child is GWSITLBridge:
			child.listen_port = port
			bridge_path = get_path_to(child)
			return
	var bridge := GWSITLBridge.new()
	bridge.name = "GWSITLBridge"
	bridge.listen_port = port
	add_child(bridge)
	bridge_path = ^"GWSITLBridge"


## Find or create a GWManualInput child (for control_source == MANUAL) and point
## manual_input_path at it. Sibling to _ensure_bridge for the drop-in facades.
func _ensure_manual_input() -> void:
	for child in get_children():
		if child is GWManualInput:
			manual_input_path = get_path_to(child)
			return
	var manual := GWManualInput.new()
	manual.name = "GWManualInput"
	add_child(manual)
	manual_input_path = ^"GWManualInput"


## Add a GWCamera child configured from `opts` (no-op if one already exists). Keys:
## protocol, resolution, fps, mount, instance, launch_ffmpeg, gimbal(bool) + gimbal_* passthrough.
## Per-vehicle ports auto-offset by `instance` so multiple vehicles don't collide.
func _ensure_camera(opts: Dictionary) -> void:
	for child in get_children():
		if child is GWCamera:
			return
	var inst: int = opts.get("instance", 0)
	var cam := GWCamera.new()
	cam.name = "GWCamera"
	cam.protocol = opts.get("protocol", GWCamera.Protocol.RTP_H264)
	cam.resolution = opts.get("resolution", Vector2i(1280, 720))
	cam.fps = opts.get("fps", 30.0)
	cam.transform = opts.get("mount", Transform3D(Basis(), Vector3(0, 0.1, -1.0)))
	cam.video_port = 5600 + 2 * inst
	cam.metadata_port = 5601 + 2 * inst
	cam.raw_tcp_port = 5566 + inst
	# false hands the raw-frame TCP server to your own pipeline instead of ffmpeg
	# — e.g. tools/gw_klv_muxer.py for STANAG4609/KLV output (see the README).
	cam.launch_ffmpeg = opts.get("launch_ffmpeg", true)
	if opts.get("gimbal", false):
		cam.gimbal_enabled = true
		cam.gimbal_pitch_channel = opts.get("gimbal_pitch_channel", 0)
		cam.gimbal_yaw_channel = opts.get("gimbal_yaw_channel", 0)
		cam.gimbal_roll_channel = opts.get("gimbal_roll_channel", 0)
		cam.gimbal_pitch_range_deg = opts.get("gimbal_pitch_range_deg", Vector2(-90.0, 0.0))
		cam.gimbal_yaw_range_deg = opts.get("gimbal_yaw_range_deg", Vector2(-180.0, 180.0))
		cam.gimbal_roll_range_deg = opts.get("gimbal_roll_range_deg", Vector2(-30.0, 30.0))
		cam.gimbal_pwm_min = opts.get("gimbal_pwm_min", 1000)
		cam.gimbal_pwm_max = opts.get("gimbal_pwm_max", 2000)
	add_child(cam)


## Assemble the ArduPilot JSON state Dictionary for GWSITLBridge.post_state().
## All quantities are already in NED/FRD, so no conversion is required.
func _build_state() -> Dictionary:
	var state := {
		"timestamp": _sim_time,
		"gyro": _omega,                                  # (p, q, r) FRD
		"accel_body": _accel_body,                       # specific force FRD
		"position": _pos_ned,                            # (N, E, D)
		"attitude": GWCoordConvert.dcm_to_ned_attitude(_dcm),
		"velocity": _vel_ned,                            # NED
		"airspeed": _airspeed,
	}
	if _wind != null and _wind.wind_speed > 0.0:
		state["wind"] = _wind.windvane()
	# Downward rangefinder (flat world at D=0): SITL's RNGFND1_TYPE=100
	# driver reads JSON "rng_1"; without it the driver reports NoData and
	# EK3 never starts optical-flow NAV aiding (measured 2026-08-04:
	# vehicle at 27 m, RFND Dist 0.00/Stat 1 all flight, GUIDED refused
	# "requires position"). Clamp: the sensor reads 0 when landed.
	state["rangefinder"] = maxf(0.0, -_pos_ned.z)
	return state
