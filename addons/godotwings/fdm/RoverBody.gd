## Wheeled ground vehicle 6-DOF dynamics — the ArduRover analog of GWFlightBody /
## GWMultirotorBody, on the shared GWVehicleBody (SITL lockstep, terrain probing,
## crash/ragdoll, wind, rendering, state reporting).
##
## The chassis is a rigid body (real inertia tensor, CG at the body origin) that
## rides on N independently-suspended wheels. Each sub-step, every wheel:
##   - finds the ground under it (per-wheel terrain raycast, cached per frame, so
##     it drives real meshes — Cesium 3D Tiles, imported terrain — or the flat
##     plane when `terrain_following` is off);
##   - pushes on the chassis through a spring/damper (preloaded to its static
##     share of the weight, with travel limits and bump stops) — this is where
##     squat, dive, body roll and rollover come from;
##   - generates tyre forces at its contact patch: a DC-motor torque curve
##     (stall torque falling to zero at the no-load top speed), neutral braking,
##     rolling resistance, and a lateral force that builds with slip velocity —
##     all clipped to a friction circle (mu_static to break loose, mu_kinetic
##     while sliding), so it drifts, spins its wheels and slides on slopes.
## Aerodynamic drag, gravity and the rigid-body integration (with the
## gyroscopic term) close the loop.
##
## Channels follow ArduRover's servo outputs: 1 = ground steering (1500 centre,
## + = right), 3 = throttle (1500 = stop, 2000 = full forward, 1000 = full
## reverse). In SKID steering mode channel 1 = left side, 3 = right side (set
## SERVO1_FUNCTION 73 / SERVO3_FUNCTION 74 on the autopilot).
class_name GWRoverBody
extends GWVehicleBody

## Default config shipped with the addon; used only if `config` is left unset.
const DEFAULT_CONFIG_PATH := "res://addons/godotwings/aircraft/Rover4WD.tres"
## Bump-stop stiffness relative to the spring once the travel limit is passed.
const BUMP_STOP_FACTOR := 12.0
## Throttle magnitude below which the motor is treated as at neutral (braking).
const NEUTRAL_DEADBAND := 0.02

@export var config: GWRoverConfig

@export_group("Manual drive")
## When control_source is MANUAL, drive it like a car: the pitch stick (↑/↓,
## channel 2) is forward/reverse throttle and the roll stick (←/→, channel 1)
## steers — no sticky throttle. Off = channels are read exactly as from
## ArduRover (1 = steering, 3 = centred throttle). Ignored under SITL.
@export var manual_drive: bool = true

@export_group("Ground probing")
## Each wheel's terrain ray starts this far above its hub (m). Keep it short so
## a low bridge or tree canopy above the vehicle isn't mistaken for the ground.
@export var probe_up: float = 0.5
## ...and reaches this far below it (m). Beyond that the wheel is in the air.
@export var probe_down: float = 25.0
## Set `spawn_altitude` from the wheel geometry (CG height at rest) so the vehicle
## spawns sitting on its wheels. Off = use `spawn_altitude` as set.
@export var auto_spawn_height: bool = true
## If ground appears ABOVE every wheel within this height (m) — a streamed tile
## loading over a vehicle parked on the flat fallback, a finer tile replacing a
## coarser one — lift the vehicle onto it. Large enough for terrain LOD steps,
## small enough that a bridge deck overhead is left alone. 0 = never lift.
@export var lift_onto_surface_m: float = 8.0

## The wheels. Left empty, they are built from `config` geometry at setup (4
## wheels: FL, FR, RL, RR). GWRover fills this from a visual model's wheel nodes
## before `_ready()` runs.
var wheels: Array[GWRoverWheel] = []
## Body box (length, width, height, m) taken from a model's hull, overriding
## `config.body_size` for the hull and inertia estimate. Zero = use the config.
var body_size_override := Vector3.ZERO

var _inertia := Vector3.ONE
var _steer := 0.0        # lagged steering command, -1..1
var _thr_left := 0.0     # lagged throttle command, left side / all wheels
var _thr_right := 0.0    # lagged throttle command, right side
var _wheelbase := 1.0    # front-to-rear hub distance of the layout (m)


func _setup_vehicle() -> bool:
	if config == null and ResourceLoader.exists(DEFAULT_CONFIG_PATH):
		config = load(DEFAULT_CONFIG_PATH)
	if config == null:
		push_error("GWRoverBody: no GWRoverConfig assigned (set the `config` property).")
		return false
	if wheels.is_empty():
		wheels = build_wheels_from_config(config)
	if wheels.size() < 3:
		push_error("GWRoverBody: needs at least 3 wheels (have %d)." % wheels.size())
		return false
	_finalize_wheels()
	_inertia = config.inertia(_body_size())
	if auto_spawn_height:
		spawn_altitude = rest_cg_height()
	return true


func _reset_dynamics() -> void:
	_steer = 0.0
	_thr_left = 0.0
	_thr_right = 0.0
	for w in wheels:
		w.reset_state()


func _vehicle_mass() -> float:
	return config.mass


## Render frame: X = width, Y = height, Z = length.
func _default_hull_size() -> Vector3:
	var s := _body_size()
	return Vector3(s.y, s.z, s.x)


func _body_size() -> Vector3:
	return body_size_override if body_size_override != Vector3.ZERO else config.body_size


## CG height above the ground when every wheel sits at its rest position (m).
func rest_cg_height() -> float:
	var h := 0.0
	for w in wheels:
		h = maxf(h, w.hub_rest.z + w.radius)
	return h


## Forward ground speed of the CG (m/s, body x).
func ground_speed() -> float:
	return (_dcm.transposed() * _vel_ned).x


## Build the standard 4-wheel layout from config geometry: hubs at the axle
## positions implied by wheelbase / track / CG position, `cg_height - radius`
## below the CG.
static func build_wheels_from_config(cfg: GWRoverConfig) -> Array[GWRoverWheel]:
	var out: Array[GWRoverWheel] = []
	var x_front := cfg.wheelbase * (1.0 - cfg.cg_position)
	var x_rear := -cfg.wheelbase * cfg.cg_position
	var z := cfg.cg_height - cfg.wheel_radius
	var half := cfg.track * 0.5
	for def in [["FL", x_front, -half], ["FR", x_front, half], ["RL", x_rear, -half], ["RR", x_rear, half]]:
		var w := GWRoverWheel.new()
		w.name = def[0]
		w.hub_rest = Vector3(def[1], def[2], z)
		w.radius = cfg.wheel_radius
		w.width = cfg.wheel_width
		out.append(w)
	return out


## Classify the wheels (front/rear axle → steers / driven), distribute the static
## load between them, and size each spring/damper from the config's natural
## frequency and damping ratio.
func _finalize_wheels() -> void:
	var x_max := -INF
	var x_min := INF
	for w in wheels:
		x_max = maxf(x_max, w.hub_rest.x)
		x_min = minf(x_min, w.hub_rest.x)
	_wheelbase = maxf(x_max - x_min, 0.05)
	var tol := 0.1 * _wheelbase
	for w in wheels:
		var front := w.hub_rest.x > x_max - tol
		w.rear = w.hub_rest.x < x_min + tol
		w.steers = config.steering_mode == GWRoverConfig.SteeringMode.ACKERMANN \
				and ((front and config.steer_front) or (w.rear and config.steer_rear))
		match config.drive_layout:
			GWRoverConfig.DriveLayout.FRONT: w.driven = front
			GWRoverConfig.DriveLayout.REAR: w.driven = w.rear
			_: w.driven = true
	_distribute_static_load()
	var wn := TAU * config.suspension_freq_hz
	for w in wheels:
		var m_i := w.static_load / G
		w.spring_k = m_i * wn * wn
		w.damper_c = 2.0 * config.suspension_damping_ratio * sqrt(w.spring_k * m_i)


## Static wheel loads N_i = w + a·x_i + b·y_i satisfying ΣN = m·g and zero net
## moment about the CG (the minimum-norm solution — exact for any symmetric
## layout, sensible for odd ones). Falls back to equal shares if degenerate.
func _distribute_static_load() -> void:
	var n := float(wheels.size())
	var mg := config.mass * G
	var sx := 0.0; var sy := 0.0; var sxx := 0.0; var syy := 0.0; var sxy := 0.0
	for w in wheels:
		sx += w.hub_rest.x; sy += w.hub_rest.y
		sxx += w.hub_rest.x * w.hub_rest.x; syy += w.hub_rest.y * w.hub_rest.y
		sxy += w.hub_rest.x * w.hub_rest.y
	var m := Basis(Vector3(n, sx, sy), Vector3(sx, sxx, sxy), Vector3(sy, sxy, syy))
	var equal := mg / n
	if absf(m.determinant()) < 1e-9:
		for w in wheels:
			w.static_load = equal
		return
	var sol := m.inverse() * Vector3(mg, 0.0, 0.0)
	for w in wheels:
		w.static_load = maxf(sol.x + sol.y * w.hub_rest.x + sol.z * w.hub_rest.y, 0.05 * equal)


# --- Terrain sampling ---------------------------------------------------------

## One terrain probe per wheel (plus one under the CG for the base class's AGL /
## rangefinder and spawn height), once per frame; the sub-steps reuse the
## sampled planes. The CG probe reaches far up like the other vehicles' so a
## spawn lands on terrain that is well above the flat fallback; the wheel
## probes stay short so a bridge or canopy above is never taken for the ground.
##
## Buried vehicle: when a long probe finds a surface ABOVE every hub, within
## `lift_onto_surface_m`, the ground has appeared over us — a streamed tile
## arriving after a spawn on the flat fallback, or a coarse tile swapped for a
## finer one metres higher. Lift the chassis onto it rather than leaving it
## driving on the plane underneath, out of sight.
func _update_ground_sample() -> void:
	var cg := _probe_ground(_pos_ned, GROUND_PROBE_UP)
	_ground_down = cg[0]
	_ground_normal = cg[1]
	var buried := 0
	var surface_down := INF   # highest surface found above a wheel (NED down, smaller = higher)
	for w in wheels:
		var hub := _pos_ned + _dcm * w.hub_rest
		var p := _probe_ground(hub, probe_up)
		w.ground_hit = p[2]
		w.ground_point = Vector3(hub.x, hub.y, p[0])
		w.ground_normal = p[1]
		if terrain_following and lift_onto_surface_m > 0.0:
			var long_probe := _probe_ground(hub, GROUND_PROBE_UP)
			if long_probe[2] and long_probe[0] < hub.z and hub.z - long_probe[0] <= lift_onto_surface_m:
				buried += 1
				surface_down = minf(surface_down, long_probe[0])
	if buried == wheels.size() and is_finite(surface_down) and not _crashed and not _ragdolling:
		_pos_ned.z = surface_down - rest_cg_height()
		_vel_ned.z = minf(_vel_ned.z, 0.0)
		for w in wheels:
			var hub := _pos_ned + _dcm * w.hub_rest
			var p := _probe_ground(hub, probe_up)
			w.ground_hit = p[2]
			w.ground_point = Vector3(hub.x, hub.y, p[0])
			w.ground_normal = p[1]
			w._droop_valid = false


## Downward ray under a NED point, starting `up` metres above it: [ground
## NED-down, normal NED, hit]. Flat plane at `ground_level` when terrain
## following is off, out of tree, or on a miss (hit = false then).
func _probe_ground(p_ned: Vector3, up: float) -> Array:
	if not terrain_following or not is_inside_tree():
		return [-ground_level, Vector3(0, 0, -1), true]
	var space := get_world_3d().direct_space_state
	if space == null:
		return [-ground_level, Vector3(0, 0, -1), true]
	var r := GWCoordConvert.ned_to_world(p_ned) - render_origin
	var params := PhysicsRayQueryParameters3D.create(
			r + Vector3(0, up, 0), r - Vector3(0, probe_down, 0), ground_collision_mask)
	var hit := space.intersect_ray(params)
	if hit.is_empty():
		return [-ground_level, Vector3(0, 0, -1), false]
	return [-(hit.position.y + render_origin.y), GWCoordConvert.world_to_ned(hit.normal).normalized(), true]


# --- Dynamics -------------------------------------------------------------------

## One sub-step: actuators, steering geometry, per-wheel suspension + tyre
## forces, aero drag, rigid-body integration, then ground-state bookkeeping.
func _integrate(h: float) -> void:
	var cmd := _drive_commands()
	var k_s := 1.0 - exp(-h / maxf(config.steer_time_const, 1e-4))
	var k_m := 1.0 - exp(-h / maxf(config.motor_time_const, 1e-4))
	_steer = lerpf(_steer, cmd[0], k_s)
	_thr_left = lerpf(_thr_left, cmd[1], k_m)
	_thr_right = lerpf(_thr_right, cmd[2], k_m)
	_apply_steering()

	var force_ned := Vector3(0.0, 0.0, config.mass * G)   # gravity, NED down = +z
	var moment_b := Vector3.ZERO
	var contacts := 0
	for w in wheels:
		var fm := _wheel_forces(w, h)
		force_ned += fm[0]
		moment_b += fm[1]
		if w.in_contact:
			contacts += 1

	var v_air_ned := _vel_ned - _wind_ned()
	var v_air_body := _dcm.transposed() * v_air_ned
	_airspeed = v_air_ned.length()
	force_ned += _dcm * _airframe_drag(v_air_body)

	var v_before := _vel_ned
	_vel_ned += (force_ned / config.mass) * h
	_pos_ned += _vel_ned * h
	_integrate_omega(moment_b, h)
	_update_ground_state(contacts, v_before)


## Steering / throttle from the command source: [steer -1..1, left -1..1,
## right -1..1]. Ackermann uses left == right.
func _drive_commands() -> Array:
	if _controls.is_empty():
		return [0.0, 0.0, 0.0]
	var skid := config.steering_mode == GWRoverConfig.SteeringMode.SKID
	if control_source == ControlSource.MANUAL and manual_drive:
		var steer := (control_norm(1) - 0.5) * 2.0
		var thr := (control_norm(2) - 0.5) * 2.0
		if skid:
			return [0.0, clampf(thr + steer, -1.0, 1.0), clampf(thr - steer, -1.0, 1.0)]
		return [steer, thr, thr]
	var ch1 := (control_norm(1) - 0.5) * 2.0
	var ch3 := (control_norm(3) - 0.5) * 2.0
	if skid:
		return [0.0, ch1, ch3]
	return [ch1, ch3, ch3]


## Ackermann geometry: the steered axle's centre-line angle is `_steer` × full
## lock; the inner wheel turns tighter and the outer shallower so all wheels
## roll about one turn centre. A four-wheel-steer rear axle counter-steers.
func _apply_steering() -> void:
	var delta_c := _steer * config.max_steer_rad()
	for w in wheels:
		if not w.steers:
			w.steer_angle = 0.0
			continue
		var sgn := -1.0 if w.rear else 1.0   # rear axle counter-steers (4WS)
		if absf(delta_c) < 1e-4:
			w.steer_angle = 0.0
			continue
		var r_turn := _wheelbase / tan(absf(delta_c))
		var y := absf(w.hub_rest.y)
		var inner := (delta_c > 0.0) == (w.hub_rest.y > 0.0)
		var r_wheel := maxf(r_turn - y, 0.05) if inner else r_turn + y
		w.steer_angle = signf(delta_c) * sgn * atan(_wheelbase / r_wheel)


## Suspension + tyre forces for one wheel. Returns [force NED, moment body FRD].
func _wheel_forces(w: GWRoverWheel, h: float) -> Array:
	w.in_contact = false
	w.normal_force = 0.0
	w.long_force = 0.0
	w.lat_force = 0.0
	var down := _dcm.z                       # body down, NED
	var n := w.ground_normal                 # ground up, NED
	var hub := _pos_ned + _dcm * w.hub_rest
	var denom := down.dot(n)
	if denom > -0.2:
		# Ground plane is edge-on or above the wheel (rolled onto its side): no contact.
		w._droop_valid = false
		_spin_free(w, h)
		return [Vector3.ZERO, Vector3.ZERO]
	# Distance from the hub to the ground plane along body-down; the wheel must
	# sit `radius` above it, the rest is suspension droop.
	var t := (w.ground_point - hub).dot(n) / denom
	var free_droop := t - w.radius
	var travel := config.suspension_travel
	if free_droop > travel:
		w.droop = travel
		w._droop_valid = false
		_spin_free(w, h)
		return [Vector3.ZERO, Vector3.ZERO]
	var d := maxf(free_droop, -travel * 1.5)
	w.droop = d
	var x := -d                                  # compression from rest
	var xdot := 0.0
	if w._droop_valid:
		xdot = -(d - w._droop_prev) / h
	w._droop_prev = d
	w._droop_valid = true
	var f_n := w.static_load + w.spring_k * x + w.damper_c * xdot
	if x > travel:
		f_n += w.spring_k * BUMP_STOP_FACTOR * (x - travel)
	f_n = maxf(f_n, 0.0)
	if f_n <= 0.0:
		_spin_free(w, h)
		return [Vector3.ZERO, Vector3.ZERO]
	w.in_contact = true
	w.normal_force = f_n

	# Contact patch kinematics.
	var r_b := w.hub_rest + Vector3(0.0, 0.0, t)          # CG -> contact, body
	var v_c := _vel_ned + _dcm * _omega.cross(r_b)         # contact point velocity, NED
	var v_t := v_c - n * v_c.dot(n)                        # in the ground plane
	var fwd := _dcm * Vector3(cos(w.steer_angle), sin(w.steer_angle), 0.0)
	fwd -= n * fwd.dot(n)
	if fwd.length_squared() < 1e-6:
		return [n * f_n, r_b.cross(_dcm.transposed() * (n * f_n))]
	fwd = fwd.normalized()
	var right := fwd.cross(n)                              # NED: fwd × up = right
	var v_long := v_t.dot(fwd)
	var v_lat := v_t.dot(right)
	w.ground_speed = v_long

	# Motor command for this wheel (per side for skid steer; equal otherwise).
	var cmd := 0.0
	if w.driven:
		cmd = _thr_left if w.hub_rest.y < 0.0 else _thr_right
	w.throttle = cmd
	var mu_eff := config.mu_kinetic if w.sliding else config.mu_static
	var stiffness := config.lateral_stiffness * f_n      # N per m/s of slip

	# Longitudinal slip speed u = ω·R − v_long: solved implicitly from the motor
	# torque curve against the tyre force, so a torquey launch spins the wheel,
	# an overrun motor brakes, and a locked-up wheel skids — and while a wheel
	# spins or skids its friction budget goes to the longitudinal direction, which
	# is what lets a skid-steer pivot and a drifting car slide.
	var u := 0.0
	var powered := w.driven and absf(cmd) >= NEUTRAL_DEADBAND
	if powered:
		u = _solve_slip(w, cmd, v_long, v_lat, f_n, stiffness, mu_eff)
	var slip := Vector2(-u, v_lat)                      # contact patch vs ground (ω·R − v_long = u)
	var slip_len := slip.length()
	var f_tyre := Vector2.ZERO
	if slip_len > 1e-6:
		f_tyre = -minf(stiffness * slip_len, mu_eff * f_n) * slip / slip_len
	w.sliding = stiffness * slip_len > mu_eff * f_n
	w.spin_angle += ((v_long + u) / w.radius) * h

	# Neutral braking (ESC brake / engine braking) and rolling resistance are
	# Coulomb-like, smoothed near zero speed so a parked vehicle holds still.
	if w.driven and not powered:
		f_tyre.x -= config.neutral_brake * config.mu_static * f_n * clampf(v_long / 0.02, -1.0, 1.0)
	f_tyre.x -= config.rolling_resistance * f_n * clampf(v_long / 0.05, -1.0, 1.0)
	var mag := f_tyre.length()
	var limit := mu_eff * f_n
	if mag > limit:
		f_tyre *= limit / mag
	var fdes := f_tyre
	w.long_force = fdes.x
	w.lat_force = fdes.y

	var f_ned := n * f_n + fwd * fdes.x + right * fdes.y
	return [f_ned, r_b.cross(_dcm.transposed() * f_ned)]


## Longitudinal slip speed u (m/s, = ω·R − v_long) at which the motor's torque
## balances the tyre's longitudinal force: τ_max·(cmd − ω/ω_noload) / R = F_x(u),
## with F_x = min(k·|s|, μ·N)·u/|s| for the slip vector s = (u, v_lat). The
## left side falls with u and the right rises, so a bisection between u = 0 and
## the motor's no-load slip is exact enough in ~24 steps.
func _solve_slip(w: GWRoverWheel, cmd: float, v_long: float, v_lat: float, f_n: float,
		stiffness: float, mu: float) -> float:
	var radius := w.radius
	var v_nl := config.max_speed
	var t_max := config.wheel_torque_max
	var u0 := cmd * v_nl - v_long        # u where the motor torque is zero
	if absf(u0) < 1e-9:
		return 0.0
	var lo := minf(0.0, u0)
	var hi := maxf(0.0, u0)
	for _i in 26:
		var mid := 0.5 * (lo + hi)
		var f_motor := t_max * (cmd - (v_long + mid) / v_nl) / radius
		var s_len := sqrt(mid * mid + v_lat * v_lat)
		var f_tyre := 0.0
		if s_len > 1e-9:
			f_tyre = minf(stiffness * s_len, mu * f_n) * mid / s_len
		var f := f_motor - f_tyre
		# f is decreasing in u: root is above mid when f > 0.
		if f > 0.0:
			lo = mid
		else:
			hi = mid
	return 0.5 * (lo + hi)


## An airborne wheel keeps spinning at its motor's no-load speed for that command.
func _spin_free(w: GWRoverWheel, h: float) -> void:
	if w.driven:
		var cmd := _thr_left if w.hub_rest.y < 0.0 else _thr_right
		w.throttle = cmd
		w.ground_speed = cmd * config.max_speed
	w.spin_angle += (w.ground_speed / w.radius) * h


## Quadratic body drag (body FRD): F = -½·rho·CdA(axis)·|v_axis|·v_axis.
func _airframe_drag(v: Vector3) -> Vector3:
	var cda := config.drag_area
	if cda == Vector3.ZERO:
		return Vector3.ZERO
	return -0.5 * GWRoverConfig.RHO * Vector3(
			cda.x * absf(v.x) * v.x, cda.y * absf(v.y) * v.y, cda.z * absf(v.z) * v.z)


## Rigid-body angular update with the inertia tensor + gyroscopic coupling:
## omega_dot = I⁻¹ (M − omega × (I·omega)).
func _integrate_omega(moment: Vector3, dt: float) -> void:
	var iom := Vector3(_inertia.x * _omega.x, _inertia.y * _omega.y, _inertia.z * _omega.z)
	var gyro := _omega.cross(iom)
	var acc := Vector3(
			(moment.x - gyro.x) / _inertia.x,
			(moment.y - gyro.y) / _inertia.y,
			(moment.z - gyro.z) / _inertia.z)
	_omega += acc * dt
	var ang := _omega.length() * dt
	if ang > 1e-9:
		_dcm = (_dcm * Basis(_omega.normalized(), ang)).orthonormalized()


## Wheels-on-ground bookkeeping: airborne / touchdown transitions (a jump), hard
## landings, and rollovers become crashes.
func _update_ground_state(contacts: int, v_before: Vector3) -> void:
	if contacts > 0:
		if not _on_ground:
			_on_ground = true
			if v_before.z > config.max_landing_speed:
				_enter_crash()
				return
			landed.emit()
	elif _on_ground:
		_on_ground = false
		took_off.emit()
	var att := GWCoordConvert.dcm_to_ned_attitude(_dcm)
	var lim := deg_to_rad(config.rollover_deg)
	if absf(att[0]) > lim or absf(att[1]) > lim:
		_enter_crash()
