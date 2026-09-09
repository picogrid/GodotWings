## Quadcopter 6-DOF dynamics — the copter analog of GWFlightBody, on the shared
## GWVehicleBody (SITL lockstep, ground/terrain, crash/ragdoll, wind, rendering).
##
## Follows ArduPilot's SITL multicopter frame model: each motor's ESC commands a
## rotor speed (first-order spool lag, separate up/down), thrust goes with
## omega^expo (quadratic by default — the real prop curve), washing out with
## axial inflow; forces are thrust + quadratic airframe drag + rotor H-force,
## moments are the motor mix + reaction torque + rotor rate damping, integrated
## as a rigid body with a real inertia tensor and the gyroscopic term. Quad X
## layout; channels 1-4 = motors. See GWMultirotorConfig for every knob (all the
## new aero terms default OFF for back-compat; QuadX.tres enables them).
class_name GWMultirotorBody
extends GWVehicleBody

## Default config shipped with the addon; used only if `config` is left unset.
const DEFAULT_CONFIG_PATH := "res://addons/godotwings/aircraft/QuadX.tres"

@export var config: GWMultirotorConfig
## SITL control-loop rate (Hz). A multirotor's rate controller needs a fast loop,
## and the ArduPilot exchange runs once per Godot physics tick — so this raises the
## engine's physics tick rate to match. The 60 Hz default makes the copter
## oscillate/vibrate (ArduCopter expects ~400 Hz). 0 = leave the project setting.
@export var control_rate_hz: int = 400

@export_group("Manual acro (FC emulation)")
## When control_source is MANUAL, run a Betaflight-style rate controller: sticks
## command angular RATES (channels 1/2/4), a rate PID + quad-X mixer drives the
## motors — hand-flyable like an FPV sim, no SITL needed. Off = channels 1-4 stay
## raw per-motor throttles (legacy; unflyable by hand). Ignored under SITL.
@export var manual_acro: bool = true
## Full-deflection roll/pitch rate (deg/s). FPV freestyle flies 600-1000.
@export var acro_max_rate_dps: float = 720.0
## Full-deflection yaw rate (deg/s).
@export var acro_yaw_rate_dps: float = 540.0
## Stick expo (0 = linear, 1 = cubic): softens centre stick, keeps the ends.
@export_range(0.0, 1.0) var acro_expo: float = 0.6
## Rate PID gains (motor fraction per rad/s [P], per rad [I], per rad/s² [D]).
@export var acro_rate_p: float = 0.055
@export var acro_rate_i: float = 0.03
@export var acro_rate_d: float = 0.0008
## Air-mode idle: motors never drop below this, so you keep authority at 0
## throttle (mid-flip, throttle chops).
@export_range(0.0, 0.2) var acro_idle: float = 0.05

const RHO := 1.225                     # air density (kg/m³), sea level ISA
const CELL_FULL := 4.2                 # LiPo cell voltage, full
const CELL_EMPTY := 3.3                # ...and sagged-empty under no load
const CELL_NOM := 3.7                  # nominal (thrust ratings assume this)

var _motor := PackedFloat32Array()    # lagged normalized rotor speed per motor, 0..1
var _pos: Array[Vector3] = []         # motor positions (body FRD, m)
var _yawdir := PackedFloat32Array()   # +1 = CCW prop, -1 = CW prop
var _inertia := Vector3.ONE
var _per_motor_max := 0.0
var _var := PackedFloat32Array()      # per-motor thrust variance factors
var _rng := RandomNumberGenerator.new()
var _wobble := Vector3.ZERO           # band-limited propwash noise state
var _propwash := 0.0                  # current propwash intensity, 0..1
var _mah_used := 0.0                  # battery charge drawn (mAh)
var _batt_scale := 1.0                # current thrust scale from pack voltage
var _acro_i := Vector3.ZERO           # rate-PID integrator
var _acro_prev_err := Vector3.ZERO
var _acro_d := Vector3.ZERO           # low-passed D term
var _acro_saturated := false          # mixer clipped last tick (anti-windup)
var _spool_rate := PackedFloat32Array()  # d(omega_norm)/dt per motor (yaw kick)


func _setup_vehicle() -> bool:
	if config == null and ResourceLoader.exists(DEFAULT_CONFIG_PATH):
		config = load(DEFAULT_CONFIG_PATH)
	if config == null:
		push_error("GWMultirotorBody: no MultirotorConfig assigned (set the `config` property).")
		return false
	_build_frame()
	_ensure_control_rate()
	return true


## Raise Godot's physics tick rate so the SITL exchange (one per tick) runs the
## ArduCopter rate loop fast enough — a 60 Hz loop makes a copter oscillate.
func _ensure_control_rate() -> void:
	if control_rate_hz <= 0 or Engine.is_editor_hint():
		return
	if Engine.physics_ticks_per_second < control_rate_hz:
		Engine.physics_ticks_per_second = control_rate_hz
		# Don't let a slow render frame starve the physics/SITL exchange: with
		# lockstep waiting (GWSITLBridge.wait_command) every allowed step
		# completes an exchange, so this cap is the realtime floor --
		# rate/10 keeps sim time honest down to ~10 rendered fps.
		Engine.max_physics_steps_per_frame = maxi(
				Engine.max_physics_steps_per_frame, ceili(control_rate_hz / 10.0))
		print("GWMultirotorBody: raised physics tick rate to %d Hz (copter rate loop)." % control_rate_hz)


## Quad X: motor 1 front-right (CCW), 2 back-left (CCW), 3 front-left (CW),
## 4 back-right (CW) — ArduCopter's standard layout. Position from a body-frame
## angle (deg, from +x forward toward +y right).
func _build_frame() -> void:
	_inertia = config.inertia()
	_per_motor_max = config.max_thrust_total() / maxf(config.motor_count, 1)
	var defs := [[45.0, 1.0], [-135.0, 1.0], [-45.0, -1.0], [135.0, -1.0]]
	_pos.clear()
	_yawdir.resize(defs.size())
	for i in defs.size():
		var a := deg_to_rad(defs[i][0])
		_pos.append(Vector3(cos(a), sin(a), 0.0) * config.arm_length)
		_yawdir[i] = defs[i][1]


func _reset_dynamics() -> void:
	_motor.resize(_pos.size())
	_motor.fill(0.0)
	_spool_rate.resize(_pos.size())
	_spool_rate.fill(0.0)
	_wobble = Vector3.ZERO
	_propwash = 0.0
	_mah_used = 0.0
	_batt_scale = 1.0
	_acro_i = Vector3.ZERO
	_acro_prev_err = Vector3.ZERO
	_acro_d = Vector3.ZERO
	_acro_saturated = false
	# Deterministic per-motor variance (seeded: reproducible runs and tests).
	_rng.seed = 0x5157524F  # "QWRO"
	_var.resize(_pos.size())
	for i in _var.size():
		_var[i] = 1.0 + _rng.randf_range(-config.motor_variance, config.motor_variance)


func _vehicle_mass() -> float:
	return config.mass


func _default_hull_size() -> Vector3:
	var d := config.arm_length * 2.0
	return Vector3(d, 0.2, d)


## One sub-step: spool the motors, then integrate attitude + position. Attitude is
## ALWAYS driven by the real motor moment — on the ground and in the air — so the
## autopilot's rate controller stays closed-loop (no force-leveling that would
## desync the gyro from its commands and tip the copter on takeoff).
func _integrate(h: float) -> void:
	var v_air_ned := _vel_ned - _wind_ned()
	var v_air_body := _dcm.transposed() * v_air_ned
	_airspeed = v_air_ned.length()

	_update_propwash(h, v_air_body)
	_update_battery(h)
	var thrusts := _update_motors(h, v_air_body)
	var total := 0.0
	var omega_sum := 0.0
	for i in thrusts.size():
		total += thrusts[i]
		omega_sum += _motor[i]

	var moment := _frame_moment(thrusts)
	moment += _rotor_damping_moment(omega_sum)
	moment += _flap_moment(v_air_body, omega_sum)
	moment += _propwash_wobble_moment()
	moment += _spool_yaw_kick()
	_integrate_omega(moment, h)

	# Forces (body FRD): thrust along body-up (-z), quadratic airframe drag,
	# rotor H-force (disc drag on lateral flow), and the legacy linear term.
	var force := Vector3(0.0, 0.0, -total)
	force += _airframe_drag(v_air_body)
	force += _rotor_drag(v_air_body, omega_sum)
	force -= config.drag_coeff * v_air_body

	var accel_ned := _dcm * (force / config.mass) + Vector3(0.0, 0.0, G)
	_vel_ned += accel_ned * h
	_pos_ned += _vel_ned * h
	_ground_contact(h)


## Spool each rotor toward its commanded speed (channels 1..N map linearly to
## normalized omega, like an ESC) and return per-motor thrust (N):
## T = omega^expo · max, washed out by axial inflow (climbing into the props
## drops blade AoA — thrust fades to zero as axial speed reaches
## prop_pitch_speed · omega, so punch-outs top out instead of running away).
func _update_motors(h: float, v_air_body: Vector3) -> PackedFloat32Array:
	var k_up := 1.0 - exp(-h / maxf(config.motor_time_const, 1e-4))
	var tc_down := config.motor_time_const_down if config.motor_time_const_down > 0.0 \
			else config.motor_time_const
	var k_down := 1.0 - exp(-h / maxf(tc_down, 1e-4))
	var cmds := _motor_commands(h)
	var thrusts := PackedFloat32Array()
	thrusts.resize(_motor.size())
	# Axial inflow: airspeed INTO the props from below is climb = -v_body.z... in
	# FRD body frame the props face -z, so inflow (air arriving at the discs from
	# their thrust side) is -v_air_body.z (positive while climbing).
	var inflow := -v_air_body.z
	var ge := _ground_effect_factor()
	var pw_loss := 1.0 - config.propwash_thrust_loss * _propwash
	for i in _motor.size():
		var cmd: float = cmds[i]
		var prev := _motor[i]
		_motor[i] = lerpf(_motor[i], cmd, k_up if cmd > _motor[i] else k_down)
		_spool_rate[i] = (_motor[i] - prev) / h
		var t := pow(_motor[i], config.thrust_expo) * _per_motor_max * _var[i] * _batt_scale
		if config.prop_pitch_speed > 0.0 and _motor[i] > 1e-3:
			# Upper cap 1.0, not 1.3: the windmill-brake bonus gave up to +30%
			# free thrust while DESCENDING (~+0.25 g near hover), making
			# throttle cuts floaty -- the quad would not drop. Washout on climb
			# (the punch-out limiter) is unchanged.
			t *= clampf(1.0 - inflow / (config.prop_pitch_speed * _motor[i]), 0.0, 1.0)
		thrusts[i] = t * ge * pw_loss
	return thrusts


## Per-motor commanded speed (0..1). SITL: channels 1..N are raw motors (the
## ArduPilot mixer already ran). MANUAL + manual_acro: channels are AETR sticks —
## run the emulated FC (rate PID + quad-X mix) so the quad is hand-flyable.
func _motor_commands(h: float) -> PackedFloat32Array:
	var cmds := PackedFloat32Array()
	cmds.resize(_motor.size())
	if control_source != ControlSource.MANUAL or not manual_acro:
		for i in cmds.size():
			cmds[i] = clampf(control_norm(i + 1), 0.0, 1.0)
		return cmds
	return _acro_mix(h)


## Betaflight-style rate controller: stick -> rate setpoint (expo curve), PID on
## the body rates, quad-X mixer with an air-mode idle floor.
func _acro_mix(h: float) -> PackedFloat32Array:
	var roll := (control_norm(1) - 0.5) * 2.0    # -1..1, right +
	var pitch := (control_norm(2) - 0.5) * 2.0   # -1..1, nose-up +
	var yaw := (control_norm(4) - 0.5) * 2.0     # -1..1, nose-right +
	var thr := clampf(control_norm(3), 0.0, 1.0)
	var target := Vector3(
			_rate_curve(roll) * deg_to_rad(acro_max_rate_dps),
			_rate_curve(pitch) * deg_to_rad(acro_max_rate_dps),
			_rate_curve(yaw) * deg_to_rad(acro_yaw_rate_dps))
	var err := target - _omega
	# Anti-windup: only integrate while the mixer isn't saturated (a real FC's
	# iterm_relax) — otherwise a held flip winds the other axes into the clamp.
	if not _acro_saturated:
		_acro_i = (_acro_i + err * acro_rate_i * h).clampf(-0.2, 0.2)
	# D on gyro (low-passed): raw derivative at 400 Hz is all spikes.
	var d_raw := (err - _acro_prev_err) / maxf(h, 1e-5)
	_acro_d = _acro_d.lerp(d_raw, 1.0 - exp(-h * TAU * 30.0))  # ~30 Hz D filter
	_acro_prev_err = err
	var out := err * acro_rate_p + _acro_i + _acro_d * acro_rate_d
	# Quad-X mix (motor order: 1 FR CCW, 2 BL CCW, 3 FL CW, 4 BR CW).
	# roll right = left motors up; pitch (nose) up = FRONT motors up (their
	# thrust lifts the nose: m.y = Σ px·T); yaw = CCW pair up.
	var base := acro_idle + thr * (1.0 - acro_idle)
	var cmds := PackedFloat32Array()
	cmds.resize(_motor.size())
	_acro_saturated = false
	for i in _motor.size():
		var mix := -signf(_pos[i].y) * out.x + signf(_pos[i].x) * out.y + _yawdir[i] * out.z
		var raw := base + mix
		if raw < 0.0 or raw > 1.0:
			_acro_saturated = true
		cmds[i] = clampf(raw, 0.0, 1.0)
	return cmds


## Stick shaping: blend linear -> cubic by acro_expo (soft centre, full ends).
func _rate_curve(x: float) -> float:
	return x * x * x * acro_expo + x * (1.0 - acro_expo)


## Quadratic airframe drag (body FRD): F = -½·rho·CdA(axis)·|v_axis|·v_axis.
func _airframe_drag(v: Vector3) -> Vector3:
	var cda := config.drag_area
	if cda == Vector3.ZERO:
		return Vector3.ZERO
	return -0.5 * RHO * Vector3(
			cda.x * absf(v.x) * v.x,
			cda.y * absf(v.y) * v.y,
			cda.z * absf(v.z) * v.z)


## Rotor H-force: the discs drag against IN-PLANE airflow proportionally to how
## fast the props spin — the dominant translational damping around hover.
func _rotor_drag(v: Vector3, omega_sum: float) -> Vector3:
	if config.rotor_drag_coeff <= 0.0:
		return Vector3.ZERO
	return -config.rotor_drag_coeff * omega_sum * Vector3(v.x, v.y, 0.0)


## Rotor-induced rate damping: flapping/H-force resist body rotation.
func _rotor_damping_moment(omega_sum: float) -> Vector3:
	if config.rate_damping <= 0.0:
		return Vector3.ZERO
	return -config.rate_damping * omega_sum * _omega


## Blade flapping: translational airflow makes the advancing blade lift more,
## tilting the rotor plane away from the motion — a nose-up moment with forward
## speed (and the mirrored roll for sideways flight).
func _flap_moment(v: Vector3, omega_sum: float) -> Vector3:
	if config.flap_moment_coeff <= 0.0:
		return Vector3.ZERO
	return config.flap_moment_coeff * omega_sum * Vector3(-v.y, v.x, 0.0)


## Propwash intensity: descending back into the rotors' own wake (mostly-vertical
## descent, props loaded) destabilises the inflow. Drives the wobble noise state
## (band-limited so it reads as turbulence, not white jitter).
func _update_propwash(h: float, v: Vector3) -> void:
	if config.propwash_moment <= 0.0:
		_propwash = 0.0
		return
	var v_down := v.z                               # FRD: +z = descending
	var v_lat := Vector2(v.x, v.y).length()
	var omega_mean := 0.0
	for m in _motor:
		omega_mean += m
	omega_mean /= maxf(_motor.size(), 1.0)
	var f: float = clampf(v_down / maxf(config.propwash_speed, 0.1), 0.0, 1.0) \
			* clampf(1.0 - v_lat / 6.0, 0.0, 1.0) \
			* clampf(omega_mean * 2.5, 0.0, 1.0)
	_propwash = f
	# ~12 Hz band-limited noise: the shaky, organic part of the wobble.
	var k := 1.0 - exp(-h * TAU * 12.0)
	var white := Vector3(_rng.randf_range(-1, 1), _rng.randf_range(-1, 1), _rng.randf_range(-1, 1))
	_wobble = _wobble.lerp(white, k)


func _propwash_wobble_moment() -> Vector3:
	if _propwash <= 0.0:
		return Vector3.ZERO
	return config.propwash_moment * _propwash * _wobble


## Battery: current draw ~ rotor power (omega³); the pack sags under load
## (I·R) and drains over the flight. Thrust scales with V² about nominal —
## fresh-pack punch, end-of-pack mush.
func _update_battery(h: float) -> void:
	if config.batt_cells <= 0:
		_batt_scale = 1.0
		return
	var power_frac := 0.0
	for m in _motor:
		power_frac += m * m * m
	power_frac /= maxf(_motor.size(), 1.0)
	var amps := config.batt_max_current * power_frac
	_mah_used += amps * h / 3.6                     # A·s -> mAh
	var soc: float = clampf(1.0 - _mah_used / maxf(config.batt_capacity_mah, 1.0), 0.0, 1.0)
	var v_rest := config.batt_cells * lerpf(CELL_EMPTY, CELL_FULL, soc)
	var v := v_rest - amps * config.batt_resistance
	_batt_scale = clampf(pow(v / (config.batt_cells * CELL_NOM), 2.0), 0.3, 1.3)


## In-ground-effect thrust cushion: T x= 1 + gain·(r/4h)², capped +30%.
func _ground_effect_factor() -> float:
	if config.prop_radius <= 0.0:
		return 1.0
	var agl := maxf(_agl(), config.prop_radius)
	var r_over := config.prop_radius / (4.0 * agl)
	return minf(1.0 + config.ground_effect_gain * r_over * r_over * 16.0, 1.3)


## Reaction to rotor angular acceleration: differential spool (rolls, flips, yaw
## punches) kicks the body in yaw — the mechanism behind a quad's snappy yaw.
func _spool_yaw_kick() -> Vector3:
	if config.rotor_inertia <= 0.0:
		return Vector3.ZERO
	var mz := 0.0
	for i in _spool_rate.size():
		mz += _yawdir[i] * config.rotor_inertia * config.rotor_max_speed * _spool_rate[i]
	return Vector3(0.0, 0.0, mz)


## Body moment (N·m): r × thrust from each motor's offset (roll/pitch) plus the
## prop reaction torque about yaw.
func _frame_moment(thrusts: PackedFloat32Array) -> Vector3:
	var m := Vector3.ZERO
	for i in thrusts.size():
		var t := thrusts[i]
		m.x += -_pos[i].y * t                          # roll  = -(py)·T
		m.y += _pos[i].x * t                           # pitch =  (px)·T
		m.z += _yawdir[i] * t * config.yaw_torque_coeff  # yaw reaction
	return m


## Rigid-body angular update with the full inertia tensor + gyroscopic coupling:
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


## Ground contact: rest at gear height, stop downward motion, brake horizontal
## sliding, and detect liftoff / hard touchdown — WITHOUT touching attitude (the
## autopilot owns that). Lifts off naturally once thrust > weight pushes it up.
func _ground_contact(_h: float) -> void:
	var gear_z := _ground_down - spawn_altitude   # NED-down at rest (z is down)
	if _pos_ned.z < gear_z:                        # higher than rest -> airborne
		if _on_ground:
			_on_ground = false
			took_off.emit()
		return
	# At or below gear height: in contact with the ground.
	if not _on_ground:                             # just touched down this step
		var att := GWCoordConvert.dcm_to_ned_attitude(_dcm)
		var slope := acos(clampf(_ground_normal.dot(Vector3(0, 0, -1)), -1.0, 1.0))
		if _vel_ned.z > CRASH_SINK_RATE or absf(att[0]) > CRASH_BANK \
				or absf(att[1]) > CRASH_PITCH or slope > CRASH_SLOPE:
			_enter_crash()
			return
		landed.emit()
	_on_ground = true
	_pos_ned.z = gear_z
	if _vel_ned.z > 0.0:
		_vel_ned.z = 0.0           # gear stops the descent
	_vel_ned.x *= clampf(1.0 - _h * 6.0, 0.0, 1.0)   # feet grip: brake sliding
	_vel_ned.y *= clampf(1.0 - _h * 6.0, 0.0, 1.0)
