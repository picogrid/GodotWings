extends SceneTree

# Rover FDM (GWRoverBody), driven directly with no tree / bridge: settles on its
# wheels, accelerates and tops out like a DC-motor drive, squats/dives, steers
# with the right sign (Ackermann and skid), reverses, slides when pushed past the
# friction circle, survives a small drop and crashes on a big one or a rollover,
# and stays rate-independent. Run:
#   Godot --headless --path . --script res://tests/test_rover.gd

var _ok := true


func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _make(tweak: Callable = Callable()) -> GWRoverBody:
	var r := GWRoverBody.new()
	var cfg: GWRoverConfig = load("res://addons/godotwings/aircraft/Rover4WD.tres").duplicate()
	if tweak.is_valid():
		tweak.call(cfg)
	r.config = cfg
	r.crash_mode = GWVehicleBody.CrashMode.SIMPLE  # no tree: no ragdoll proxy
	r._ready()  # builds wheels + resets (no tree/bridge needed)
	return r


## ArduRover servo layout: ch1 steering (+right), ch3 throttle (+forward), both
## centred at 1500 µs. Skid steer: ch1 = left side, ch3 = right side.
func _drive(r: GWRoverBody, ch1: float, ch3: float) -> void:
	var pwm := PackedInt32Array()
	pwm.resize(16)
	pwm.fill(1500)
	pwm[0] = 1500 + int(roundf(clampf(ch1, -1.0, 1.0) * 500.0))
	pwm[2] = 1500 + int(roundf(clampf(ch3, -1.0, 1.0) * 500.0))
	r._update_controls(pwm)


func _run(r: GWRoverBody, seconds: float, dt := 0.02) -> void:
	for _i in int(seconds / dt):
		r._step(dt)


func _att(r: GWRoverBody) -> Array:
	return GWCoordConvert.dcm_to_ned_attitude(r._dcm)


func _contacts(r: GWRoverBody) -> int:
	var n := 0
	for w in r.wheels:
		if w.in_contact:
			n += 1
	return n


func _initialize() -> void:
	_test_layout()
	_test_rest()
	_test_accelerate_and_top_speed()
	_test_squat_and_dive()
	_test_steering()
	_test_skid_steer()
	_test_reverse()
	_test_sliding()
	_test_drops_and_rollover()
	_test_manual_drive()
	_test_rate_independence()
	print("test_rover: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)


func _test_layout() -> void:
	print("[layout]")
	var r := _make()
	_check(r.wheels.size() == 4, "config builds 4 wheels")
	var cfg := r.config
	var fl := r.wheels[0]
	_check(fl.name == "FL" and fl.hub_rest.x > 0.0 and fl.hub_rest.y < 0.0, "FL is front-left (x>0, y<0)")
	_check(absf(fl.hub_rest.x - (-r.wheels[2].hub_rest.x)) < 1e-6, "centred CG: front/rear symmetric")
	_check(absf(r.spawn_altitude - cfg.cg_height) < 1e-6, "spawn height = CG height at rest (%.2f)" % r.spawn_altitude)
	var total := 0.0
	for w in r.wheels:
		total += w.static_load
	_check(absf(total - cfg.mass * GWVehicleBody.G) < 1e-3, "preloads sum to weight (%.1f N)" % total)
	_check(absf(r.wheels[0].static_load - r.wheels[3].static_load) < 1e-6, "symmetric layout -> equal preloads")
	_check(fl.steers and not r.wheels[2].steers, "front steers, rear does not")
	_check(fl.driven and r.wheels[2].driven, "4WD: all wheels driven")
	_check(fl.spring_k > 0.0 and fl.damper_c > 0.0, "spring/damper sized (k=%.0f c=%.0f)" % [fl.spring_k, fl.damper_c])
	# Nose-heavy CG puts more load on the front axle.
	var heavy := _make(func(c: GWRoverConfig): c.cg_position = 0.7)
	_check(heavy.wheels[0].static_load > heavy.wheels[2].static_load, "forward CG loads the front axle more")
	r.free(); heavy.free()


func _test_rest() -> void:
	print("[rest]")
	var r := _make()
	_drive(r, 0.0, 0.0)
	_run(r, 3.0)
	var att := _att(r)
	var sum_n := 0.0
	for w in r.wheels:
		sum_n += w.normal_force
	_check(r._vel_ned.length() < 0.02, "sits still (v=%.3f m/s)" % r._vel_ned.length())
	_check(absf(-r._pos_ned.z - r.spawn_altitude) < 0.02, "rests at ride height (%.3f vs %.3f)" % [-r._pos_ned.z, r.spawn_altitude])
	_check(absf(att[0]) < 0.01 and absf(att[1]) < 0.01, "level (roll=%.3f pitch=%.3f)" % [att[0], att[1]])
	_check(_contacts(r) == 4, "all 4 wheels in contact")
	_check(absf(sum_n - r.config.mass * GWVehicleBody.G) < 2.0, "wheels carry the weight (%.1f N)" % sum_n)
	_check(r._on_ground and not r._crashed, "on ground, not crashed")
	r.free()


func _test_accelerate_and_top_speed() -> void:
	print("[accelerate / top speed]")
	var r := _make()
	_drive(r, 0.0, 1.0)
	_run(r, 3.0)
	var v3 := r.ground_speed()
	_check(v3 > 2.0, "accelerates under full throttle (%.1f m/s after 3 s)" % v3)
	_check(r._vel_ned.x > 0.0 and absf(r._vel_ned.y) < 0.05 * r._vel_ned.x, "drives straight north on heading 0")
	_run(r, 25.0)
	var vt := r.ground_speed()
	_check(vt > 0.6 * r.config.max_speed and vt <= r.config.max_speed * 1.02,
			"tops out below the no-load speed (%.2f of %.1f m/s)" % [vt, r.config.max_speed])
	_check(_contacts(r) == 4 and r._on_ground, "still on its wheels at speed")
	# Throttle to neutral: neutral brake + rolling resistance stop it.
	_drive(r, 0.0, 0.0)
	_run(r, 8.0)
	_check(absf(r.ground_speed()) < 0.05, "neutral throttle brakes to a stop (%.3f m/s)" % r.ground_speed())
	r.free()


func _test_squat_and_dive() -> void:
	print("[squat / dive]")
	var r := _make()
	_drive(r, 0.0, 0.0)
	_run(r, 2.0)
	_drive(r, 0.0, 1.0)
	_run(r, 0.4)
	var pitch_accel: float = _att(r)[1]
	_check(pitch_accel > 0.005, "launch squats (nose up, pitch=%.3f rad)" % pitch_accel)
	_check(r.wheels[2].normal_force > r.wheels[0].normal_force, "load shifts to the rear under acceleration")
	_run(r, 6.0)
	_drive(r, 0.0, -1.0)   # hard reverse throttle = braking
	_run(r, 0.4)
	var pitch_brake: float = _att(r)[1]
	_check(pitch_brake < -0.005, "braking dives (nose down, pitch=%.3f rad)" % pitch_brake)
	r.free()


func _test_steering() -> void:
	print("[steering]")
	var r := _make()
	_drive(r, 0.0, 0.6)
	_run(r, 3.0)
	_drive(r, 1.0, 0.6)   # full right
	_run(r, 1.5)
	_check(r._omega.z > 0.2, "steer right -> positive yaw rate (r=%.2f rad/s)" % r._omega.z)
	_check(r.wheels[0].steer_angle > 0.0 and r.wheels[1].steer_angle > 0.0, "front wheels turned right")
	_check(r.wheels[1].steer_angle > r.wheels[0].steer_angle, "Ackermann: inner (right) wheel steers tighter")
	_check(absf(r.wheels[2].steer_angle) < 1e-9, "rear wheels straight")
	var roll: float = _att(r)[0]
	_check(roll < -0.005, "body rolls outward (left) in a right turn (roll=%.3f)" % roll)
	_check(r.wheels[0].normal_force > r.wheels[1].normal_force, "outer (left) wheels load up in the turn")
	var yaw_total := 0.0
	var east_max := 0.0
	for _i in 300:
		r._step(0.02)
		yaw_total += r._omega.z * 0.02
		east_max = maxf(east_max, r._pos_ned.y)
	_check(not r._crashed, "full-lock turn at speed slides rather than rolling over")
	_check(east_max > 2.0, "right turn curves east (max E=%.1f m)" % east_max)
	_check(yaw_total > TAU, "keeps circling: %.1f rad of yaw in 6 s" % yaw_total)
	# Mirror: left turn.
	var l := _make()
	_drive(l, 0.0, 0.6)
	_run(l, 3.0)
	_drive(l, -1.0, 0.6)
	_run(l, 1.5)
	_check(l._omega.z < -0.2, "steer left -> negative yaw rate (r=%.2f rad/s)" % l._omega.z)
	# Steady-state turn radius follows the geometry (v / r ≈ wheelbase / tan(δ)).
	var s := _make()
	_drive(s, 0.0, 0.35)
	_run(s, 4.0)
	_drive(s, 0.5, 0.35)
	_run(s, 6.0)
	var radius_meas := absf(s.ground_speed() / s._omega.z)
	var radius_geo := s.config.wheelbase / tan(0.5 * s.config.max_steer_rad())
	_check(absf(radius_meas - radius_geo) / radius_geo < 0.35,
			"gentle turn radius near Ackermann geometry (%.2f m vs %.2f m)" % [radius_meas, radius_geo])
	r.free(); l.free(); s.free()


func _test_skid_steer() -> void:
	print("[skid steer]")
	# Skid steer needs torque beyond the tyres' grip to spin them into a pivot.
	var r := _make(func(c: GWRoverConfig):
		c.steering_mode = GWRoverConfig.SteeringMode.SKID
		c.wheel_torque_max = 25.0)
	_check(not r.wheels[0].steers, "skid: no wheel steers")
	_drive(r, 0.8, 0.8)    # both sides equal -> straight
	_run(r, 3.0)
	_check(r.ground_speed() > 1.5 and absf(r._omega.z) < 0.02, "equal sides drive straight (v=%.1f r=%.3f)" % [r.ground_speed(), r._omega.z])
	_drive(r, 0.8, 0.2)    # left faster than right -> turns right
	_run(r, 2.0)
	_check(r._omega.z > 0.2, "left > right -> turns right (r=%.2f rad/s)" % r._omega.z)
	_drive(r, 1.0, -1.0)   # pivot
	_run(r, 3.0)
	_check(r._omega.z > 0.5 and absf(r.ground_speed()) < 0.3, "opposite sides pivot in place (r=%.2f v=%.2f)" % [r._omega.z, r.ground_speed()])
	var spinning := 0
	for w in r.wheels:
		if w.sliding:
			spinning += 1
	_check(spinning == 4, "pivot: all four tyres scrubbing (%d)" % spinning)
	r.free()


func _test_reverse() -> void:
	print("[reverse]")
	var r := _make()
	_drive(r, 0.0, -0.7)
	_run(r, 3.0)
	_check(r.ground_speed() < -1.0, "negative throttle reverses (%.1f m/s)" % r.ground_speed())
	_check(r.wheels[0].spin_angle < 0.0, "wheels spin backwards in reverse")
	r.free()


func _test_sliding() -> void:
	print("[sliding]")
	var r := _make()
	_drive(r, 0.0, 0.0)
	_run(r, 2.0)
	# Shove it sideways (east) at 6 m/s: tyres can't hold that slip -> friction
	# circle saturates, forces clip to mu_kinetic * N.
	r._vel_ned = Vector3(0.0, 6.0, 0.0)
	r._step(0.02)
	var any_slide := false
	var clipped := true
	for w in r.wheels:
		if w.sliding:
			any_slide = true
			var f := Vector2(w.long_force, w.lat_force).length()
			if f > r.config.mu_kinetic * w.normal_force * 1.001:
				clipped = false
	_check(any_slide, "big lateral slip -> tyres sliding")
	_check(clipped, "sliding force clipped to mu_kinetic * N")
	_run(r, 4.0)
	_check(absf(r._vel_ned.y) < 0.1, "friction scrubs off the slide (vE=%.2f)" % r._vel_ned.y)
	_check(not r.wheels[0].sliding, "grips again once slow")
	# Launch wheelspin: a very torquey motor exceeds grip at standstill.
	var t := _make(func(c: GWRoverConfig): c.wheel_torque_max = 40.0)
	_drive(t, 0.0, 1.0)
	_run(t, 0.5)
	_check(t.wheels[2].sliding, "excess launch torque spins the driven wheels")
	r.free(); t.free()


func _test_drops_and_rollover() -> void:
	print("[drops / rollover]")
	var crashes := [0]
	var landings := [0]
	var r := _make()
	r.crashed.connect(func(): crashes[0] += 1)
	r.landed.connect(func(): landings[0] += 1)
	r.took_off.connect(func(): pass)
	_drive(r, 0.0, 0.0)
	# Small drop (0.3 m): lands, suspension absorbs it.
	r._pos_ned.z -= 0.3
	r._on_ground = false
	_run(r, 3.0)
	_check(landings[0] == 1 and crashes[0] == 0, "0.3 m drop: lands without crashing")
	_check(absf(-r._pos_ned.z - r.spawn_altitude) < 0.03 and _contacts(r) == 4, "settles back on its wheels")
	# Big drop (2 m ≈ 6.3 m/s): a crash.
	r._pos_ned.z -= 2.0
	r._on_ground = false
	_run(r, 2.0)
	_check(crashes[0] == 1, "2 m drop: crashes (%d)" % crashes[0])
	# Rollover: tipped past the limit.
	var t := _make()
	var tips := [0]
	t.crashed.connect(func(): tips[0] += 1)
	_drive(t, 0.0, 0.0)
	t._dcm = GWCoordConvert.attitude_to_dcm(deg_to_rad(80.0), 0.0, 0.0)
	t._step(0.02)
	_check(tips[0] == 1 and t._crashed, "80 deg roll = rollover crash")
	# Tipped only 20 deg: it rights itself on the suspension, no crash.
	var m := _make()
	var m_crash := [0]
	m.crashed.connect(func(): m_crash[0] += 1)
	_drive(m, 0.0, 0.0)
	m._dcm = GWCoordConvert.attitude_to_dcm(deg_to_rad(20.0), 0.0, 0.0)
	m._pos_ned.z -= 0.15
	_run(m, 4.0)
	_check(m_crash[0] == 0 and absf(_att(m)[0]) < 0.02, "20 deg tilt settles level (roll=%.3f)" % _att(m)[0])
	r.free(); t.free(); m.free()


func _test_manual_drive() -> void:
	print("[manual drive]")
	var r := _make()
	r.control_source = GWVehicleBody.ControlSource.MANUAL
	r.manual_drive = true
	# Manual: ch2 (pitch stick, ↑) = throttle, ch1 = steering; ch3 stays at 1500.
	var pwm := PackedInt32Array()
	pwm.resize(16)
	pwm.fill(1500)
	pwm[1] = 2000   # stick forward
	pwm[0] = 1650   # a little right
	r._update_controls(pwm)
	_run(r, 3.0)
	_check(not r._crashed, "manual: drives without crashing")
	_check(r.ground_speed() > 1.0, "manual: pitch stick drives forward (%.1f m/s)" % r.ground_speed())
	_check(r._omega.z > 0.1, "manual: roll stick steers right (r=%.2f)" % r._omega.z)
	# With manual_drive off, the same channels are read the ArduRover way: ch3 is
	# centred = stop, so it should not move.
	var s := _make()
	s.control_source = GWVehicleBody.ControlSource.MANUAL
	s.manual_drive = false
	s._update_controls(pwm)
	_run(s, 2.0)
	_check(absf(s.ground_speed()) < 0.05, "raw layout: centred ch3 = no throttle")
	r.free(); s.free()


func _test_rate_independence() -> void:
	print("[rate independence]")
	var a := _make()
	var b := _make()
	_drive(a, 0.4, 0.8)
	_drive(b, 0.4, 0.8)
	_run(a, 6.0, 0.02)     # 50 Hz host
	_run(b, 6.0, 0.005)    # 200 Hz host
	var d := a._pos_ned.distance_to(b._pos_ned)
	var travelled := a._pos_ned.length()
	_check(d / maxf(travelled, 1.0) < 0.05,
			"trajectory rate-independent (50Hz vs 200Hz differ %.2f m over %.1f m)" % [d, travelled])
	_check(is_finite(a._pos_ned.x) and is_finite(a._omega.z), "state finite")
	a.free(); b.free()
