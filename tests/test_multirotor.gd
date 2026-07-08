extends SceneTree

# Quadcopter FDM (GWMultirotorBody): motor mixing produces the right body moments,
# full thrust climbs and stays upright, below-hover stays grounded, and the fixed
# sub-stepping keeps it rate-independent. (Absolute yaw-reaction sign vs ArduCopter
# is validated in SITL; here we check relative correctness + dynamics.)

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


## `matched` zeroes the per-motor thrust variance: these tests drive raw motors
## open-loop (no FC correcting), where any real-world motor mismatch tips the
## frame over — exactly as it would a real quad without a flight controller.
func _make(matched := true) -> GWMultirotorBody:
	var q := GWMultirotorBody.new()
	var cfg: GWMultirotorConfig = load("res://addons/godotwings/aircraft/QuadX.tres").duplicate()
	if matched:
		cfg.motor_variance = 0.0
	q.config = cfg
	q._ready()  # builds frame + reset (no tree/bridge needed)
	return q


func _set4(q: GWMultirotorBody, a: float, b: float, c: float, d: float) -> void:
	var pwm := PackedInt32Array()
	pwm.resize(16)
	pwm.fill(1000)
	pwm[0] = 1000 + int(a * 1000); pwm[1] = 1000 + int(b * 1000)
	pwm[2] = 1000 + int(c * 1000); pwm[3] = 1000 + int(d * 1000)
	q._update_controls(pwm)


## AETR stick channels for MANUAL acro mode (0.5 = centered, throttle 0..1).
func _set_aetr(q: GWMultirotorBody, roll: float, pitch: float, thr: float, yaw: float) -> void:
	_set4(q, roll, pitch, thr, yaw)


func _fly_alt(q: GWMultirotorBody, seconds: float, dt: float) -> float:
	var steps := int(seconds / dt)
	for _i in steps:
		q._step(dt)
	return -q._pos_ned.z


func _initialize() -> void:
	var q := _make()

	# --- motor mixing -> body moment (pure, deterministic) ---
	_check(q._frame_moment(PackedFloat32Array([1, 1, 1, 1])).length() < 1e-5,
		"equal thrust -> no moment")
	# motors 1 (front-right) + 3 (front-left) = front
	_check(q._frame_moment(PackedFloat32Array([1, 0, 1, 0])).y > 0.0,
		"front motors -> +pitch (nose up)")
	# motors 1 (front-right) + 4 (back-right) = right; right-heavy rolls left (m.x<0)
	_check(q._frame_moment(PackedFloat32Array([1, 0, 0, 1])).x < 0.0,
		"right motors -> roll left (-m.x)")
	# motors 1,2 = CCW (+yaw reaction); 3,4 = CW (-yaw reaction)
	_check(q._frame_moment(PackedFloat32Array([1, 1, 0, 0])).z > 0.0, "CCW props -> +yaw reaction")
	_check(q._frame_moment(PackedFloat32Array([0, 0, 1, 1])).z < 0.0, "CW props -> -yaw reaction")

	# --- full throttle climbs and stays upright (acro FC holding zero rates:
	# with blade flapping modelled the frame is open-loop unstable, like a real
	# quad — a controller must close the loop; sticks centered, throttle high) ---
	var climb := _make(false)  # even with per-motor variance: the FC corrects it
	climb.control_source = GWVehicleBody.ControlSource.MANUAL
	_set_aetr(climb, 0.5, 0.5, 0.8, 0.5)
	var alt := _fly_alt(climb, 3.0, 0.0025)
	var att := GWCoordConvert.dcm_to_ned_attitude(climb._dcm)
	_check(alt > 5.0, "full throttle climbs (alt=%.1f m)" % alt)
	_check(absf(att[0]) < 0.1 and absf(att[1]) < 0.1, "acro FC keeps it level while climbing")
	_check(is_finite(climb._pos_ned.z) and is_finite(att[2]), "state finite")

	# --- below hover stays on the ground ---
	var grounded := _make()
	_set4(grounded, 0.15, 0.15, 0.15, 0.15)  # 0.15 < ~0.2 hover (5:1 TWR) -> ground hold
	var galt := _fly_alt(grounded, 2.0, 0.01)
	_check(absf(galt - grounded.spawn_altitude) < 0.05, "below-hover stays grounded (alt=%.2f)" % galt)

	# --- pitch authority: front motors > back -> pitches nose-up. Sampled early:
	# the snappy default has very low inertia, so a held differential tumbles fast.
	var pitcher := _make()
	_set4(pitcher, 0.6, 0.4, 0.6, 0.4)  # 1,3 front high; 2,4 back low
	_fly_alt(pitcher, 0.1, 0.005)
	var pitch: float = GWCoordConvert.dcm_to_ned_attitude(pitcher._dcm)[1]
	_check(pitch > 0.05 and pitch < 1.5, "front-heavy pitches nose up (pitch=%.2f)" % pitch)

	# --- FPV physics: quadratic thrust curve puts hover near sqrt(1/TWR) ---
	# Just below hover omega sinks, just above climbs (start airborne at 50 m).
	var twr: float = (_make().config as GWMultirotorConfig).thrust_to_weight
	var hov := sqrt(1.0 / twr)
	var lo := _make(); lo._pos_ned.z = -50.0; lo._on_ground = false
	_set4(lo, hov - 0.05, hov - 0.05, hov - 0.05, hov - 0.05)
	var lo_alt := _fly_alt(lo, 2.0, 0.01)
	var hi := _make(); hi._pos_ned.z = -50.0; hi._on_ground = false
	_set4(hi, hov + 0.05, hov + 0.05, hov + 0.05, hov + 0.05)
	var hi_alt := _fly_alt(hi, 2.0, 0.01)
	_check(lo_alt < 50.0 and hi_alt > 50.0,
		"hover sits near sqrt(1/TWR)=%.2f stick (below:%.1fm above:%.1fm)" % [hov, lo_alt, hi_alt])

	# --- inflow washout: a full-throttle punch-out reaches a terminal climb
	# (through the acro FC so the open-loop flap instability stays corrected) ---
	var punch := _make(); punch._pos_ned.z = -10.0; punch._on_ground = false
	punch.control_source = GWVehicleBody.ControlSource.MANUAL
	_set_aetr(punch, 0.5, 0.5, 1.0, 0.5)
	_fly_alt(punch, 6.0, 0.0025)
	var vclimb := -punch._vel_ned.z
	_check(vclimb > 15.0 and vclimb < 45.0,
		"punch-out tops out (terminal climb %.1f m/s)" % vclimb)

	# --- acro FC: full right stick tracks the configured roll rate ---
	var acro := _make(); acro._pos_ned.z = -50.0; acro._on_ground = false
	acro.control_source = GWVehicleBody.ControlSource.MANUAL
	_set_aetr(acro, 1.0, 0.5, 0.45, 0.5)
	for _i in 160:  # 0.4 s at 400 Hz
		acro._step(0.0025)
	var want := deg_to_rad(acro.acro_max_rate_dps)
	var got := acro._omega.x
	_check(absf(got - want) < want * 0.25,
		"acro tracks commanded roll rate (%.0f of %.0f deg/s)" % [rad_to_deg(got), rad_to_deg(want)])

	# --- ground effect: extra thrust in the cushion near the ground ---
	var ge_cfg: GWMultirotorConfig = load("res://addons/godotwings/aircraft/QuadX.tres")
	var ge_lo := _make(); ge_lo._pos_ned.z = -ge_cfg.prop_radius  # skimming
	ge_lo._on_ground = false
	var f_lo: float = ge_lo._ground_effect_factor()
	var ge_hi := _make(); ge_hi._pos_ned.z = -50.0; ge_hi._on_ground = false
	var f_hi: float = ge_hi._ground_effect_factor()
	_check(f_lo > 1.1 and absf(f_hi - 1.0) < 0.01,
		"ground effect cushions low hover (low x%.2f, high x%.2f)" % [f_lo, f_hi])

	# --- battery: sustained full throttle sags the pack (thrust scale drops) ---
	var batt := _make(); batt._pos_ned.z = -500.0; batt._on_ground = false
	batt.control_source = GWVehicleBody.ControlSource.MANUAL
	_set_aetr(batt, 0.5, 0.5, 1.0, 0.5)
	var fresh: float = -1.0
	for _i in 40 * 400:  # 40 s at 400 Hz
		batt._step(0.0025)
		if fresh < 0.0:
			fresh = batt._batt_scale
	_check(fresh > 1.1 and batt._batt_scale < fresh - 0.2,
		"battery sags under load (fresh x%.2f -> x%.2f)" % [fresh, batt._batt_scale])

	# --- propwash: descending into the wake near hover shakes the frame ---
	var wash := _make(); wash._pos_ned.z = -100.0; wash._on_ground = false
	wash.control_source = GWVehicleBody.ControlSource.MANUAL
	wash._vel_ned = Vector3(0, 0, 3.5)  # sinking 3.5 m/s
	_set_aetr(wash, 0.5, 0.5, sqrt(1.0 / twr), 0.5)  # ~hover throttle
	for _i in 200:
		wash._step(0.0025)
	_check(wash._propwash > 0.3 and wash._propwash_wobble_moment().length() > 0.0,
		"propwash active in descent (intensity %.2f)" % wash._propwash)

	# --- quadratic drag: a throttle-cut fall reaches a sane terminal velocity ---
	var faller := _make(); faller._pos_ned.z = -2000.0; faller._on_ground = false
	_set4(faller, 0.0, 0.0, 0.0, 0.0)
	_fly_alt(faller, 12.0, 0.005)
	var vfall := faller._vel_ned.z
	_check(vfall > 10.0 and vfall < 60.0,
		"throttle-cut fall has terminal velocity (%.1f m/s)" % vfall)

	# --- rate independence: climb similar at 50 Hz vs 200 Hz ---
	var a50 := _make(); _set4(a50, 0.8, 0.8, 0.8, 0.8)
	var a200 := _make(); _set4(a200, 0.8, 0.8, 0.8, 0.8)
	var alt50 := _fly_alt(a50, 2.0, 0.02)
	var alt200 := _fly_alt(a200, 2.0, 0.005)
	var rel := absf(alt50 - alt200) / maxf(alt50, 1.0)
	_check(rel < 0.15, "altitude rate-independent (50Hz=%.1f 200Hz=%.1f, %.1f%%)" % [alt50, alt200, rel * 100.0])

	print("test_multirotor: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
