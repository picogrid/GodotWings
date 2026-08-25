extends SceneTree

# Headless smoke test for GWGroundPTZ: command handling (clamp/wrap, relative,
# continuous, stop, errors) plus the JSON socket end-to-end over a real
# StreamPeerTCP client. Rendering can't run under --headless, so this only
# exercises pose math + the control protocol, not the video path.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _approx(a: float, b: float, eps := 1e-4) -> bool:
	return absf(a - b) < eps


func _initialize() -> void:
	var ptz := GWGroundPTZ.new()
	ptz.launch_ffmpeg = false      # no ffmpeg on CI; camera child still gets created
	ptz.raw_tcp_port = 5599
	ptz.control_port = 8799
	ptz.tilt_min_deg = -90.0
	ptz.tilt_max_deg = 30.0
	ptz.zoom_max = 40.0
	ptz.metadata_enabled = true
	ptz.metadata_port = 5711
	ptz.latitude = 51.5
	ptz.longitude = -0.1
	ptz.altitude_m = 42.0
	get_root().add_child(ptz)

	# _ready() (and so the auto-created GWCamera child) lands on the next tick,
	# not synchronously. Frame-grab needs a real GPU texture readback, which the
	# dummy headless rasterizer can't provide (see test_camera.gd) — not what
	# this test is about — so stop it as soon as the camera exists, before the
	# many awaited frames below give it a chance to fire.
	while ptz._camera == null:
		await process_frame
	ptz._camera.set_process(false)

	await process_frame
	await process_frame

	_check(ptz._camera != null, "GWCamera child auto-created")
	_check(ptz._server.is_listening(), "JSON control server listening")
	_check(_approx(ptz.pan, 0.0) and _approx(ptz.tilt, 0.0) and _approx(ptz.zoom, 1.0),
			"initial pose is (0, 0, 1)")

	# --- direct command handling: clamp + wrap ---
	var r := ptz._handle_command({"cmd": "absolute", "pan": 200.0, "tilt": 100.0, "zoom": 999.0})
	_check(r["ok"] == true, "absolute out-of-range accepted")
	_check(_approx(r["pan"], -160.0), "pan 200 wraps to -160 (%.1f)" % r["pan"])
	_check(_approx(r["tilt"], 30.0), "tilt 100 clamps to tilt_max 30 (%.1f)" % r["tilt"])
	_check(_approx(r["zoom"], 40.0), "zoom 999 clamps to zoom_max 40 (%.1f)" % r["zoom"])
	_check(_approx(ptz._camera.fov, ptz.base_fov / 40.0), "fov = base_fov / zoom pushed to GWCamera")

	# --- absolute with partial fields leaves the rest unchanged ---
	ptz._handle_command({"cmd": "absolute", "pan": 0.0, "tilt": 0.0, "zoom": 1.0})
	r = ptz._handle_command({"cmd": "absolute", "pan": 10.0})
	_check(_approx(r["pan"], 10.0) and _approx(r["tilt"], 0.0) and _approx(r["zoom"], 1.0),
			"absolute with only 'pan' leaves tilt/zoom unchanged")

	# --- relative deltas ---
	r = ptz._handle_command({"cmd": "relative", "rpan": 5.0, "rtilt": -5.0, "rzoom": 0.5})
	_check(_approx(r["pan"], 15.0) and _approx(r["tilt"], -5.0) and _approx(r["zoom"], 1.5),
			"relative applies deltas onto current pose")

	# --- continuous motion integrates over physics ticks, then times out ---
	ptz._handle_command({"cmd": "absolute", "pan": 0.0, "tilt": 0.0, "zoom": 1.0})
	r = ptz._handle_command({"cmd": "continuous", "pan_speed": 100.0, "timeout": 0.5})
	_check(r["ok"] == true and not r.has("pan"), "continuous replies bare {ok:true}")
	ptz._physics_process(0.2)
	_check(_approx(ptz.pan, 0.2 * ptz.max_pan_rate_deg, 1e-2),
			"continuous pan_speed=100 moves at max_pan_rate_deg/sec (%.2f)" % ptz.pan)
	ptz._physics_process(0.4)  # exceeds the 0.5s timeout -> auto-stop
	_check(not ptz._cont_active, "continuous motion auto-stops after its timeout")

	# --- stop halts an in-flight continuous move ---
	ptz._handle_command({"cmd": "continuous", "pan_speed": -100.0, "timeout": 5.0})
	r = ptz._handle_command({"cmd": "stop"})
	_check(r["ok"] == true and not ptz._cont_active, "stop halts continuous motion")

	# --- errors ---
	r = ptz._handle_command({"cmd": "bogus"})
	_check(r["ok"] == false and r.has("error"), "unknown cmd -> ok:false + error")
	r = ptz._handle_command({})
	_check(r["ok"] == false and r.has("error"), "missing cmd -> ok:false + error")
	r = ptz._handle_command({"cmd": "absolute", "pan": "north"})
	_check(r["ok"] == false, "non-numeric field -> ok:false")

	# --- telemetry sidecar: one JSON packet/frame over UDP, pose + geodetic ---
	var telemetry := PacketPeerUDP.new()
	_check(telemetry.bind(5711, "127.0.0.1") == OK, "telemetry: bound a listener on 5711")
	ptz._handle_command({"cmd": "absolute", "pan": 30.0, "tilt": -10.0, "zoom": 2.0})
	var got_telemetry := false
	for i in 20:
		await process_frame
		if telemetry.get_available_packet_count() > 0:
			var pkt: Variant = JSON.parse_string(telemetry.get_packet().get_string_from_utf8())
			if pkt is Dictionary:
				got_telemetry = true
				_check(_approx(pkt.get("lat", -999.0), 51.5) and _approx(pkt.get("lon", -999.0), -0.1)
						and _approx(pkt.get("alt", -999.0), 42.0), "telemetry carries the configured lat/lon/alt")
				_check(_approx(pkt.get("pan_deg", -999.0), 30.0) and _approx(pkt.get("tilt_deg", -999.0), -10.0)
						and _approx(pkt.get("zoom", -999.0), 2.0), "telemetry carries the live pan/tilt/zoom")
				_check(_approx(pkt.get("vfov_deg", -999.0), ptz.base_fov / 2.0), "telemetry vfov_deg = base_fov / zoom")
				break
	_check(got_telemetry, "telemetry: at least one packet received")
	telemetry.close()

	# --- real socket round-trip ---
	var client := StreamPeerTCP.new()
	_check(client.connect_to_host("127.0.0.1", 8799) == OK, "client dials JSON control port")
	for i in 5:
		await process_frame
		client.poll()
	client.put_data(JSON.stringify({"cmd": "status"}).to_utf8_buffer() + "\n".to_utf8_buffer())
	var reply_line := ""
	for i in 10:
		await process_frame
		client.poll()
		if client.get_available_bytes() > 0:
			reply_line += client.get_utf8_string(client.get_available_bytes())
			if reply_line.find("\n") >= 0:
				break
	var reply: Variant = JSON.parse_string(reply_line.strip_edges())
	_check(reply is Dictionary and reply.get("ok") == true and reply.has("pan"),
			"socket round-trip: status -> {ok:true,pan,tilt,zoom} (%s)" % reply_line.strip_edges())

	client.disconnect_from_host()
	ptz.queue_free()
	await process_frame

	print("test_ground_ptz: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
