@icon("res://addons/godotwings/sensors/camera_icon.svg")
class_name GWGroundPTZ
extends Node3D

## Ground-emplaced pan/tilt/zoom camera (a tripod/mast vantage, not attached to
## a vehicle). Reuses GWCamera verbatim for rendering + RTSP publish — this node
## only points GWCamera's mount and adjusts its `fov` for zoom — and adds a
## dumb, newline-delimited JSON control socket so an external process can drive
## pan/tilt/zoom and read pose back. No camera protocol lives here; see the
## README "groundPTZ" section for the exact JSON contract.
##
## Position it in the scene via this node's own transform (drag it in the 3D
## view or set `position` in the Inspector, same as any other node); set
## `reference_heading_deg` to the compass heading pan = 0 should point at.

@export_group("Mount")
## Heading (deg) that pan = 0 points at (added to `pan` before applying yaw).
@export var reference_heading_deg: float = 0.0
## Camera3D.fov (deg) at zoom = 1 (widest). Applied as fov = base_fov / zoom.
@export var base_fov: float = 50.0
## Tilt (pitch) clamp, degrees. Positive = up, negative = down.
@export var tilt_min_deg: float = -90.0
@export var tilt_max_deg: float = 30.0
## Max optical zoom factor (zoom is clamped to [1, zoom_max]).
@export var zoom_max: float = 40.0

@export_group("Continuous motion rates")
## Rate at speed = 100 (% of max), i.e. deg/sec for pan/tilt, zoom-units/sec for zoom.
@export var max_pan_rate_deg: float = 60.0
@export var max_tilt_rate_deg: float = 30.0
@export var max_zoom_rate: float = 8.0
## Continuous motion auto-stops after this long if the request omits "timeout".
@export var default_continuous_timeout: float = 5.0

@export_group("Streaming camera")
## Matches GWCamera's own export — RTSP is the point of groundPTZ, but the
## other protocols work too if you'd rather pull raw RTP/MPEG-TS.
@export var protocol: GWCamera.Protocol = GWCamera.Protocol.RTSP
@export var resolution: Vector2i = Vector2i(1280, 720)
@export var fps: float = 30.0
@export var video_host: String = "127.0.0.1"
@export var video_port: int = 5610
## RTSP path/name this camera publishes to (its own path — distinct from any
## vehicle camera's).
@export var rtsp_url: String = "rtsp://127.0.0.1:8554/groundptz"
@export var ffmpeg_path: String = "ffmpeg"
@export var launch_ffmpeg: bool = true
@export var bitrate_kbps: int = 4000
@export var raw_tcp_port: int = 5568

@export_group("JSON control")
@export var control_host: String = "0.0.0.0"
@export var control_port: int = 8770

@export_group("Telemetry (FMV metadata)")
## Emit one JSON packet per rendered frame over UDP: pose + geodetic position,
## for an external process to fold into FMV metadata (e.g. MISB ST0601 KLV)
## alongside the RTSP/MPEG-TS video. Independent of the JSON control socket
## above — this is a fire-and-forget sidecar, same pattern as GWCamera's own
## `metadata_enabled` (which stays off on the internal GWCamera here, since its
## body-relative pose has no vehicle to be relative to).
@export var metadata_enabled: bool = false
@export var metadata_host: String = "127.0.0.1"
@export var metadata_port: int = 5611
## True WGS84 position of the mount. Purely informational (doesn't affect
## rendering) — it's what lets a KLV/FMV consumer geolocate the sensor.
@export var latitude: float = 0.0
@export var longitude: float = 0.0
@export var altitude_m: float = 0.0

## Current pose — degrees for pan/tilt, factor for zoom. Read-only from the
## outside; drive it through the JSON socket (or _handle_command in-process).
var pan: float = 0.0
var tilt: float = 0.0
var zoom: float = 1.0

var _camera: GWCamera
var _server := TCPServer.new()
var _peers: Array[StreamPeerTCP] = []
var _peer_buffers: Array[String] = []

var _cont_active := false
var _cont_pan_speed := 0.0
var _cont_tilt_speed := 0.0
var _cont_zoom_speed := 0.0
var _cont_time_left := 0.0

var _meta := PacketPeerUDP.new()
var _meta_frame_id := 0
var _meta_accum := 0.0


func _ready() -> void:
	_ensure_camera()
	_set_pose(0.0, 0.0, 1.0)
	if _server.listen(control_port, control_host) != OK:
		push_error("GWGroundPTZ: could not listen on tcp://%s:%d for JSON control." %
				[control_host, control_port])
	else:
		print("GWGroundPTZ: JSON control on tcp://%s:%d" % [control_host, control_port])
	if metadata_enabled:
		_meta.set_dest_address(metadata_host, metadata_port)
		print("GWGroundPTZ: telemetry -> udp://%s:%d" % [metadata_host, metadata_port])


## Add a GWCamera child if one isn't already there (lets a scene author drop
## their own pre-configured GWCamera under this node instead).
func _ensure_camera() -> void:
	for child in get_children():
		if child is GWCamera:
			_camera = child
			return
	var cam := GWCamera.new()
	cam.name = "GWCamera"
	cam.protocol = protocol
	cam.resolution = resolution
	cam.fps = fps
	cam.fov = base_fov
	cam.video_host = video_host
	cam.video_port = video_port
	cam.rtsp_url = rtsp_url
	cam.ffmpeg_path = ffmpeg_path
	cam.launch_ffmpeg = launch_ffmpeg
	cam.bitrate_kbps = bitrate_kbps
	cam.raw_tcp_port = raw_tcp_port
	cam.metadata_enabled = false  # static mount; poll "status" over the JSON socket instead
	add_child(cam)
	_camera = cam


func _physics_process(delta: float) -> void:
	if _cont_active:
		_step_continuous(delta)


func _process(delta: float) -> void:
	_accept_connections()
	_poll_peers()
	if metadata_enabled:
		_meta_accum += delta
		var interval := 1.0 / maxf(fps, 1.0)
		if _meta_accum >= interval:
			_meta_accum -= interval
			_send_metadata()


## One JSON packet per (nominal) rendered frame: pose + geodetic position, for
## an external process to fold into FMV metadata (e.g. MISB ST0601 KLV) keyed
## to the RTSP/MPEG-TS video this same node publishes. `zoom` gives the vertical
## FOV directly (`fov_deg`); `hfov_deg` derives the horizontal FOV from the
## configured resolution's aspect ratio.
func _send_metadata() -> void:
	_meta_frame_id += 1
	var vfov := base_fov / zoom
	var aspect := float(resolution.x) / maxf(float(resolution.y), 1.0)
	var hfov := rad_to_deg(2.0 * atan(tan(deg_to_rad(vfov) * 0.5) * aspect))
	var payload := {
		"frame_id": _meta_frame_id,
		"unix_time": Time.get_unix_time_from_system(),
		"lat": latitude,
		"lon": longitude,
		"alt": altitude_m,
		"heading_deg": wrapf(reference_heading_deg + pan, 0.0, 360.0),  # absolute pointing bearing
		"pan_deg": pan,             # relative to reference_heading_deg (this mount is fixed, so pan IS the sensor-relative azimuth)
		"tilt_deg": tilt,
		"zoom": zoom,
		"vfov_deg": vfov,
		"hfov_deg": hfov,
		"width": resolution.x,
		"height": resolution.y,
	}
	_meta.put_packet(JSON.stringify(payload).to_utf8_buffer())


func _accept_connections() -> void:
	while _server.is_connection_available():
		var peer := _server.take_connection()
		peer.set_no_delay(true)
		_peers.append(peer)
		_peer_buffers.append("")


func _poll_peers() -> void:
	for i in range(_peers.size() - 1, -1, -1):
		var peer := _peers[i]
		peer.poll()
		if peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
			_peers.remove_at(i)
			_peer_buffers.remove_at(i)
			continue
		var avail := peer.get_available_bytes()
		if avail <= 0:
			continue
		var chunk := peer.get_utf8_string(avail)
		_peer_buffers[i] += chunk
		# Newline-delimited: consume every complete line, keep any trailing partial.
		while true:
			var nl := _peer_buffers[i].find("\n")
			if nl < 0:
				break
			var line := _peer_buffers[i].substr(0, nl).strip_edges()
			_peer_buffers[i] = _peer_buffers[i].substr(nl + 1)
			if line.is_empty():
				continue
			var reply := _handle_line(line)
			peer.put_data((JSON.stringify(reply) + "\n").to_utf8_buffer())


func _handle_line(line: String) -> Dictionary:
	var parsed: Variant = JSON.parse_string(line)
	if not (parsed is Dictionary):
		return {"ok": false, "error": "expected a JSON object"}
	return _handle_command(parsed)


## Execute one control request. Exposed as its own function (rather than being
## folded into the socket loop) so it's easy to unit-test headlessly.
func _handle_command(msg: Dictionary) -> Dictionary:
	if not msg.has("cmd"):
		return {"ok": false, "error": "missing 'cmd'"}
	var cmd: Variant = msg["cmd"]
	match cmd:
		"status":
			return _pose_reply()
		"absolute":
			var p = msg.get("pan", pan)
			var t = msg.get("tilt", tilt)
			var z = msg.get("zoom", zoom)
			if not (_is_num(p) and _is_num(t) and _is_num(z)):
				return {"ok": false, "error": "pan/tilt/zoom must be numbers"}
			_stop_continuous()
			_set_pose(float(p), float(t), float(z))
			return _pose_reply()
		"relative":
			var dp = msg.get("rpan", 0.0)
			var dt = msg.get("rtilt", 0.0)
			var dz = msg.get("rzoom", 0.0)
			if not (_is_num(dp) and _is_num(dt) and _is_num(dz)):
				return {"ok": false, "error": "rpan/rtilt/rzoom must be numbers"}
			_stop_continuous()
			_set_pose(pan + float(dp), tilt + float(dt), zoom + float(dz))
			return _pose_reply()
		"continuous":
			var sx = msg.get("pan_speed", 0.0)
			var sy = msg.get("tilt_speed", 0.0)
			var sz = msg.get("zoom_speed", 0.0)
			var to = msg.get("timeout", default_continuous_timeout)
			if not (_is_num(sx) and _is_num(sy) and _is_num(sz) and _is_num(to)):
				return {"ok": false, "error": "pan_speed/tilt_speed/zoom_speed/timeout must be numbers"}
			_cont_pan_speed = clampf(float(sx), -100.0, 100.0)
			_cont_tilt_speed = clampf(float(sy), -100.0, 100.0)
			_cont_zoom_speed = clampf(float(sz), -100.0, 100.0)
			_cont_time_left = maxf(float(to), 0.0)
			_cont_active = true
			return {"ok": true}
		"stop":
			_stop_continuous()
			return _pose_reply()
		_:
			return {"ok": false, "error": "unknown cmd '%s'" % str(cmd)}


func _is_num(v: Variant) -> bool:
	return v is float or v is int


func _pose_reply() -> Dictionary:
	return {"ok": true, "pan": pan, "tilt": tilt, "zoom": zoom}


func _stop_continuous() -> void:
	_cont_active = false
	_cont_pan_speed = 0.0
	_cont_tilt_speed = 0.0
	_cont_zoom_speed = 0.0
	_cont_time_left = 0.0


func _step_continuous(delta: float) -> void:
	_cont_time_left -= delta
	if _cont_time_left <= 0.0:
		_stop_continuous()
		return
	var dp := _cont_pan_speed / 100.0 * max_pan_rate_deg * delta
	var dt := _cont_tilt_speed / 100.0 * max_tilt_rate_deg * delta
	var dz := _cont_zoom_speed / 100.0 * max_zoom_rate * delta
	_set_pose(pan + dp, tilt + dt, zoom + dz)


## Clamp + apply a pose to the mount and push it to GWCamera. Pan wraps (it's a
## full-rotation yaw, e.g. continuous panning past ±180 shouldn't stick at the
## limit); tilt and zoom are hard-clamped (physical travel limits).
func _set_pose(p: float, t: float, z: float) -> void:
	pan = wrapf(p, -180.0, 180.0)
	tilt = clampf(t, tilt_min_deg, tilt_max_deg)
	zoom = clampf(z, 1.0, zoom_max)
	if _camera == null:
		return
	var yaw := deg_to_rad(reference_heading_deg + pan)
	var pitch := deg_to_rad(tilt)
	# Same yaw-then-pitch stacking (and yaw negation) as GWCamera's own servo
	# gimbal: yaw around world-up first, tilt about the now-yawed local right axis.
	_camera.transform = Transform3D(Basis(Vector3.UP, -yaw) * Basis(Vector3.RIGHT, pitch), Vector3.ZERO)
	_camera.fov = base_fov / zoom


func _exit_tree() -> void:
	for peer in _peers:
		peer.disconnect_from_host()
	_peers.clear()
	_peer_buffers.clear()
	_server.stop()
	_meta.close()
