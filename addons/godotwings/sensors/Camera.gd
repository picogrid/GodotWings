@icon("res://addons/godotwings/sensors/camera_icon.svg")
class_name GWCamera
extends Node3D

## Off-screen camera that streams H.264 out of Godot for an external CV pipeline,
## plus a UDP metadata side-channel carrying the camera pose stamped on the SITL
## clock (one packet per frame, correlated by frame_id).
##
## It renders a SubViewport sharing the main World3D, grabs RGBA frames on a timer,
## and hosts a local TCP server that ffmpeg pulls from as a rawvideo client (then
## emits RTP/RTSP/MPEG-TS). The video path reads pose off the GWVehicleBody but is
## otherwise independent of the SITL bridge — it never blocks the physics loop.
##
## Mount it under a GWVehicleBody (or tick `enable_camera` on a facade). This node's
## transform IS the mount: identity looks out the nose, -90° about X looks down.
##
## With `gimbal_enabled`, the camera acts as an ArduPilot servo mount: the autopilot
## resolves the active mount mode into pitch/yaw/roll servo PWM (over the SITL link)
## and the camera follows. See the gimbal exports and the README for the AP params.

enum Protocol {
	RTP_H264,    ## RTP/H.264 to udp://host:port — QGroundControl's native video.
	MPEGTS_H264, ## H.264 in MPEG-TS to udp://host:port — easiest for cv2.VideoCapture("udp://...").
	RTSP,        ## Push H.264 to an RTSP server (e.g. MediaMTX) at rtsp_url.
}

@export var enabled := true
@export var resolution := Vector2i(1280, 720)
@export var fps := 30.0
## Vertical field of view in degrees. Live-updatable (e.g. for zoom): assigning
## after _ready() pushes straight to the render camera, no restart needed.
@export var fov := 70.0:
	set(v):
		fov = v
		if _cam != null:
			_cam.fov = v
## Some platforms hand back a vertically-flipped readback; toggle if the feed is upside-down.
@export var flip_v := false

@export_group("Aircraft")
## GWVehicleBody to read pose from. Empty = auto-find the nearest ancestor.
@export var flight_body_path: NodePath

@export_group("Encoder")
## Path to the ffmpeg binary. "ffmpeg" also triggers a search of common install locations.
@export var ffmpeg_path := "ffmpeg"
## If false, GWCamera only hosts the raw-frame TCP server; you launch your own ffmpeg/pipeline.
@export var launch_ffmpeg := true
@export var bitrate_kbps := 4000
## Local TCP port Godot listens on for ffmpeg to pull raw frames from.
@export var raw_tcp_port := 5566

@export_group("Video out")
@export var protocol := Protocol.RTP_H264
@export var video_host := "127.0.0.1"
@export var video_port := 5600
@export var rtsp_url := "rtsp://127.0.0.1:8554/godotwings"

@export_group("Metadata out")
@export var metadata_enabled := true
@export var metadata_host := "127.0.0.1"
@export var metadata_port := 5601

@export_group("Gimbal (ArduPilot servo mount)")
## Drive the camera from ArduPilot mount servo outputs. ArduPilot side: set
## MNT1_TYPE=1 (Servo) and SERVOn_FUNCTION = 7/6/8 for pitch/yaw/roll, then any
## mount mode (MAVLink/ROI/Home/SysID/RC) points the camera over the SITL link.
@export var gimbal_enabled := false
## SITL channels (1-16) carrying the mount pitch / yaw / roll servo PWM — match
## your SERVOn_FUNCTION assignments. 0 = that axis is fixed.
@export_range(0, 16) var gimbal_pitch_channel := 0
@export_range(0, 16) var gimbal_yaw_channel := 0
@export_range(0, 16) var gimbal_roll_channel := 0
## Angle (deg) the PWM range maps to: x at min PWM, y at max PWM. Match
## MNT1_PITCH_MIN/MAX, MNT1_YAW_MIN/MAX, MNT1_ROLL_MIN/MAX. Swap x/y to invert.
@export var gimbal_pitch_range_deg := Vector2(-90.0, 0.0)
@export var gimbal_yaw_range_deg := Vector2(-180.0, 180.0)
@export var gimbal_roll_range_deg := Vector2(-30.0, 30.0)
## Servo PWM endpoints the angle range spans (SERVOn_MIN..SERVOn_MAX).
@export var gimbal_pwm_min := 1000
@export var gimbal_pwm_max := 2000

var _viewport: SubViewport
var _cam: Camera3D
var _body: GWVehicleBody
var _server := TCPServer.new()
var _peer: StreamPeerTCP
var _meta := PacketPeerUDP.new()
var _ffmpeg_pid := -1
var _frame_id := 0
var _accum := 0.0
var _frame_interval := 1.0 / 30.0


func _ready() -> void:
	if not enabled:
		set_process(false)
		set_physics_process(false)
		return
	_frame_interval = 1.0 / maxf(fps, 1.0)

	# Off-screen render target sharing the main scene's world.
	_viewport = SubViewport.new()
	_viewport.size = resolution
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_viewport.handle_input_locally = false
	add_child(_viewport)
	_viewport.world_3d = get_world_3d()

	_cam = Camera3D.new()
	_cam.fov = fov
	_cam.current = true
	_viewport.add_child(_cam)

	_body = _resolve_body()
	if _body == null and metadata_enabled:
		push_warning("GWCamera: no GWVehicleBody found; metadata pose will be zero.")

	if not _server.listen(raw_tcp_port, "127.0.0.1") == OK:
		push_error("GWCamera: could not listen on TCP %d for raw frames." % raw_tcp_port)
		set_process(false)
		set_physics_process(false)
		return

	if metadata_enabled:
		_meta.set_dest_address(metadata_host, metadata_port)

	if launch_ffmpeg:
		_start_ffmpeg()

	print("GWCamera: %dx%d @%.0ffps, raw frames on tcp://127.0.0.1:%d" %
			[resolution.x, resolution.y, fps, raw_tcp_port])


func _resolve_body() -> GWVehicleBody:
	if not flight_body_path.is_empty():
		return get_node_or_null(flight_body_path) as GWVehicleBody
	var n := get_parent()
	while n != null:
		if n is GWVehicleBody:
			return n
		n = n.get_parent()
	return null


func _find_ffmpeg() -> String:
	if FileAccess.file_exists(ffmpeg_path):
		return ffmpeg_path
	if ffmpeg_path != "ffmpeg":
		return ffmpeg_path  # user gave an explicit (if unverified) path
	for cand in ["/opt/homebrew/bin/ffmpeg", "/usr/local/bin/ffmpeg",
			"/usr/bin/ffmpeg", "/usr/local/sbin/ffmpeg"]:
		if FileAccess.file_exists(cand):
			return cand
	return "ffmpeg"  # last resort: hope it's on PATH


func _ffmpeg_args() -> PackedStringArray:
	var a := PackedStringArray([
		"-loglevel", "warning",
		"-f", "rawvideo",
		"-pixel_format", "rgba",
		"-video_size", "%dx%d" % [resolution.x, resolution.y],
		"-framerate", str(fps),
		"-i", "tcp://127.0.0.1:%d" % raw_tcp_port,
		"-an",
		"-c:v", "libx264",
		"-preset", "ultrafast",
		"-tune", "zerolatency",
		"-bf", "0",                 # no B-frames: nothing reordered/held back
		"-pix_fmt", "yuv420p",
		"-b:v", "%dk" % bitrate_kbps,
		"-g", str(int(maxf(fps, 1.0))),
		# Encoder/mux low-latency: don't accumulate, flush every packet immediately.
		"-fflags", "nobuffer",
		"-flags", "low_delay",
		"-max_delay", "0",
		"-flush_packets", "1",
	])
	match protocol:
		Protocol.RTP_H264:
			a.append_array(["-f", "rtp", "rtp://%s:%d" % [video_host, video_port]])
		Protocol.MPEGTS_H264:
			# muxdelay/muxpreload 0 stops the TS muxer from pre-buffering ~0.7s by default.
			a.append_array(["-muxdelay", "0", "-muxpreload", "0",
					"-f", "mpegts", "udp://%s:%d?pkt_size=1316" % [video_host, video_port]])
		Protocol.RTSP:
			a.append_array(["-rtsp_transport", "tcp", "-f", "rtsp", rtsp_url])
	return a


func _start_ffmpeg() -> void:
	var bin := _find_ffmpeg()
	var args := _ffmpeg_args()
	_ffmpeg_pid = OS.create_process(bin, args, false)
	if _ffmpeg_pid < 0:
		push_error("GWCamera: failed to launch ffmpeg ('%s'). Install ffmpeg or set "
				% bin + "ffmpeg_path / launch_ffmpeg=false. Raw frames still served on tcp://127.0.0.1:%d."
				% raw_tcp_port)
		return
	print("GWCamera: launched ffmpeg (pid %d): %s %s" % [_ffmpeg_pid, bin, " ".join(args)])
	if protocol == Protocol.RTP_H264:
		_write_sdp()


## RTP carries no setup info; write an SDP so ffmpeg-backed clients (OpenCV) can decode.
func _write_sdp() -> void:
	var sdp := "v=0\r\no=- 0 0 IN IP4 %s\r\ns=GodotWings\r\nc=IN IP4 %s\r\nt=0 0\r\n" % [video_host, video_host]
	sdp += "m=video %d RTP/AVP 96\r\na=rtpmap:96 H264/90000\r\n" % video_port
	var path := "user://godotwings_cam.sdp"
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f:
		f.store_string(sdp)
		f.close()
		print("GWCamera: wrote RTP SDP to %s" % ProjectSettings.globalize_path(path))


func _physics_process(_delta: float) -> void:
	# Update the camera in _physics_process (after the body, in tree order) so it
	# locks to the SAME physics state as the model. In _process the two drift by a
	# sub-tick each frame, which reads as the model jittering in the feed.
	if _cam != null:
		_cam.global_transform = global_transform * Transform3D(_gimbal_basis(), Vector3.ZERO)


## Camera-local rotation from the mount servos (identity when off), applied
## yaw→pitch→roll like a pan/tilt head. The servo angles are mechanical
## (body-relative) — ArduPilot already folded in vehicle attitude — so applying
## them in the camera-local frame reproduces the commanded pointing. Yaw and roll
## are negated to match ArduPilot's sense against Godot's camera frame.
func _gimbal_basis() -> Basis:
	if not gimbal_enabled:
		return Basis()
	var pitch := deg_to_rad(_gimbal_axis_deg(gimbal_pitch_channel, gimbal_pitch_range_deg))
	var yaw := deg_to_rad(_gimbal_axis_deg(gimbal_yaw_channel, gimbal_yaw_range_deg))
	var roll := deg_to_rad(_gimbal_axis_deg(gimbal_roll_channel, gimbal_roll_range_deg))
	return Basis(Vector3.UP, -yaw) * Basis(Vector3.RIGHT, pitch) * Basis(Vector3.BACK, -roll)


## Decode one mount servo channel's PWM into an angle (deg) by mapping the
## [gimbal_pwm_min, gimbal_pwm_max] range onto [range.x, range.y].
func _gimbal_axis_deg(channel: int, range_deg: Vector2) -> float:
	if channel < 1 or _body == null:
		return 0.0
	var pwm := _body.control_pwm(channel)
	if pwm <= 0:
		return 0.0  # nothing received on this channel yet
	var span := float(maxi(gimbal_pwm_max - gimbal_pwm_min, 1))
	var t := clampf((pwm - gimbal_pwm_min) / span, 0.0, 1.0)
	return lerpf(range_deg.x, range_deg.y, t)


func _process(delta: float) -> void:
	# Accept ffmpeg's (or a user pipeline's) connection once it dials in.
	if _peer == null and _server.is_connection_available():
		_peer = _server.take_connection()
		_peer.set_no_delay(true)
		print("GWCamera: frame consumer connected.")

	_accum += delta
	if _accum < _frame_interval:
		return
	_accum -= _frame_interval  # keep the remainder so pacing stays on-target
	_grab_and_send()


func _grab_and_send() -> void:
	var tex := _viewport.get_texture()
	if tex == null:
		return
	var img := tex.get_image()
	if img == null:
		return
	if img.get_format() != Image.FORMAT_RGBA8:
		img.convert(Image.FORMAT_RGBA8)
	if flip_v:
		img.flip_y()

	_frame_id += 1

	if _peer != null:
		_peer.poll()
		if _peer.get_status() == StreamPeerTCP.STATUS_CONNECTED:
			if _peer.put_data(img.get_data()) != OK:
				push_warning("GWCamera: frame consumer dropped; awaiting reconnect.")
				_peer = null
		else:
			_peer = null

	if metadata_enabled:
		_send_metadata()


func _send_metadata() -> void:
	var sim_time := 0.0
	var pos_ned := Vector3.ZERO
	var quat := Quaternion.IDENTITY
	if _body != null:
		sim_time = _body._sim_time
		pos_ned = _body._pos_ned
		quat = _body._dcm.get_rotation_quaternion()  # body(FRD) -> NED
	var payload := {
		"frame_id": _frame_id,
		"sim_time": sim_time,
		"pos_ned": [pos_ned.x, pos_ned.y, pos_ned.z],
		"quat_body_to_ned": [quat.w, quat.x, quat.y, quat.z],
		"fov_deg": fov,
		"width": resolution.x,
		"height": resolution.y,
		# Static mount offset from the body origin (this node's own `transform.origin`
		# — e.g. `camera_mount`'s translation), in the SAME Godot render-frame axes
		# (+X body-right, +Y body-up, -Z body-forward) as `mount_basis` below — so a
		# consumer can place the sensor exactly, not just the vehicle CG.
		"mount_pos": [transform.origin.x, transform.origin.y, transform.origin.z],
		# Camera orientation relative to the aircraft body (render frame), including
		# the live gimbal rotation — so CV can recover world pointing as
		# body_basis * mount_basis.
		"mount_basis": _basis_to_array(transform.basis * _gimbal_basis()),
	}
	if gimbal_enabled:
		payload["gimbal_deg"] = [
			_gimbal_axis_deg(gimbal_pitch_channel, gimbal_pitch_range_deg),
			_gimbal_axis_deg(gimbal_yaw_channel, gimbal_yaw_range_deg),
			_gimbal_axis_deg(gimbal_roll_channel, gimbal_roll_range_deg),
		]
	_meta.put_packet(JSON.stringify(payload).to_utf8_buffer())


func _basis_to_array(b: Basis) -> Array:
	return [b.x.x, b.x.y, b.x.z, b.y.x, b.y.y, b.y.z, b.z.x, b.z.y, b.z.z]


func _exit_tree() -> void:
	if _ffmpeg_pid >= 0:
		OS.kill(_ffmpeg_pid)
		_ffmpeg_pid = -1
	if _peer != null:
		_peer.disconnect_from_host()
	_server.stop()
	_meta.close()
