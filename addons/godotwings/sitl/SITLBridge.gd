## Threaded UDP bridge implementing the ArduPilot JSON SITL protocol.
##
## ArduPilot -> Godot : binary packet (PWM), UDP port 9002.
## Godot -> ArduPilot : JSON text state, replied to the sender.
##
## PROTOCOL IS LOCKSTEP: ArduPilot blocks until it receives our state reply, so
## exactly one physics step must be produced per received PWM packet. A dedicated
## I/O thread touches only the socket and mutex-guarded buffers (never the scene
## tree); the physics owner drives the handshake on the main thread via
## has_command() / take_command() / post_state() — see GWVehicleBody._physics_process.
class_name GWSITLBridge
extends Node

const MAGIC := 18458          ## validates inbound packets (0x481A). NOTE: the
                              ## design brief said 29569, but the real ArduPilot
                              ## JSON servo packet (verified on ArduPlane 4.5.7)
                              ## uses 18458.
const PACKET_SIZE := 40       ## 2+2+4 + 16*2 bytes
const LISTEN_PORT := 9002

## Emitted (deferred, on main thread) the first time a valid packet arrives.
signal connected(ip: String, port: int)

@export var listen_port: int = LISTEN_PORT

var _thread: Thread
var _mutex: Mutex
var _state_sem: Semaphore        # main posts after producing a reply for a cmd
var _running := false

# --- shared state (guard with _mutex) ----------------------------------------
var _has_new_cmd := false
var _in_pwm := PackedInt32Array()
var _in_frame_count := 0
var _sender_ip := ""
var _sender_port := 0
var _out_json := PackedByteArray()
var _ever_connected := false

# main-thread-only
var _last_frame_count := -1


func _ready() -> void:
	_mutex = Mutex.new()
	_state_sem = Semaphore.new()
	_in_pwm.resize(16)
	_running = true
	_thread = Thread.new()
	_thread.start(_sitl_loop)


func _exit_tree() -> void:
	_running = false
	# Unblock the thread if it is parked waiting for a state reply.
	if _state_sem:
		_state_sem.post()
	if _thread and _thread.is_started():
		_thread.wait_to_finish()


## True if a freshly received PWM command is waiting to be processed.
func has_command() -> bool:
	_mutex.lock()
	var v := _has_new_cmd
	_mutex.unlock()
	return v


## Consume the pending command. Returns a Dictionary:
##   pwm          : PackedInt32Array(16), microseconds
##   aileron/elevator/throttle/rudder : normalized floats
##   frame_count  : int
##   reset        : bool — true if ArduPilot restarted (frame_count went backwards)
## Must be paired with exactly one [method post_state] call.
func take_command() -> Dictionary:
	_mutex.lock()
	var pwm := _in_pwm.duplicate()
	var fc := _in_frame_count
	_has_new_cmd = false
	_mutex.unlock()

	var reset := fc < _last_frame_count
	_last_frame_count = fc

	return {
		"pwm": pwm,
		"aileron": (pwm[0] - 1500.0) / 500.0,
		"elevator": (pwm[1] - 1500.0) / 500.0,
		"throttle": (pwm[2] - 1000.0) / 1000.0,
		"rudder": (pwm[3] - 1500.0) / 500.0,
		"frame_count": fc,
		"reset": reset,
	}


## Provide the post-step aircraft state and unblock the thread to reply.
## `state` must contain: timestamp, gyro(Vector3 frd), accel_body(Vector3 frd),
## position(Vector3 ned), attitude([roll,pitch,yaw]), velocity(Vector3 ned).
## Optional: airspeed, wind(Dictionary), battery(Dictionary).
func post_state(state: Dictionary) -> void:
	var json := _build_json(state)
	_mutex.lock()
	_out_json = json
	_mutex.unlock()
	_state_sem.post()


# -----------------------------------------------------------------------------
# Thread: socket I/O only. Never touch the scene tree from here.
# -----------------------------------------------------------------------------
func _sitl_loop() -> void:
	var udp := PacketPeerUDP.new()
	var err := udp.bind(listen_port)
	if err != OK:
		push_error("GWSITLBridge: failed to bind UDP port %d (err %d)" % [listen_port, err])
		return

	while _running:
		if udp.get_available_packet_count() <= 0:
			OS.delay_msec(1)
			continue

		var packet := udp.get_packet()
		var parsed := _parse_binary(packet)
		if parsed.is_empty():
			continue # bad magic / short packet — ignore

		var ip := udp.get_packet_ip()
		var port := udp.get_packet_port()

		_mutex.lock()
		_in_pwm = parsed["pwm"]
		_in_frame_count = parsed["frame_count"]
		_sender_ip = ip
		_sender_port = port
		_has_new_cmd = true
		var first := not _ever_connected
		_ever_connected = true
		_mutex.unlock()

		if first:
			# Cross-thread: hop to the main thread before emitting.
			call_deferred("emit_signal", "connected", ip, port)

		# Block until the main thread produces the reply for THIS command.
		_state_sem.wait()
		if not _running:
			break

		_mutex.lock()
		var reply := _out_json
		var dip := _sender_ip
		var dport := _sender_port
		_mutex.unlock()

		udp.set_dest_address(dip, dport)
		udp.put_packet(reply)

	udp.close()


## Parse the inbound binary PWM packet. Returns {} on invalid input.
func _parse_binary(packet: PackedByteArray) -> Dictionary:
	if packet.size() < PACKET_SIZE:
		return {}
	var spb := StreamPeerBuffer.new()
	spb.data_array = packet
	spb.big_endian = false
	var magic := spb.get_u16()
	if magic != MAGIC:
		return {}
	var _frame_rate := spb.get_u16()
	var frame_count := spb.get_u32()
	var pwm := PackedInt32Array()
	pwm.resize(16)
	for i in 16:
		pwm[i] = spb.get_u16()
	return {"frame_count": frame_count, "pwm": pwm}


## Serialize aircraft state to an ArduPilot JSON state packet.
func _build_json(s: Dictionary) -> PackedByteArray:
	var gyro: Vector3 = s["gyro"]
	var accel: Vector3 = s["accel_body"]
	var pos: Vector3 = s["position"]
	var att: Array = s["attitude"]
	var vel: Vector3 = s["velocity"]

	var data := {
		"timestamp": s["timestamp"],
		"imu": {
			"gyro": [gyro.x, gyro.y, gyro.z],
			"accel_body": [accel.x, accel.y, accel.z],
		},
		"position": [pos.x, pos.y, pos.z],
		"attitude": [att[0], att[1], att[2]],
		"velocity": [vel.x, vel.y, vel.z],
	}
	if s.has("airspeed"):
		data["airspeed"] = s["airspeed"]
	if s.has("wind"):
		data["windvane"] = s["wind"]
	if s.has("battery"):
		data["battery"] = s["battery"]
	if s.has("rangefinder"):
		# ArduPilot JSON backend optional field: SITL rangefinder instance 1
		# (meters). Feeds RNGFND1_TYPE=100 -> EK3 optical-flow height scaling.
		data["rng_1"] = s["rangefinder"]

	# Trailing newline so ArduPilot reliably detects the end of the frame.
	return (JSON.stringify(data) + "\n").to_utf8_buffer()
