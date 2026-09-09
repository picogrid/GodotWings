extends SceneTree

## Lockstep pacing. ArduPilot blocks until we reply, and its next PWM packet
## lands about a round trip after we post state — but Godot runs a frame's
## physics ticks back to back. Without waiting for that packet inside the tick,
## every tick after the first finds nothing and is skipped, the exchange rate
## collapses to one per RENDERED frame, and sim time runs at
## render_fps / control_rate_hz of realtime (measured 0.36x at 144 fps and
## 400 Hz; the copter raises the tick rate to 400).

var _ok := true


func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


## A command source shaped like GWSITLBridge, whose packet "arrives" only while
## wait_command is waiting — exactly the case a tick burst misses.
class LateSource:
	var pending := false
	var wait_calls := 0
	var posted := 0

	func has_command() -> bool:
		return pending

	func wait_command(_timeout_usec: int = 4000) -> bool:
		wait_calls += 1
		pending = true
		return true

	func take_command() -> Dictionary:
		pending = false
		var pwm := PackedInt32Array()
		pwm.resize(16)
		pwm.fill(1500)
		pwm[2] = 1000
		return {"pwm": pwm, "aileron": 0.0, "elevator": 0.0, "throttle": 0.0,
				"rudder": 0.0, "frame_count": posted, "reset": false}

	func post_state(_state: Dictionary) -> void:
		posted += 1


## The old contract: no wait_command at all (GWManualInput), so a tick with no
## command is simply skipped.
class NoWaitSource extends LateSource:
	func wait_command(_timeout_usec: int = 4000) -> bool:
		push_error("wait_command must not be called on a source without it")
		return false


func _initialize() -> void:
	_test_every_tick_completes_an_exchange()
	_test_source_without_wait_is_unchanged()
	_test_wait_command_guards()
	print("test_sitl_pacing: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)


func _make_body() -> GWFlightBody:
	var fb := GWFlightBody.new()
	fb.config = load("res://addons/godotwings/aircraft/Skywalker.tres")
	fb._ready()
	return fb


func _test_every_tick_completes_an_exchange() -> void:
	print("[tick burst]")
	var fb := _make_body()
	var source := LateSource.new()
	fb._source = source
	# A burst of ticks, as Godot runs them at frame start with nothing pending.
	for _i in 20:
		fb._physics_process(1.0 / 400.0)
	_check(source.posted == 20, "every tick in a burst completed an exchange (%d of 20)" % source.posted)
	_check(source.wait_calls == 20, "each waited for its own packet (%d)" % source.wait_calls)
	_check(fb._sim_time > 0.0, "sim time advanced (%.4f s)" % fb._sim_time)
	fb.free()


func _test_source_without_wait_is_unchanged() -> void:
	print("[source without wait_command]")
	var fb := _make_body()
	var source := NoWaitSource.new()
	fb._source = source
	for _i in 5:
		fb._physics_process(1.0 / 400.0)
	_check(source.posted == 0, "a tick with no command is skipped, not waited on (%d)" % source.posted)
	source.pending = true
	fb._physics_process(1.0 / 400.0)
	_check(source.posted == 1, "a pending command is still consumed")
	fb.free()


## The waits must cost nothing when there is no autopilot: a scene with the
## bridge present but SITL never started, or stopped mid-run, must not stall
## 4 ms per tick (at 400 Hz that would be most of the frame).
func _test_wait_command_guards() -> void:
	print("[guards]")
	var bridge := GWSITLBridge.new()
	bridge._mutex = Mutex.new()
	bridge._state_sem = Semaphore.new()

	var start := Time.get_ticks_usec()
	var never := bridge.wait_command()
	var never_usec := Time.get_ticks_usec() - start
	_check(not never and never_usec < 1000,
			"never connected: returns false at once (%d us)" % never_usec)

	# Connected, but the last packet was long ago: paused or killed.
	bridge._ever_connected = true
	bridge._last_rx_usec = Time.get_ticks_usec() - 600_000
	start = Time.get_ticks_usec()
	var quiet := bridge.wait_command()
	var quiet_usec := Time.get_ticks_usec() - start
	_check(not quiet and quiet_usec < 1000,
			"gone quiet: returns false at once (%d us)" % quiet_usec)

	# A command already in hand needs no wait at all.
	bridge._has_new_cmd = true
	start = Time.get_ticks_usec()
	var ready := bridge.wait_command()
	_check(ready and Time.get_ticks_usec() - start < 500, "a pending command returns immediately")

	# Live but nothing yet: waits, and no longer than its budget.
	bridge._has_new_cmd = false
	bridge._last_rx_usec = Time.get_ticks_usec()
	start = Time.get_ticks_usec()
	var timed_out := bridge.wait_command(2000)
	var waited := Time.get_ticks_usec() - start
	_check(not timed_out and waited >= 1500 and waited < 6000,
			"a live link with no packet waits its budget and gives up (%d us)" % waited)
	bridge.free()
