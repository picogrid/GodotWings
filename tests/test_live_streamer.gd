extends SceneTree

# Network-free integration coverage for the live worker. Tileset metadata and
# glTF sentinel bytes are synthetic; planning, queues, residency, and promotion
# use the production implementation.

const ROOT_URL := "https://synthetic.test/root.json"
var GLTF_SENTINEL := PackedByteArray([0x67, 0x6c, 0x54, 0x46])


class SyntheticStreamer extends GWTiles3DStreamer:
	var responses: Dictionary = {}
	var request_counts: Dictionary = {}
	var expected_sessions: Dictionary = {}
	var request_urls: Array[String] = []

	func _ready() -> void:
		_mutex = Mutex.new()
		_wake_sem = Semaphore.new()
		_running = true

	func _exit_tree() -> void:
		if _mutex == null:
			return
		_mutex.lock()
		_running = false
		_mutex.unlock()

	func _http_get(url: String, _retried: bool = false) -> Dictionary:
		var key := url.split("?", true, 1)[0]
		request_urls.append(url)
		if expected_sessions.has(key):
			var expected := "session=" + String(expected_sessions[key])
			var query := url.split("?", true, 1)
			if query.size() < 2 or expected not in String(query[1]).split("&"):
				return {"ok": false, "status": 403}
		request_counts[key] = int(request_counts.get(key, 0)) + 1
		var response_key := url if responses.has(url) else key
		if not responses.has(response_key):
			return {"ok": false, "status": 404}
		var response = responses[response_key]
		if response is Dictionary and response.has("__status"):
			return {"ok": false, "status": int(response["__status"])}
		if response is PackedByteArray:
			return {"ok": true, "status": 200, "body": response}
		return {
			"ok": true,
			"status": 200,
			"body": JSON.stringify(response).to_utf8_buffer(),
		}

	func _place_new_tile(tile: Dictionary) -> void:
		# GLB decoding has its own live check. Keep the real admission and
		# placement state machine, replacing only the renderer-facing decode.
		var wrapper := Node3D.new()
		var visible_mesh := MeshInstance3D.new()
		wrapper.add_child(visible_mesh)
		add_child(wrapper)
		_accept_placed_tile(tile, wrapper)


var _ok := true
var _anchor_ecef: Vector3


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  PASS  ", message)
	else:
		push_error("FAIL: " + message)
		_ok = false


func _sphere(center: Vector3, radius: float = 25.0) -> Dictionary:
	return {"sphere": [center.x, center.y, center.z, radius]}


func _content_tile(uri: String, center: Vector3, geometric_error: float = 0.0,
		children: Array = []) -> Dictionary:
	var tile := {
		"boundingVolume": _sphere(center),
		"geometricError": geometric_error,
		"content": {"uri": uri},
	}
	if not children.is_empty():
		tile["children"] = children
	return tile


func _contentless_tile(center: Vector3, geometric_error: float,
		children: Array) -> Dictionary:
	return {
		"boundingVolume": _sphere(center),
		"geometricError": geometric_error,
		"children": children,
	}


func _document(root_tile: Dictionary) -> Dictionary:
	return {"asset": {"version": "1.1"}, "geometricError": 1000.0, "root": root_tile}


func _new_streamer(document: Dictionary, max_loaded: int = 64,
		placement_budget: int = 64) -> SyntheticStreamer:
	var streamer := SyntheticStreamer.new()
	streamer._tileset_root_url = ROOT_URL
	streamer.max_tiles_loaded = max_loaded
	streamer.tiles_per_frame_budget = placement_budget
	streamer.responses[ROOT_URL] = document
	root.add_child(streamer)
	return streamer


func _serve_content(streamer: SyntheticStreamer, paths: Array) -> void:
	for path in paths:
		streamer.responses["https://synthetic.test/" + String(path)] = GLTF_SENTINEL


func _view(forward: Vector3) -> Dictionary:
	var normalized := forward.normalized()
	var up := Vector3.UP
	if absf(normalized.dot(up)) > 0.9:
		up = Vector3.RIGHT
	var right := normalized.cross(up).normalized()
	up = right.cross(normalized).normalized()
	return {
		"position": Vector3.ZERO,
		"forward": normalized,
		"right": right,
		"up": up,
		"tan_half_fov_x": 0.5,
		"tan_half_fov_y": 0.5,
		"viewport_height": 1000.0,
		"near": 0.1,
		"far": 10000.0,
	}


func _known_ids(streamer: SyntheticStreamer) -> PackedStringArray:
	var known := PackedStringArray(streamer._loaded_tiles.keys())
	for tile in streamer._incoming_tiles:
		known.append(String(tile["id"]))
	for tile in streamer._pending_results:
		known.append(String(tile["id"]))
	return known


func _snapshot(streamer: SyntheticStreamer, revision: int,
		views: Array = []) -> Dictionary:
	return {
		"lat": 0.0,
		"lon": 0.0,
		"alt": 0.0,
		"anchor_lat": 0.0,
		"anchor_lon": 0.0,
		"anchor_alt": 0.0,
		"local_center": Vector3.ZERO,
		"local_radius": 5000.0,
		"far_radius": 10000.0,
		"maximum_sse": 1.0,
		"generation": 0,
		"view_revision": revision,
		"views": views,
		"known_ids": _known_ids(streamer),
	}


func _run_and_drain(streamer: SyntheticStreamer, revision: int,
		views: Array = [], drain_count: int = 4) -> void:
	streamer._run_selection(_snapshot(streamer, revision, views))
	for unused in drain_count:
		streamer._drain_results()


func _id(path: String, content_index: int = 0) -> String:
	return GWTiles3DStreamer._stable_tile_id(
			"https://synthetic.test/" + path, Transform3D.IDENTITY, content_index)


func _is_visible(streamer: SyntheticStreamer, id: String) -> bool:
	return streamer._loaded_tiles.has(id) \
			and streamer._loaded_tiles[id]["wrapper"].visible


func _visible_count(streamer: SyntheticStreamer, ids: PackedStringArray) -> int:
	var count := 0
	for id in ids:
		if _is_visible(streamer, id):
			count += 1
	return count


func _test_same_pass_parent_and_children() -> void:
	var root_tile := _content_tile("parent.glb", _anchor_ecef, 100.0, [
		_content_tile("child-a.glb", _anchor_ecef),
		_content_tile("child-b.glb", _anchor_ecef),
	])
	var streamer := _new_streamer(_document(root_tile), 3, 1)
	_serve_content(streamer, ["parent.glb", "child-a.glb", "child-b.glb"])
	streamer._run_selection(_snapshot(streamer, 1, [_view(Vector3.FORWARD)]))

	var parent_id := _id("parent.glb")
	var child_a := _id("child-a.glb")
	var child_b := _id("child-b.glb")
	_check(streamer._pending_plans.has(2) and streamer._pending_plans.has(3)
			and streamer._pending_results.size() == 3
			and streamer._pending_results[0]["id"] == parent_id,
			"worker publishes coarse and fine control independently, with the fallback payload first")

	streamer._drain_results()
	_check(_is_visible(streamer, parent_id)
			and not streamer._loaded_tiles.has(child_a)
			and not streamer._loaded_tiles.has(child_b),
			"renderer exposes the coarse fallback before admitting replacement payloads")
	streamer._drain_results()
	_check(_is_visible(streamer, parent_id) and not _is_visible(streamer, child_a),
			"an incomplete replacement remains hidden behind its parent")
	streamer._drain_results()
	_check(not _is_visible(streamer, parent_id)
			and _is_visible(streamer, child_a) and _is_visible(streamer, child_b)
			and streamer._has_active_coverage(parent_id),
			"the complete replacement atomically promotes without a coverage gap")


func _test_nested_contentless_multicontent() -> void:
	var nested_root := {
		"boundingVolume": _sphere(_anchor_ecef),
		"geometricError": 0.0,
		"contents": [{"uri": "fine-a.glb"}, {"uri": "fine-b.glb"}],
	}
	var root_tile := {
		"boundingVolume": _sphere(_anchor_ecef),
		"geometricError": 100.0,
		"refine": "REPLACE",
		"contents": [{"uri": "coarse-a.glb"}, {"uri": "coarse-b.glb"}],
		"children": [
			_contentless_tile(_anchor_ecef, 100.0, [
				_contentless_tile(_anchor_ecef, 100.0, [
					_content_tile("nested.json", _anchor_ecef),
				]),
			]),
		],
	}
	var streamer := _new_streamer(_document(root_tile), 4, 4)
	streamer.responses["https://synthetic.test/nested.json"] = _document(nested_root)
	_serve_content(streamer, ["coarse-a.glb", "coarse-b.glb", "fine-a.glb", "fine-b.glb"])
	_run_and_drain(streamer, 1, [_view(Vector3.FORWARD)])

	var ids := PackedStringArray([
		_id("coarse-a.glb", 0), _id("coarse-b.glb", 1),
		_id("fine-a.glb", 0), _id("fine-b.glb", 1),
	])
	_check(not _is_visible(streamer, ids[0]) and not _is_visible(streamer, ids[1])
			and _is_visible(streamer, ids[2]) and _is_visible(streamer, ids[3])
			and _visible_count(streamer, ids) == 2,
			"nested contentless metadata preserves an atomic multi-content REPLACE cut")


func _test_off_frustum_sibling_turn() -> void:
	var offset := Vector3(0.0, 1000.0, 0.0)
	var front_ecef := _anchor_ecef + offset
	var back_ecef := _anchor_ecef - offset
	var root_tile := _contentless_tile(_anchor_ecef, 100.0, [
		_content_tile("front-coarse.glb", front_ecef, 100.0, [
			_content_tile("front-fine.glb", front_ecef),
		]),
		_content_tile("back-coarse.glb", back_ecef, 100.0, [
			_content_tile("back-fine.glb", back_ecef),
		]),
	])
	root_tile["boundingVolume"] = _sphere(_anchor_ecef, 2500.0)
	var streamer := _new_streamer(_document(root_tile), 4, 4)
	_serve_content(streamer, [
		"front-coarse.glb", "front-fine.glb", "back-coarse.glb", "back-fine.glb",
	])
	var front_local: Vector3 = GWTiles3DTraversal.bounding_sphere(
			_sphere(front_ecef), Transform3D.IDENTITY, 0.0, 0.0, 0.0)["center"]
	var front_view := _view(front_local)
	_run_and_drain(streamer, 1, [front_view])

	var front_coarse := _id("front-coarse.glb")
	var front_fine := _id("front-fine.glb")
	var back_coarse := _id("back-coarse.glb")
	var back_fine := _id("back-fine.glb")
	_check(_is_visible(streamer, front_fine) and not _is_visible(streamer, front_coarse)
			and _is_visible(streamer, back_coarse),
			"the complete coarse sibling frontier remains visible outside the initial frustum")

	_run_and_drain(streamer, 2, [_view(-front_local)])
	_check(_is_visible(streamer, front_coarse) and not _is_visible(streamer, front_fine)
			and not _is_visible(streamer, back_coarse) and _is_visible(streamer, back_fine),
			"a 180-degree turn swaps complete cuts without exposing an unrequested hole")


func _test_contentless_root_does_not_force_offscreen_children() -> void:
	var offset := Vector3(0.0, 8000.0, 0.0)
	var visible_ecef := _anchor_ecef + offset
	var hidden_ecef := _anchor_ecef - offset
	var children := []
	for index in 8:
		children.append(_content_tile("hidden-%d.glb" % index, hidden_ecef))
	children.append(_content_tile("visible.glb", visible_ecef))
	var root_tile := _contentless_tile(_anchor_ecef, 100.0, children)
	root_tile["boundingVolume"] = _sphere(_anchor_ecef, 9000.0)
	var streamer := _new_streamer(_document(root_tile), 1, 1)
	_serve_content(streamer, ["visible.glb"])
	var visible_local: Vector3 = GWTiles3DTraversal.bounding_sphere(
			_sphere(visible_ecef), Transform3D.IDENTITY, 0.0, 0.0, 0.0)["center"]

	_run_and_drain(streamer, 1, [_view(visible_local)])
	_check(_is_visible(streamer, _id("visible.glb"))
			and streamer._loaded_tiles.size() == 1,
			"a broad contentless root does not spend a tight budget on off-frustum children")



func _test_budget_prioritizes_high_sse_branch() -> void:
	# All branches share a sightline, but metadata order is medium, low, high.
	# Three coarse tiles leave one slot: high detail fills it, the next external
	# document confirms exhaustion, and later optional metadata must be skipped.
	var high_ecef := _anchor_ecef + Vector3(0.0, 500.0, 0.0)
	var blocked_ecef := _anchor_ecef + Vector3(0.0, 1200.0, 0.0)
	var late_ecef := _anchor_ecef + Vector3(0.0, 2000.0, 0.0)
	var root_tile := _contentless_tile(_anchor_ecef, 100.0, [
		_content_tile("priority-blocked-coarse.glb", blocked_ecef, 100.0, [
			_content_tile("blocked.json", blocked_ecef),
		]),
		_content_tile("priority-late-coarse.glb", late_ecef, 100.0, [
			_content_tile("late.json", late_ecef),
		]),
		_content_tile("priority-high-coarse.glb", high_ecef, 100.0, [
			_content_tile("priority-high-fine.glb", high_ecef),
		]),
	])
	root_tile["boundingVolume"] = _sphere(_anchor_ecef, 2500.0)
	var streamer := _new_streamer(_document(root_tile), 4, 4)
	_serve_content(streamer, [
		"priority-blocked-coarse.glb", "priority-late-coarse.glb",
		"priority-high-coarse.glb", "priority-high-fine.glb",
		"blocked-fine.glb", "late-fine.glb",
	])
	streamer.responses["https://synthetic.test/blocked.json"] = _document(
			_content_tile("blocked-fine.glb", blocked_ecef))
	streamer.responses["https://synthetic.test/late.json"] = _document(
			_content_tile("late-fine.glb", late_ecef))
	var high_local: Vector3 = GWTiles3DTraversal.bounding_sphere(
			_sphere(high_ecef), Transform3D.IDENTITY, 0.0, 0.0, 0.0)["center"]

	_run_and_drain(streamer, 1, [_view(high_local)])
	_check(_is_visible(streamer, _id("priority-high-fine.glb"))
			and not _is_visible(streamer, _id("priority-high-coarse.glb"))
			and _is_visible(streamer, _id("priority-blocked-coarse.glb"))
			and _is_visible(streamer, _id("priority-late-coarse.glb")),
			"a tight cut gives its only detail slot to the highest-SSE branch")
	_check(streamer.request_counts.has("https://synthetic.test/blocked.json")
			and not streamer.request_counts.has("https://synthetic.test/late.json"),
			"confirmed capacity exhaustion stops later optional metadata traversal")


func _test_later_phase_zero_retains_visible_detail() -> void:
	var offset := Vector3(0.0, 8000.0, 0.0)
	var first_ecef := _anchor_ecef + offset
	var later_ecef := _anchor_ecef - offset
	var root_tile := _contentless_tile(_anchor_ecef, 100.0, [
		_content_tile("first-parent.glb", first_ecef, 100.0, [
			_content_tile("first-fine.glb", first_ecef),
		]),
		_content_tile("later-parent.glb", later_ecef, 100.0, [
			_content_tile("later-fine.glb", later_ecef),
		]),
	])
	root_tile["boundingVolume"] = _sphere(_anchor_ecef, 9000.0)
	var streamer := _new_streamer(_document(root_tile), 4, 4)
	_serve_content(streamer, [
		"first-parent.glb", "first-fine.glb",
		"later-parent.glb", "later-fine.glb",
	])
	var first_local: Vector3 = GWTiles3DTraversal.bounding_sphere(
			_sphere(first_ecef), Transform3D.IDENTITY, 0.0, 0.0, 0.0)["center"]
	var later_local: Vector3 = GWTiles3DTraversal.bounding_sphere(
			_sphere(later_ecef), Transform3D.IDENTITY, 0.0, 0.0, 0.0)["center"]
	_run_and_drain(streamer, 1, [_view(first_local)])

	var first_parent := _id("first-parent.glb")
	var first_fine := _id("first-fine.glb")
	_check(not _is_visible(streamer, first_parent) and _is_visible(streamer, first_fine),
			"the initial detailed cut is visible before the later view arrives")

	streamer._run_selection(_snapshot(streamer, 2, [_view(later_local)]))
	_check(streamer._pending_plans.has(4),
			"a later view publishes its own coarse control plan")
	if not streamer._pending_plans.has(4):
		return
	var phase_zero: Dictionary = streamer._pending_plans[4]
	var phase_one: Dictionary = streamer._pending_plans[5]
	_check(streamer._pending_results.size() == 2
			and streamer._pending_results[0]["id"] == _id("later-parent.glb"),
			"a later-view coarse plan is independently available before its fine payload")

	streamer._pending_plans.clear()
	streamer._pending_plans[4] = phase_zero
	streamer.tiles_per_frame_budget = 0
	streamer._drain_results()
	_check(not _is_visible(streamer, first_parent) and _is_visible(streamer, first_fine),
			"phase zero additively retains visible detail without overlapping its parent")

	streamer._pending_plans[5] = phase_one
	streamer._drain_results()
	_check(not _is_visible(streamer, first_parent) and not _is_visible(streamer, first_fine),
			"only authoritative phase one retires the prior off-view cut")


func _many_content_fixture(count: int) -> Dictionary:
	var coarse := []
	var fine := []
	for index in count:
		coarse.append({"uri": "cap-coarse-%03d.glb" % index})
		fine.append(_content_tile("cap-fine-%03d.glb" % index, _anchor_ecef))
	return _document({
		"boundingVolume": _sphere(_anchor_ecef),
		"geometricError": 100.0,
		"refine": "REPLACE",
		"contents": coarse,
		"children": fine,
	})


func _serve_many_content(streamer: SyntheticStreamer, count: int) -> void:
	for index in count:
		streamer.responses["https://synthetic.test/cap-coarse-%03d.glb" % index] = GLTF_SENTINEL
		streamer.responses["https://synthetic.test/cap-fine-%03d.glb" % index] = GLTF_SENTINEL


func _test_queue_pressure_and_budget_rollback() -> void:
	const CUT_SIZE := 257
	var streamer := _new_streamer(_many_content_fixture(CUT_SIZE), CUT_SIZE, CUT_SIZE)
	_serve_many_content(streamer, CUT_SIZE)
	# The bounded payload queue admits only 256 coarse payloads. A subsequent
	# real pass fetches the missing member and completes the valid coarse cut.
	_run_and_drain(streamer, 1, [_view(Vector3.FORWARD)], 1)
	_run_and_drain(streamer, 2, [_view(Vector3.FORWARD)], 2)
	var coarse_ids := PackedStringArray()
	for index in CUT_SIZE:
		coarse_ids.append(_id("cap-coarse-%03d.glb" % index, index))
	_check(streamer._loaded_tiles.size() == CUT_SIZE
			and _visible_count(streamer, coarse_ids) == CUT_SIZE,
			"more than 256 candidate payloads converge to a complete valid coarse cut")

	# Plan the detailed cut while capacity permits it, filling the bounded
	# producer queue, then impose the hard cap before main-thread placement.
	streamer.max_tiles_loaded = CUT_SIZE * 2
	streamer._run_selection(_snapshot(streamer, 3, [_view(Vector3.FORWARD)]))
	_check(streamer._pending_results.size() == GWTiles3DStreamer.MAX_PENDING_RESULTS
			and streamer._pending_plans.has(7),
			"fine control remains independently queued when the payload channel is full")
	streamer.max_tiles_loaded = CUT_SIZE
	streamer._drain_results()
	_check(streamer._accepted_plan_serial == 7
			and streamer._loaded_tiles.size() == CUT_SIZE
			and _visible_count(streamer, coarse_ids) == CUT_SIZE,
			"fine control is accepted under a full payload queue without evicting the occupied fallback cut")

	# With the real hard budget restored, refinement must roll back every
	# speculative child and publish coarse coverage, not an incomplete cut.
	_run_and_drain(streamer, 4, [_view(Vector3.FORWARD)], 2)
	_check(streamer._accepted_plan_serial == 9
			and streamer._loaded_tiles.size() == CUT_SIZE
			and _visible_count(streamer, coarse_ids) == CUT_SIZE,
			"budget rollback retains the complete coarse cut after cap pressure")


func _test_completed_older_snapshot_is_accepted() -> void:
	var root_tile := _content_tile("moving-parent.glb", _anchor_ecef, 100.0, [
		_content_tile("moving-a.glb", _anchor_ecef),
		_content_tile("moving-b.glb", _anchor_ecef),
	])
	var streamer := _new_streamer(_document(root_tile), 3, 3)
	_serve_content(streamer, ["moving-parent.glb", "moving-a.glb", "moving-b.glb"])

	# Model a newer camera snapshot arriving while revision 1 is completing:
	# its finished control and payloads remain useful until revision 2 itself
	# has completed, rather than being discarded against the posted target.
	streamer._target_view_revision = 2
	streamer._run_selection(_snapshot(streamer, 1, [_view(Vector3.FORWARD)]))
	streamer._drain_results()
	_check(streamer._accepted_view_revision == 1
			and streamer._accepted_plan_serial == 3
			and _is_visible(streamer, _id("moving-a.glb"))
			and _is_visible(streamer, _id("moving-b.glb")),
			"a completed older snapshot is accepted while a newer camera revision is pending")

	_run_and_drain(streamer, 2, [_view(Vector3.FORWARD)])
	_check(streamer._accepted_view_revision == 2
			and streamer._accepted_plan_serial == 5,
			"the subsequently completed newer snapshot monotonically supersedes it")


func _test_failed_payloads_preserve_nested_fallbacks() -> void:
	var nested_root := _content_tile("middle.glb", _anchor_ecef, 100.0, [
		_content_tile("deep-a.glb", _anchor_ecef),
		_content_tile("deep-b.glb", _anchor_ecef),
	])
	var root_tile := _content_tile("outer.glb", _anchor_ecef, 100.0, [
		_content_tile("failure-nested.json", _anchor_ecef),
	])
	var streamer := _new_streamer(_document(root_tile), 4, 4)
	streamer.responses["https://synthetic.test/failure-nested.json"] = _document(nested_root)
	streamer.responses["https://synthetic.test/outer.glb"] = GLTF_SENTINEL
	streamer.responses["https://synthetic.test/middle.glb"] = "not a glTF".to_utf8_buffer()
	streamer.responses["https://synthetic.test/deep-a.glb"] = GLTF_SENTINEL
	streamer.responses["https://synthetic.test/deep-b.glb"] = GLTF_SENTINEL
	_run_and_drain(streamer, 1, [_view(Vector3.FORWARD)])

	var outer := _id("outer.glb")
	var middle := _id("middle.glb")
	var deep_a := _id("deep-a.glb")
	var deep_b := _id("deep-b.glb")
	_check(_is_visible(streamer, outer) and not streamer._loaded_tiles.has(middle)
			and streamer._loaded_tiles.has(deep_a) and streamer._loaded_tiles.has(deep_b)
			and not _is_visible(streamer, deep_a) and not _is_visible(streamer, deep_b),
			"ready nested descendants neither overlap nor retire an unpromoted malformed ancestor")

	var coarse_snapshot := _snapshot(streamer, 2, [_view(Vector3.FORWARD)])
	coarse_snapshot["views"][0]["position"] = Vector3(0.0, 0.0, 2000.0)
	coarse_snapshot["maximum_sse"] = 64.0
	streamer._run_selection(coarse_snapshot)
	streamer._drain_results()
	_check(_is_visible(streamer, outer) and not streamer._loaded_tiles.has(deep_a)
			and not streamer._loaded_tiles.has(deep_b),
			"zooming out releases hidden descendants whose parent never loaded")

	var sibling_root := _content_tile("failure-parent.glb", _anchor_ecef, 100.0, [
		_content_tile("failure-good.glb", _anchor_ecef),
		_content_tile("failure-bad.glb", _anchor_ecef),
	])
	var sibling := _new_streamer(_document(sibling_root), 3, 3)
	_serve_content(sibling, ["failure-parent.glb", "failure-good.glb"])
	sibling.responses["https://synthetic.test/failure-bad.glb"] = {"__status": 503}
	_run_and_drain(sibling, 1, [_view(Vector3.FORWARD)])
	_check(_is_visible(sibling, _id("failure-parent.glb"))
			and not _is_visible(sibling, _id("failure-good.glb")),
			"one failed replacement sibling preserves the visible parent fallback")


func _test_nested_sessions_are_bound_to_requests() -> void:
	var nested_a := {
		"boundingVolume": _sphere(_anchor_ecef),
		"geometricError": 0.0,
		"contents": [
			{"uri": "session-a-explicit.glb?session=A"},
			{"uri": "shared-session.json"},
		],
	}
	var nested_b := {
		"boundingVolume": _sphere(_anchor_ecef),
		"geometricError": 0.0,
		"contents": [
			{"uri": "session-b-explicit.glb?session=B"},
			{"uri": "shared-session.json"},
		],
	}
	var root_tile := _content_tile("session-parent.glb", _anchor_ecef, 100.0, [
		_content_tile("session-a.json", _anchor_ecef),
		_content_tile("session-b.json", _anchor_ecef),
	])
	var streamer := _new_streamer(_document(root_tile), 5, 5)
	streamer.responses["https://synthetic.test/session-a.json"] = _document(nested_a)
	streamer.responses["https://synthetic.test/session-b.json"] = _document(nested_b)
	streamer.responses["https://synthetic.test/shared-session.json?session=A"] = _document(
			_content_tile("session-a-implicit.glb", _anchor_ecef))
	streamer.responses["https://synthetic.test/shared-session.json?session=B"] = _document(
			_content_tile("session-b-implicit.glb", _anchor_ecef))
	_serve_content(streamer, [
		"session-parent.glb",
		"session-a-explicit.glb", "session-a-implicit.glb",
		"session-b-explicit.glb", "session-b-implicit.glb",
	])
	streamer.expected_sessions["https://synthetic.test/session-a-explicit.glb"] = "A"
	streamer.expected_sessions["https://synthetic.test/session-a-implicit.glb"] = "A"
	streamer.expected_sessions["https://synthetic.test/session-b-explicit.glb"] = "B"
	streamer.expected_sessions["https://synthetic.test/session-b-implicit.glb"] = "B"
	_run_and_drain(streamer, 1, [_view(Vector3.FORWARD)])

	var parent_id := _id("session-parent.glb")
	var content_ids := PackedStringArray([
		_id("session-a-explicit.glb?session=A", 0),
		_id("session-a-implicit.glb", 0),
		_id("session-b-explicit.glb?session=B", 0),
		_id("session-b-implicit.glb", 0),
	])
	_check(not _is_visible(streamer, parent_id)
			and _visible_count(streamer, content_ids) == content_ids.size(),
			"deferred payloads, including sessionless URIs, retain their owning document session")

	_run_and_drain(streamer, 2, [_view(Vector3.FORWARD)])
	_check(streamer.request_counts.get(ROOT_URL, 0) == 1
			and streamer.request_counts.get("https://synthetic.test/session-a.json", 0) == 1
			and streamer.request_counts.get("https://synthetic.test/session-b.json", 0) == 1
			and streamer.request_counts.get("https://synthetic.test/shared-session.json", 0) == 2
			and "https://synthetic.test/shared-session.json?session=A" in streamer.request_urls
			and "https://synthetic.test/shared-session.json?session=B" in streamer.request_urls
			and streamer._document_cache.size() == 5,
			"valid A and B sessions retain separate effective-URL cache entries")


func _test_nested_403_preserves_parent_session() -> void:
	var streamer := _new_streamer(_document(
			_content_tile("unused.glb", _anchor_ecef)))
	var nested_url := "https://synthetic.test/refinement-failure.json"
	var sibling_url := "https://synthetic.test/sessionless-sibling.glb"
	streamer._auth_params = ["session=PARENT"]
	streamer._document_cache["stale"] = {}
	streamer.responses[nested_url] = {"__status": 403}
	streamer.responses[sibling_url] = GLTF_SENTINEL
	streamer.expected_sessions[nested_url] = "PARENT"
	streamer.expected_sessions[sibling_url] = "PARENT"

	var failed := streamer._walk_tileset_live(
			nested_url, Transform3D.IDENTITY, _snapshot(streamer, 1),
			{}, {}, "parent", 1, true, true, [])
	var sibling_id := _id("sessionless-sibling.glb")
	var fetched := streamer._fetch_content(
			sibling_url, sibling_id, Transform3D.IDENTITY,
			_sphere(_anchor_ecef), {"center": Vector3.ZERO, "radius": 25.0},
			"parent", _snapshot(streamer, 1))
	_check(not failed["ok"] and streamer._document_cache.is_empty()
			and fetched and streamer._pending_results.size() == 1
			and "https://synthetic.test/sessionless-sibling.glb?session=PARENT" \
					in streamer.request_urls,
			"a nested metadata 403 clears cache without stripping its parent document session")


func _translation_matrix(origin: Vector3) -> Array:
	return [
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		origin.x, origin.y, origin.z, 1.0,
	]


func _test_transformed_instances_have_stable_identity() -> void:
	var transform_a := GWTiles3DTraversal.parse_gltf_transform(
			_translation_matrix(_anchor_ecef + Vector3(0.0, -50.0, 0.0)))
	var transform_b := GWTiles3DTraversal.parse_gltf_transform(
			_translation_matrix(_anchor_ecef + Vector3(0.0, 50.0, 0.0)))
	var root_tile := _contentless_tile(_anchor_ecef, 100.0, [
		{
			"boundingVolume": _sphere(Vector3.ZERO),
			"geometricError": 0.0,
			"transform": _translation_matrix(transform_a.origin),
			"content": {"uri": "shared-instance.glb"},
		},
		{
			"boundingVolume": _sphere(Vector3.ZERO),
			"geometricError": 0.0,
			"transform": _translation_matrix(transform_b.origin),
			"content": {"uri": "shared-instance.glb"},
		},
	])
	var streamer := _new_streamer(_document(root_tile), 2, 2)
	_serve_content(streamer, ["shared-instance.glb"])
	_run_and_drain(streamer, 1)

	var id_a := GWTiles3DStreamer._stable_tile_id(
			"https://synthetic.test/shared-instance.glb", transform_a, 0)
	var id_b := GWTiles3DStreamer._stable_tile_id(
			"https://synthetic.test/shared-instance.glb", transform_b, 0)
	_check(id_a != id_b and streamer._loaded_tiles.has(id_a)
			and streamer._loaded_tiles.has(id_b)
			and streamer._loaded_tiles[id_a]["ecef_transform"] == transform_a
			and streamer._loaded_tiles[id_b]["ecef_transform"] == transform_b,
			"one canonical URI produces distinct stable instances for exact transforms")

	_run_and_drain(streamer, 2)
	_check(streamer.request_counts.get(
			"https://synthetic.test/shared-instance.glb", 0) == 2
			and streamer._loaded_tiles.size() == 2,
			"a later selection reuses both transformed identities without duplicate payloads")


func _test_payload_scheduler_yields_only_to_real_target_changes() -> void:
	var streamer := _new_streamer(_document(
			_content_tile("scheduler-root.glb", _anchor_ecef)), 8, 8)
	var desired := {}
	for index in 8:
		var path := "scheduler-%d.glb" % index
		var url := "https://synthetic.test/" + path
		streamer.responses[url] = GLTF_SENTINEL
		desired[_id(path)] = {
			"url": url,
			"transform": Transform3D.IDENTITY,
			"bounding_volume": _sphere(_anchor_ecef),
			"bounds": {"center": Vector3.ZERO, "radius": 25.0},
			"pending_parent": "",
		}
	var snapshot := _snapshot(streamer, 5)
	var sent := {}

	streamer._target_change_revision = 5
	streamer._download_selection(desired, snapshot, sent)
	_check(streamer._pending_results.size() == 8 and sent.size() == 8,
			"an unchanged target lets the worker finish the full payload batch")

	streamer._pending_results.clear()
	sent.clear()
	streamer._target_change_revision = 6
	streamer._download_selection(desired, snapshot, sent)
	_check(streamer._pending_results.size() == 4 and sent.size() == 4,
			"a genuinely newer target yields the old download after four payload attempts")


func _initialize() -> void:
	call_deferred("_run")


func _run() -> void:
	_anchor_ecef = GWGeodeticConvert.geodetic_to_ecef(0.0, 0.0, 0.0)
	_test_same_pass_parent_and_children()
	_test_nested_contentless_multicontent()
	_test_off_frustum_sibling_turn()
	_test_contentless_root_does_not_force_offscreen_children()
	_test_budget_prioritizes_high_sse_branch()
	_test_later_phase_zero_retains_visible_detail()
	_test_queue_pressure_and_budget_rollback()
	_test_completed_older_snapshot_is_accepted()
	_test_failed_payloads_preserve_nested_fallbacks()
	_test_transformed_instances_have_stable_identity()
	_test_nested_sessions_are_bound_to_requests()
	_test_nested_403_preserves_parent_session()
	_test_payload_scheduler_yields_only_to_real_target_changes()

	print("\ntest_live_streamer: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
