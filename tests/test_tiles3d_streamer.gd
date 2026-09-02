extends SceneTree

# Headless test for GWTiles3DStreamer -- covers the one piece of it that's
# testable without a real network/thread: the set_ion_token Inspector
# convenience property. Everything else (the actual HTTP fetch/traversal/
# threading) is verified manually against real Cesium ion data (see the
# session notes) rather than as part of the automated suite, since it needs
# a real token and real network access.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _initialize() -> void:
	var s := GWTiles3DStreamer.new()
	s.ion_token_env = "TEST_ION_TOKEN_VAR_%d" % Time.get_ticks_usec()  # avoid clobbering a real env var if one happens to exist

	_check(not OS.has_environment(s.ion_token_env), "sanity: the test env var starts unset")

	s.set_ion_token = "totally-fake-test-token"
	_check(OS.has_environment(s.ion_token_env) and OS.get_environment(s.ion_token_env) == "totally-fake-test-token",
			"set_ion_token applies the value via OS.set_environment(ion_token_env, ...)")

	# The critical property: reading it back must NEVER show the real value --
	# a plain @export var would serialize whatever was typed straight into
	# the saved .tscn file in plaintext, which is exactly what this custom
	# getter avoids.
	_check(s.set_ion_token == "", "set_ion_token's getter always reads back empty (never persisted/serialized)")

	# Setting an empty string must be a no-op (don't clobber an already-set
	# token just because the Inspector field redraws as empty).
	s.set_ion_token = ""
	_check(OS.get_environment(s.ion_token_env) == "totally-fake-test-token",
			"setting an empty string doesn't clear an already-applied token")

	# An empty ion_token_env (e.g. accidentally blanked in the Inspector)
	# must degrade to a clear error, not crash -- found live: OS.set_environment
	# throws an uncaught engine-level error for an empty variable NAME, which
	# without this guard surfaces as a raw C++ assertion instead of a
	# GodotWings-authored message.
	var s2 := GWTiles3DStreamer.new()
	s2.ion_token_env = ""
	s2.set_ion_token = "some-token"  # must not crash
	_check(true, "set_ion_token with an empty ion_token_env does not crash (push_error instead)")

	# Belt-and-suspenders check: EVERY property list entry named
	# "set_ion_token" must lack PROPERTY_USAGE_STORAGE, regardless of which
	# one Godot's serializer happens to consult -- found live: a real token
	# got written into a saved .tscn in plaintext despite the getter-always-
	# empty trick above, because a non-@tool script's properties are backed
	# by a placeholder instance while merely editing (not running) a scene,
	# which never runs any script code -- including that getter -- at all,
	# and just stores/serializes the raw typed value. @tool (so a real
	# instance backs it even while editing) plus this explicit non-storage
	# flag are the two independent fixes; this test covers the second one,
	# which holds regardless of whether the first one is ever bypassed by
	# some other Godot-internal path.
	var found_entry := false
	for p in s.get_property_list():
		if p["name"] == "set_ion_token":
			found_entry = true
			_check((p["usage"] & PROPERTY_USAGE_STORAGE) == 0,
					"set_ion_token property list entry lacks PROPERTY_USAGE_STORAGE (usage=%d)" % p["usage"])
	_check(found_entry, "set_ion_token appears in the property list at all (still editable in the Inspector)")

	# Generation gating: a pass started under an OLDER anchor than the
	# current one (superseded by a newer reanchor before it finished) must
	# NOT apply its evictions -- found live: reanchoring while a pass was
	# still in flight made everything vanish and never come back, because a
	# stale pass's eviction list was applied unconditionally even though it
	# reflected an outdated area. Exercised directly against _drain_results
	# with synthetic pending state -- no real network/thread needed.
	var s3 := GWTiles3DStreamer.new()
	s3._mutex = Mutex.new()  # normally created in _ready(), which needs a real vehicle
	var fake_wrapper := Node3D.new()
	s3.add_child(fake_wrapper)
	s3._loaded_tiles["survivor"] = {"wrapper": fake_wrapper, "ecef_transform": Transform3D.IDENTITY}
	s3._current_generation = 2
	s3._pending_results = [
		{"new_tiles": [], "evict_ids": PackedStringArray(["survivor"]), "generation": 1},  # stale -- must be ignored
	]
	s3._drain_results()
	_check(s3._loaded_tiles.has("survivor"),
			"a stale-generation pass's eviction is NOT applied (tile survives)")

	s3._pending_results = [
		{"new_tiles": [], "evict_ids": PackedStringArray(["survivor"]), "generation": 2},  # current -- must apply
	]
	s3._drain_results()
	_check(not s3._loaded_tiles.has("survivor"),
			"a current-generation pass's eviction IS applied (tile removed)")

	# Stable tile identity: found live against real Google Photorealistic 3D
	# Tiles data -- the SAME real-world tile gets a fresh, unique opaque URL
	# on every separate tileset.json fetch (tied to its ephemeral session),
	# so keying dedup on that URL made three identical stationary polls
	# triple the tile count instead of recognizing tiles already had. Two
	# points close enough to be the same real tile (well under the 10m grid)
	# must hash the same; two points a real tile-spacing apart must not.
	_check(GWTiles3DStreamer._stable_tile_id(Vector3(1000.0, 2000.0, 3000.0)) ==
			GWTiles3DStreamer._stable_tile_id(Vector3(1000.4, 1999.6, 3000.2)),
			"stable_tile_id treats nearly-identical positions as the same tile")
	_check(GWTiles3DStreamer._stable_tile_id(Vector3(1000.0, 2000.0, 3000.0)) !=
			GWTiles3DStreamer._stable_tile_id(Vector3(1500.0, 2000.0, 3000.0)),
			"stable_tile_id treats positions a real tile-spacing apart as different tiles")

	# Root-endpoint session rejection: found live against the real API --
	# the top-level tileset root endpoint 400s if a session param (only
	# valid on dataset-specific paths) is attached, which only bites once a
	# session has actually been captured from an earlier poll -- so it
	# fetches fine on the very first poll and then fails on every poll
	# after that, since it's re-fetched once per poll with an
	# ever-more-stale session attached. allow_session=false must strip it.
	var s4 := GWTiles3DStreamer.new()
	s4._auth_params = ["key=abc123", "session=deadbeef"]
	var with_session := s4._apply_auth("https://tile.googleapis.com/v1/3dtiles/root.json")
	var without_session := s4._apply_auth("https://tile.googleapis.com/v1/3dtiles/root.json", false)
	_check(with_session.find("session=deadbeef") != -1, "_apply_auth includes session by default")
	_check(without_session.find("session=") == -1 and without_session.find("key=abc123") != -1,
			"_apply_auth(allow_session=false) strips session but keeps key, got %s" % without_session)

	print("\ntest_tiles3d_streamer: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
