extends SceneTree

## Network-free checks for Inspector token handling and root authentication.
## Streaming and replacement behavior is covered by test_live_streamer.gd.

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

	s.free()
	s4.free()

	print("\ntest_tiles3d_streamer: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
