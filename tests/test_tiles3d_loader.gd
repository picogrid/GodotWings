extends SceneTree

# Headless smoke test for GWTiles3DLoader, using a SYNTHETIC glb + manifest
# built in-process (no network, no real gw_3dtiles_prefetch.py run needed).
# The glb itself is produced by round-tripping a tiny scene through
# GLTFDocument's own exporter (append_from_scene + generate_buffer) --
# real Godot glTF I/O end to end, not a hand-rolled byte blob -- then fed
# back through the same append_from_buffer()/generate_scene() path
# GWTiles3DLoader and gw_3dtiles_prefetch.py's own verification both use.
#
# The manifest's "basis" is deliberately non-identity (scale 2x, with axes
# permuted/negated) to catch a row/column mix-up in
# GWTiles3DLoader._transform_from_manifest -- the exact bug class that bit
# the ENU/rotation math earlier in this project's development.
#
# The exported mesh node ALSO carries its own non-identity transform (not
# identity like a naive fixture would use) to catch a different, real bug
# found live against actual Cesium ion data: Google Photorealistic 3D Tiles
# bakes each tile's real-world anchor into the glTF's OWN root node matrix
# (the tileset.json transform chain stays identity throughout), so
# overwriting the loaded scene's transform with the manifest's tileset-chain
# transform -- instead of wrapping it -- silently discarded that anchor and
# put every tile at the identical wrong position. This fixture fails that
# way if the loader regresses to overwriting.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _approx_v3(a: Vector3, b: Vector3, eps := 0.001) -> bool:
	return a.distance_to(b) < eps


const CONTENT_OWN_OFFSET := Vector3(1000.0, 2000.0, 3000.0)


func _initialize() -> void:
	# 1. Build a source scene whose OWN root node carries a non-identity
	# transform (simulating Google's baked-in content anchor -- see comment
	# above) and export it to a real glb buffer.
	var mesh_inst := MeshInstance3D.new()
	mesh_inst.mesh = BoxMesh.new()
	mesh_inst.transform = Transform3D(Basis(), CONTENT_OWN_OFFSET)

	var export_doc := GLTFDocument.new()
	var export_state := GLTFState.new()
	_check(export_doc.append_from_scene(mesh_inst, export_state) == OK, "exported synthetic scene to GLTFState")
	var glb_bytes := export_doc.generate_buffer(export_state)
	_check(glb_bytes.size() > 0, "generate_buffer produced non-empty glb")
	_check(glb_bytes.slice(0, 4).get_string_from_ascii() == "glTF", "exported buffer has glTF magic")

	var glb_path := "user://tiles3d_test_tile.glb"
	var f := FileAccess.open(glb_path, FileAccess.WRITE)
	f.store_buffer(glb_bytes)
	f.close()

	# 2. Write a manifest pointing at it, with a non-identity basis (2x scale,
	# axes permuted: local X -> Godot +X, local Y -> Godot -Z, local Z -> Godot +Y)
	# and a distinct position, to catch any row/column transposition.
	# fetched_at is "now" (not a hardcoded past date) -- GWTiles3DLoader
	# enforces max_cache_age_hours by default, and a hardcoded date would
	# eventually make this whole fixture look stale and fail for a reason
	# unrelated to what it's testing.
	var now_iso := Time.get_datetime_string_from_unix_time(Time.get_unix_time_from_system(), true) + "Z"
	var manifest := {
		"fetched_at": now_iso,
		"source_tileset": "synthetic-test",
		"asset_id": null,
		"content_attributions": ["<span>Test Attribution</span>"],
		"origin_lat": 12.5,
		"origin_lon": -45.5,
		"origin_alt": 10.0,
		"radius_km": 1.0,
		"detail_m": 10.0,
		"tiles": [
			{
				"file": "tiles3d_test_tile.glb",
				"source_url": "synthetic",
				"position": [10.0, 20.0, 30.0],
				# Row-major (matches gw_3dtiles_prefetch.py's numpy .tolist()):
				# m[i][j] = row i, column j -> column j (where local axis j
				# lands) = Vector3(m[0][j], m[1][j], m[2][j]). This encodes
				# columns (2,0,0), (0,0,-2), (0,2,0) -- verify against those,
				# not against the rows as written here.
				"basis": [[2.0, 0.0, 0.0], [0.0, 0.0, 2.0], [0.0, -2.0, 0.0]],
			}
		],
	}
	var manifest_path := "user://tiles3d_test_manifest.json"
	var mf := FileAccess.open(manifest_path, FileAccess.WRITE)
	mf.store_string(JSON.stringify(manifest))
	mf.close()

	# 3. Load it through the real loader node.
	var loader := GWTiles3DLoader.new()
	loader.manifest_path = manifest_path
	get_root().add_child(loader)
	await process_frame

	_check(loader.tile_count == 1, "loader reports 1 tile loaded (got %d)" % loader.tile_count)
	_check(is_equal_approx(loader.origin_lat, 12.5), "manifest origin_lat read")
	_check(is_equal_approx(loader.origin_lon, -45.5), "manifest origin_lon read")
	_check(loader.source_tileset == "synthetic-test", "manifest source_tileset read")
	_check(loader.attributions.size() == 1 and loader.attributions[0] == "<span>Test Attribution</span>",
			"manifest content_attributions read, got %s" % [loader.attributions])

	var tile_node: Node3D = null
	for c in loader.get_children():
		if c is Node3D:
			tile_node = c
			break
	_check(tile_node != null, "a Node3D child was added for the tile")

	if tile_node != null:
		_check(_approx_v3(tile_node.position, Vector3(10.0, 20.0, 30.0)), "tile position matches manifest")
		_check(_approx_v3(tile_node.transform.basis.x, Vector3(2.0, 0.0, 0.0)),
				"basis column 0 (local X) placed correctly, got %s" % tile_node.transform.basis.x)
		_check(_approx_v3(tile_node.transform.basis.y, Vector3(0.0, 0.0, -2.0)),
				"basis column 1 (local Y) placed correctly, got %s" % tile_node.transform.basis.y)
		_check(_approx_v3(tile_node.transform.basis.z, Vector3(0.0, 2.0, 0.0)),
				"basis column 2 (local Z) placed correctly, got %s" % tile_node.transform.basis.z)

		# The content's own internal offset must be PRESERVED (composed via
		# the wrapper), not overwritten -- the exact regression this fixture
		# targets (see the file-header comment).
		var content_root: Node3D = null
		for c2 in tile_node.get_children():
			if c2 is Node3D:
				content_root = c2
				break
		_check(content_root != null, "content scene was added as a child of the wrapper (not merged into it)")
		if content_root != null:
			_check(_approx_v3(content_root.position, CONTENT_OWN_OFFSET, 0.01),
					"content's own internal offset survived (not overwritten), got %s" % content_root.position)
			var expected_global := tile_node.transform * content_root.transform
			var actual_global := content_root.get_global_transform()
			_check(_approx_v3(actual_global.origin, expected_global.origin, 0.01),
					"manifest transform and content's own offset compose correctly, got %s expected %s" %
					[actual_global.origin, expected_global.origin])

		# Confirm the exported box mesh really did round-trip into a real mesh
		# somewhere under the loaded scene (not just an empty Node3D).
		var found_mesh := false
		var stack: Array = [tile_node]
		while stack.size() > 0:
			var n = stack.pop_back()
			if n is MeshInstance3D and n.mesh != null:
				found_mesh = true
				break
			for c2 in n.get_children():
				stack.push_back(c2)
		_check(found_mesh, "loaded tile scene contains a real MeshInstance3D with a mesh")

	# 4. GOOGLE_YUP_ECEF_CORRECTION regression check, against the exact real
	# values that caught this bug live: a real downloaded tile's raw glTF
	# node-matrix translation (Google's own axis convention) decoded to a
	# location near the Seychelles instead of Helsinki until this exact
	# correction was applied -- and an earlier, wrong version of this same
	# constant (transposed, from misremembering Basis(a,b,c)'s semantics)
	# passed every other check in this file while still computing a
	# thousands-of-km-off position, because nothing here isolated the
	# correction matrix itself. This does.
	var raw_google_vec := Vector3(2877380.3167647473, 5512911.69498128, -1343554.773555934)
	var corrected := GWTiles3DContent.GOOGLE_YUP_ECEF_CORRECTION * raw_google_vec
	var expected_corrected := Vector3(2877380.3167647473, 1343554.773555934, 5512911.69498128)
	_check(_approx_v3(corrected, expected_corrected, 0.5),
			"GOOGLE_YUP_ECEF_CORRECTION matches the real verified conversion, got %s expected %s" %
			[corrected, expected_corrected])

	# 5. Missing-file / empty-manifest-path robustness (should warn, not crash).
	var loader2 := GWTiles3DLoader.new()
	loader2.manifest_path = ""
	get_root().add_child(loader2)
	await process_frame
	_check(loader2.tile_count == 0, "empty manifest_path loads zero tiles without error")

	# 6. Staleness enforcement: caching real Cesium ion / Google 3D Tiles
	# content is session-scoped by the provider's terms, not a license for a
	# permanent local mirror (see gw_3dtiles_prefetch.py's docstring and
	# .gitignore) -- max_cache_age_hours is the mechanism that actually
	# enforces that on the Godot side instead of just documenting it.
	var stale_manifest := manifest.duplicate(true)
	stale_manifest["fetched_at"] = "2020-01-01T00:00:00Z"
	var stale_path := "user://tiles3d_test_manifest_stale.json"
	var sf := FileAccess.open(stale_path, FileAccess.WRITE)
	sf.store_string(JSON.stringify(stale_manifest))
	sf.close()

	var loader3 := GWTiles3DLoader.new()
	loader3.manifest_path = stale_path
	get_root().add_child(loader3)
	await process_frame
	_check(loader3.tile_count == 0, "stale manifest (default max_cache_age_hours) refuses to load")

	var loader4 := GWTiles3DLoader.new()
	loader4.manifest_path = stale_path
	loader4.max_cache_age_hours = 0.0  # explicitly disabled -- must load despite the same old fetched_at
	get_root().add_child(loader4)
	await process_frame
	_check(loader4.tile_count == 1, "max_cache_age_hours = 0 disables the staleness check (got %d)" % loader4.tile_count)

	print("\ntest_tiles3d_loader: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
