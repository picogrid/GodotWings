extends SceneTree

# Headless smoke test for GWImportedTerrain, using small SYNTHETIC files built
# in-process (no network, no real gw_terrain_import.py run needed) — mirrors
# the R+G-packed height encoding gw_terrain_import.py actually writes, with
# known values at each corner so orientation (N/S/E/W) and the height decode
# are both checked against ground truth we set ourselves.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _approx(a: float, b: float, eps := 0.05) -> bool:
	return absf(a - b) < eps


func _pack_pixel(h16: int) -> Color:
	var hi := (h16 >> 8) & 0xFF
	var lo := h16 & 0xFF
	return Color(hi / 255.0, lo / 255.0, 0.0)


func _initialize() -> void:
	# 5x5 heightmap: corners get distinct, easily-identified heights; middle flat.
	# NW(0,0)=0, NE(0,4)=65535, SW(4,0)=32768, SE(4,4)=16384, center=8192.
	var n := 5
	var hm := Image.create(n, n, false, Image.FORMAT_RGB8)
	for y in n:
		for x in n:
			hm.set_pixel(x, y, _pack_pixel(8192))  # flat fill, corners overridden below
	hm.set_pixel(0, 0, _pack_pixel(0))       # NW: pixel row 0 = north edge, col 0 = west edge
	hm.set_pixel(n - 1, 0, _pack_pixel(65535))  # NE
	hm.set_pixel(0, n - 1, _pack_pixel(32768))  # SW
	hm.set_pixel(n - 1, n - 1, _pack_pixel(16384))  # SE
	var hm_path := "user://test_terrain_heightmap.png"
	_check(hm.save_png(hm_path) == OK, "wrote synthetic heightmap")

	var tex := Image.create(4, 4, false, Image.FORMAT_RGB8)
	tex.fill(Color(0.3, 0.5, 0.2))
	var tex_path := "user://test_terrain_texture.png"
	_check(tex.save_png(tex_path) == OK, "wrote synthetic texture")

	var elev_min := 0.0
	var elev_max := 100.0
	var elev_center := 50.0  # decoded center height16=8192 -> elev = 0 + (8192/65535)*100 = 12.5; center offset chosen separately
	var size_m := 1000.0
	var meta := {
		"size_m": size_m, "elevation_min_m": elev_min, "elevation_max_m": elev_max,
		"elevation_center_m": elev_center, "center_lat": 10.0, "center_lon": 20.0,
	}
	var meta_path := "user://test_terrain_meta.json"
	var f := FileAccess.open(meta_path, FileAccess.WRITE)
	f.store_string(JSON.stringify(meta))
	f.close()

	var t := GWImportedTerrain.new()
	# _decode_height16 unit checks, independent of the mesh -- exact round trip,
	# same encoding gw_terrain_import.py uses.
	_check(t._decode_height16(Color(0, 0, 0)) == 0, "decode: (0,0) -> 0")
	_check(t._decode_height16(Color(1, 1, 0)) == 65535, "decode: (1,1) -> 65535")
	_check(t._decode_height16(Color(1.0, 0, 0)) == 65280, "decode: hi-byte only -> 255*256")
	_check(t._decode_height16(Color(0, 1.0 / 255.0, 0)) == 1, "decode: lo-byte only -> 1")

	t.heightmap_path = hm_path
	t.texture_path = tex_path
	t.metadata_path = meta_path
	t.max_vertices_per_side = n  # no downsampling -- exercise every corner exactly
	get_root().add_child(t)
	await process_frame

	_check(t.size_m == size_m, "metadata size_m loaded")
	_check(_approx(t.elevation_center_m, elev_center), "metadata elevation_center_m loaded")
	_check(t.mesh != null, "mesh built")

	if t.mesh != null:
		var aabb := t.mesh.get_aabb()
		_check(_approx(aabb.size.x, size_m, 1.0), "mesh spans size_m in X (got %.1f)" % aabb.size.x)
		_check(_approx(aabb.size.z, size_m, 1.0), "mesh spans size_m in Z (got %.1f)" % aabb.size.z)

	var col: StaticBody3D = null
	for c in t.get_children():
		if c is StaticBody3D:
			col = c
			break
	_check(col != null, "collision StaticBody3D generated")

	if col != null:
		var space := t.get_world_3d().direct_space_state
		var half := size_m * 0.5

		# NW corner (heightmap row 0, col 0) -> easting=-half, northing=+half
		# -> Godot (x=-half, z=-half) per ned_to_world (x=E, z=-N). h16=0 -> elev=elev_min=0 -> y = 0 - elev_center.
		var hit_nw := space.intersect_ray(PhysicsRayQueryParameters3D.create(
				Vector3(-half, 1000, -half), Vector3(-half, -1000, -half)))
		if hit_nw.is_empty():
			_check(false, "NW corner raycast hit")
		else:
			var expected_nw := elev_min - elev_center
			_check(_approx(hit_nw.position.y, expected_nw, 2.0),
					"NW corner (h16=0) -> y=%.2f (expected ~%.2f)" % [hit_nw.position.y, expected_nw])

		# NE corner (row 0, col n-1) -> easting=+half, northing=+half -> Godot (x=+half, z=-half).
		# h16=65535 -> elev=elev_max=100.
		var hit_ne := space.intersect_ray(PhysicsRayQueryParameters3D.create(
				Vector3(half, 1000, -half), Vector3(half, -1000, -half)))
		if hit_ne.is_empty():
			_check(false, "NE corner raycast hit")
		else:
			var expected_ne := elev_max - elev_center
			_check(_approx(hit_ne.position.y, expected_ne, 2.0),
					"NE corner (h16=65535) -> y=%.2f (expected ~%.2f, confirms east = +x)" %
					[hit_ne.position.y, expected_ne])

		# SW corner (row n-1, col 0) -> easting=-half, northing=-half -> Godot (x=-half, z=+half).
		# h16=32768 -> elev=50.
		var hit_sw := space.intersect_ray(PhysicsRayQueryParameters3D.create(
				Vector3(-half, 1000, half), Vector3(-half, -1000, half)))
		if hit_sw.is_empty():
			_check(false, "SW corner raycast hit")
		else:
			var expected_sw := elev_min + (32768.0 / 65535.0) * (elev_max - elev_min) - elev_center
			_check(_approx(hit_sw.position.y, expected_sw, 2.0),
					"SW corner (h16=32768) -> y=%.2f (expected ~%.2f, confirms south = +z)" %
					[hit_sw.position.y, expected_sw])

	print("\ntest_imported_terrain: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
