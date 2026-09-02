extends SceneTree

# Headless test for GWTiles3DTraversal -- ports the exact live-verified checks
# already done in Python this session (against real Cesium ion / Google
# Photorealistic 3D Tiles data and a real public Cesium sample tileset) into
# GDScript, plus a synthetic multi-level tileset tree exercising the pure
# decision function (evaluate_tile) with no network at all.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _approx_v3(a: Vector3, b: Vector3, eps: float) -> bool:
	return a.distance_to(b) < eps


func _initialize() -> void:
	# 1. box_intersects_sphere: the REAL Google Photorealistic 3D Tiles root
	# box (~7645km half-extents centered on Earth's center -- i.e. it
	# encloses the whole planet) must be judged to overlap a small Helsinki
	# AOI. This is the exact case that broke a naive corner-projection
	# approach live (root got wrongly pruned) before the Cartesian
	# closest-point-on-OBB fix.
	var root_box := [0, 0, 0, 7645212, 0, 0, 0, 7645212, 0, 0, 0, 7645212]
	var helsinki_center := GWGeodeticConvert.geodetic_to_ecef(60.221088825593135, 25.018208331290666, 0.0)
	var helsinki_radius := 500.0
	_check(GWTiles3DTraversal.box_intersects_sphere(root_box, Transform3D.IDENTITY, helsinki_center, helsinki_radius),
			"real Google root box (whole-Earth-enclosing) overlaps a small Helsinki AOI")

	# A small box far away (London) must NOT overlap the same small Helsinki AOI.
	var london_center := GWGeodeticConvert.geodetic_to_ecef(51.5, -0.1, 0.0)
	var small_box := [0, 0, 0, 100, 0, 0, 0, 100, 0, 0, 0, 100]
	var london_transform := Transform3D(Basis(), london_center)
	_check(not GWTiles3DTraversal.box_intersects_sphere(small_box, london_transform, helsinki_center, helsinki_radius),
			"small London box does not overlap a small Helsinki AOI")

	# 2. local_transform_to_godot, against the exact real numbers verified
	# live this session for a real downloaded Google tile (post axis-
	# correction -- see Tiles3DContent.GOOGLE_YUP_ECEF_CORRECTION; this test
	# feeds the ALREADY-corrected ECEF position, matching what the streamer
	# will do after applying that correction upstream) -- expected result
	# hand-derived independently in Python and cross-checked against what
	# Godot actually placed the real mesh at.
	var corrected_ecef := Vector3(2877380.3167647473, 1343554.773555934, 5512911.69498128)
	var google_transform := Transform3D(Basis(), corrected_ecef)
	var godot_xform := GWTiles3DTraversal.local_transform_to_godot(
			google_transform, 60.221088825593135, 25.018208331290666, 0.0)
	var expected_pos := Vector3(631.5, 66.3, -205.3)
	_check(_approx_v3(godot_xform.origin, expected_pos, 1.0),
			"local_transform_to_godot matches the real verified Google tile position, got %s expected ~%s" %
			[godot_xform.origin, expected_pos])

	# And the real Cesium sample Dragon tileset's root ECEF transform
	# (verified live: decodes to lat=40.0425, lon=-75.6121, alt=503.75 --
	# using that same point as the anchor should give ~(0, 503.75, 0)).
	var dragon_transform := Transform3D(
		Basis(Vector3(96.86356343768793, 24.848542777253734, 0),
			Vector3(-15.986465724980844, 62.317780594908875, 76.5566922962899),
			Vector3(19.02322243409411, -74.15554020821229, 64.3356267137516)),
		Vector3(1215107.7612304366, -4736682.902037748, 4081926.095098698))
	var dragon_godot := GWTiles3DTraversal.local_transform_to_godot(
			dragon_transform, 40.04253061142592, -75.61209430782448, 0.0)
	_check(_approx_v3(dragon_godot.origin, Vector3(0.0, 503.75, 0.0), 1.0),
			"local_transform_to_godot matches the real verified Dragon tileset position, got %s" % dragon_godot.origin)
	_check(absf(dragon_godot.basis.x.length() - 100.0) < 0.5,
			"local_transform_to_godot preserves the Dragon transform's real 100x scale, got %.2f" % dragon_godot.basis.x.length())

	# 3. unwrap_b3dm: a synthetic b3dm header wrapping a fake "glTF" payload,
	# and passthrough for already-plain glb/gltf data.
	var fake_glb := "glTFfakepayload".to_utf8_buffer()
	var header := PackedByteArray()
	header.append_array("b3dm".to_utf8_buffer())
	header.append_array(PackedByteArray([1, 0, 0, 0]))  # version
	var total_len: int = 28 + fake_glb.size()
	header.append_array(_u32le(total_len))  # byteLength
	header.append_array(_u32le(0))  # featureTableJSONByteLength
	header.append_array(_u32le(0))  # featureTableBinaryByteLength
	header.append_array(_u32le(0))  # batchTableJSONByteLength
	header.append_array(_u32le(0))  # batchTableBinaryByteLength
	var b3dm := header + fake_glb
	var unwrapped := GWTiles3DTraversal.unwrap_b3dm(b3dm)
	_check(unwrapped == fake_glb, "unwrap_b3dm recovers the embedded glb exactly")
	var plain := "glTFalreadyplain".to_utf8_buffer()
	_check(GWTiles3DTraversal.unwrap_b3dm(plain) == plain, "unwrap_b3dm passes through non-b3dm data unchanged")

	# 4. evaluate_tile against a synthetic multi-level tree (mirrors the real
	# public Dragon sample's structure: root has its own content AND
	# children with progressively finer geometricError) -- no network.
	var aoi_bbox: Array = GWTiles3DTraversal.bbox_from_center(60.0, 25.0, 1.0)
	var aoi_center := GWGeodeticConvert.geodetic_to_ecef(60.0, 25.0, 0.0)
	var aoi_radius := 1000.0
	var synthetic_root := {
		"geometricError": 1.0,
		"content": {"uri": "coarse.glb"},
		"boundingVolume": {"sphere": [0, 0, 0, 10.0]},
		"children": [
			{
				"geometricError": 0.1,
				"content": {"uri": "fine.glb"},
				"boundingVolume": {"sphere": [0, 0, 0, 10.0]},
			},
		],
	}
	var root_transform := Transform3D(Basis(), aoi_center)  # tile sits at the AOI center
	var coarse_result := GWTiles3DTraversal.evaluate_tile(synthetic_root, root_transform, 1.0, aoi_bbox, aoi_center, aoi_radius)
	_check(coarse_result["action"] == "content" and coarse_result["contents"][0]["uri"] == "coarse.glb",
			"detail_m=1.0 stops at the root's own content (ge=1.0 <= 1.0), got %s" % coarse_result)

	var fine_result := GWTiles3DTraversal.evaluate_tile(synthetic_root, root_transform, 0.05, aoi_bbox, aoi_center, aoi_radius)
	_check(fine_result["action"] == "recurse" and fine_result["children"].size() == 1,
			"detail_m=0.05 recurses past the root (ge=1.0 > 0.05), got %s" % fine_result)
	if fine_result["action"] == "recurse":
		var child_entry: Dictionary = fine_result["children"][0]
		var child_result := GWTiles3DTraversal.evaluate_tile(child_entry["tile"], child_entry["transform"], 0.05,
				aoi_bbox, aoi_center, aoi_radius)
		_check(child_result["action"] == "content" and child_result["contents"][0]["uri"] == "fine.glb",
				"recursion reaches the finer child's content, got %s" % child_result)

	# A tile with NO content of its own (pure grouping/LOD node) must recurse
	# regardless of detail_m -- the exact bug found live against the real
	# Google/Cesium root structure.
	var grouping_tile := {
		"geometricError": 1e100,
		"boundingVolume": {"sphere": [0, 0, 0, 10.0]},
		"children": [{"geometricError": 5.0, "content": {"uri": "leaf.glb"}, "boundingVolume": {"sphere": [0, 0, 0, 10.0]}}],
	}
	var grouping_result := GWTiles3DTraversal.evaluate_tile(grouping_tile, root_transform, 50.0, aoi_bbox, aoi_center, aoi_radius)
	_check(grouping_result["action"] == "recurse",
			"a tile with no content of its own recurses regardless of geometricError, got %s" % grouping_result)

	# Outside the AOI -> pruned regardless of detail_m.
	var far_center := GWGeodeticConvert.geodetic_to_ecef(-33.9, 151.2, 0.0)  # Sydney, far from the Helsinki-ish AOI above
	var far_tile := {"geometricError": 0.01, "content": {"uri": "far.glb"}, "boundingVolume": {"sphere": [0, 0, 0, 10.0]}}
	var far_transform := Transform3D(Basis(), far_center)
	var far_result := GWTiles3DTraversal.evaluate_tile(far_tile, far_transform, 50.0, aoi_bbox, aoi_center, aoi_radius)
	_check(far_result["action"] == "prune", "a tile far outside the AOI is pruned, got %s" % far_result)

	print("\ntest_tiles3d_traversal: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)


func _u32le(v: int) -> PackedByteArray:
	var b := PackedByteArray()
	b.resize(4)
	b.encode_u32(0, v)
	return b
