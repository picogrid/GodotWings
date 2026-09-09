extends SceneTree

# Synthetic, network-free coverage for camera-aware 3D Tiles bounds and SSE.

var _ok := true


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  PASS  ", message)
	else:
		push_error("FAIL: " + message)
		_ok = false


func _view(position: Vector3, forward: Vector3, tan_half_fov: float = 1.0,
		viewport_height: float = 1000.0, near_distance: float = 1.0,
		far_distance: float = 1000.0) -> Dictionary:
	return {
		"position": position,
		"forward": forward.normalized(),
		"right": Vector3.RIGHT,
		"up": Vector3.UP,
		"tan_half_fov_x": tan_half_fov,
		"tan_half_fov_y": tan_half_fov,
		"viewport_height": viewport_height,
		"near": near_distance,
		"far": far_distance,
	}


func _initialize() -> void:
	var anchor_ecef := GWGeodeticConvert.geodetic_to_ecef(0.0, 0.0, 0.0)
	var scaled_basis := Basis(
			Vector3(2.0, 0.0, 0.0),
			Vector3(0.0, 3.0, 0.0),
			Vector3(0.0, 0.0, 4.0))
	var scaled_transform := Transform3D(scaled_basis, anchor_ecef)
	var scaled_sphere := GWTiles3DTraversal.bounding_sphere(
			{"sphere": [0.0, 0.0, 0.0, 10.0]}, scaled_transform, 0.0, 0.0, 0.0)
	_check(scaled_sphere["center"].length() < 0.1,
			"sphere center is converted into the anchor-local frame")
	_check(absf(scaled_sphere["radius"] - 40.0) < 0.001,
			"sphere radius uses the transform's largest scale")
	_check(absf(scaled_sphere["error_scale"] - 4.0) < 0.001,
			"bounds retain the cumulative transform scale for geometric error")

	var scaled_box := GWTiles3DTraversal.bounding_sphere(
			{"box": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0]},
			scaled_transform, 0.0, 0.0, 0.0)
	_check(absf(scaled_box["radius"] - sqrt(184.0)) < 0.001,
			"box sphere encloses transformed corners under non-uniform scale")

	# Regions use radians and cross the antimeridian eastward. They must ignore
	# the tile transform and conservatively include curved surface edges.
	var region := {"region": [deg_to_rad(179.0), deg_to_rad(-1.0),
			deg_to_rad(-179.0), deg_to_rad(1.0), 0.0, 100.0]}
	var region_bounds := GWTiles3DTraversal.bounding_sphere(
			region, Transform3D(Basis(), Vector3(999999.0, 0.0, 0.0)), 0.0, 180.0, 50.0)
	var north_edge_xyz := GWGeodeticConvert.geodetic_to_ecef_xyz(1.0, 180.0, 100.0)
	var north_edge := GWGeodeticConvert.ecef_xyz_to_godot_position(
			north_edge_xyz[0], north_edge_xyz[1], north_edge_xyz[2], 0.0, 180.0, 50.0)
	_check(region_bounds["center"].length() < 1.0,
			"antimeridian region midpoint is anchor-local and transform-independent")
	_check(region_bounds["center"].distance_to(north_edge) <= region_bounds["radius"],
			"region sphere conservatively contains a curved surface edge")

	var view := _view(Vector3.ZERO, Vector3(0.0, 0.0, -1.0))
	var scaled_lod_transform := Transform3D(
			scaled_basis, anchor_ecef + Vector3(0.0, 0.0, 100.0))
	var scaled_lod_bounds := GWTiles3DTraversal.bounding_sphere(
			{"sphere": [0.0, 0.0, 0.0, 2.0]}, scaled_lod_transform, 0.0, 0.0, 0.0)
	var unscaled_error := GWTiles3DTraversal.screen_space_error(10.0, {
		"center": scaled_lod_bounds["center"],
		"radius": scaled_lod_bounds["radius"],
		"error_scale": 1.0,
	}, [view])
	var transformed_error := GWTiles3DTraversal.screen_space_error(
			10.0, scaled_lod_bounds, [view])
	_check(absf(transformed_error - 4.0 * unscaled_error) < 0.001,
			"non-uniform cumulative scale multiplies geometric error by its largest axis")

	var near_shear_basis := Basis(
			Vector3(1.0, 0.0, 0.0),
			Vector3(0.0000009, 1.0, 0.0),
			Vector3(0.0, 0.0, 1.0))
	var near_shear_transform := Transform3D(
			near_shear_basis, anchor_ecef + Vector3(0.0, 0.0, 100.0))
	var near_shear_bounds := GWTiles3DTraversal.bounding_sphere(
			{"sphere": [0.0, 0.0, 0.0, 10.0]}, near_shear_transform, 0.0, 0.0, 0.0)
	var stretched_direction := Vector3(1.0, 1.0, 0.0).normalized()
	var stretched_radius := (near_shear_basis * stretched_direction).length() * 10.0
	_check(near_shear_bounds["radius"] + 0.0000001 >= stretched_radius,
			"near-orthogonal shear still produces a sphere containing transformed points")
	var no_shear_scale_error := GWTiles3DTraversal.screen_space_error(10.0, {
		"center": near_shear_bounds["center"],
		"radius": near_shear_bounds["radius"],
		"error_scale": 1.0,
	}, [view])
	var near_shear_error := GWTiles3DTraversal.screen_space_error(
			10.0, near_shear_bounds, [view])
	_check(near_shear_error > no_shear_scale_error * 1.0000004,
			"sub-epsilon shear conservatively scales projected geometric error")
	_check(not GWTiles3DTraversal.bounds_visible(
			{"center": Vector3(0.0, 0.0, 10.0), "radius": 1.0}, view),
			"sphere wholly behind the camera is rejected")
	_check(GWTiles3DTraversal.bounds_visible(
			{"center": Vector3(105.0, 0.0, -100.0), "radius": 4.0}, view),
			"sphere intersecting an off-axis side plane remains visible")
	_check(not GWTiles3DTraversal.bounds_visible(
			{"center": Vector3(107.0, 0.0, -100.0), "radius": 4.0}, view),
			"sphere wholly outside an off-axis side plane is rejected")
	_check(GWTiles3DTraversal.bounds_visible(
			{"center": Vector3(0.0, 0.0, -0.5), "radius": 0.5}, view),
			"sphere touching the near plane remains visible")
	_check(not GWTiles3DTraversal.bounds_visible(
			{"center": Vector3(0.0, 0.0, -1011.0), "radius": 10.0}, view),
			"sphere wholly beyond the far plane is rejected")

	var bounds := {"center": Vector3(0.0, 0.0, -100.0), "radius": 10.0}
	var wide_error := GWTiles3DTraversal.screen_space_error(10.0, bounds, [view])
	var zoomed_view := _view(Vector3.ZERO, Vector3(0.0, 0.0, -1.0), 0.5)
	var zoomed_error := GWTiles3DTraversal.screen_space_error(10.0, bounds, [zoomed_view])
	_check(wide_error < 80.0 and zoomed_error > 80.0,
			"narrower FOV raises pixel error and crosses a refinement threshold")
	_check(absf(zoomed_error - 2.0 * wide_error) < 0.001,
			"projected error scales inversely with tan(half vertical FOV)")

	var away_view := _view(Vector3.ZERO, Vector3(0.0, 0.0, 1.0), 0.25)
	var close_view := _view(Vector3(0.0, 0.0, 50.0), Vector3(0.0, 0.0, -1.0), 1.0)
	var close_error := GWTiles3DTraversal.screen_space_error(10.0, bounds, [close_view])
	var multi_error := GWTiles3DTraversal.screen_space_error(10.0, bounds, [away_view, close_view])
	_check(absf(multi_error - close_error) < 0.001,
			"two-camera selection uses the maximum error among intersecting views")
	_check(GWTiles3DTraversal.screen_space_error(10.0, bounds, [away_view]) == 0.0,
			"geometry invisible to every camera has zero screen-space error")
	_check(GWTiles3DTraversal.screen_space_error(10.0, bounds, []) == 0.0,
			"an empty camera snapshot has zero screen-space error")
	_check(is_inf(GWTiles3DTraversal.screen_space_error(
			10.0, {"center": Vector3.ZERO, "radius": 1.0}, [view])),
			"camera inside a sphere requests refinement conservatively")

	print("\ntest_camera_selection: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
