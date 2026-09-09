## Pure, network-free 3D Tiles traversal/pruning logic -- ports the exact
## algorithms already written and verified against real Cesium ion / Google
## Photorealistic 3D Tiles data in tools/gw_3dtiles_prefetch.py this session:
## AOI-vs-boundingVolume overlap tests (region/box/sphere), geometricError-
## threshold LOD stopping, tile transform-chain composition, and .b3dm
## unwrapping. Kept separate from GWTiles3DStreamer (which owns the actual
## networking/threading) so this can be unit-tested against synthetic
## tileset-shaped Dictionaries with no network or thread involved.
##
## boundingVolume/tileset.json dictionaries here are exactly what
## JSON.parse_string() produces from a real tileset.json -- no remapping.
class_name GWTiles3DTraversal
extends RefCounted

const R_EQ := GWGeodeticConvert.R_EQ


## Same flat-tangent approximation gw_terrain_import.py's own
## bbox_from_center used -- for AOI overlap testing (region volumes) only,
## not for any position math (that's all GWGeodeticConvert). Returns
## [min_lon, min_lat, max_lon, max_lat].
static func bbox_from_center(lat: float, lon: float, radius_km: float) -> Array:
	var r_m := radius_km * 1000.0
	var dlat := rad_to_deg(r_m / R_EQ)
	var dlon := rad_to_deg(r_m / (R_EQ * cos(deg_to_rad(lat))))
	return [lon - dlon, lat - dlat, lon + dlon, lat + dlat]


static func bbox_overlaps(a: Array, b: Array) -> bool:
	return a[0] <= b[2] and a[2] >= b[0] and a[1] <= b[3] and a[3] >= b[1]


## 3D Tiles `transform`: 16 numbers, COLUMN-major (glTF/3D-Tiles convention).
static func parse_gltf_transform(m: Array) -> Transform3D:
	var basis := Basis()
	basis.x = Vector3(m[0], m[1], m[2])
	basis.y = Vector3(m[4], m[5], m[6])
	basis.z = Vector3(m[8], m[9], m[10])
	return Transform3D(basis, Vector3(m[12], m[13], m[14]))


## Closest-point-on-OBB test against a sphere, entirely in Cartesian
## (ECEF-ish) space -- NOT a geodetic envelope of the box's corners. Verified
## live against the real Google Photorealistic 3D Tiles root tile, whose box
## is ~7645km half-extents centered on Earth's center (i.e. it encloses the
## whole planet): its 8 corners project to a tight, WRONG lat/lon envelope,
## because the box's FACES (not corners) pass nearest the poles and the
## antimeridian -- corner-sampling a box through a nonlinear (geodetic)
## projection simply doesn't bound it. A Cartesian test has no such failure
## mode. `box`: [centerX,Y,Z, halfX_x,y,z, halfY_x,y,z, halfZ_x,y,z] (12
## numbers) in the tile's own local frame, pushed through world_transform.
static func box_intersects_sphere(box: Array, world_transform: Transform3D,
		sphere_center: Vector3, sphere_radius: float) -> bool:
	var center_local := Vector3(box[0], box[1], box[2])
	var center_world := world_transform * center_local
	var r := world_transform.basis
	var axes_world: Array[Vector3] = [
		r * Vector3(box[3], box[4], box[5]),
		r * Vector3(box[6], box[7], box[8]),
		r * Vector3(box[9], box[10], box[11]),
	]

	var d := sphere_center - center_world
	var closest := center_world
	for axis: Vector3 in axes_world:
		var half_extent: float = axis.length()
		if half_extent < 1e-9:
			continue  # degenerate (flat) axis
		var axis_dir: Vector3 = axis / half_extent
		var dist_along: float = clampf(d.dot(axis_dir), -half_extent, half_extent)
		closest += dist_along * axis_dir
	return sphere_center.distance_to(closest) <= sphere_radius


## 3D Tiles `boundingVolume.sphere`: [centerX,Y,Z, radius] in the tile's
## local frame. Same Cartesian-space reasoning as box_intersects_sphere.
static func sphere_bv_intersects_sphere(bv_sphere: Array, world_transform: Transform3D,
		sphere_center: Vector3, sphere_radius: float) -> bool:
	var center_local := Vector3(bv_sphere[0], bv_sphere[1], bv_sphere[2])
	var radius: float = bv_sphere[3]
	var center_world := world_transform * center_local
	# Scale by the transform's own scale (assumed uniform across all three
	# axes -- true for every real transform seen this session).
	var scale := world_transform.basis.x.length()
	if scale < 1e-9:
		scale = 1.0
	return sphere_center.distance_to(center_world) <= sphere_radius + radius * scale


## `region` is native geodetic (a fixed EPSG:4979-like frame per the 3D
## Tiles spec, independent of the tile's own local transform) -- an exact
## comparison against the AOI's own geodetic bbox, no projection involved.
## `box`/`sphere` are tested in Cartesian space against
## (aoi_sphere_center, aoi_sphere_radius) (see box_intersects_sphere). An
## unrecognized volume type is NOT pruned (conservative: over-fetch rather
## than silently drop real content).
static func bounding_volume_overlaps_aoi(bv: Dictionary, world_transform: Transform3D,
		aoi_bbox: Array, aoi_sphere_center: Vector3, aoi_sphere_radius: float) -> bool:
	if bv.has("region"):
		var region: Array = bv["region"]
		var west := rad_to_deg(region[0])
		var south := rad_to_deg(region[1])
		var east := rad_to_deg(region[2])
		var north := rad_to_deg(region[3])
		return bbox_overlaps([west, south, east, north], aoi_bbox)
	if bv.has("box"):
		return box_intersects_sphere(bv["box"], world_transform, aoi_sphere_center, aoi_sphere_radius)
	if bv.has("sphere"):
		return sphere_bv_intersects_sphere(bv["sphere"], world_transform, aoi_sphere_center, aoi_sphere_radius)
	return true


## Converts a 3D Tiles bounding volume into one conservative sphere in the
## streamer's current anchor-local Godot frame, with `error_scale` carrying
## the cumulative transform's largest scale for geometric-error projection.
## `region` is already geodetic and therefore does not inherit the tile
## transform for its geometry. Unknown/malformed volumes deliberately return
## an infinite sphere so callers never cull data merely because a provider
## introduced a volume form we do not understand.
static func bounding_sphere(bv: Dictionary, world_transform: Transform3D,
		anchor_lat: float, anchor_lon: float, anchor_alt: float) -> Dictionary:
	if bv.has("sphere"):
		var sphere: Array = bv["sphere"]
		if sphere.size() >= 4:
			var ecef_center := world_transform * Vector3(
					float(sphere[0]), float(sphere[1]), float(sphere[2]))
			var error_scale := _basis_radius_scale(world_transform.basis)
			return {
				"center": GWGeodeticConvert.ecef_xyz_to_godot_position(
						ecef_center.x, ecef_center.y, ecef_center.z,
						anchor_lat, anchor_lon, anchor_alt),
				"radius": absf(float(sphere[3])) * error_scale,
				"error_scale": error_scale,
			}

	if bv.has("box"):
		var box: Array = bv["box"]
		if box.size() >= 12:
			var ecef_center := world_transform * Vector3(
					float(box[0]), float(box[1]), float(box[2]))
			var axis_x := world_transform.basis * Vector3(
					float(box[3]), float(box[4]), float(box[5]))
			var axis_y := world_transform.basis * Vector3(
					float(box[6]), float(box[7]), float(box[8]))
			var axis_z := world_transform.basis * Vector3(
					float(box[9]), float(box[10]), float(box[11]))
			var radius := maxf(
					maxf((axis_x + axis_y + axis_z).length(), (axis_x + axis_y - axis_z).length()),
					maxf((axis_x - axis_y + axis_z).length(), (-axis_x + axis_y + axis_z).length()))
			return {
				"center": GWGeodeticConvert.ecef_xyz_to_godot_position(
						ecef_center.x, ecef_center.y, ecef_center.z,
						anchor_lat, anchor_lon, anchor_alt),
				"radius": radius,
				"error_scale": _basis_radius_scale(world_transform.basis),
			}

	if bv.has("region"):
		var region: Array = bv["region"]
		if region.size() >= 6:
			var west := float(region[0])
			var south := float(region[1])
			var east := float(region[2])
			var north := float(region[3])
			var min_height := float(region[4])
			var max_height := float(region[5])
			# Unwrap eastward across the antimeridian before taking the
			# midpoint. The radius uses a surface-path upper bound rather than
			# a corner fit: corner-only spheres can cut through the ellipsoid's
			# curved edges and silently omit real terrain.
			var lon_span := fposmod(east - west, TAU)
			if is_equal_approx(lon_span, 0.0) and not is_equal_approx(east, west):
				lon_span = TAU
			var center_lon := wrapf(west + 0.5 * lon_span, -PI, PI)
			var center_lat := 0.5 * (south + north)
			var center_height := 0.5 * (min_height + max_height)
			var center_xyz := GWGeodeticConvert.geodetic_to_ecef_xyz(
					rad_to_deg(center_lat), rad_to_deg(center_lon), center_height)
			var min_abs_lat := 0.0 if south <= 0.0 and north >= 0.0 \
					else minf(absf(south), absf(north))
			var max_parallel_factor := cos(minf(min_abs_lat, PI * 0.5))
			var curvature_upper_bound := R_EQ / (1.0 - GWGeodeticConvert.E2) \
					+ maxf(absf(min_height), absf(max_height))
			var surface_path_bound := curvature_upper_bound * (
					0.5 * absf(north - south)
					+ minf(PI, 0.5 * lon_span) * max_parallel_factor)
			return {
				"center": GWGeodeticConvert.ecef_xyz_to_godot_position(
						center_xyz[0], center_xyz[1], center_xyz[2],
						anchor_lat, anchor_lon, anchor_alt),
				"radius": surface_path_bound + 0.5 * absf(max_height - min_height),
				"error_scale": _basis_radius_scale(world_transform.basis),
			}

	return {
		"center": Vector3.ZERO,
		"radius": INF,
		"error_scale": _basis_radius_scale(world_transform.basis),
	}


## Conservative largest stretch applied to a sphere and geometric error.
## The spectral norm squared of a basis is the largest eigenvalue of its Gram
## matrix A^T A; the maximum absolute row sum bounds that eigenvalue. This is
## exact for rotations/axis scales and remains safe for arbitrarily small
## shear, without a fragile "nearly orthogonal" epsilon branch.
static func _basis_radius_scale(basis: Basis) -> float:
	var xx := basis.x.length_squared()
	var yy := basis.y.length_squared()
	var zz := basis.z.length_squared()
	var xy := absf(basis.x.dot(basis.y))
	var xz := absf(basis.x.dot(basis.z))
	var yz := absf(basis.y.dot(basis.z))
	return sqrt(maxf(xx + xy + xz, maxf(yy + xy + yz, zz + xz + yz)))


## Conservative perspective-frustum sphere test in camera-axis space.
## Side-plane comparisons retain the sphere radius in plane-normal units, so
## a sphere crossing an off-axis boundary remains visible. Near/far are
## likewise intersection tests, not center tests.
static func bounds_visible(bounds: Dictionary, view: Dictionary) -> bool:
	var radius: float = float(bounds.get("radius", INF))
	if not is_finite(radius):
		return true
	radius = absf(radius)
	var center: Vector3 = bounds.get("center", Vector3.ZERO)
	var position: Vector3 = view.get("position", Vector3.ZERO)
	var forward: Vector3 = view.get("forward", Vector3.FORWARD)
	var right: Vector3 = view.get("right", Vector3.RIGHT)
	var up: Vector3 = view.get("up", Vector3.UP)
	var offset := center - position
	var depth := offset.dot(forward)
	var horizontal := offset.dot(right)
	var vertical := offset.dot(up)
	var near_distance := maxf(0.0, float(view.get("near", 0.0)))
	var far_distance := float(view.get("far", INF))
	if depth + radius < near_distance or depth - radius > far_distance:
		return false
	var tan_x := maxf(0.0, float(view.get("tan_half_fov_x", 0.0)))
	var tan_y := maxf(0.0, float(view.get("tan_half_fov_y", 0.0)))
	var horizontal_margin := radius * sqrt(1.0 + tan_x * tan_x)
	var vertical_margin := radius * sqrt(1.0 + tan_y * tan_y)
	return absf(horizontal) <= depth * tan_x + horizontal_margin \
			and absf(vertical) <= depth * tan_y + vertical_margin


## Maximum transformed geometric error, in pixels, among views whose frusta
## intersect the bounds. Distance is to the nearest point of the sphere,
## making camera-inside bounds request refinement conservatively.
static func screen_space_error(geometric_error: float, bounds: Dictionary, views: Array) -> float:
	if geometric_error <= 0.0:
		return 0.0
	if views.is_empty():
		return 0.0
	var radius: float = absf(float(bounds.get("radius", INF)))
	if not is_finite(radius):
		return INF
	var error_scale := absf(float(bounds.get("error_scale", 1.0)))
	if not is_finite(error_scale):
		return INF
	var center: Vector3 = bounds.get("center", Vector3.ZERO)
	var maximum := 0.0
	for view_variant in views:
		var view: Dictionary = view_variant
		if not bounds_visible(bounds, view):
			continue
		var distance := center.distance_to(view.get("position", Vector3.ZERO)) - radius
		if distance <= 1e-6:
			return INF
		var tan_y := float(view.get("tan_half_fov_y", 0.0))
		var viewport_height := float(view.get("viewport_height", 0.0))
		if tan_y <= 0.0 or viewport_height <= 0.0:
			return INF
		maximum = maxf(maximum,
				geometric_error * error_scale * viewport_height / (2.0 * tan_y * distance))
	return maximum


## b3dm = [28-byte header][feature table JSON+bin][batch table JSON+bin]
## [embedded glb]. Header: magic(4)='b3dm', version(4), byteLength(4),
## featureTableJSONByteLength(4), featureTableBinaryByteLength(4),
## batchTableJSONByteLength(4), batchTableBinaryByteLength(4). Returns data
## unchanged if it isn't b3dm (already a plain glb/gltf).
static func unwrap_b3dm(data: PackedByteArray) -> PackedByteArray:
	if data.size() < 28 or data.slice(0, 4).get_string_from_ascii() != "b3dm":
		return data
	var ft_json_len := data.decode_u32(12)
	var ft_bin_len := data.decode_u32(16)
	var bt_json_len := data.decode_u32(20)
	var bt_bin_len := data.decode_u32(24)
	var glb_start := 28 + ft_json_len + ft_bin_len + bt_json_len + bt_bin_len
	return data.slice(glb_start)


## Given one already-fetched/parsed tile Dictionary and its parent's
## composed transform, decides what to do -- WITHOUT any I/O. Mirrors
## gw_3dtiles_prefetch.py's Walker._walk_tile decision logic exactly (prune
## outside the AOI; stop and emit content once geometricError <= detail_m OR
## there are no children -- a tile with no content of its own, e.g. a pure
## LOD/grouping node, has nothing to stop on regardless of its
## geometricError and must recurse instead; otherwise recurse into
## children), just split out as a pure function so it's unit-testable
## against a synthetic tile tree with no network involved. Returns a
## Dictionary tagged by "action":
##   {"action": "prune"}
##   {"action": "content", "transform": Transform3D, "contents": [Dictionary, ...]}
##   {"action": "recurse", "children": [{"tile": Dictionary, "transform": Transform3D}, ...]}
##   {"action": "none"}  -- dead end: no content, no children
static func evaluate_tile(tile: Dictionary, parent_transform: Transform3D, detail_m: float,
		aoi_bbox: Array, aoi_sphere_center: Vector3, aoi_sphere_radius: float) -> Dictionary:
	var transform := parent_transform
	if tile.has("transform"):
		transform = parent_transform * parse_gltf_transform(tile["transform"])

	var bv: Dictionary = tile.get("boundingVolume", {})
	if not bounding_volume_overlaps_aoi(bv, transform, aoi_bbox, aoi_sphere_center, aoi_sphere_radius):
		return {"action": "prune"}

	var geometric_error: float = tile.get("geometricError", 0.0)
	var children: Array = tile.get("children", [])

	var contents = tile.get("contents", null)
	if contents == null and tile.has("content"):
		contents = [tile["content"]]
	if contents == null:
		contents = []

	if contents.size() > 0 and (geometric_error <= detail_m or children.size() == 0):
		return {"action": "content", "transform": transform, "contents": contents}
	elif children.size() > 0:
		var out := []
		for child in children:
			out.append({"tile": child, "transform": transform})
		return {"action": "recurse", "children": out}
	return {"action": "none"}


## A tile's absolute world_transform (already composed down from the
## tileset root -- see WalkResult below) -> its Godot-frame wrapper
## Transform3D relative to (anchor_lat, anchor_lon, anchor_alt). Position
## conversion stays in double precision (GWGeodeticConvert); this is the
## same math gw_3dtiles_prefetch.py's local_transform_to_godot does.
static func local_transform_to_godot(world_transform: Transform3D,
		anchor_lat: float, anchor_lon: float, anchor_alt: float) -> Transform3D:
	var o := world_transform.origin
	var pos := GWGeodeticConvert.ecef_xyz_to_godot_position(o.x, o.y, o.z, anchor_lat, anchor_lon, anchor_alt)
	var basis := GWGeodeticConvert.ecef_rotation_to_godot(world_transform.basis, anchor_lat, anchor_lon)
	return Transform3D(basis, pos)
