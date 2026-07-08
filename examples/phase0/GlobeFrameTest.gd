extends Node3D
## Phase 0 validation — does GodotWings' render frame line up with the Cesium globe?
##
## The whole globe plan rests on ONE assumption: in CesiumGeoreference's
## CartographicOrigin mode, the engine frame is a local ENU/EUS tangent at the
## origin lat/lon/alt, and that tangent coincides with GWCoordConvert.ned_to_world
## (East=+X, Up=+Y, South=+Z). This scene proves (or disproves) that WITHOUT a
## Cesium Ion token or any streamed tiles — it uses the georeference's own
## (network-free, convention-exact) coordinate math:
##   * lat_lon_alt_rad_to_ecef()  : home geodetic -> ECEF origin
##   * eus_at_ecef()              : the East/Up/South basis at a point (ECEF)
##   * ecef_to_lat_lon_alt_deg()  : ECEF -> geodetic, to read where a probe landed
## It maps a few known NED offsets through ned_to_world + the EUS basis and checks
## they land North/East/Up on the WGS84 ellipsoid.
##
## HOW TO RUN
##   Quick (numeric only): just run this scene. It creates a CesiumGeoreference,
##   runs the checks, and prints PASS/FAIL to the Output panel. No network needed.
##
##   Visual (optional): in the editor, add a CesiumGeoreference under this root via
##   the Cesium dock, then add a Cesium World Terrain tileset under it (sets the Ion
##   token for you). Run again — this script reuses the existing georeference, so the
##   terrain renders and you can eyeball the North/East/Up markers on the globe.
##
## READING THE RESULT
##   PASS  -> ned_to_world already matches Cesium's EUS tangent. No correction
##            needed; proceed to Phase 1 as planned.
##   FAIL  -> the per-axis deltas show which engine axis maps to which geodetic
##            direction, telling us what fixed correction Basis to fold into the
##            NED->world render path.

## Geodetic home — match the SITL HOME_LOCATION you fly from.
@export var home_lat: float = -35.363261
@export var home_lon: float = 149.165230
@export var home_alt: float = 584.0
## Offset distance used for each axis probe (m). Big enough to read cleanly.
@export var test_dist: float = 1000.0
## Spawn coloured markers + a camera so an (optional) terrain tileset is viewable.
@export var build_visuals: bool = true

# Cesium origin-type enum value for CartographicOrigin (lat/lon/alt drive origin).
const CARTOGRAPHIC_ORIGIN := 0


func _ready() -> void:
	if not ClassDB.class_exists("CesiumGeoreference"):
		push_error("Phase0: 'CesiumGeoreference' class not found. Enable the cesium_godot " +
				"extension and use a build with the native lib for this platform. (On macOS, " +
				"if the dylib was downloaded via a browser, clear quarantine: " +
				"xattr -dr com.apple.quarantine addons/cesium_godot/lib/)")
		return

	var geo := _find_georeference(self)
	var created := false
	if geo == null:
		geo = ClassDB.instantiate("CesiumGeoreference")
		geo.name = "CesiumGeoreference"
		add_child(geo)
		created = true
	geo.set("origin_type", CARTOGRAPHIC_ORIGIN)
	geo.set("latitude", home_lat)
	geo.set("longitude", home_lon)
	geo.set("altitude", home_alt)

	# Let the native node settle (also lets an editor-added tileset start streaming).
	await get_tree().process_frame
	await get_tree().process_frame

	_run_checks(geo, created)
	if build_visuals:
		_build_visuals()


# --- the actual validation ---------------------------------------------------
func _run_checks(geo: Node, created: bool) -> void:
	if not geo.has_method("eus_at_ecef") or not geo.has_method("lat_lon_alt_rad_to_ecef"):
		push_error("Phase0: node is not a usable CesiumGeoreference (missing native converters).")
		return

	# Home in ECEF, and the East/Up/South basis there (columns: X=East, Y=Up, Z=South).
	var origin_ecef: Vector3 = geo.call("lat_lon_alt_rad_to_ecef",
			Vector3(deg_to_rad(home_lat), deg_to_rad(home_lon), home_alt))
	var eus: Basis = geo.call("eus_at_ecef", origin_ecef)
	var up_ecef := eus.y.normalized()

	print("\n========== Phase 0: Globe frame validation ==========")
	print("CesiumGeoreference : %s" % ("created at runtime" if created else "found in scene"))
	print("Home geodetic      : lat=%.6f  lon=%.6f  alt=%.1f" % [home_lat, home_lon, home_alt])
	print("Origin ECEF (m)    : (%.1f, %.1f, %.1f)" % [origin_ecef.x, origin_ecef.y, origin_ecef.z])
	print("EUS basis @ origin : East=%s Up=%s South=%s"
			% [_v(eus.x.normalized()), _v(eus.y.normalized()), _v(eus.z.normalized())])
	print("-----------------------------------------------------")

	var coslat := cos(deg_to_rad(home_lat))
	# Each probe: an NED offset and which geodetic axis should respond, plus its
	# expected magnitude (flat-tangent approximation) and whether the move should be
	# tangent to (up-dot 0) or along (up-dot 1) the local up. The PASS test checks
	# the axis MAPPING — correct sign, on-axis magnitude within 5%, off-axis terms
	# small, and clean tangency. The leftover off-axis "coupling" is the expected
	# WGS84 ellipsoid curvature (Cesium models it exactly; the flat FDM ignores it).
	var probes := [
		{"name": "North", "ned": Vector3(test_dist, 0, 0),
			"axis": "lat", "exp": rad_to_deg(test_dist / 6378137.0), "exp_updot": 0.0},
		{"name": "East ", "ned": Vector3(0, test_dist, 0),
			"axis": "lon", "exp": rad_to_deg(test_dist / (6378137.0 * coslat)), "exp_updot": 0.0},
		{"name": "Up   ", "ned": Vector3(0, 0, -test_dist),  # NED down is +z, so up = -z
			"axis": "alt", "exp": test_dist, "exp_updot": 1.0},
	]

	var all_pass := true
	for p in probes:
		var engine: Vector3 = GWCoordConvert.ned_to_world(p["ned"])  # NED -> Godot (EUS)
		var d_ecef := eus * engine                                    # EUS metres -> ECEF delta
		var ecef := origin_ecef + d_ecef
		var g: Vector3 = geo.call("ecef_to_lat_lon_alt_deg", ecef)    # -> (lat, lon, alt) deg
		var d_lat := g.x - home_lat
		var d_lon := g.y - home_lon
		var d_alt := g.z - home_alt
		var updot := d_ecef.normalized().dot(up_ecef)

		# On-axis response and the two off-axis "coupling" terms, in comparable units:
		# horizontal arcs as metres (deg * arc-per-deg), altitude already in metres.
		var lat_m := deg_to_rad(d_lat) * 6378137.0
		var lon_m := deg_to_rad(d_lon) * 6378137.0 * coslat
		var on_axis: float
		var off_axis: float
		var exp_m: float = (p["exp"] if p["axis"] == "alt" else deg_to_rad(p["exp"])
				* (6378137.0 if p["axis"] == "lat" else 6378137.0 * coslat))
		match p["axis"]:
			"lat": on_axis = lat_m; off_axis = maxf(abs(lon_m), abs(d_alt))
			"lon": on_axis = lon_m; off_axis = maxf(abs(lat_m), abs(d_alt))
			_:     on_axis = d_alt; off_axis = maxf(abs(lat_m), abs(lon_m))

		# PASS: on-axis within 5% of expected metres, off-axis coupling < 1% of the
		# move, and tangency (up-dot) as expected within 0.02.
		var ok_axis := _close(on_axis, exp_m, abs(exp_m) * 0.05)
		var ok_off := off_axis < test_dist * 0.01
		var ok_up := _close(updot, p["exp_updot"], 0.02)
		var ok: bool = ok_axis and ok_off and ok_up
		all_pass = all_pass and ok

		print("NED %s +%.0fm -> engine(%.1f, %.1f, %.1f)"
				% [p["name"], test_dist, engine.x, engine.y, engine.z])
		print("   on-axis(%s)=%+.2fm (exp %+.2fm)  coupling=%.2fm  up-dot=%+.4f (exp %+.2f)  -> %s"
				% [p["axis"], on_axis, exp_m, off_axis, updot, p["exp_updot"], "OK" if ok else "FAIL"])
		print("   geodetic d: dLat=%+.6f  dLon=%+.6f  dAlt=%+.2fm  (off-axis = ellipsoid curvature)"
				% [d_lat, d_lon, d_alt])

	print("-----------------------------------------------------")
	if all_pass:
		print("RESULT: PASS  ned_to_world matches Cesium CartographicOrigin EUS tangent.")
		print("        Axes align (N->+lat, E->+lon, Up->+alt); only sub-% ellipsoid")
		print("        coupling remains. No frame correction needed — proceed to Phase 1.")
	else:
		push_warning("Phase0 RESULT: FAIL — an axis is mis-mapped (see on-axis/coupling " +
				"above); a fixed correction Basis is needed in the NED->world render path.")
		print("RESULT: FAIL  (see on-axis vs coupling above)")
	print("=====================================================\n")


# --- helpers -----------------------------------------------------------------
func _close(a: float, b: float, tol: float) -> bool:
	return abs(a - b) <= tol


func _v(v: Vector3) -> String:
	return "(%+.3f,%+.3f,%+.3f)" % [v.x, v.y, v.z]


func _scene_root() -> Node:
	return get_tree().current_scene if get_tree() and get_tree().current_scene else self


func _find_camera(n: Node) -> Camera3D:
	if n is Camera3D:
		return n
	for c in n.get_children():
		var f := _find_camera(c)
		if f != null:
			return f
	return null


func _find_georeference(n: Node) -> Node:
	if n.is_class("CesiumGeoreference") or n.get_class() == "CesiumGeoreference":
		return n
	for c in n.get_children():
		var f := _find_georeference(c)
		if f != null:
			return f
	return null


## Coloured markers at the origin and each NED probe, plus a camera + sun so an
## optional terrain tileset is viewable. Origin=white, North=red, East=green, Up=blue.
func _build_visuals() -> void:
	_marker(Vector3.ZERO, Color.WHITE, 40.0)
	_marker(GWCoordConvert.ned_to_world(Vector3(test_dist, 0, 0)), Color.RED, 30.0)
	_marker(GWCoordConvert.ned_to_world(Vector3(0, test_dist, 0)), Color.GREEN, 30.0)
	_marker(GWCoordConvert.ned_to_world(Vector3(0, 0, -test_dist)), Color.BLUE, 30.0)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-50, -40, 0)
	add_child(sun)

	# If you've added your own camera (e.g. a CesiumDynamicCam), don't fight it for
	# the active view — just leave the markers and let your camera drive.
	if _find_camera(_scene_root()) != null:
		return

	var cam := Camera3D.new()
	# Match Cesium's CartographicOrigin near/far. A tiny near with this huge far
	# degenerates the directional-light frustum culler (the "prepare_camera: !res"
	# spam), so use near=9 like AbstractCesiumCamera does.
	cam.near = 9.0
	cam.far = 35358652.0
	cam.position = Vector3(test_dist * 2.0, test_dist * 2.0, test_dist * 3.0)
	add_child(cam)  # must be in-tree before look_at (needs a global transform)
	cam.look_at(GWCoordConvert.ned_to_world(Vector3(test_dist * 0.5, test_dist * 0.5, -test_dist * 0.5)), Vector3.UP)
	cam.current = true


func _marker(pos: Vector3, color: Color, radius: float) -> void:
	var m := MeshInstance3D.new()
	var sphere := SphereMesh.new()
	sphere.radius = radius
	sphere.height = radius * 2.0
	m.mesh = sphere
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	mat.emission_enabled = true
	mat.emission = color
	m.material_override = mat
	m.position = pos
	add_child(m)
