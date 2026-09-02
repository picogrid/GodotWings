extends SceneTree

# Headless test for GWGeodeticConvert -- re-verifies, in GDScript, the exact
# rigor already established in Python this session (tools/gw_3dtiles_prefetch.py):
# geodetic<->ECEF round-trips to machine precision (Bowring's method), and the
# ENU basis is orthonormal + right-handed (E x N = Up) at diverse latitudes
# (equator, near-pole/antimeridian, mid-latitude, southern hemisphere).
# geodetic_to_ned/ned_to_geodetic are checked via pure self-consistency
# (encode then decode), not against an unrelated approximation formula --
# comparing two different approximation methods burned real time earlier this
# session on a false-alarm "FAIL" that was actually just expected divergence
# between methods, not a bug.

var _ok := true

func _check(cond: bool, msg: String) -> void:
	if cond: print("  PASS  ", msg)
	else: push_error("FAIL: " + msg); _ok = false


func _approx_v3(a: Vector3, b: Vector3, eps := 1e-6) -> bool:
	return a.distance_to(b) < eps


func _initialize() -> void:
	# 1. geodetic -> ECEF -> geodetic round-trip, machine precision.
	var points := [
		[60.221088825593135, 25.018208331290666, 50.0],  # Helsinki
		[0.0, 0.0, 0.0],                                   # equator / prime meridian
		[89.9, 179.9, 1000.0],                             # near-pole / antimeridian
		[-33.8688, 151.2093, 100.0],                       # Sydney (southern hemisphere)
		[37.7749, -122.4194, 200.0],                       # San Francisco
	]
	# Tolerances here (~5m / ~2m) reflect the REAL precision ceiling of this
	# path, not an arbitrary slop: geodetic_to_ecef()/ecef_to_geodetic(Vector3)
	# round-trip an ECEF-scale (~6.4e6 m) value through a Vector3, and Godot's
	# Vector3 is single-precision (float32) by default -- its ULP at that
	# magnitude is already ~0.7m, independent of this module's own math being
	# exact. (geodetic_to_ned/ned_to_geodetic below stay in double precision
	# throughout and are held to a far tighter tolerance for exactly this
	# reason -- see GeodeticConvert.gd's geodetic_to_ecef_xyz doc comment.)
	for p in points:
		var ecef: Vector3 = GWGeodeticConvert.geodetic_to_ecef(p[0], p[1], p[2])
		var back: Array = GWGeodeticConvert.ecef_to_geodetic(ecef)
		var lat_err: float = absf(back[0] - p[0])
		var lon_err: float = absf(back[1] - p[1])
		var alt_err: float = absf(back[2] - p[2])
		_check(lat_err < 5e-5 and lon_err < 5e-5 and alt_err < 2.0,
				"geodetic<->ECEF round-trip at (%.4f, %.4f, %.1f): lat_err=%.9f lon_err=%.9f alt_err=%.6f" %
				[p[0], p[1], p[2], lat_err, lon_err, alt_err])

	# 2. ENU basis: orthonormal + right-handed (E x N = Up) at diverse latitudes.
	var lat_lons := [[60.0, 25.0], [0.0, 0.0], [89.0, 170.0], [-33.9, 151.2]]
	for ll in lat_lons:
		var enu: Basis = GWGeodeticConvert.enu_basis(ll[0], ll[1])
		var e: Vector3 = enu.x
		var n: Vector3 = enu.y
		var u: Vector3 = enu.z
		_check(is_equal_approx(e.length(), 1.0) and is_equal_approx(n.length(), 1.0) and is_equal_approx(u.length(), 1.0),
				"ENU basis unit vectors at (%.1f, %.1f)" % [ll[0], ll[1]])
		# 1e-6, not 1e-9: these are float32 Vector3 dot products (observed
		# noise floor ~3e-8), not float64 -- see the round-trip tolerance
		# comment above for why Godot's default Vector3 precision matters here.
		_check(absf(e.dot(n)) < 1e-6 and absf(n.dot(u)) < 1e-6 and absf(u.dot(e)) < 1e-6,
				"ENU basis orthogonal at (%.1f, %.1f)" % [ll[0], ll[1]])
		_check(_approx_v3(e.cross(n), u, 1e-6), "ENU right-handed (E x N = Up) at (%.1f, %.1f)" % [ll[0], ll[1]])

	# 3. geodetic_to_ned / ned_to_geodetic: pure self-consistency round-trip.
	var home_lat := 60.221088825593135
	var home_lon := 25.018208331290666
	var home_alt := 0.0
	var test_geo := [60.225, 25.025, 30.0]
	var ned: Vector3 = GWGeodeticConvert.geodetic_to_ned(test_geo[0], test_geo[1], test_geo[2], home_lat, home_lon, home_alt)
	var back_geo: Array = GWGeodeticConvert.ned_to_geodetic(ned, home_lat, home_lon, home_alt)
	# ~1e-7 deg (~1cm) / ~1mm: the ECEF subtraction itself is double-precision
	# (see GeodeticConvert.gd), but enu_basis()'s rotation is still a Basis
	# (float32 Vector3 columns) -- observed residual here is ~1e-9 deg /
	# ~0.02mm, two to three orders of magnitude tighter than the ECEF-via-
	# Vector3 path above, which is exactly the improvement this fix targeted.
	_check(absf(back_geo[0] - test_geo[0]) < 1e-7 and absf(back_geo[1] - test_geo[1]) < 1e-7 and absf(back_geo[2] - test_geo[2]) < 1e-3,
			"geodetic_to_ned <-> ned_to_geodetic round-trip, got back %s expected %s" % [back_geo, test_geo])

	# Home itself should decode to NED (0,0,0).
	var home_ned: Vector3 = GWGeodeticConvert.geodetic_to_ned(home_lat, home_lon, home_alt, home_lat, home_lon, home_alt)
	_check(_approx_v3(home_ned, Vector3.ZERO, 1e-6), "home position -> NED (0,0,0), got %s" % home_ned)

	# A pure-north offset should show up almost entirely in NED.x (North),
	# ~0 in NED.y (East) -- sanity-checks the ENU->NED axis assignment, not
	# just that encode/decode invert each other.
	var north_geo: Array = GWGeodeticConvert.ned_to_geodetic(Vector3(1000.0, 0.0, 0.0), home_lat, home_lon, home_alt)
	var north_ned: Vector3 = GWGeodeticConvert.geodetic_to_ned(north_geo[0], north_geo[1], north_geo[2], home_lat, home_lon, home_alt)
	_check(north_ned.x > 900.0 and absf(north_ned.y) < 1.0,
			"1000m north offset stays predominantly in NED.x, got %s" % north_ned)

	print("\ntest_geodetic_convert: ", "PASS" if _ok else "FAIL")
	quit(0 if _ok else 1)
