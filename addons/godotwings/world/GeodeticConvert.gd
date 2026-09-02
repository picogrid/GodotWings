## WGS84 geodetic (lat/lon/alt) <-> ECEF <-> NED conversions -- ports the exact
## math already written and verified (round-trips to machine precision at
## equator/pole/antimeridian/Helsinki/Sydney test points; ENU basis confirmed
## orthonormal + right-handed) in tools/gw_3dtiles_prefetch.py this session.
##
## Separate from GWCoordConvert (sitl/CoordConvert.gd): that module only maps
## NED<->Godot render space and has no lat/lon awareness at all. This module
## is the missing link -- real-world geodetic position <-> the vehicle's
## native NED frame, needed by anything that talks to a real-world data
## source (e.g. GWTiles3DStreamer) rather than just rendering.
##
## Frames: see sitl/CoordConvert.gd -- NED: x=North, y=East, z=Down. ECEF:
## X toward (0 deg lat, 0 deg lon), Y toward (0 deg lat, 90 deg E), Z toward
## the north pole. ENU here is a 3x3 Basis whose COLUMNS (.x/.y/.z) are the
## East/North/Up unit vectors in ECEF at a given lat/lon.
class_name GWGeodeticConvert
extends RefCounted

const R_EQ := 6378137.0                    ## WGS84 semi-major axis (m)
const FLATTENING := 1.0 / 298.257223563
const E2 := FLATTENING * (2.0 - FLATTENING) ## eccentricity squared


## Geodetic -> ECEF, kept as three plain (64-bit double) floats rather than a
## Vector3. ECEF magnitudes are ~6.4 million meters; Godot's Vector3 is
## single-precision (float32) by default, whose ULP at that magnitude is
## already ~1m -- fine for an absolute position, but subtracting two such
## Vector3s to get a small LOCAL offset (as geodetic_to_ned does) would
## amplify that into a ~meter-scale error in the small result via
## catastrophic cancellation, regardless of how physically close the two
## points are. Doing the subtraction on these plain doubles first, and only
## packing the (already small) result into a Vector3 afterward, avoids that
## entirely -- verified live: this was a real, silent precision bug caught
## by the round-trip test, not just an overly strict tolerance.
static func geodetic_to_ecef_xyz(lat_deg: float, lon_deg: float, alt_m: float) -> PackedFloat64Array:
	var lat := deg_to_rad(lat_deg)
	var lon := deg_to_rad(lon_deg)
	var sin_lat := sin(lat)
	var cos_lat := cos(lat)
	var n := R_EQ / sqrt(1.0 - E2 * sin_lat * sin_lat)
	var x := (n + alt_m) * cos_lat * cos(lon)
	var y := (n + alt_m) * cos_lat * sin(lon)
	var z := (n * (1.0 - E2) + alt_m) * sin_lat
	return PackedFloat64Array([x, y, z])


## Geodetic (deg, deg, meters above the WGS84 ellipsoid) -> ECEF (meters), as
## a Vector3. Fine for an absolute position on its own; see
## geodetic_to_ecef_xyz's doc comment before subtracting two of these.
static func geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Vector3:
	var xyz := geodetic_to_ecef_xyz(lat_deg, lon_deg, alt_m)
	return Vector3(xyz[0], xyz[1], xyz[2])


## ECEF (as three plain doubles, see geodetic_to_ecef_xyz) -> geodetic, via
## Bowring's closed-form method. Returns [lat_deg, lon_deg, alt_m].
static func ecef_xyz_to_geodetic(x: float, y: float, z: float) -> Array:
	var lon := atan2(y, x)
	var dist_xy := sqrt(x * x + y * y)
	var b := R_EQ * (1.0 - FLATTENING)
	var ep2 := (R_EQ * R_EQ - b * b) / (b * b)
	var theta := atan2(z * R_EQ, dist_xy * b)
	var sin_t := sin(theta)
	var cos_t := cos(theta)
	var lat := atan2(z + ep2 * b * sin_t * sin_t * sin_t, dist_xy - E2 * R_EQ * cos_t * cos_t * cos_t)
	var sin_lat := sin(lat)
	var n := R_EQ / sqrt(1.0 - E2 * sin_lat * sin_lat)
	var alt := dist_xy / cos(lat) - n
	return [rad_to_deg(lat), rad_to_deg(lon), alt]


## ECEF (meters, as a Vector3) -> geodetic. Returns [lat_deg, lon_deg, alt_m].
## Precision is bounded by whatever precision `p` itself already has (e.g. a
## tile's raw glTF-encoded position is float32-limited by Godot's glTF
## importer regardless of what this function does) -- for a case you control
## yourself (e.g. subtracting two absolute positions), prefer
## geodetic_to_ned/ned_to_geodetic, which stay in double precision throughout.
static func ecef_to_geodetic(p: Vector3) -> Array:
	return ecef_xyz_to_geodetic(p.x, p.y, p.z)


## Basis whose columns (.x/.y/.z) are the East, North, Up unit vectors (in
## ECEF) at (lat, lon) -- orthonormal, right-handed (E x N = Up).
static func enu_basis(lat_deg: float, lon_deg: float) -> Basis:
	var lat := deg_to_rad(lat_deg)
	var lon := deg_to_rad(lon_deg)
	var sl := sin(lat)
	var cl := cos(lat)
	var so := sin(lon)
	var co := cos(lon)
	var east := Vector3(-so, co, 0.0)
	var north := Vector3(-sl * co, -sl * so, cl)
	var up := Vector3(cl * co, cl * so, sl)
	return Basis(east, north, up)  # Basis(a,b,c) sets .x=a, .y=b, .z=c (columns) directly


## Real-world geodetic position -> the vehicle's native NED offset from
## (home_lat, home_lon, home_alt). NED = (North, East, -Up).
static func geodetic_to_ned(lat_deg: float, lon_deg: float, alt_m: float,
		home_lat: float, home_lon: float, home_alt: float) -> Vector3:
	var home_xyz := geodetic_to_ecef_xyz(home_lat, home_lon, home_alt)
	var target_xyz := geodetic_to_ecef_xyz(lat_deg, lon_deg, alt_m)
	# Subtract as doubles first (see geodetic_to_ecef_xyz) -- the result is
	# small (the actual offset, typically meters to a few km), so packing
	# it into a Vector3 here loses essentially nothing.
	var offset := Vector3(target_xyz[0] - home_xyz[0], target_xyz[1] - home_xyz[1], target_xyz[2] - home_xyz[2])
	var enu := enu_basis(home_lat, home_lon)
	var local := enu.transposed() * offset  # (E, N, U)
	return Vector3(local.y, local.x, -local.z)


## Inverse of geodetic_to_ned: a NED offset from (home_lat, home_lon,
## home_alt) -> real-world geodetic [lat_deg, lon_deg, alt_m].
static func ned_to_geodetic(ned: Vector3, home_lat: float, home_lon: float, home_alt: float) -> Array:
	var home_xyz := geodetic_to_ecef_xyz(home_lat, home_lon, home_alt)
	var enu := enu_basis(home_lat, home_lon)
	var enu_vec := Vector3(ned.y, ned.x, -ned.z)  # (E, N, U)
	# enu * enu_vec is the small offset (fine as a Vector3); add it to the
	# double-precision home position component-wise so the large absolute
	# result never round-trips through a float32 Vector3 before decoding.
	var delta := enu * enu_vec
	return ecef_xyz_to_geodetic(home_xyz[0] + delta.x, home_xyz[1] + delta.y, home_xyz[2] + delta.z)


## ENU (East, North, Up) -> Godot EUS (East, Up, South) axes: Godot's East =
## ENU's East, Godot's Up = ENU's Up, Godot's -Z (South, per GWCoordConvert)
## = -ENU's North. Same permutation gw_3dtiles_prefetch.py's _ENU_TO_EUS used.
static func _enu_to_eus(v: Vector3) -> Vector3:
	return Vector3(v.x, v.z, -v.y)


## An absolute ECEF position (as three plain doubles -- see
## geodetic_to_ecef_xyz; e.g. a tile's world-space translation, however you
## arrived at it) -> a Godot-frame position (East, Up, South) relative to
## (anchor_lat, anchor_lon, anchor_alt). Stays in double precision for the
## subtraction, same reasoning as geodetic_to_ned.
static func ecef_xyz_to_godot_position(x: float, y: float, z: float,
		anchor_lat: float, anchor_lon: float, anchor_alt: float) -> Vector3:
	var anchor_xyz := geodetic_to_ecef_xyz(anchor_lat, anchor_lon, anchor_alt)
	var offset := Vector3(x - anchor_xyz[0], y - anchor_xyz[1], z - anchor_xyz[2])
	var enu := enu_basis(anchor_lat, anchor_lon)
	return _enu_to_eus(enu.transposed() * offset)


## An ECEF-frame rotation (e.g. a tile's world-space basis) -> a Godot-frame
## rotation relative to (anchor_lat, anchor_lon). Pure rotation, no
## precision concerns the way position has -- ordinary Vector3/Basis
## arithmetic is fine here.
static func ecef_rotation_to_godot(rot: Basis, anchor_lat: float, anchor_lon: float) -> Basis:
	var enu := enu_basis(anchor_lat, anchor_lon)
	var rot_in_enu := enu.transposed() * rot
	var b := Basis()
	b.x = _enu_to_eus(rot_in_enu.x)
	b.y = _enu_to_eus(rot_in_enu.y)
	b.z = _enu_to_eus(rot_in_enu.z)
	return b
