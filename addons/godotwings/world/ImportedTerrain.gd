@tool
@icon("res://addons/godotwings/sensors/camera_icon.svg")
## Real-world terrain baked by tools/gw_terrain_import.py: a heightmapped mesh
## textured with real satellite imagery (Sentinel-2 L2A + Copernicus DEM
## GLO-30), built from the tool's output files — same "generate procedurally"
## pattern as examples/Terrain.gd, just driven by real data instead of noise.
## No network access here; the download/reprojection happens offline, once,
## in the Python tool.
##
## Positioned so the tile's geographic center sits at this node's own origin —
## leave this node's transform at identity and a GWVehicleBody's default spawn
## (NED 0,0) lands exactly there: the AOI center IS the takeoff point. Set
## HOME_LOCATION (docker-compose.yml) / GWGeoReference.home_lat/home_lon to
## `center_lat`/`center_lon` (also printed by the tool) so ArduPilot's own GPS
## origin matches too.
class_name GWImportedTerrain
extends MeshInstance3D

@export_file("*.png") var heightmap_path: String = ""  ## R+G-packed 16-bit height (see _decode_height16)
@export_file("*.png") var texture_path: String = ""    ## satellite imagery, same extent as the heightmap
@export_file("*.json") var metadata_path: String = ""  ## sidecar gw_terrain_import.py writes alongside them
## Downsamples the heightmap grid to at most this many vertices per side.
@export var max_vertices_per_side: int = 200
## Tick to rebuild after changing the exports above (matches Terrain.gd's own
## regenerate-on-demand pattern — rebuilding on every keystroke while editing
## a path would be wasteful).
@export var regenerate: bool = false:
	set(v):
		if v:
			_generate()

## Populated by _generate() from metadata_path — read these after _ready()
## for what the tool computed (elevation range, source scene ids, ...).
var size_m: float = 0.0
var elevation_min_m: float = 0.0
var elevation_max_m: float = 0.0
var elevation_center_m: float = 0.0
var center_lat: float = 0.0
var center_lon: float = 0.0


func _ready() -> void:
	_generate()


func _generate() -> void:
	for c in get_children():
		if c is StaticBody3D:
			c.free()
	if heightmap_path.is_empty() or texture_path.is_empty() or metadata_path.is_empty():
		return

	var meta := _load_metadata(metadata_path)
	if meta.is_empty():
		push_error("GWImportedTerrain: could not read metadata %s" % metadata_path)
		return
	size_m = meta.get("size_m", 0.0)
	elevation_min_m = meta.get("elevation_min_m", 0.0)
	elevation_max_m = meta.get("elevation_max_m", 0.0)
	elevation_center_m = meta.get("elevation_center_m", 0.0)
	center_lat = meta.get("center_lat", 0.0)
	center_lon = meta.get("center_lon", 0.0)

	var hm := _load_png(heightmap_path)
	if hm == null:
		push_error("GWImportedTerrain: could not load heightmap %s" % heightmap_path)
		return

	var tex := _load_png(texture_path)
	var has_tex := tex != null
	if not has_tex:
		push_warning("GWImportedTerrain: could not load texture %s; terrain will be untextured." % texture_path)

	mesh = _build_mesh(hm)

	var mat := StandardMaterial3D.new()
	if has_tex:
		mat.albedo_texture = ImageTexture.create_from_image(tex)
	material_override = mat

	create_trimesh_collision()  # StaticBody3D + ConcavePolygonShape3D on layer 1, matches Terrain.gd


func _load_metadata(path: String) -> Dictionary:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		return {}
	var parsed: Variant = JSON.parse_string(f.get_as_text())
	return parsed if parsed is Dictionary else {}


## Read + decode a PNG via raw bytes rather than Image.load(path) — behaviorally
## identical when running from source, but avoids a Godot warning about
## res:// image loads not working in an exported/packed build (verified: no
## warning either way here, since this is a source/editor-time dev tool, but
## this is the cleaner form regardless).
func _load_png(path: String) -> Image:
	if not FileAccess.file_exists(path):
		return null
	var bytes := FileAccess.get_file_as_bytes(path)
	var img := Image.new()
	return img if img.load_png_from_buffer(bytes) == OK else null


## Decode one R+G-packed 16-bit height sample: hi=round(r*255), lo=round(g*255),
## h16=hi*256+lo. Godot's Image has no true 16-bit single-channel format —
## Image.load() on a 16-bit grayscale PNG silently truncates to FORMAT_L8 (256
## levels), verified directly rather than assumed — so gw_terrain_import.py
## packs the value across two 8-bit-exact channels of an ordinary RGB8 PNG
## instead. B is unused.
func _decode_height16(c: Color) -> int:
	var hi := roundi(c.r * 255.0)
	var lo := roundi(c.g * 255.0)
	return hi * 256 + lo


## Builds the grid mesh: vertex (i, j) sits at the geographic location of
## heightmap pixel (px_col, px_row) — easting/northing computed exactly like
## gw_terrain_import.py's own AEQD grid (rasterio.transform.from_origin(-half,
## half, pixel_size, pixel_size)), then mapped into Godot axes the same way
## GWCoordConvert.ned_to_world does (x=East, z=-North, y=up) — so this tile
## composes correctly with the rest of the scene's NED-based world.
func _build_mesh(hm: Image) -> ArrayMesh:
	var hm_px := hm.get_width()
	var grid_n := maxi(mini(max_vertices_per_side, hm_px), 2)
	var half := size_m * 0.5
	var erange := maxf(elevation_max_m - elevation_min_m, 0.0001)

	# Sample once per grid vertex (not once per triangle-corner) so shared
	# vertices between adjacent triangles read the same value.
	var positions: Array[Vector3] = []
	var uvs: Array[Vector2] = []
	positions.resize(grid_n * grid_n)
	uvs.resize(grid_n * grid_n)
	for j in grid_n:
		var t_row := float(j) / float(grid_n - 1)
		var px_row := roundi(t_row * (hm_px - 1))
		var northing := half - t_row * size_m
		for i in grid_n:
			var t_col := float(i) / float(grid_n - 1)
			var px_col := roundi(t_col * (hm_px - 1))
			var easting := -half + t_col * size_m
			var h16 := _decode_height16(hm.get_pixel(px_col, px_row))
			var elevation := elevation_min_m + (float(h16) / 65535.0) * erange
			var y := elevation - elevation_center_m
			var idx := j * grid_n + i
			positions[idx] = Vector3(easting, y, -northing)
			uvs[idx] = Vector2(t_col, t_row)

	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for j in grid_n - 1:
		for i in grid_n - 1:
			var i00 := j * grid_n + i
			var i10 := j * grid_n + i + 1
			var i01 := (j + 1) * grid_n + i
			var i11 := (j + 1) * grid_n + i + 1
			# Same winding as Terrain.gd's own grid (normals face +Y via generate_normals()).
			for idx in [i00, i10, i01, i10, i11, i01]:
				st.set_uv(uvs[idx])
				st.add_vertex(positions[idx])
	st.generate_normals()
	return st.commit()
