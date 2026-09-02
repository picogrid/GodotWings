@tool
@icon("res://addons/godotwings/sensors/camera_icon.svg")
## LOCAL DEBUGGING FIXTURE ONLY -- not the compliant way to use real Cesium
## ion / Google 3D Tiles data. Use GWTiles3DStreamer (Tiles3DStreamer.gd) for
## that: it fetches live every session and holds content in memory only,
## never touching disk. This node loads .glb tiles that
## tools/gw_3dtiles_prefetch.py downloaded and cached to disk -- exactly the
## "prefetch once, run offline" architecture that Cesium ion's and Google's
## actual Terms of Service prohibit (see gw_3dtiles_prefetch.py's module
## docstring for the pulled-live excerpts). Kept around for debugging the
## transform/placement math against a small area, ideally fed by a public
## unauthenticated sample tileset rather than a real ion token.
##
## Loads the cached .glb tiles and places each at the manifest's precomputed
## Transform3D — same "generate from an offline tool's output, no network at
## runtime" pattern as GWImportedTerrain, just for real building/terrain
## meshes instead of a heightmapped grid. No tileset.json traversal, no LOD
## selection, no network access here — that all happened once, offline, in
## the Python tool. Positioned so the AOI center (origin_lat/origin_lon in
## the manifest) sits at this node's own origin — same convention as
## GWImportedTerrain, so a GWVehicleBody's default spawn (NED 0,0) lands at
## the AOI center and the two systems compose in the same scene.
##
## The manifest/tiles directory is a SESSION-SCOPED CACHE (see
## gw_3dtiles_prefetch.py's docstring and .gitignore) — point manifest_path
## at wherever the tool last wrote .cache/3dtiles/.../manifest.json, not at
## a path you intend to commit.
class_name GWTiles3DLoader
extends Node3D

@export_file("*.json") var manifest_path: String = ""  ## manifest.json written by gw_3dtiles_prefetch.py
## Tick to reload after changing manifest_path (matches GWImportedTerrain's
## regenerate-on-demand pattern).
@export var regenerate: bool = false:
	set(v):
		if v:
			_generate()
## Refuse to load a manifest older than this (re-run gw_3dtiles_prefetch.py
## instead) — the tool's own `--purge-stale-hours` enforces the same thing
## on disk, but nothing made that automatic on the Godot side until this:
## caching real Cesium ion / Google 3D Tiles content is session-scoped by
## the provider's terms, not a license for a permanent local mirror, and a
## manifest sitting around for weeks untouched is exactly how "session-
## scoped" quietly turns into "permanent" if nothing ever checks. 0 disables
## the check (e.g. for a manifest you know is intentionally long-lived, like
## a public-sample fixture with no such restriction).
@export var max_cache_age_hours: float = 24.0

## Populated by _generate() from the manifest — read after _ready() for what
## the tool fetched (AOI, source tileset, fetch time).
var origin_lat: float = 0.0
var origin_lon: float = 0.0
var origin_alt: float = 0.0
var source_tileset: String = ""
var fetched_at: String = ""
var tile_count: int = 0
## Attribution HTML snippets from the ion endpoint response (see
## gw_3dtiles_prefetch.py's `content_attributions`). Cesium ion's and the
## content provider's terms REQUIRE these be displayed wherever the content
## itself is shown to anyone besides you — this property exists so your own
## UI can actually render them; it is not satisfied by just having this
## list exist unused. Empty for content with no attribution requirement
## (e.g. a public unauthenticated sample tileset).
var attributions: Array = []
var _content_axis_correction: String = ""


func _ready() -> void:
	_generate()


func _generate() -> void:
	for c in get_children():
		c.free()
	tile_count = 0
	if manifest_path.is_empty():
		return

	var manifest := _load_json(manifest_path)
	if manifest.is_empty():
		push_error("GWTiles3DLoader: could not read manifest %s" % manifest_path)
		return

	origin_lat = manifest.get("origin_lat", 0.0)
	origin_lon = manifest.get("origin_lon", 0.0)
	origin_alt = manifest.get("origin_alt", 0.0)
	source_tileset = manifest.get("source_tileset", "")
	fetched_at = manifest.get("fetched_at", "")
	_content_axis_correction = manifest.get("content_axis_correction", "")
	if _content_axis_correction == null:
		_content_axis_correction = ""
	attributions = manifest.get("content_attributions", [])
	if attributions == null:
		attributions = []
	for html in attributions:
		push_warning("GWTiles3DLoader: attribution required by the content provider's terms -- render this in your own UI: %s" % html)

	if max_cache_age_hours > 0.0 and not fetched_at.is_empty():
		var age_hours := (Time.get_unix_time_from_system() - Time.get_unix_time_from_datetime_string(fetched_at)) / 3600.0
		if age_hours > max_cache_age_hours:
			var msg := "GWTiles3DLoader: manifest %s is %.1fh old (max_cache_age_hours=%.1f) -- caching this content " + \
					"is session-scoped by the provider's terms, not a permanent mirror. Re-run " + \
					"gw_3dtiles_prefetch.py, or raise/disable max_cache_age_hours if this manifest is a fixture " + \
					"exempt from that (e.g. a public unauthenticated sample)."
			push_error(msg % [manifest_path, age_hours, max_cache_age_hours])
			return

	var tiles_dir := manifest_path.get_base_dir()
	var tiles: Array = manifest.get("tiles", [])
	for tile in tiles:
		_load_tile(tiles_dir, tile)


func _load_tile(tiles_dir: String, tile: Dictionary) -> void:
	var glb_path := tiles_dir.path_join(String(tile.get("file", "")))
	if not FileAccess.file_exists(glb_path):
		push_warning("GWTiles3DLoader: missing cached tile %s (re-run gw_3dtiles_prefetch.py?)" % glb_path)
		return

	var bytes := FileAccess.get_file_as_bytes(glb_path)
	var wrapper_transform := GWTiles3DContent.transform_from_manifest(tile)
	var tile_name: String = tile.get("file", "tile").get_basename()
	var wrapper := GWTiles3DContent.place_tile(self, bytes, wrapper_transform, _content_axis_correction, tile_name)
	if wrapper == null:
		push_warning("GWTiles3DLoader: failed to parse/place %s" % glb_path)
		return
	tile_count += 1


func _load_json(path: String) -> Dictionary:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		return {}
	var parsed: Variant = JSON.parse_string(f.get_as_text())
	return parsed if parsed is Dictionary else {}
