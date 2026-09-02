## Shared tile-placement logic for GWTiles3DLoader (offline/backup path) and
## GWTiles3DStreamer (the real, live path) -- pulled out because both bugs
## fixed here were real, found live against actual Cesium ion data, and easy
## to reintroduce independently in either caller:
##
## 1. GLTFDocument.generate_scene() already bakes the content's OWN internal
##    positioning into the returned scene's transform. Google Photorealistic
##    3D Tiles bakes a real ECEF offset into the glTF's own root node matrix
##    (the tileset.json transform chain stays identity throughout); other
##    providers (e.g. Cesium's b3dm samples) put it in the tileset transform
##    chain instead and leave content in trivial local space. Overwriting
##    scene.transform instead of wrapping it silently discards whichever
##    half of the position lives in the content -- verified live: every
##    downloaded Google tile landed at the identical wrong position
##    (elevation off by Earth's radius) until this was a wrapper, not an
##    overwrite.
## 2. Google's baked offset uses a DIFFERENT ECEF axis convention than the
##    standard one this project's own ECEF math (GWGeodeticConvert) uses --
##    verified live by brute-force axis-permutation search against a real
##    downloaded tile's known location.
class_name GWTiles3DContent
extends RefCounted

## Google's (X,Y,Z) = standard ECEF's (X, Z, -Y), i.e. Y points at the north
## pole instead of Z. Godot loads a glTF node matrix raw -- it has no idea
## it's "ECEF" at all -- so this fixed rotation must be applied to the
## WRAPPER's basis (not its position, which always comes from the tileset
## transform chain in the standard convention) whenever the content came
## from Google's proxy.
## Basis(a,b,c) sets .x=a, .y=b, .z=c DIRECTLY (columns, exactly matching
## property assignment and the official docs) -- this constant is built from
## the matrix's COLUMNS, i.e. the transpose of how the row-form correction
## reads in prose ("(X,Y,Z) -> (X,-Z,Y)").
const GOOGLE_YUP_ECEF_CORRECTION := Basis(Vector3(1, 0, 0), Vector3(0, 0, 1), Vector3(0, -1, 0))


## Parses glb_bytes via GLTFDocument, WRAPS it (never overwrites the parsed
## scene's own transform -- see the class doc comment above), applies
## content_axis_correction if set, adds the wrapper under `parent` named
## `tile_name`, and returns it. Returns null if parsing/generation failed --
## the caller should push_warning with whatever path/URL context it has,
## since this helper doesn't know it.
static func place_tile(parent: Node3D, glb_bytes: PackedByteArray, wrapper_transform: Transform3D,
		content_axis_correction: String, tile_name: String) -> Node3D:
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	if doc.append_from_buffer(glb_bytes, "", state) != OK:
		return null
	var scene := doc.generate_scene(state)
	if scene == null:
		return null

	var wrapper := Node3D.new()
	wrapper.name = tile_name
	var xform := wrapper_transform
	if content_axis_correction == "google_yup_ecef":
		xform.basis = xform.basis * GOOGLE_YUP_ECEF_CORRECTION
	wrapper.transform = xform
	parent.add_child(wrapper)
	if Engine.is_editor_hint():
		wrapper.owner = parent.get_tree().edited_scene_root
	wrapper.add_child(scene)
	if Engine.is_editor_hint():
		scene.owner = parent.get_tree().edited_scene_root
	return wrapper


## Manifest position is [east, up, -south] meters (already Godot EUS). Basis
## is stored ROW-major (json's nested-list-of-rows, from numpy's .tolist())
## whose COLUMNS are where the tile's local X/Y/Z axes land in Godot space --
## so Basis.x/.y/.z (Godot's column vectors) are built by reading down each
## column, not each row. Columns may carry non-uniform scale from the tile's
## own transform (verified against a real sample: orthogonal columns,
## uniform 100x scale) -- Godot's Basis holds that fine.
static func transform_from_manifest(tile: Dictionary) -> Transform3D:
	var pos: Array = tile.get("position", [0.0, 0.0, 0.0])
	var m: Array = tile.get("basis", [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
	var basis := Basis()
	basis.x = Vector3(m[0][0], m[1][0], m[2][0])
	basis.y = Vector3(m[0][1], m[1][1], m[2][1])
	basis.z = Vector3(m[0][2], m[1][2], m[2][2])
	return Transform3D(basis, Vector3(pos[0], pos[1], pos[2]))
