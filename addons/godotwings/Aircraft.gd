@tool
@icon("res://addons/godotwings/sensors/camera_icon.svg")
## Drop-in fixed-wing aircraft: one node that self-assembles a working SITL setup
## — flight model + UDP bridge + visual model + optional camera. It IS a
## GWFlightBody, so all the FDM exports are here too. Pick a `flight_model` (or
## Custom with your own `spec`/`config`), optionally a `model_scene` and a camera.
## At runtime it adds a GWSITLBridge child (port offset by `sitl_instance`),
## instances the model, and adds a GWCamera if `enable_camera` is set.
class_name GWAircraft
extends GWFlightBody

## glTF authoring frames we know how to rotate into Godot's -Z-forward / +Y-up.
enum ModelOrientation {
	GLTF_Y_UP,     ## Standard glTF / Godot: nose -Z, up +Y. No rotation.
	ONSHAPE_Z_UP,  ## Onshape/CAD export: Z up, nose +Y. Rotate -90 deg about X.
}

## Built-in flight models, selectable from the inspector. CUSTOM defers to `spec`
## (or a raw `config`) so you can build or hand-tune your own.
enum FlightModel { TRAINER, FOAMIE, FPV_WING, CUSTOM }

## Bundled visual + orientation so a freshly-dropped node is immediately visible.
const DEFAULT_MODEL := preload("res://addons/godotwings/aircraft/wingbody.gltf")
const DEFAULT_MODEL_ORIENTATION := ModelOrientation.ONSHAPE_Z_UP

@export_group("Aircraft definition")
## Pick a built-in flight model, or CUSTOM to use `spec` / `config` below.
@export var flight_model: FlightModel = FlightModel.TRAINER:
	set(v):
		flight_model = v
		notify_property_list_changed()
		update_configuration_warnings()
## Custom physical spec (used only when `flight_model` is CUSTOM). If set, it
## builds `config` for you; leave empty to supply a raw `config` instead.
@export var spec: GWAircraftSpec:
	set(v):
		spec = v
		update_configuration_warnings()

@export_group("Visual model")
## Leave empty to use the bundled wing. Drop your own glTF here to override it.
@export var model_scene: PackedScene:
	set(v):
		model_scene = v
		_refresh_model_preview()
		update_configuration_warnings()
@export var model_orientation: ModelOrientation = ModelOrientation.GLTF_Y_UP:
	set(v):
		model_orientation = v
		_refresh_model_preview()
## Extra yaw (deg) about the up axis — flip to 180 if the model renders tail-first.
@export var model_yaw_offset_deg: float = 0.0:
	set(v):
		model_yaw_offset_deg = v
		_refresh_model_preview()
## Uniform scale applied to the model (visual only).
@export var model_scale: float = 1.0:
	set(v):
		model_scale = v
		_refresh_model_preview()

@export_group("SITL")
## Vehicle index for multi-aircraft setups: bridge listens on 9002 + 10*instance
## (ArduPilot's per-instance port convention).
@export var sitl_instance: int = 0

@export_group("Streaming camera")
@export var enable_camera: bool = false
@export var camera_protocol: GWCamera.Protocol = GWCamera.Protocol.RTP_H264
@export var camera_resolution: Vector2i = Vector2i(1280, 720)
@export var camera_fps: float = 30.0
## Camera mount relative to the body. Default: 1 m ahead of CG, looking out the nose.
@export var camera_mount: Transform3D = Transform3D(Basis(), Vector3(0, 0.1, -1.0))
## Off hands the raw-frame TCP server to your own pipeline instead of ffmpeg —
## e.g. `tools/gw_klv_muxer.py` for STANAG4609/KLV FMV output (see the README's
## "STANAG 4609 / MISB ST 0601 KLV metadata" section). `camera_protocol` /
## `rtsp_url` etc. are then unused; only `resolution`/`fps` still apply.
@export var camera_launch_ffmpeg: bool = true
## Make the camera an ArduPilot servo gimbal: it reads the mount servo PWM (set
## MNT1_TYPE=1 + SERVOn_FUNCTION on the autopilot) and follows every mount mode
## (MAVLink/ROI/Home/SysID/RC). See GWCamera for details.
@export var camera_gimbal: bool = false:
	set(v):
		camera_gimbal = v
		notify_property_list_changed()
## SITL channels (1-16) carrying mount pitch/yaw/roll servo PWM; 0 = axis fixed.
@export_range(0, 16) var gimbal_pitch_channel: int = 0
@export_range(0, 16) var gimbal_yaw_channel: int = 0
@export_range(0, 16) var gimbal_roll_channel: int = 0
## Angle (deg) each PWM range maps to (x at min PWM, y at max); match MNT1_*_MIN/MAX.
@export var gimbal_pitch_range_deg: Vector2 = Vector2(-90.0, 0.0)
@export var gimbal_yaw_range_deg: Vector2 = Vector2(-180.0, 180.0)
@export var gimbal_roll_range_deg: Vector2 = Vector2(-30.0, 30.0)
## Servo PWM endpoints the angle ranges span (SERVOn_MIN..MAX).
@export var gimbal_pwm_min: int = 1000
@export var gimbal_pwm_max: int = 2000


func _ready() -> void:
	if Engine.is_editor_hint():
		set_physics_process(false)
		_instance_model()   # visual preview only; bridge/camera are runtime-only
		return
	var resolved := _resolve_spec()
	if resolved != null:
		config = resolved.build()
	if control_source == ControlSource.MANUAL:
		_ensure_manual_input()  # direct keyboard/joypad/RC; no SITL bridge
	else:
		_ensure_bridge(sitl_instance)
	_instance_model()
	if enable_camera:
		_ensure_camera(_camera_opts())
	super._ready()


func _instance_model() -> void:
	if get_node_or_null("Model") != null:
		return
	# Fall back to the bundled wing (with its own orientation) when none is set,
	# so a bare GWAircraft is visible out of the box.
	var scene := model_scene if model_scene != null else DEFAULT_MODEL
	var orient := model_orientation if model_scene != null else DEFAULT_MODEL_ORIENTATION
	var wrapper := Node3D.new()
	wrapper.name = "Model"
	var basis := _orientation_basis(orient)
	if not is_zero_approx(model_yaw_offset_deg):
		basis = Basis(Vector3.UP, deg_to_rad(model_yaw_offset_deg)) * basis
	if not is_equal_approx(model_scale, 1.0):
		basis = basis.scaled(Vector3.ONE * model_scale)
	wrapper.transform = Transform3D(basis, Vector3.ZERO)
	wrapper.add_child(scene.instantiate())
	add_child(wrapper)


## Rebuild the visual preview in the editor when a model export changes.
func _refresh_model_preview() -> void:
	if not Engine.is_editor_hint() or not is_inside_tree():
		return
	var existing := get_node_or_null("Model")
	if existing != null:
		existing.free()
	_instance_model()


func _orientation_basis(orient: ModelOrientation) -> Basis:
	match orient:
		ModelOrientation.ONSHAPE_Z_UP:
			return Basis.from_euler(Vector3(-PI / 2.0, 0.0, 0.0))
		_:
			return Basis()


## The spec backing the chosen flight model: a built-in preset, or the user's
## `spec` when CUSTOM. Returns null when CUSTOM with no spec (use raw `config`).
func _resolve_spec() -> GWAircraftSpec:
	match flight_model:
		FlightModel.TRAINER: return GWAircraftSpec.trainer()
		FlightModel.FOAMIE: return GWAircraftSpec.foamie()
		FlightModel.FPV_WING: return GWAircraftSpec.fpv_wing()
		_: return spec


## Build the camera/gimbal options dict for GWVehicleBody._ensure_camera.
func _camera_opts() -> Dictionary:
	return {
		"protocol": camera_protocol, "resolution": camera_resolution, "fps": camera_fps,
		"mount": camera_mount, "instance": sitl_instance, "gimbal": camera_gimbal,
		"launch_ffmpeg": camera_launch_ffmpeg,
		"gimbal_pitch_channel": gimbal_pitch_channel, "gimbal_yaw_channel": gimbal_yaw_channel,
		"gimbal_roll_channel": gimbal_roll_channel, "gimbal_pitch_range_deg": gimbal_pitch_range_deg,
		"gimbal_yaw_range_deg": gimbal_yaw_range_deg, "gimbal_roll_range_deg": gimbal_roll_range_deg,
		"gimbal_pwm_min": gimbal_pwm_min, "gimbal_pwm_max": gimbal_pwm_max,
	}


# Gimbal exports are only meaningful when camera_gimbal is on.
const _GIMBAL_PROPS := [
	"gimbal_pitch_channel", "gimbal_yaw_channel", "gimbal_roll_channel",
	"gimbal_pitch_range_deg", "gimbal_yaw_range_deg", "gimbal_roll_range_deg",
	"gimbal_pwm_min", "gimbal_pwm_max",
]


## Hide the custom `spec` slot unless the flight model is CUSTOM, and the gimbal
## fields unless `camera_gimbal` is on.
func _validate_property(property: Dictionary) -> void:
	if property.name == "spec" and flight_model != FlightModel.CUSTOM:
		property.usage &= ~PROPERTY_USAGE_EDITOR
	elif not camera_gimbal and property.name in _GIMBAL_PROPS:
		property.usage &= ~PROPERTY_USAGE_EDITOR


func _get_configuration_warnings() -> PackedStringArray:
	var w: PackedStringArray = []
	if flight_model == FlightModel.CUSTOM and spec == null and config == null:
		w.append("Flight model is Custom but no `spec` or `config` set — will fall back to the bundled Skywalker.")
	return w
