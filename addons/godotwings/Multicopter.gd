@tool
@icon("res://addons/godotwings/sensors/camera_icon.svg")
## Drop-in quadcopter: the ArduCopter counterpart of GWAircraft. One node that
## self-assembles a working SITL copter — flight model (GWMultirotorConfig) + UDP
## bridge + visual model + optional camera. It IS a GWMultirotorBody, so all the
## FDM exports are here too. Run the SITL side with VEHICLE=ArduCopter.
class_name GWMulticopter
extends GWMultirotorBody

## Bundled placeholder model so a freshly-dropped node is visible out of the box.
const DEFAULT_MODEL := preload("res://addons/godotwings/aircraft/quad_model.tscn")

@export_group("Visual model")
## Leave empty to use the bundled placeholder quad. Drop your own glTF to override.
@export var model_scene: PackedScene:
	set(v):
		model_scene = v
		_refresh_model_preview()
## Extra yaw (deg) about the up axis, if your model faces the wrong way.
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
## Vehicle index for multi-copter setups: bridge listens on 9002 + 10*instance.
@export var sitl_instance: int = 0

@export_group("Streaming camera")
@export var enable_camera: bool = false
@export var camera_protocol: GWCamera.Protocol = GWCamera.Protocol.RTP_H264
@export var camera_resolution: Vector2i = Vector2i(1280, 720)
@export var camera_fps: float = 30.0
## Camera mount relative to the body. Default: a forward-down FPV-ish look.
@export var camera_mount: Transform3D = Transform3D(Basis(), Vector3(0, 0.05, -0.15))
## Off hands the raw-frame TCP server to your own pipeline instead of ffmpeg —
## e.g. `tools/gw_klv_muxer.py` for STANAG4609/KLV FMV output (see the README's
## "STANAG 4609 / MISB ST 0601 KLV metadata" section). `camera_protocol` /
## `rtsp_url` etc. are then unused; only `resolution`/`fps` still apply.
@export var camera_launch_ffmpeg: bool = true
## Make the camera an ArduPilot servo gimbal (reads mount servo PWM). See GWCamera.
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
	if control_source == ControlSource.MANUAL:
		_ensure_manual_input()  # direct keyboard/joypad/RC; raw motor channels, no SITL
	else:
		_ensure_bridge(sitl_instance)
	_instance_model()
	if enable_camera:
		_ensure_camera(_camera_opts())
	super._ready()


func _instance_model() -> void:
	if get_node_or_null("Model") != null:
		return
	var scene := model_scene if model_scene != null else DEFAULT_MODEL
	var wrapper := Node3D.new()
	wrapper.name = "Model"
	var basis := Basis()
	if not is_zero_approx(model_yaw_offset_deg):
		basis = Basis(Vector3.UP, deg_to_rad(model_yaw_offset_deg))
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


func _validate_property(property: Dictionary) -> void:
	if not camera_gimbal and property.name in _GIMBAL_PROPS:
		property.usage &= ~PROPERTY_USAGE_EDITOR
