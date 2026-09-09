## Ground-vehicle parameters — the rover analog of GWAircraftConfig /
## GWMultirotorConfig. Drives GWRoverBody.
##
## Everything here is a measurable quantity (mass, wheelbase, tyre grip, motor
## torque…), not a "feel" knob. Geometry (wheelbase, track, wheel radius, CG
## height, body box) is only used when the vehicle has no visual model to read
## it from: a GWRover with a `model_scene` takes wheel count/positions/radii and
## the hull box from the model's nodes instead (see GWRover for the naming
## standard), and the rest of this resource still applies.
@tool
class_name GWRoverConfig
extends Resource

const G := 9.80665
const RHO := 1.225

## Which wheels the motor(s) drive.
enum DriveLayout { ALL, FRONT, REAR }
## How the vehicle turns: Ackermann steers an axle; skid steer drives the left
## and right sides at different speeds (tracked / differential-drive rovers).
enum SteeringMode { ACKERMANN, SKID }

@export var mass: float = 25.0            ## kg, all-up

@export_group("Inertia (kg·m²)")
## Leave at 0 to estimate from mass + body box (solid-box model). Set explicitly
## if you have a CAD / measured tensor.
@export var Ixx: float = 0.0   ## roll
@export var Iyy: float = 0.0   ## pitch
@export var Izz: float = 0.0   ## yaw

@export_group("Geometry (used without a model)")
## Front-to-rear axle distance (m).
@export var wheelbase: float = 0.9
## Left-to-right hub distance (m).
@export var track: float = 0.7
@export var wheel_radius: float = 0.15
@export var wheel_width: float = 0.08
## Centre of mass height above the ground at rest (m). Higher = rolls more in
## corners and tips sooner.
@export var cg_height: float = 0.26
## Fore/aft position of the CG between the axles: 0 = over the rear axle,
## 1 = over the front axle, 0.5 = centred.
@export_range(0.0, 1.0) var cg_position: float = 0.5
## Body box (length, width, height) in metres — the collision hull and the
## inertia estimate, and the placeholder visual.
@export var body_size: Vector3 = Vector3(1.1, 0.6, 0.3)
## Body box centre height above the ground at rest (m) — for the placeholder
## visual and the hull.
@export var body_center_height: float = 0.3

@export_group("Suspension")
## Undamped natural frequency of the sprung mass on its springs (Hz). Off-road
## soft ≈ 1.2–1.5; road car ≈ 1.5–2; stiff RC buggy ≈ 2.5–3.5. Spring rates are
## derived from this and each wheel's static load.
@export_range(0.5, 6.0) var suspension_freq_hz: float = 2.0
## Damping ratio: 0.3 = floaty, 0.5–0.7 = well damped, 1.0 = critically damped.
@export_range(0.05, 1.5) var suspension_damping_ratio: float = 0.6
## Wheel travel each way from the rest position (m). Bump stops beyond this.
@export var suspension_travel: float = 0.08

@export_group("Drive")
@export var drive_layout: DriveLayout = DriveLayout.ALL
## Peak (stall) torque per DRIVEN wheel (N·m). With a DC motor model the torque
## falls linearly with wheel speed to zero at `max_speed`, so this sets the
## launch acceleration and hill-climbing ability.
@export var wheel_torque_max: float = 12.0
## No-load top speed on flat ground (m/s). Torque reaches zero here.
@export var max_speed: float = 6.0
## Motor/ESC response lag (s) — first order on the throttle command.
@export var motor_time_const: float = 0.08
## Braking applied when throttle is at neutral, as a fraction of the tyre's
## static grip (an ESC in brake mode / engine braking). 0 = coasts.
@export_range(0.0, 1.0) var neutral_brake: float = 0.3

@export_group("Steering")
@export var steering_mode: SteeringMode = SteeringMode.ACKERMANN
## Full-lock steering angle (deg) of the steered axle's centre-line (Ackermann
## geometry gives the inner wheel more, the outer less).
@export_range(1.0, 60.0) var max_steer_deg: float = 30.0
## Steering servo lag (s).
@export var steer_time_const: float = 0.12
## Steer the front axle (the wheels furthest forward).
@export var steer_front: bool = true
## Also counter-steer the rear axle (four-wheel steering).
@export var steer_rear: bool = false

@export_group("Tyres")
## Peak (static) friction coefficient tyre–ground. ~1.0 dry tarmac, 0.6–0.8
## dirt/grass, 0.3 wet mud, 0.1 ice.
@export var mu_static: float = 0.9
## Sliding friction coefficient once a tyre lets go (< mu_static).
@export var mu_kinetic: float = 0.7
## Cornering stiffness per unit load per m/s of lateral slip velocity — how
## fast lateral force builds before the tyre saturates (grip ÷ this = the slip
## speed at which it starts sliding). ~8 firm tyre, ~3 soft knobbly tyre.
@export var lateral_stiffness: float = 6.0
## Rolling resistance coefficient (force = C·N). ~0.01 tarmac, 0.05 grass, 0.1 sand.
@export var rolling_resistance: float = 0.03

@export_group("Aero")
## Drag area CdA (m²) per body axis (x=fwd, y=right, z=down): F = ½·rho·CdA·v².
@export var drag_area: Vector3 = Vector3(0.25, 0.4, 0.6)

@export_group("Limits")
## Roll or pitch beyond this = rolled over (a crash).
@export var rollover_deg: float = 65.0
## Touching down faster than this (m/s, vertical) after a jump = a crash. 4 m/s
## is roughly a 0.8 m drop.
@export var max_landing_speed: float = 4.0


## Inertia tensor (kg·m²), auto-estimating any axis left at 0 from the body box.
func inertia(size: Vector3 = body_size) -> Vector3:
	var i := Vector3(Ixx, Iyy, Izz)
	var est := estimate_inertia(size)
	if i.x <= 0.0: i.x = est.x
	if i.y <= 0.0: i.y = est.y
	if i.z <= 0.0: i.z = est.z
	return i


## Solid-box inertia for `size` = (length, width, height): a rover's mass is
## spread fairly evenly (battery, motors, frame) so the box is a fair model.
func estimate_inertia(size: Vector3) -> Vector3:
	var l2 := size.x * size.x
	var w2 := size.y * size.y
	var h2 := size.z * size.z
	return Vector3(mass / 12.0 * (w2 + h2), mass / 12.0 * (l2 + h2), mass / 12.0 * (l2 + w2))


## Full-lock steer angle in radians.
func max_steer_rad() -> float:
	return deg_to_rad(max_steer_deg)
