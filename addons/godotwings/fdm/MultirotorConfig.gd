## Quadcopter (multirotor) parameters — the copter analog of GWAircraftConfig.
##
## Drives GWMultirotorBody, whose dynamics follow ArduPilot's SITL multicopter
## frame: per-motor thrust + reaction torque, summed into body force/moment, then
## integrated with a real inertia tensor (+ gyroscopic term). Geometry is a quad X.
@tool
class_name GWMultirotorConfig
extends Resource

const G := 9.80665

@export var mass: float = 0.7            ## kg (all-up weight)

@export_group("Inertia (kg·m²)")
## Leave at 0 to auto-estimate from mass + arm length (point-mass model). Set
## explicitly if you have a measured/CAD inertia tensor.
@export var Ixx: float = 0.0   ## roll
@export var Iyy: float = 0.0   ## pitch
@export var Izz: float = 0.0   ## yaw

@export_group("Geometry")
## Motor distance from the centre (m). Smaller = lower inertia = snappier rotation.
@export var arm_length: float = 0.13
## Motor count (quad X assumed; 4).
@export var motor_count: int = 4

@export_group("Propulsion")
## Full-throttle total thrust ÷ weight. ~6 = snappy FPV freestyle, ~2 = cinematic,
## 1.5 = heavy lifter. NB: with the quadratic thrust curve hover throttle ≈
## sqrt(1/this) — keep MOT_THST_HOVER / MOT_THST_EXPO in sync.
@export var thrust_to_weight: float = 5.0
## First-order motor spool-UP lag (s). Lower = crisper throttle response.
@export var motor_time_const: float = 0.02
## Spool-DOWN lag (s). Real motors brake slower than they accelerate (gazebo's
## iris uses ~2x the up constant). 0 = same as motor_time_const.
@export var motor_time_const_down: float = 0.0
## Thrust curve exponent: thrust = omega^expo · max. ESCs command rotor SPEED
## roughly linearly with PWM but thrust goes with omega², so 2.0 is physical
## (soft low stick, punchy top end — what MOT_THST_EXPO linearises on the FC).
## 1.0 = legacy linear.
@export_range(1.0, 3.0) var thrust_expo: float = 2.0
## Reaction (yaw) torque produced per Newton of motor thrust (m). Real 5" props
## sit near 0.015. NEGATE this if the copter spins up uncontrollably in SITL (it
## flips the assumed CW/CCW sense — the one thing not validatable without
## ArduCopter).
@export var yaw_torque_coeff: float = 0.06
## Axial airspeed (m/s) at FULL rotor speed where the blades stop producing
## thrust (≈ prop pitch × max rev/s). Climbing into the props washes thrust out
## linearly toward this, so punch-outs top out realistically instead of
## accelerating forever. 0 = no washout (legacy).
@export var prop_pitch_speed: float = 0.0

@export_group("Aero")
## Lumped LINEAR translational drag (N per m/s). Legacy catch-all — prefer the
## quadratic `drag_area` + `rotor_drag_coeff` below and leave this near 0.
@export var drag_coeff: float = 0.25
## Quadratic airframe drag area CdA (m²) per body axis (x=fwd, y=right, z=down):
## F = ½·rho·CdA(axis)·v². The z entry is the flat frame + prop-disc face (the
## largest). This is what makes a dive "air-brake" when you chop throttle.
## Zero = no quadratic drag (legacy behaviour).
@export var drag_area: Vector3 = Vector3.ZERO
## Rotor H-force: drag on the rotor discs proportional to prop speed × lateral
## airspeed (N per m/s per unit summed normalized rotor speed). THE dominant
## hover-regime damping on a real quad — gazebo's rotorDragCoefficient.
## Zero = off (legacy).
@export var rotor_drag_coeff: float = 0.0
## Rotor-induced rate damping (N·m per rad/s per unit summed normalized rotor
## speed). Blade flapping / H-force resist rotation; without it rates feel
## frictionless. Zero = off (legacy).
@export var rate_damping: float = 0.0
## Blade-flapping moment (N·m per m/s per unit summed rotor speed): translational
## flight makes the advancing blade lift more, tilting the rotor plane back — the
## "nose-up tug" as speed builds (gazebo's rollingMomentCoefficient). Zero = off.
@export var flap_moment_coeff: float = 0.0

@export_group("Ground effect")
## Rotor radius (m) — 5" prop ≈ 0.0635. Used by ground effect. Zero = off.
@export var prop_radius: float = 0.0
## Thrust boost at h = prop_radius above ground: T ×= 1 + gain·(r/4h)², capped
## +30%. ~1.0 = textbook in-ground-effect cushion on landing.
@export var ground_effect_gain: float = 1.0

@export_group("Propwash")
## Peak wobble moment (N·m) injected when descending into the prop wake near
## hover thrust — THE signature FPV "propwash jitter". Zero = off.
@export var propwash_moment: float = 0.0
## Descent rate (m/s) at which the wobble reaches full strength.
@export var propwash_speed: float = 4.0
## Thrust dropout fraction at full wobble (momentary lift loss in the wake).
@export_range(0.0, 0.5) var propwash_thrust_loss: float = 0.15

@export_group("Battery")
## Cell count (e.g. 4 = 4S). 0 = no battery model (constant full thrust).
@export var batt_cells: int = 0
## Pack capacity (mAh) — drains over the flight, softening the top end.
@export var batt_capacity_mah: float = 1300.0
## Total current draw at full throttle on all motors (A). Scales sag with load.
@export var batt_max_current: float = 90.0
## Pack + ESC internal resistance (ohm). Voltage sag = I·R under load.
@export var batt_resistance: float = 0.025

@export_group("Motor imperfections")
## Per-motor static thrust variance (fraction, e.g. 0.02 = ±2%) — real motors
## never match, so a hover micro-drifts instead of sitting frozen. Zero = off.
@export_range(0.0, 0.1) var motor_variance: float = 0.0
## Rotor moment of inertia (kg·m²) per prop+bell. Spool changes kick yaw the
## other way (reaction to rotor angular acceleration). 5" ≈ 2e-6. Zero = off.
@export var rotor_inertia: float = 0.0
## Max rotor speed (rad/s) at full throttle — converts normalized spool rate to
## the yaw-kick torque above. ~2800 for a 5" 4S setup.
@export var rotor_max_speed: float = 2800.0


## Full-throttle total thrust (N).
func max_thrust_total() -> float:
	return thrust_to_weight * mass * G


## Inertia tensor (kg·m²), auto-estimating any axis left at 0.
func inertia() -> Vector3:
	var i := Vector3(Ixx, Iyy, Izz)
	var est := _estimate_inertia()
	if i.x <= 0.0: i.x = est.x
	if i.y <= 0.0: i.y = est.y
	if i.z <= 0.0: i.z = est.z
	return i


## Lumped estimate: mass at the arm radius with a fill factor for how strongly
## the mass concentrates at the centre (battery/FC/stack) vs the tip motors.
## Real quads sit FAR below the point-mass bound: gazebo's iris (m=1.5,
## arm≈0.256) has Ixx=0.008 = 0.08·m·r²; a 5" freestyle quad measures ~0.15·m·r²
## roll/pitch. Yaw is roughly double (mass spread in the plane). Measure/CAD the
## tensor for fidelity.
func _estimate_inertia() -> Vector3:
	var r2 := arm_length * arm_length
	return Vector3(mass * r2 * 0.15, mass * r2 * 0.15, mass * r2 * 0.3)
