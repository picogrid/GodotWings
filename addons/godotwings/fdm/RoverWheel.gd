## One wheel of a GWRoverBody: where it is on the chassis, how big it is, what
## it does (steers / drives which side), and its live contact state. Built from
## GWRoverConfig geometry, or from the wheel nodes of a visual model (see the
## GWRover model standard), so the physics has one wheel representation either way.
class_name GWRoverWheel
extends RefCounted

## Node name (from the model) or a generated label ("FL", "RR", …).
var name := ""
## Hub centre at rest, body FRD relative to the CG (m): x fwd, y right, z down.
var hub_rest := Vector3.ZERO
var radius := 0.15
var width := 0.08
## Steered wheel (Ackermann) — set from position by the body.
var steers := false
## Rear-axle wheel: counter-steers when the config enables four-wheel steering.
var rear := false
## Motor drives this wheel.
var driven := true
## Static load share at rest (N) — the suspension static_load.
var static_load := 0.0
## Spring rate (N/m) and damper rate (N·s/m), derived from the static_load and the
## config's natural frequency / damping ratio.
var spring_k := 0.0
var damper_c := 0.0

# --- live state (updated each sub-step) --------------------------------------
var in_contact := false
## Hub displacement from rest along body-down (m): positive = wheel dropped
## (droop), negative = pushed up (compressed).
var droop := 0.0
var _droop_prev := 0.0
var _droop_valid := false
var normal_force := 0.0     ## N, along the ground normal
var long_force := 0.0       ## N, along the wheel's rolling direction (+ = pushes body forward)
var lat_force := 0.0        ## N, along the wheel's right
var sliding := false        ## friction circle saturated this step
var steer_angle := 0.0      ## rad, + = nose right
var spin_angle := 0.0       ## rad, accumulated rolling rotation (for visuals)
var ground_speed := 0.0     ## m/s along the rolling direction at the contact patch
var throttle := 0.0         ## lagged motor command driving this wheel, -1..1

# --- per-frame terrain sample under this wheel (NED) --------------------------
var ground_hit := false
var ground_point := Vector3.ZERO
var ground_normal := Vector3(0, 0, -1)

# --- visual binding (used by GWRover to animate a model's wheel node) ---------
var node: Node3D = null
var node_rest: Transform3D = Transform3D.IDENTITY
## Axle (spin) axis in the wheel node's local frame, pointing to the vehicle's right.
var axle_local := Vector3.RIGHT
## Vehicle up direction expressed in the wheel node's PARENT frame.
var up_in_parent := Vector3.UP
## Metres per unit of the parent frame (model scale), to move the node by droop.
var parent_scale := 1.0


## Contact patch position at rest, body FRD (hub + radius straight down).
func contact_rest() -> Vector3:
	return hub_rest + Vector3(0.0, 0.0, radius)


func reset_state() -> void:
	in_contact = false
	droop = 0.0
	_droop_prev = 0.0
	_droop_valid = false
	normal_force = 0.0
	long_force = 0.0
	lat_force = 0.0
	sliding = false
	steer_angle = 0.0
	spin_angle = 0.0
	ground_speed = 0.0
	throttle = 0.0
