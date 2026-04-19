## Quadcopter-style flight controller — CW/CCW differential yaw.
##
## Extends FlightFpsController; translation (throttle, pitch, strafe) is
## identical to the parent.  Yaw is replaced with the angular-momentum
## model used by real quadcopters:
##
##   1. Per-motor throttle differential — CW motors are throttled up
##      while CCW motors are throttled down (or vice versa), mirroring
##      how real ESCs change prop RPM to create a yaw couple.
##
##   2. Direct `apply_torque` on the body — simulates the aerodynamic
##      drag torque that spinning props generate.  This is the physical
##      mechanism behind prop-torque yaw: each spinning prop drags on
##      the air, and the reaction pushes the body the other way.
##      Differential RPM upsets this balance, leaving a net torque.
##
## Both layers are active simultaneously so the response matches
## real-quad feel: the motors visibly change speed *and* the body
## receives the correct angular impulse.
##
## ── Motor spin-sign convention ────────────────────────────────────
## Spin signs are derived automatically from motor position in the
## body's local XZ plane — the standard X-quad layout:
##
##     Front-Left  (x < 0, z < 0) → x·z > 0 → CW  (+1)
##     Front-Right (x > 0, z < 0) → x·z < 0 → CCW (−1)
##     Rear-Right  (x > 0, z > 0) → x·z > 0 → CW  (+1)
##     Rear-Left   (x < 0, z > 0) → x·z < 0 → CCW (−1)
##
## At equal throttle the CW and CCW drag torques cancel → no yaw.
## To yaw, the two diagonal pairs are offset against each other.
## Override per-motor signs via `spin_sign_override` for non-standard
## layouts (e.g. hex, Y-frame, V-tail).
##
## ── What is NOT used from the parent ──────────────────────────────
## The following FlightFpsController exports are inherited but have
## no effect in this controller:
##   • `yaw_tilt_angle`         — motor nodes are never tilted.
##   • `yaw_idle_thrust`        — replaced by `yaw_motor_idle`.
##   • `yaw_throttle_attenuation` — not applicable to this model.
##   • `_motor_rest_basis`       — internal cache, never populated.
##
## All other parent exports (`power_authority`, throttle/pitch/roll/
## strafe action names) function identically to the parent.
extends FlightFpsController
class_name FlightQuadController


## Peak yaw throttle delta applied at full yaw-stick deflection.
## CW motors are raised by this amount and CCW motors lowered (or vice
## versa), clamped to [0, 1] per motor.
##
## Increasing this value sharpens the motor-speed difference visible
## during a yaw manoeuvre.  If yaw authority feels weak at low
## throttle, raise `yaw_reaction_torque` instead — at zero thrust
## there is nothing for the differential to act on (a zero-speed prop
## produces no drag either way), so torque is the more direct knob.
@export_range(0.0, 1.0, 0.001) var yaw_differential: float = 0.25

## Torque (N·m) applied directly to the body per unit of yaw input.
## This represents the aerodynamic drag couple from differential prop
## RPM — the "angular momentum" mechanism — and is the primary source
## of yaw authority at low throttle.
##
## Scale this against the drone's mass and moment of inertia.
## A starting point: set to roughly 5–10× the body mass in kg.
## If the drone spins too fast at full yaw, lower this value; if yaw
## feels sluggish and unresponsive, raise it.
@export_range(0.0, 2000.0, 1.0) var yaw_reaction_torque: float = 50.0

## Minimum throttle floor applied to *all* motors when the yaw stick
## is deflected, scaled by |yaw|.  Keeps props "spinning" during a
## pure-yaw input so the differential has something to act on.
##
## Analogous to `yaw_idle_thrust` in the parent tilt-rotor controller.
## Set to 0 to disable (rely entirely on `yaw_reaction_torque`).
@export_range(0.0, 1.0, 0.001) var yaw_motor_idle: float = 0.10

## Per-thruster spin-sign overrides.
##   Key:   the Node3D thruster reference (assign at runtime or from
##          a ready function with `spin_sign_override[get_node(...)] = 1`).
##   Value: +1 (CW) or −1 (CCW).
##
## When empty (default) spin signs are auto-derived from motor position.
var spin_sign_override: Dictionary = {}


func update_mix(body: RigidBody3D, thrusters: Array[Node]) -> void:
	if thrusters.is_empty():
		return

	# ── Build base thrust from player input ─────────────────────────
	# Magnitude-preserving composition — see fps-flight-controller-
	# guide.md §3 for the full rationale.  The direction is shared by
	# all motors; only the per-motor *throttle* differs for yaw.
	var lift:    float = get_axis_value(throttle_down_action,   throttle_up_action)
	var strafe:  float = get_axis_value(roll_left_action,       roll_right_action)
	var forward: float = get_axis_value(pitch_backward_action,  pitch_forward_action)
	var yaw:     float = get_axis_value(yaw_left_action,        yaw_right_action)

	var direction   := Vector3(strafe, lift, -forward)
	var base_magnitude: float = minf(direction.length(), 1.0)

	# Unit direction all motors will push along.  Falls back to UP so
	# yaw-only input (no translation) gives motors a valid direction.
	var thrust_dir: Vector3 = Vector3.UP
	if base_magnitude > 0.0001:
		thrust_dir = direction / base_magnitude  # cheap unit vector

	# Player-commanded base throttle, bounded by power_authority.
	var base_scalar: float = base_magnitude * power_authority

	# ── Yaw motor idle floor ────────────────────────────────────────
	# Raise the floor for all motors proportionally to |yaw| so the
	# differential has some RPM to work with even at zero throttle.
	var yaw_abs: float = absf(yaw)
	var idle_floor: float = 0.0
	if yaw_motor_idle > 0.0 and yaw_abs > 0.0:
		idle_floor = yaw_motor_idle * yaw_abs * power_authority

	var effective_base: float = maxf(base_scalar, idle_floor)

	# ── Per-motor throttle differential ─────────────────────────────
	# motor_throttle = clamp(effective_base − yaw × differential × spin, 0, 1)
	#
	# Sign derivation (yaw > 0 = body yaws CCW = left in Godot +Y):
	#   CCW motor (spin = −1): effective_base − yaw × diff × (−1)
	#                        = effective_base + yaw × diff  →  raised  ✓
	#   CW  motor (spin = +1): effective_base − yaw × diff × (+1)
	#                        = effective_base − yaw × diff  →  lowered ✓
	# Differential RPM makes CCW props spin faster, CW slower →
	# net CCW drag tops CW drag → reaction torque on body is CW...
	# wait — drag on body IS the reaction, so:
	#   faster CCW prop → more CCW drag on air → +Y torque on body ✓
	var any_active := false
	for t in thrusters:
		if not (t is Node3D):
			continue

		var spin: int = _get_spin_sign(t as Node3D, body)
		var motor_thr: float = clampf(
			effective_base - yaw * yaw_differential * float(spin),
			0.0, 1.0
		)

		# `set_thrust_directed` separates direction from magnitude and
		# avoids an extra normalise inside set_thrust.
		if t.has_method("set_thrust_directed"):
			t.set_thrust_directed(thrust_dir, motor_thr)
		elif t.has_method("set_thrust"):
			t.set_thrust(thrust_dir * motor_thr)

		if motor_thr > 0.0:
			any_active = true

	# ── Yaw reaction torque ─────────────────────────────────────────
	# Applies the net angular impulse that differential prop RPM
	# produces via aerodynamic drag.  Scaled along the body's current
	# local-Y axis so it always rotates around the drone's own "up",
	# regardless of the drone's attitude in the world.
	#
	# `apply_torque` in _physics_process applies a torque for one
	# physics step — identical semantics to how FlightThruster calls
	# `apply_force`.
	if body != null and yaw_abs > 0.0:
		body.apply_torque(body.basis.y * yaw * yaw_reaction_torque)

	# Silence all engines when player releases every input.
	if not any_active and yaw_abs < 0.001:
		_silence_all(thrusters)


## Returns the CW/CCW spin sign (+1 or −1) for a single motor node.
##
## Checks `spin_sign_override` first.  If no override is registered,
## deduces the sign from the motor's position in the body's local XZ
## plane using the standard X-quad rule (see class header).
func _get_spin_sign(motor: Node3D, body: RigidBody3D) -> int:
	# Manual override takes precedence over position detection.
	if spin_sign_override.has(motor):
		var s: int = spin_sign_override[motor]
		return 1 if s >= 0 else -1

	# Resolve local position relative to the body origin.
	# Using body.to_local handles motors nested under arm sub-nodes
	# as well as the common case of motors as direct body children.
	var local_pos: Vector3
	if body != null:
		local_pos = body.to_local(motor.global_position)
	else:
		local_pos = motor.transform.origin

	# X-quad rule: same-sign quadrant → CW, opposite-sign → CCW.
	var xz: float = local_pos.x * local_pos.z
	if absf(xz) < 0.0001:
		# Motor lies on the X or Z axis (indeterminate).  Default CW.
		push_warning(
			"QuadFlightController: motor '%s' is on a body axis (x·z ≈ 0); "
			% motor.name +
			"spin sign defaults to CW (+1).  Set spin_sign_override to silence."
		)
		return 1

	return 1 if xz > 0.0 else -1
