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
##   2. Body-Y reaction torque — simulates the aerodynamic drag
##      couple spinning props create.  Each prop drags on the air;
##      the reaction pushes the body the opposite way.  Differential
##      RPM upsets the balance, leaving a net torque
##
##          τ_yaw = κR · Σ sᵢ · Tᵢ
##
##      where Tᵢ is each motor's *current* thrust (N).  Because Tᵢ
##      already reflects the differential from layer (1), yaw
##      authority scales naturally with throttle — no hand-tuned
##      constant torque (see flight-dynamics-equations.md §5).
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
## during a yaw manoeuvre and, because τ_yaw is derived from the
## resulting thrust split, also increases the body's yaw authority.
## If yaw feels weak at low throttle, raise `yaw_motor_idle` so the
## differential has some baseline RPM to act on.
@export_range(0.0, 1.0, 0.001) var yaw_differential: float = 0.25

## Propeller torque coefficient (metres) — combined $\kappa R$ from
## the momentum-theory rotor torque model $Q = \kappa\,T\,R$
## (flight-dynamics-equations.md §5.1).  The net yaw torque on the
## body is then
##
##     τ_yaw = κR · Σ sᵢ · Tᵢ
##
## where sᵢ is each motor's spin sign and Tᵢ is its current thrust
## in Newtons (`max_force · motor_throttle`).  Because Tᵢ already
## encodes the yaw differential computed below, yaw authority scales
## naturally with throttle — there is no separate "constant torque"
## knob to tune.
##
## Physical values for a 5-inch quad are roughly 0.01–0.03 m
## (κ ≈ 0.05–0.15, R ≈ 0.0635 m).  Game drones with unrealistic mass
## or moment of inertia may need larger values for a responsive feel;
## the range is widened accordingly.  If yaw is too twitchy, lower
## this; if it feels sluggish, raise it.
@export_range(0.0, 1.0, 0.001) var prop_torque_coefficient: float = 0.02

## Minimum throttle floor applied to *all* motors when the yaw stick
## is deflected, scaled by |yaw|.  Keeps props "spinning" during a
## pure-yaw input so the differential has something to act on.
##
## Analogous to `yaw_idle_thrust` in the parent tilt-rotor controller.
## Set to 0 to rely entirely on the player-commanded base thrust; at
## zero base throttle and zero idle, all Tᵢ are zero and the derived
## τ_yaw collapses to zero as well (a zero-RPM prop produces no
## drag — physically correct, but means yaw authority disappears).
@export_range(0.0, 1.0, 0.001) var yaw_motor_idle: float = 0.10

## ── Per-axis aerodynamic angular drag ──────────────────────────────
##
## Godot's built-in `angular_damp` on RigidBody3D is *isotropic* — it
## applies the same resistance to pitch, roll, and yaw.  Real drones
## are not isotropic: a quad's four props present a large flat disc
## to yaw rotation but only edge-on to pitch and roll, so yaw
## aerodynamic damping is typically 3–5× the pitch/roll damping.
##
## These exports apply an explicit drag torque in the body's local
## frame each physics tick:
##
##     τ_drag_local = −Vector3(k_pitch · ω_x,  k_yaw · ω_y,  k_roll · ω_z)
##
## where ω is the body's angular velocity expressed in its own local
## frame.  The torque is then rotated back to world space and handed
## to `apply_torque`.  Units are N·m per rad/s (linear drag), so
## doubling ω doubles the opposing torque (matches low-speed
## aerodynamic drag on an immersed disc/plate).
##
## Recommended workflow: **set the RigidBody3D's `angular_damp` to 0**
## and tune these three values instead.  If `angular_damp` is left
## non-zero it will stack on top of these — both are additive — which
## is fine for quick tuning but makes the yaw-rate calculations in
## units-reference.md §5.3 inexact.
##
## A good starting point for the 0.5 kg / 10-inch-prop baseline:
##   angular_drag_pitch = 0.05, angular_drag_roll = 0.05,
##   angular_drag_yaw   = 0.25  (yaw ≈ 5× pitch/roll).
##
## Left at 0 by default so existing scenes are not silently changed.
@export_range(0.0, 10.0, 0.001) var angular_drag_pitch: float = 0.0

## See `angular_drag_pitch`.  Roll drag around the drone's local Z
## axis (forward/back axis).  Usually similar to pitch drag.
@export_range(0.0, 10.0, 0.001) var angular_drag_roll: float = 0.0

## See `angular_drag_pitch`.  Yaw drag around the drone's local Y
## (up) axis.  Typically 3–5× the pitch/roll values because the prop
## discs act like flat plates resisting yaw rotation.
@export_range(0.0, 10.0, 0.001) var angular_drag_yaw: float = 0.0

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

	# Running Σ sᵢ·Tᵢ (signed thrust, N) accumulated across motors.
	# Used below to derive the physically-scaled yaw torque.
	var signed_thrust_sum: float = 0.0

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

		# Tᵢ = max_force · throttle.  `get_max_force` is the
		# FlightThruster API; fall back to 1.0 for exotic thrusters
		# so the torque still scales with the relative differential.
		#
		# Read the thruster's *actual* throttle property rather than
		# the `motor_thr` we just commanded — on a thruster that
		# implements PowerRamp, `t.throttle` holds the ramped value
		# that is being applied as force this tick, so the yaw
		# reaction torque lags motor RPM in exactly the same way.
		# The commanded value is saved to `_commanded_throttle`
		# inside the thruster and the next physics tick will ramp
		# toward it.  Falls back to `motor_thr` for exotic thrusters
		# that don't expose a `throttle` property.
		var motor_max: float = 1.0
		if t.has_method("get_max_force"):
			motor_max = t.get_max_force()
		var actual_thr: float = motor_thr
		if "throttle" in t:
			actual_thr = t.throttle
		signed_thrust_sum += float(spin) * motor_max * actual_thr

		if motor_thr > 0.0:
			any_active = true

	# ── Yaw reaction torque (physically derived) ────────────────────
	# τ_yaw = κR · Σ sᵢ · Tᵢ   (flight-dynamics-equations.md §5.2)
	#
	# With our spin convention (spin = +1 CW, −1 CCW), yaw > 0 raises
	# CCW throttle and lowers CW throttle, making the signed sum
	# negative.  Applying `+κR · sum` along `body.basis.y` therefore
	# produces a −Y torque, i.e. CW rotation from above — matching
	# "yaw right stick → drone yaws right".  No extra sign flip
	# needed; unlike the previous constant-torque layer, the sign
	# falls out of the thrust split directly.
	#
	# `apply_torque` applies for one physics step, identical to how
	# FlightThruster calls `apply_force`.
	if body != null and prop_torque_coefficient > 0.0 and absf(signed_thrust_sum) > 0.0001:
		body.apply_torque(body.basis.y * prop_torque_coefficient * signed_thrust_sum)

	# ── Per-axis aerodynamic angular drag ───────────────────────────
	# Apply τ_drag = −k · ω component-wise in the body's local frame,
	# then rotate the resulting torque back into world space before
	# handing it to `apply_torque`.  This lets yaw be more damped
	# than pitch/roll (see export doc on `angular_drag_pitch`).
	#
	# Skipped when all three coefficients are zero — cheap early-out
	# so users who haven't opted in don't pay for the transform math.
	if body != null and (angular_drag_pitch > 0.0 or angular_drag_roll > 0.0 or angular_drag_yaw > 0.0):
		# RigidBody3D bases are orthonormal, so transposed() == inverse().
		var w_local: Vector3 = body.basis.transposed() * body.angular_velocity
		var drag_local := Vector3(
			-w_local.x * angular_drag_pitch,  # local-X = pitch axis
			-w_local.y * angular_drag_yaw,    # local-Y = yaw   axis
			-w_local.z * angular_drag_roll,   # local-Z = roll  axis
		)
		# Rotate back to world frame for apply_torque, which takes
		# a world-space torque vector.
		body.apply_torque(body.basis * drag_local)

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
