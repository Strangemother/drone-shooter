## Minimal FPS-style flight controller.
##
## Reads player input axes and pushes a single thrust vector to every
## thruster.  No mixing, no PID, no stabilisation — exactly what the
## thruster does with that vector (where it pushes the body, what
## torque the off-centre application creates) is what the player
## feels.  This is the foundation a more advanced FPS drone model
## can be built on top of.
##
## ── Input mapping ──────────────────────────────────────────────────
##   throttle_up / throttle_down   → +Y / -Y component (vertical)
##   pitch_forward / pitch_backward → -Z / +Z component (forward/back)
##   roll_left / roll_right         → -X / +X component (strafe L/R)
##   yaw_left / yaw_right           → opposite Y-axis tilt per pair
##
## All in the *thruster's local frame*.  Because each thruster's
## basis follows the parent body, "forward" means "wherever the body
## is pointing" — which is what an FPS player expects.
##
## ── Yaw model (iteration 2) ────────────────────────────────────────
## Yaw is produced by tilting each motor's thrust vector *tangent
## to its own arm* — front/rear motors tilt in ±X, side motors tilt
## in ±Z.  All four tangential components point the same rotational
## way around Y, which sums into a pure yaw couple with no net
## translation.  The tilt is built from sin/cos so total thrust
## magnitude per motor is preserved — yawing does not change lift.
## Works in full hover because the tangential component is added
## rather than derived from rotating an existing horizontal part.
##
## ── How the thrust vector becomes movement ─────────────────────────
## Every tick we build a unit-ish direction vector from input and
## scale it by `power_authority` ∈ [0, 1].  Each thruster receives
## the same `set_thrust(vector)` call.  The thruster then:
##   • takes the vector's length as throttle (clamped to [0, 1]),
##   • takes the vector's direction as its local force axis,
##   • applies `max_force * throttle` along that axis at its own
##     world position — which automatically produces translation
##     plus any torque from its lever arm to the body centre.
##
## So if all thrusters are arranged symmetrically around the body
## centre and you push `Vector3.UP * 0.5`, the drone rises straight
## up.  If they're asymmetric (one engine farther from centre than
## the others) the body rotates as well — that's a real-world
## physical result, not a bug.
extends FlightControllerBase
class_name FlightFpsController


## Maximum power (0..1) the player input can command.  At 1.0 a full
## stick deflection sends `set_thrust(direction * 1.0)` to every
## thruster — i.e. each thruster runs at full throttle.  Lower values
## make the controls less twitchy.
@export_range(0.0, 1.0, 0.001) var power_authority: float = 1.0

## Peak yaw tilt angle (radians) applied to each motor at full
## yaw-stick deflection.  Each motor tilts its thrust tangent to
## its own arm (perpendicular to the arm direction, in the body's
## horizontal plane) — front/rear motors tilt in ±X, left/right
## motors tilt in ±Z.  The four tangential components form a pure
## yaw couple, so the drone spins on Y without drifting.
##
## Unlike a Y-rotation of the base thrust, this mechanism adds a
## horizontal component even when the base thrust is purely
## vertical — yaw works in full hover.
@export_range(0.0, 1.5708, 0.001) var yaw_tilt_angle: float = 0.35

## Sign flip for the yaw response.  If pushing the yaw stick spins
## the drone the wrong way, toggle this in the inspector rather
## than editing code.
@export var invert_yaw: bool = false


func update_mix(body: RigidBody3D, thrusters: Array[Node]) -> void:
	if thrusters.is_empty():
		return

	# Build a single thrust vector in body-local terms from raw input.
	# X = strafe (roll axis), Y = lift (throttle), Z = pitch.
	# The signs match Godot's "forward = -Z" convention — pushing
	# the pitch-forward action sends thrust along -Z (forward).
	# Inherited from: (parent) FlightControllerBase.get_axis_value
	var lift: float = get_axis_value(throttle_down_action, throttle_up_action)
	var strafe: float = get_axis_value(roll_left_action, roll_right_action)
	var forward: float = get_axis_value(pitch_backward_action, pitch_forward_action)
	var yaw: float = get_axis_value(yaw_left_action, yaw_right_action)

	var direction := Vector3(strafe, lift, -forward)
	if direction == Vector3.ZERO and is_zero_approx(yaw):
		_silence_all(thrusters)
		return

	# Preserve the *magnitude* of the player input — the trigger
	# value at 0.3 should produce 30% throttle, not full throttle.
	# `normalized()` would discard that magnitude and only keep the
	# direction.  Instead we clamp the length to 1.0 so a worst-case
	# diagonal like (1, 1, 1) (\u221a3 \u2248 1.73) stays inside the
	# thruster's [0, 1] throttle range.  Then scale by player
	# authority for the inspector-tunable ceiling.
	var magnitude: float = min(direction.length(), 1.0)
	var base_thrust: Vector3 = Vector3.ZERO
	if magnitude > 0.0:
		base_thrust = direction.normalized() * magnitude * power_authority

	# ── Yaw: tilt each motor's thrust tangent to its own arm ───────
	# Model: imagine each motor sits on a hinge at its arm's end.
	# The hinge axis runs *along the arm* (from body centre outward
	# to the motor).  Tilting the hinge swings the motor's thrust
	# in the direction perpendicular to the arm (and to Y).
	#
	# For a front motor (arm along +Z), perpendicular-to-arm in the
	# horizontal plane is ±X.  For a side motor (arm along +X), it
	# is ±Z.  The general rule for a motor at body-local offset
	# (x, _, z) is:
	#     tangent_dir = Vector3(-z, 0, x).normalized() * sign(...)
	# which is the 2D perpendicular in the XZ plane — the same as
	# `Vector3.UP.cross(arm)`.  Signing this so all motors push
	# clockwise (or all anti-clockwise) around the Y axis gives a
	# pure yaw couple.
	#
	# The magnitude we inject is `base_thrust.length() * sin(angle)`
	# so the sideways push scales with how hard the motor is already
	# pushing — zero throttle = zero yaw authority, which matches a
	# real rotor.  The vertical component is scaled by cos(angle)
	# so the *total* thrust magnitude stays equal to the base, i.e.
	# yawing never increases or decreases lift.
	var yaw_angle: float = yaw * yaw_tilt_angle
	if invert_yaw:
		yaw_angle = -yaw_angle

	var tilt_sin: float = sin(yaw_angle)
	var tilt_cos: float = cos(yaw_angle)

	for t in thrusters:
		if not t.has_method("set_thrust"):
			continue

		var per_thrust := base_thrust
		if not is_zero_approx(yaw_angle) and body != null:
			# arm = horizontal offset from body centre to this motor.
			# Its length is the lever arm; its direction tells us
			# which way "tangent to the arm" points.
			var body_offset: Vector3 = body.to_local(t.global_position)
			var arm := Vector3(body_offset.x, 0.0, body_offset.z)
			if arm.length_squared() > 0.0001:
				# tangent_dir is perpendicular to arm in the XZ plane,
				# oriented so all motors push the same rotational way
				# around Y (positive yaw = anti-clockwise viewed from
				# above, i.e. right-hand rule about +Y).
				var tangent_dir: Vector3 = Vector3.UP.cross(arm).normalized()
				var base_magnitude: float = base_thrust.length()
				# Preserve total thrust: the vertical (and any pre-
				# existing horizontal) part is scaled by cos, while
				# sin times magnitude goes into the tangent direction.
				per_thrust = base_thrust * tilt_cos + tangent_dir * base_magnitude * tilt_sin

		t.set_thrust(per_thrust)


## Cut all engines — used when input is fully released so they
## don't latch at the last commanded throttle.
func _silence_all(thrusters: Array[Node]) -> void:
	for t in thrusters:
		if t.has_method("set_thrust"):
			t.set_thrust(Vector3.ZERO)
		elif t.has_method("set_throttle"):
			t.set_throttle(0.0)
