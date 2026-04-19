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
## ── Yaw model (iteration 2 — motor tilt) ───────────────────────────
## Yaw is produced by *physically rotating each motor node* around
## its own arm axis (the horizontal line from body centre out to the
## motor).  Because a thruster's `force_axis` is `UP` in its own
## local frame, tilting the motor tilts the thrust vector along
## with it — the controller does not compute any rotated vectors;
## it just sets the motor's transform and lets the thruster do its
## normal job.
##
## This is the "obvious planar" model: move the motor, the impulse
## follows.  It is visually self-evident in the editor (you can see
## each motor tilt), works in dead hover (the tilt adds a horizontal
## component that would otherwise be absent), and produces a pure
## yaw couple because each motor's tangential push cancels its
## opposite-side partner's translationally while reinforcing about
## body-Y.
##
## The controller caches each motor's rest transform on first use
## (or after `reset_state()`) and rebuilds the tilted transform
## fresh every tick, so there is no drift from accumulated
## rotations.
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
## yaw-stick deflection.  The controller rotates each motor node
## around its own arm axis by this angle (signed so all four
## motors push the same rotational way around body-Y), yielding
## a pure yaw couple.
##
## Works in full hover because tilting the motor adds a horizontal
## thrust component that is independent of the base thrust.
@export_range(0.0, 1.5708, 0.001) var yaw_tilt_angle: float = 0.35

## Sign flip for the yaw response.  If pushing the yaw stick spins
## the drone the wrong way, toggle this in the inspector rather
## than editing code.
@export var invert_yaw: bool = false

## Baseline throttle applied along local UP while the yaw stick is
## deflected, scaled by |yaw|.  Exists because this is a tilt-rotor
## yaw model: the motors produce a yaw couple by redirecting their
## thrust tangentially, so with zero thrust there is nothing to
## redirect and yaw authority vanishes at idle.  Real quadcopters
## never have this problem — their props must spin to hover, so
## yaw (from differential prop RPM) is always hot.  Adding a small
## idle along UP whenever the player asks for yaw mimics that
## "props are already spinning" feel without touching the rest of
## the flight model.
##
## Only raises the UP component to the idle floor; if the player
## is already commanding more throttle than the idle, their input
## wins and this has no effect.  Set to 0 to disable.
@export_range(0.0, 1.0, 0.001) var yaw_idle_thrust: float = 0.15

## How much the yaw tilt is attenuated when the player is also
## commanding translation thrust.  At 0.0 the tilt angle is constant
## regardless of throttle (original behaviour).  At 1.0 the tilt
## fades linearly to zero as base-thrust magnitude rises toward 1,
## pushing almost all yaw authority onto the idle-thrust path.
##
## Rationale: the tilt-and-idle combo gives the most drone-like feel
## at low throttle; when the player is already pushing hard, the
## extra tilt stacks on top of high thrust and over-rotates.  Biasing
## yaw toward the idle path keeps the response consistent across the
## throttle range.
@export_range(0.0, 1.0, 0.001) var yaw_throttle_attenuation: float = 0.6


# Rest basis of each motor, captured the first time we see it.
# We rebuild each motor's transform from its rest state every
# tick so tilt commands don't accumulate frame-to-frame.
var _motor_rest_basis: Dictionary = {}


func reset_state() -> void:
	# Drop cached rest bases; they'll be re-captured on next tick.
	# Call this after programmatically repositioning motors.
	_motor_rest_basis.clear()


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

	# ── Yaw idle: give the tilt something to redirect ──────────────
	# With a tilt-rotor yaw model, the yaw couple is produced by
	# rotating each motor's thrust vector tangent to its arm.  If
	# the motors are producing zero thrust (player not touching
	# throttle/pitch/roll), the tilt rotates a zero vector and the
	# drone doesn't yaw.  Real quads hide this because their props
	# must spin to hover; we mimic it by raising the UP component
	# of base_thrust to a small floor whenever the yaw stick is
	# deflected.  Scaled by |yaw| so a light yaw input adds only a
	# light idle — no audible motor jump when the stick is barely
	# touched.
	var yaw_abs: float = absf(yaw)
	if yaw_idle_thrust > 0.0 and yaw_abs > 0.0:
		var idle: float = yaw_idle_thrust * yaw_abs * power_authority
		if base_thrust.y < idle:
			base_thrust.y = idle

	# ── Yaw: rotate each motor node around its arm axis ────────────
	# The motor's arm axis is the horizontal vector from the body
	# centre to the motor.  Rotating the motor node around this
	# axis tilts its local UP — and therefore its thrust — tangent
	# to the arm.  All four motors tilted by the same signed angle
	# around their outward-pointing arm axis produce tangential
	# pushes that add up to a pure yaw couple (see yaw-model
	# header above for the derivation).
	var yaw_angle: float = yaw * yaw_tilt_angle
	if invert_yaw:
		yaw_angle = -yaw_angle

	# Attenuate tilt in proportion to player-commanded translation
	# thrust.  `magnitude` here is the pre-idle base-thrust magnitude
	# (i.e. how hard the player is pushing translation stick inputs),
	# so attenuation is driven purely by the player's own throttle
	# and does not self-cancel against the yaw idle we just added.
	if yaw_throttle_attenuation > 0.0 and magnitude > 0.0:
		yaw_angle *= 1.0 - yaw_throttle_attenuation * magnitude

	for t in thrusters:
		if not (t is Node3D):
			continue

		# Capture rest basis the first time we see each motor.
		if not _motor_rest_basis.has(t):
			_motor_rest_basis[t] = (t as Node3D).transform.basis

		var rest_basis: Basis = _motor_rest_basis[t]

		# Compute the hinge axis in the motor's *parent* frame
		# (which is the body frame, since motors are direct
		# children of the RigidBody3D).  This is just the
		# horizontal part of the motor's local position — no
		# matter how the body itself is oriented in the world,
		# the local offset is constant per motor.
		if yaw_angle != 0.0 and body != null:
			var local_pos: Vector3 = (t as Node3D).transform.origin
			var arm := Vector3(local_pos.x, 0.0, local_pos.z)
			if arm.length_squared() > 0.0001:
				var hinge: Vector3 = arm.normalized()
				# Rotate the rest basis around the hinge (in
				# parent/body frame) by the yaw angle.  All four
				# motors get the same signed angle; because each
				# hinge points outward from the centre, the four
				# tangential tilts swirl the same way around Y.
				(t as Node3D).transform.basis = rest_basis.rotated(hinge, yaw_angle)
			else:
				(t as Node3D).transform.basis = rest_basis
		else:
			# No yaw input — snap back to rest so we don't hold a
			# stale tilt from a previous tick.
			(t as Node3D).transform.basis = rest_basis

		# Finally, hand the base thrust to the motor.  The motor's
		# now-tilted basis will rotate `UP` into the correct world
		# direction for us — no per-motor vector math required.
		if t.has_method("set_thrust"):
			t.set_thrust(base_thrust)

	if base_thrust == Vector3.ZERO and yaw_angle == 0.0:
		_silence_all(thrusters)


## Cut all engines — used when input is fully released so they
## don't latch at the last commanded throttle.
func _silence_all(thrusters: Array[Node]) -> void:
	for t in thrusters:
		if t.has_method("set_thrust"):
			t.set_thrust(Vector3.ZERO)
		elif t.has_method("set_throttle"):
			t.set_throttle(0.0)
