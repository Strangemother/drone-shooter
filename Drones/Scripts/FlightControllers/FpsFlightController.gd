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
##   yaw_left / yaw_right           → unused at this stage
##
## All in the *thruster's local frame*.  Because each thruster's
## basis follows the parent body, "forward" means "wherever the body
## is pointing" — which is what an FPS player expects.
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


func update_mix(_body: RigidBody3D, thrusters: Array[Node]) -> void:
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

	var direction := Vector3(strafe, lift, -forward)
	if direction == Vector3.ZERO:
		_silence_all(thrusters)
		return

	# Normalise so diagonal inputs aren't √2 stronger than cardinals,
	# then scale by player authority.  The thruster will treat the
	# resulting length as its throttle.
	var thrust := direction.normalized() * power_authority

	for t in thrusters:
		if t.has_method("set_thrust"):
			t.set_thrust(thrust)


## Cut all engines — used when input is fully released so they
## don't latch at the last commanded throttle.
func _silence_all(thrusters: Array[Node]) -> void:
	for t in thrusters:
		if t.has_method("set_thrust"):
			t.set_thrust(Vector3.ZERO)
		elif t.has_method("set_throttle"):
			t.set_throttle(0.0)