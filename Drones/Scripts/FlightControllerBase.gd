## Base class for drone flight controllers.
##
## Attach a subclass of this as a child of a `DroneScene` to control its
## thrusters.  If no controller is attached the drone runs in "raw"
## mode — no automatic mixing; external code sets each thruster's
## throttle directly via `set_throttle`.
##
## Subclasses must override `update_mix()`.  Optionally override
## `reset_state()` if the controller accumulates state (PID integrals,
## filters) that needs clearing after a teleport.
##
## ── Input convention ───────────────────────────────────────────────
## Input action names are exported so you can remap controls in the
## inspector without editing code.  An empty `StringName` (&"") means
## "this axis has no input bound"; the controller must tolerate that.
##
## ── Naming ─────────────────────────────────────────────────────────
## Class name is `FlightControllerBase` (the file of the same name).
## Subclasses should be named `Flight<Mode>Controller`, e.g.
## `FlightAngleController`, `FlightAcroController`.
extends Node

class_name FlightControllerBase


# ── input action bindings ─────────────────────────────────────────

## Input action names — must match entries in Project > Input Map.
## Change these to remap controls without editing code.  An empty
## StringName means "axis is unbound" and returns 0 from
## `get_axis_value`.
@export var throttle_up_action: StringName = &"jump"
@export var throttle_down_action: StringName = &"crouch"
@export var pitch_forward_action: StringName = &"moveForward"
@export var pitch_backward_action: StringName = &"moveBackward"
@export var roll_left_action: StringName = &"moveLeft"
@export var roll_right_action: StringName = &"moveRight"
@export var yaw_left_action: StringName = &""
@export var yaw_right_action: StringName = &""


# ── virtual overrides ─────────────────────────────────────────────

## Called every physics tick by `DroneScript` with the vehicle body
## and its list of discovered thrusters.  Override in subclasses to
## implement your mixing strategy.  The base implementation is a
## no-op — a controller that doesn't override this does nothing.
func update_mix(_body: RigidBody3D, _thrusters: Array[Node]) -> void:
	pass


## Called after a position reset (teleport, respawn) to clear any
## accumulated internal state.  Override in subclasses that track
## state (e.g. PID integrals, velocity filters).
func reset_state() -> void:
	pass


# ── shared helpers available to every controller ──────────────────

## Sum of every thruster's `get_max_force()`.  Used to normalise
## per-thruster throttle commands.
func get_total_max_force(thrusters: Array[Node]) -> float:
	var total := 0.0
	for t in thrusters:
		total += t.get_max_force()
	return total


## Longest lever arm on `axis` ("x" or "z") — the distance from the
## body centre to the farthest thruster along that axis.  Used to
## convert "I want this much torque" into per-thruster throttle
## differentials.
func get_max_arm_length(body: RigidBody3D, thrusters: Array[Node], axis: String) -> float:
	var max_arm := 0.0
	for t in thrusters:
		var offset := body.to_local(t.global_position)
		var v := offset.x if axis == "x" else offset.z
		max_arm = maxf(max_arm, absf(v))
	return max_arm


## Returns a scalar in [-1, 1] for the axis formed by two input
## actions.  Unbound actions (empty StringName) contribute 0.
func get_axis_value(negative_action: StringName, positive_action: StringName) -> float:
	return _action_strength(positive_action) - _action_strength(negative_action)


func _action_strength(action: StringName) -> float:
	if action == &"":
		return 0.0
	if not InputMap.has_action(action):
		return 0.0
	return Input.get_action_strength(action)
