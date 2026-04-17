## Base class for drone flight controllers.
## Attach a subclass of this as a child of a DroneScene to control its
## thrusters.  If no controller is attached the drone runs in "raw" mode —
## no automatic mixing, the developer (or another script) sets each
## thruster's throttle directly.
extends Node

class_name DroneFlightController

## Input action names — must match entries in Project > Input Map.
## Change these to remap controls without editing code.
@export var throttle_up_action: StringName = &"jump"
@export var throttle_down_action: StringName = &"crouch"
@export var pitch_forward_action: StringName = &"moveForward"
@export var pitch_backward_action: StringName = &"moveBackward"
@export var roll_left_action: StringName = &"moveLeft"
@export var roll_right_action: StringName = &"moveRight"


## Called every physics tick by DroneScript with the vehicle body and
## its list of discovered thrusters.  Override this in subclasses to
## implement your mixing strategy.
func update_mix(body: RigidBody3D, thrusters: Array[Node]) -> void:
	pass


# ── shared helpers available to every controller ──────────────────

func get_total_max_force(thrusters: Array[Node]) -> float:
	var total := 0.0
	for t in thrusters:
		total += t.get_max_force()
	return total


func get_max_arm_length(body: RigidBody3D, thrusters: Array[Node], axis: String) -> float:
	var max_arm := 0.0
	for t in thrusters:
		var offset := body.to_local(t.global_position)
		var v := offset.x if axis == "x" else offset.z
		max_arm = maxf(max_arm, absf(v))
	return max_arm


func get_axis_value(negative_action: StringName, positive_action: StringName) -> float:
	return _action_strength(positive_action) - _action_strength(negative_action)


func _action_strength(action: StringName) -> float:
	if action == &"":
		return 0.0
	if not InputMap.has_action(action):
		return 0.0
	return Input.get_action_strength(action)
