extends RigidBody3D

@export_range(0.0, 1.0, 0.01) var bullet_density : float = 1.0

@export_range(0.0, 20.0, 0.01) var anti_gravity : float = 0.0
@export_range(-1.0, 1.0, 0.001) var throttle_trim : float = 0.0
@export_range(0.0, 1.0, 0.001) var collective_authority : float = 0.25
@export_range(0.0, 1.0, 0.001) var pitch_authority : float = 0.18
@export_range(0.0, 1.0, 0.001) var roll_authority : float = 0.18
@export var throttle_up_action: StringName = &"jump"
@export var throttle_down_action: StringName = &"crouch"
@export var pitch_forward_action: StringName = &"moveForward"
@export var pitch_backward_action: StringName = &"moveBackward"
@export var roll_left_action: StringName = &"moveLeft"
@export var roll_right_action: StringName = &"moveRight"

var thrusters: Array[Node] = []


func _ready() -> void:
	_collect_thrusters()
	_update_thruster_mix()


func _physics_process(_delta: float) -> void:
	if thrusters.is_empty():
		return

	_update_thruster_mix()


func _collect_thrusters() -> void:
	thrusters.clear()

	for child in find_children("*", "", true, false):
		if child.is_in_group("vehicle_thrusters") and child.has_method("set_throttle") and child.has_method("get_max_force"):
			thrusters.append(child)

	if thrusters.is_empty():
		push_warning("DroneScene has no mounted thrusters.")


func _update_thruster_mix() -> void:
	var total_max_force := _get_total_max_force()
	if total_max_force <= 0.0:
		return

	var collective := _get_hover_throttle(total_max_force)
	collective += throttle_trim
	collective += _get_axis_value(throttle_down_action, throttle_up_action) * collective_authority

	var pitch_input := _get_axis_value(pitch_backward_action, pitch_forward_action)
	var roll_input := _get_axis_value(roll_left_action, roll_right_action)
	var max_arm_x := _get_max_arm_length("x")
	var max_arm_z := _get_max_arm_length("z")

	for thruster in thrusters:
		var local_offset := to_local(thruster.global_position)
		var pitch_mix := 0.0
		var roll_mix := 0.0

		if max_arm_z > 0.0:
			pitch_mix = (local_offset.z / max_arm_z) * pitch_input * pitch_authority

		if max_arm_x > 0.0:
			roll_mix = (-local_offset.x / max_arm_x) * roll_input * roll_authority

		thruster.set_throttle(clampf(collective + pitch_mix + roll_mix, 0.0, 1.0))


func _get_total_max_force() -> float:
	var total := 0.0
	for thruster in thrusters:
		total += thruster.get_max_force()
	return total


func _get_hover_throttle(total_max_force: float) -> float:
	var gravity_acceleration := maxf(float(ProjectSettings.get_setting("physics/3d/default_gravity")) - anti_gravity, 0.0)
	var required_force := mass * gravity_acceleration
	return clampf(required_force / total_max_force, 0.0, 1.0)


func _get_max_arm_length(axis: String) -> float:
	var max_arm := 0.0
	for thruster in thrusters:
		var local_offset := to_local(thruster.global_position)
		var axis_value := local_offset.x if axis == "x" else local_offset.z
		max_arm = maxf(max_arm, absf(axis_value))
	return max_arm


func _get_axis_value(negative_action: StringName, positive_action: StringName) -> float:
	return _get_action_strength(positive_action) - _get_action_strength(negative_action)


func _get_action_strength(action_name: StringName) -> float:
	if action_name == &"":
		return 0.0

	if not InputMap.has_action(action_name):
		return 0.0

	return Input.get_action_strength(action_name)

func get_bullet_density() -> float:
	return bullet_density

func hitscanHit(propulForce : float, propulDir: Vector3, propulPos : Vector3):
	var hitPos : Vector3 = propulPos - global_transform.origin#set the position to launch the object at
	if propulDir != Vector3.ZERO: apply_impulse(propulDir * propulForce, hitPos)
	
func projectileHit(propulForce : float, propulDir: Vector3):
	if propulDir != Vector3.ZERO: apply_central_force((global_transform.origin - propulDir) * propulForce)
	
