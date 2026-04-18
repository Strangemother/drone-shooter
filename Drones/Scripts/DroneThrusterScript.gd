extends Node3D

class_name FlightThruster

@export_node_path("RigidBody3D") var target_body_path: NodePath = NodePath("..")
@export_range(0.0, 500.0, 0.1) var max_force: float = 45.0
@export_range(0.0, 1.0, 0.001) var throttle: float = 0.0:
	set(value):
		throttle = clampf(value, 0.0, 1.0)
@export var force_axis: Vector3 = Vector3.UP
@export var enabled: bool = true

var target_body: RigidBody3D


func _enter_tree() -> void:
	add_to_group("vehicle_thrusters")


func _ready() -> void:
	target_body = _resolve_target_body()


func _physics_process(_delta: float) -> void:
	if not enabled:
		return

	if throttle <= 0.0:
		return

	if target_body == null:
		target_body = _resolve_target_body()
		if target_body == null:
			return

	var normalized_axis := force_axis.normalized()
	if normalized_axis == Vector3.ZERO:
		return

	var force := global_transform.basis * normalized_axis * max_force * throttle
	var application_point := global_position - target_body.global_position
	target_body.sleeping = false
	target_body.apply_force(force, application_point)


func set_throttle(value: float) -> void:
	throttle = value


func get_max_force() -> float:
	return max_force


func _resolve_target_body() -> RigidBody3D:
	var node := get_node_or_null(target_body_path)
	if node is RigidBody3D:
		return node

	push_warning("Thruster target is not a RigidBody3D: %s" % [target_body_path])
	return null
