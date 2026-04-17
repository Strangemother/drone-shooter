extends Node3D

## Path to the drone node to reset. Set this in the inspector to match
## your scene tree (e.g. "ShootingRangeTargetManager/DroneScene").
@export var drone_path: NodePath = NodePath("ShootingRangeTargetManager/DroneScene")

## The drone's spawn transform, captured automatically on _ready().
var _drone_spawn_transform: Transform3D


func _ready() -> void:
	var drone := _get_drone()
	if drone:
		_drone_spawn_transform = drone.global_transform


func _notification(what):
	if what == NOTIFICATION_WM_CLOSE_REQUEST:
		get_tree().quit() # default behavior


func _input(event):
	if event.as_text() == 'Escape':
		get_tree().quit() # default behavior


func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("resetDrone"):
		reset_drone()


func reset_drone() -> void:
	print('reset_drone')
	var drone := _get_drone()
	if drone == null:
		print('No drone to reset.')
		return
	print('Reseting drone: ', drone)
	# Zero all motion so it doesn't carry momentum from before the reset.
	drone.linear_velocity = Vector3.ZERO
	drone.angular_velocity = Vector3.ZERO

	# Snap back to spawn position and orientation.
	drone.global_transform = _drone_spawn_transform


func _get_drone() -> RigidBody3D:
	var node := get_node_or_null(drone_path)
	if node is RigidBody3D:
		return node
	return null
