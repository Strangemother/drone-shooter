extends Node3D

class_name FlightThruster

@export_node_path("RigidBody3D") var target_body_path: NodePath = NodePath("..")
@export_range(0.0, 500.0, 0.1) var max_force: float = 45.0
@export_range(0.0, 1.0, 0.001) var throttle: float = 0.0:
	set(value):
		throttle = clampf(value, 0.0, 1.0)

## Thrust direction in the thruster's *local* frame.  The thruster's
## own `global_transform.basis` rotates this into world space each
## tick, so a force_axis of UP follows the thruster's orientation
## (and therefore the parent body's orientation) for free.
##
## A controller that wants to vector-thrust at runtime should call
## `set_thrust(Vector3)` rather than poking `force_axis` and
## `throttle` separately — see that method's docs.
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


## Set throttle in [0, 1].  Direction stays whatever `force_axis` was.
## Use this when you only want to vary power and the thruster's
## orientation is fixed (the common case).
func set_throttle(value: float) -> void:
	throttle = value


## Set thrust as a single vector.  The vector's *length* is the
## throttle (clamped to [0, 1]) and its *direction* is the local-
## frame axis the thrust is applied along.
##
## This is the recommended API for runtime controllers — the
## developer thinks in "I want this much push in this direction"
## terms, and the thruster handles converting that into a body
## force at the correct application point.  Body translation and
## torque from off-centre application happen automatically.
##
## Examples:
##   • `set_thrust(Vector3.UP * 0.5)` — half-throttle straight up.
##   • `set_thrust(Vector3.ZERO)` — engine off.
##   • `set_thrust(Vector3(0, 0.7, -0.3).normalized() * 0.8)` —
##     80% throttle, vectored mostly up with a forward component.
##
## Note: like `force_axis`, the direction is in the thruster's
## *local* frame.  If you want world-space thrust, transform with
## `global_transform.basis.inverse() * world_vector` first.
func set_thrust(thrust: Vector3) -> void:
	var power := thrust.length()
	if power <= 0.0001:
		throttle = 0.0
		return
	force_axis = thrust / power
	throttle = clampf(power, 0.0, 1.0)


## Convenience: set direction and power independently in one call.
## Equivalent to `set_thrust(direction.normalized() * power)` but
## skips a normalize when the caller already has a unit vector.
func set_thrust_directed(direction: Vector3, power: float) -> void:
	if direction == Vector3.ZERO or power <= 0.0:
		throttle = 0.0
		return
	force_axis = direction.normalized()
	throttle = clampf(power, 0.0, 1.0)


func get_max_force() -> float:
	return max_force


func _resolve_target_body() -> RigidBody3D:
	var node := get_node_or_null(target_body_path)
	if node is RigidBody3D:
		return node

	push_warning("Thruster target is not a RigidBody3D: %s" % [target_body_path])
	return null
