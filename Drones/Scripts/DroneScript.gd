extends RigidBody3D

@export_range(0.0, 1.0, 0.01) var bullet_density : float = 1.0

## The discovered thruster nodes (children in "vehicle_thrusters" group).
var thrusters: Array[Node] = []

## The active flight controller child (FlightAngleController, FlightAcroController, etc).
## If null the drone is in "raw" mode — no automatic mixing, you set
## each thruster's throttle yourself from external code.
var flight_controller: FlightControllerBase


func _ready() -> void:
	add_to_group("drones")
	_collect_thrusters()
	flight_controller = _find_flight_controller()
	if flight_controller:
		flight_controller.update_mix(self, thrusters)


func _physics_process(_delta: float) -> void:
	if thrusters.is_empty():
		return
	# Raw mode: no controller attached — do nothing, let external code drive.
	if flight_controller == null:
		return
	flight_controller.update_mix(self, thrusters)


func _collect_thrusters() -> void:
	thrusters.clear()
	for child in find_children("*", "", true, false):
		if child.is_in_group("vehicle_thrusters") and child.has_method("set_throttle") and child.has_method("get_max_force"):
			thrusters.append(child)
	if thrusters.is_empty():
		push_warning("DroneScene has no mounted thrusters.")


## Finds the first FlightControllerBase child.  Returns null for raw mode.
func _find_flight_controller() -> FlightControllerBase:
	for child in get_children():
		if child is FlightControllerBase:
			return child
	return null


## Called after a position reset to clear any accumulated controller state.
func reset_flight_controller() -> void:
	if flight_controller and flight_controller.has_method("reset_state"):
		flight_controller.reset_state()


## Set a waypoint target on the flight controller (if it supports waypoints).
## Call from anywhere:  get_tree().call_group("drones", "set_waypoint", some_node3d)
func set_waypoint(target: Node3D) -> void:
	if flight_controller and flight_controller.has_method("set_waypoint"):
		flight_controller.set_waypoint(target)


## Clear the current waypoint — the drone reverts to hover-in-place.
## Call from anywhere:  get_tree().call_group("drones", "clear_waypoint")
func clear_waypoint() -> void:
	if flight_controller and flight_controller.has_method("clear_waypoint"):
		flight_controller.clear_waypoint()

func get_bullet_density() -> float:
	return bullet_density

func hitscanHit(propulForce : float, propulDir: Vector3, propulPos : Vector3):
	var hitPos : Vector3 = propulPos - global_transform.origin#set the position to launch the object at
	if propulDir != Vector3.ZERO: apply_impulse(propulDir * propulForce, hitPos)
	
func projectileHit(propulForce : float, propulDir: Vector3):
	if propulDir != Vector3.ZERO: apply_central_force((global_transform.origin - propulDir) * propulForce)
	
