extends RigidBody3D

@export_range(0.0, 1.0, 0.01) var bullet_density : float = 1.0

## ── Motor-point-mass inertia override (physics-accuracy-review §2.L) ──
##
## Godot's `RigidBody3D.inertia` defaults to a shape-derived tensor
## computed from the body's collision shapes — typically a `BoxShape3D`
## treated as a solid of uniform density.  For a real quadcopter this
## is **wrong**: almost all the rotational inertia comes from four
## motors/batteries concentrated near the tips of the arms, not from a
## uniformly dense block.  The shape-derived number overestimates yaw
## inertia (mass near the axis contributes less than a uniform
## distribution suggests) and underestimates pitch/roll inertia
## (lumped corner masses are further from the body centre than an
## equivalent uniform box's effective radius).
##
## When `use_motor_point_inertia` is `true`, `_ready()` computes the
## inertia tensor as four point masses at the thruster positions:
##
##     m_motor = mass · motor_mass_fraction / N_thrusters
##     I_xx   = Σ m_motor · (y² + z²)     (pitch axis, body-local)
##     I_yy   = Σ m_motor · (x² + z²)     (yaw   axis)
##     I_zz   = Σ m_motor · (x² + y²)     (roll  axis)
##
## and writes it directly to `inertia`.  The remaining mass
## `(1 − motor_mass_fraction)` is treated as a point at the body
## origin (contributes 0 to rotational inertia), which is a passable
## approximation for a central flight-controller stack + battery.
##
## Left off by default so existing scenes are not silently changed.
## Enable on drones whose motors sit well outboard — the effect is
## most visible on frames with large arm length.
@export var use_motor_point_inertia: bool = false

## Fraction of total mass treated as concentrated at the motor
## positions.  The remainder is assumed to be at the body centre and
## contributes nothing to rotational inertia.  Real 5"–10" quads are
## roughly 60–80 % motor-mass-fraction (each motor+prop+ESC weighs a
## surprising amount vs. the central stack).  Racing frames skew
## higher; cinelifters with heavy camera payloads skew lower.
##
## Only consulted when `use_motor_point_inertia` is true.
@export_range(0.0, 1.0, 0.01) var motor_mass_fraction: float = 0.75

## The discovered thruster nodes (children in "vehicle_thrusters" group).
var thrusters: Array[Node] = []

## The active flight controller child (FlightAngleController, FlightAcroController, etc).
## If null the drone is in "raw" mode — no automatic mixing, you set
## each thruster's throttle yourself from external code.
var flight_controller: FlightControllerBase


func _ready() -> void:
	add_to_group("drones")
	_collect_thrusters()
	if use_motor_point_inertia:
		_apply_motor_point_inertia()
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


## Compute and apply the point-mass inertia tensor described at
## `use_motor_point_inertia`.  Called once from `_ready()` after
## thrusters are collected.  Silently no-ops if there are no
## thrusters (shape-derived inertia stays in effect).
func _apply_motor_point_inertia() -> void:
	if thrusters.is_empty():
		return

	var m_motor: float = mass * motor_mass_fraction / float(thrusters.size())
	# Accumulate Σ m·(a² + b²) for each of the three body-local axes.
	# Godot's `inertia` Vector3 is (I_xx, I_yy, I_zz) — pitch, yaw, roll.
	var i_xx := 0.0  # around body X → pitch axis
	var i_yy := 0.0  # around body Y → yaw   axis
	var i_zz := 0.0  # around body Z → roll  axis
	for t in thrusters:
		if not (t is Node3D):
			continue
		var p: Vector3 = to_local((t as Node3D).global_position)
		i_xx += m_motor * (p.y * p.y + p.z * p.z)
		i_yy += m_motor * (p.x * p.x + p.z * p.z)
		i_zz += m_motor * (p.x * p.x + p.y * p.y)

	inertia = Vector3(i_xx, i_yy, i_zz)


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
	
