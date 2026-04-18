@tool
extends Node3D
class_name WaypointPath

## Multi-waypoint path follower (Stage 1 — straight segments + early swap).
##
## Feeds a drone one child Node3D at a time via the existing
## `set_waypoint(Node3D)` API on the drone.  When the drone gets within
## `swap_radius` of the current target, the next child is handed over.
##
## Nothing in `DroneScript` or the flight controller changes — this node
## is a pure *consumer* of the same public API a human would call from
## code:
##
##     get_tree().call_group("drones", "set_waypoint", some_node3d)
##
## ── Usage ───────────────────────────────────────────────────────────
##   1. Add this script to a Node3D in the scene.
##   2. Parent 2+ Node3D children under it — these are the waypoints,
##      visited in scene-tree order (top to bottom).
##   3. Either:
##        • leave `drone_path` empty → the path auto-binds to every
##          drone in the "drones" group (simple single-drone scenes);
##        • or drag a specific drone into `drone_path` to bind just one.
##   4. Press play.  Watch the `waypoint_reached` / `path_completed`
##      signals if gameplay needs to react.

# ── signals ────────────────────────────────────────────────────────

signal path_started
signal waypoint_reached(index: int, waypoint: Node3D)
signal path_completed
signal path_looped
signal path_cancelled


# ── exports ────────────────────────────────────────────────────────

## Optional: specific drone (RigidBody3D running DroneScript) to bind
## to.  Leave empty to broadcast to every node in the "drones" group.
@export var drone_path: NodePath = NodePath("")

## Distance (metres) at which the current waypoint is considered
## "reached" and the next one is swapped in.  Tune against the
## drone's `arrival_radius` in the flight controller — this should
## usually be ≥ that value so the path advances *before* the drone
## bothers to stop.  Default 2 m feels good for a cruise speed of
## ~6 m/s.
@export_range(0.1, 50.0, 0.1) var swap_radius: float = 2.0

## End-of-path behaviour.
enum LoopMode { NONE, LOOP, PING_PONG }
@export var loop_mode: LoopMode = LoopMode.NONE

## If true, binds & starts automatically in `_ready`.  Turn off if
## gameplay code wants to trigger the path manually via `start()`.
@export var auto_start: bool = true

## If true, prints the current index / distance a few times a second.
@export var debug_print: bool = false


# ── internal state ────────────────────────────────────────────────

var _waypoints: Array[Node3D] = []
var _index: int = 0
var _direction: int = 1                 # +1 forward, -1 reverse (ping-pong)
var _active: bool = false
var _bound_drones: Array[Node] = []
var _debug_accum: float = 0.0


func _ready() -> void:
	if Engine.is_editor_hint():
		return
	_refresh_waypoints()
	if auto_start:
		# Defer so all drones have had their own _ready and are in the
		# "drones" group before we try to bind.
		call_deferred("start")


func _physics_process(delta: float) -> void:
	if Engine.is_editor_hint():
		return
	if not _active or _waypoints.is_empty() or _bound_drones.is_empty():
		return

	var current: Node3D = _waypoints[_index]
	if current == null or not is_instance_valid(current):
		# Waypoint was freed mid-flight — resync.
		_refresh_waypoints()
		if _waypoints.is_empty():
			cancel()
			return
		_index = clamp(_index, 0, _waypoints.size() - 1)
		_push_waypoint(_waypoints[_index])
		return

	# Distance is measured from the *first* bound drone.  Multi-drone
	# use cases need one WaypointPath per drone (see design brief §7.5).
	var drone := _first_valid_drone()
	if drone == null:
		return

	var dist := drone.global_position.distance_to(current.global_position)

	if debug_print:
		_debug_accum += delta
		if _debug_accum > 0.5:
			_debug_accum = 0.0
			print("[WaypointPath] idx=%d dist=%.2f target=%s" % [_index, dist, current.name])

	if dist <= swap_radius:
		emit_signal("waypoint_reached", _index, current)
		_advance()


# ── public API ────────────────────────────────────────────────────

## Start (or restart) following the path from the beginning.
func start() -> void:
	_refresh_waypoints()
	if _waypoints.is_empty():
		push_warning("WaypointPath '%s' has no waypoint children." % name)
		return
	_bound_drones = _resolve_drones()
	if _bound_drones.is_empty():
		push_warning("WaypointPath '%s' could not find a drone to bind to." % name)
		return
	_index = 0
	_direction = 1
	_active = true
	_push_waypoint(_waypoints[_index])
	emit_signal("path_started")


## Stop following, clear the drone's waypoint, and reset progress.
func cancel() -> void:
	if not _active:
		return
	_active = false
	for drone in _bound_drones:
		if is_instance_valid(drone) and drone.has_method("clear_waypoint"):
			drone.clear_waypoint()
	emit_signal("path_cancelled")


## Re-scan child Node3Ds.  Call after adding / removing waypoints at
## runtime.
func refresh() -> void:
	_refresh_waypoints()


# ── internals ─────────────────────────────────────────────────────

func _advance() -> void:
	match loop_mode:
		LoopMode.NONE:
			if _index + 1 >= _waypoints.size():
				_active = false
				for drone in _bound_drones:
					if is_instance_valid(drone) and drone.has_method("clear_waypoint"):
						drone.clear_waypoint()
				emit_signal("path_completed")
				return
			_index += 1
		LoopMode.LOOP:
			_index = (_index + 1) % _waypoints.size()
			if _index == 0:
				emit_signal("path_looped")
		LoopMode.PING_PONG:
			var next: int = _index + _direction
			if next >= _waypoints.size() or next < 0:
				_direction = -_direction
				next = _index + _direction
				emit_signal("path_looped")
			_index = next

	_push_waypoint(_waypoints[_index])


func _push_waypoint(wp: Node3D) -> void:
	for drone in _bound_drones:
		if is_instance_valid(drone) and drone.has_method("set_waypoint"):
			drone.set_waypoint(wp)


func _refresh_waypoints() -> void:
	_waypoints.clear()
	for child in get_children():
		if child is Node3D:
			_waypoints.append(child)


func _resolve_drones() -> Array[Node]:
	var out: Array[Node] = []
	if drone_path != NodePath(""):
		var n := get_node_or_null(drone_path)
		if n != null:
			out.append(n)
		else:
			push_warning("WaypointPath: drone_path '%s' did not resolve." % drone_path)
		return out
	# Fall back to the "drones" group.
	out.assign(get_tree().get_nodes_in_group("drones"))
	return out


func _first_valid_drone() -> Node3D:
	for drone in _bound_drones:
		if is_instance_valid(drone) and drone is Node3D:
			return drone
	return null


func _get_configuration_warnings() -> PackedStringArray:
	var warnings := PackedStringArray()
	var count := 0
	for child in get_children():
		if child is Node3D:
			count += 1
	if count < 2:
		warnings.append("WaypointPath needs at least 2 Node3D children to form a path.")
	return warnings
