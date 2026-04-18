@tool
extends Node3D
class_name WaypointPathCurve

## Multi-waypoint path follower (Stage 3 — Curve3D-backed smooth path).
##
## Same integration contract as `WaypointPath` and `WaypointPathSmooth`:
##   • No changes to DroneScript or any FlightController.
##   • Consumes the drone's public `set_waypoint(Node3D)` API.
##
## The difference is *what curve the virtual target sweeps along*.
## Stage 2 (`WaypointPathSmooth`) uses straight-line polyline segments
## between waypoints — corners are rounded only because pursuit
## lookahead cuts them.  Stage 3 replaces the polyline with a real
## curve (`Curve3D`) whose control handles are computed automatically
## from the surrounding waypoints (Catmull-Rom-style).  The path
## itself is now curved, so even a slow drone traces a smooth line.
##
## See `docs/multi-waypoint-design-brief.md` §5 stage 3.
##
## ── Usage ───────────────────────────────────────────────────────────
##   1. Add this script to a Node3D in the scene.
##   2. Parent 2+ Node3D children under it — curve control points,
##      visited in scene-tree order.
##   3. Either leave `drone_path` empty (auto-binds every drone in the
##      "drones" group) or drag a specific drone into it.
##   4. Press play.
##
## ── Tuning ──────────────────────────────────────────────────────────
##   • `cruise_speed` ≤ flight controller's `max_approach_speed`.
##   • `lookahead_distance` ≥ flight controller's brake distance
##     (`max_approach_speed / pos_p`, default 24 m) so the controller
##     commands full cruise between waypoints.
##   • `curve_tension` controls how tightly the curve hugs the
##     waypoints.  0 = straight (collapses to Stage 2 behaviour),
##     0.5 = Catmull-Rom (default, smooth arc through each point),
##     1.0 = loopy, overshoots — usually too much.
##
## At the end of a non-looping path the drone is given the *final*
## waypoint directly so the flight controller's arrival logic engages.

# ── signals ────────────────────────────────────────────────────────

signal path_started
signal waypoint_reached(index: int, waypoint: Node3D)
signal path_completed
signal path_looped
signal path_cancelled


# ── exports ────────────────────────────────────────────────────────

## Optional: specific drone to bind to.  Leave empty to broadcast to
## every node in the "drones" group.
@export var drone_path: NodePath = NodePath("")

## How fast the virtual target advances along the curve (m/s).
@export_range(0.1, 50.0, 0.1) var cruise_speed: float = 6.0

## How far ahead of the drone's current arc-length the virtual target
## sits (metres).  See `docs/waypoint-controller-guide.md` §brake
## distance — usually ≥ `max_approach_speed / pos_p` (default 24 m).
@export_range(0.1, 200.0, 0.1) var lookahead_distance: float = 25.0

## Maximum metres the virtual target is allowed to lead the drone.
## Prevents a mis-tuned cruise_speed from outrunning the drone (brief
## §7.3).  Set 0 to disable.
@export_range(0.0, 200.0, 0.1) var max_lead_distance: float = 30.0

## Distance to the *final* point at which the real Node3D is handed
## to the controller so its arrival/brake logic takes over.
@export_range(0.1, 20.0, 0.1) var arrival_handoff_distance: float = 2.0

## Curve smoothing.  0 = straight (polyline), 0.5 = Catmull-Rom,
## 1.0 = exaggerated loops.  See class docs.
@export_range(0.0, 1.0, 0.01) var curve_tension: float = 0.5

## End-of-path behaviour.
enum LoopMode { NONE, LOOP, PING_PONG }
@export var loop_mode: LoopMode = LoopMode.NONE

## If true, binds & starts automatically in `_ready`.
@export var auto_start: bool = true

## Print arc-length / lookahead a few times a second.
@export var debug_print: bool = false


# ── internal state ────────────────────────────────────────────────

var _waypoints: Array[Node3D] = []

## The real smoothing engine.  Rebuilt whenever waypoints change.
## We deliberately use Godot's Curve3D (not a polyline) so we get
## `sample_baked(offset_metres)` for arc-length parameterisation
## for free (brief §4.5 — parametric `t` is wrong for constant-speed
## motion, baked is right).
var _curve: Curve3D = null

## Arc-length of each *authored* waypoint along the baked curve.
## Used to fire `waypoint_reached` at the right moments.
var _waypoint_arc_lengths: PackedFloat32Array = PackedFloat32Array()

## True when `loop_mode == LOOP`.  The curve has a synthetic closing
## span so the virtual target can sweep last→first smoothly.
var _closed: bool = false

var _last_announced_index: int = -1
var _virtual_target: Node3D = null
var _s_drone: float = 0.0
var _s_target: float = 0.0
var _direction: int = 1
var _active: bool = false
var _bound_drones: Array[Node] = []
var _debug_accum: float = 0.0
var _completed_emitted: bool = false


func _ready() -> void:
	if Engine.is_editor_hint():
		return
	_ensure_virtual_target()
	_refresh_waypoints()
	if auto_start:
		call_deferred("start")


func _physics_process(delta: float) -> void:
	if Engine.is_editor_hint():
		return
	if not _active or _curve == null or _bound_drones.is_empty():
		return
	var path_length := _curve.get_baked_length()
	if path_length <= 0.0:
		return

	var drone := _first_valid_drone()
	if drone == null:
		return

	# Project drone onto the curve.  `sample_baked` + local search
	# is the poor-man's "closest point on curve" — Curve3D has no
	# built-in projection, but sampling the baked points and picking
	# the nearest works well for the sub-hundred-m paths this system
	# is built for.
	_s_drone = _project_to_arc_length(drone.global_position, _s_drone, path_length)

	# Advance the virtual target's arc-length.
	_s_target += cruise_speed * delta
	if _closed:
		_s_target = fposmod(_s_target, path_length)

	# Lead clamp — same logic as Stage 2, handled modulo in closed.
	if max_lead_distance > 0.0:
		var lead: float = _s_target - _s_drone
		if _closed:
			lead = fposmod(lead, path_length)
		if lead > max_lead_distance:
			_s_target = _s_drone + max_lead_distance
			if _closed:
				_s_target = fposmod(_s_target, path_length)

	var chase_s: float = _s_drone + lookahead_distance

	if _closed:
		if chase_s >= path_length:
			_last_announced_index = -1
			emit_signal("path_looped")
		chase_s = fposmod(chase_s, path_length)
		_handle_progress(chase_s)
	else:
		_handle_progress(chase_s)
		if _handle_end_of_path(drone, chase_s, path_length):
			return
		chase_s = clamp(chase_s, 0.0, path_length)

	_virtual_target.global_position = _curve.sample_baked(chase_s)

	if debug_print:
		_debug_accum += delta
		if _debug_accum > 0.5:
			_debug_accum = 0.0
			print("[WaypointPathCurve] s_drone=%.2f s_target=%.2f chase=%.2f len=%.2f" % [_s_drone, _s_target, chase_s, path_length])


# ── public API ────────────────────────────────────────────────────

## Start (or restart) the path from its beginning.
func start() -> void:
	_ensure_virtual_target()
	_refresh_waypoints()
	if _waypoints.size() < 2 or _curve == null:
		push_warning("WaypointPathCurve '%s' needs at least 2 waypoint children." % name)
		return
	_bound_drones = _resolve_drones()
	if _bound_drones.is_empty():
		push_warning("WaypointPathCurve '%s' could not find a drone to bind to." % name)
		return

	_s_drone = 0.0
	_s_target = 0.0
	_direction = 1
	_last_announced_index = -1
	_completed_emitted = false
	_active = true

	_virtual_target.global_position = _curve.sample_baked(0.0)
	_push_waypoint(_virtual_target)
	emit_signal("path_started")


## Stop following, clear the drone's waypoint, and halt.
func cancel() -> void:
	if not _active:
		return
	_active = false
	for drone in _bound_drones:
		if is_instance_valid(drone) and drone.has_method("clear_waypoint"):
			drone.clear_waypoint()
	emit_signal("path_cancelled")


## Re-scan children and rebuild the curve.  Call after adding or
## removing waypoint nodes at runtime.
func refresh() -> void:
	_refresh_waypoints()


# ── internals ─────────────────────────────────────────────────────

func _ensure_virtual_target() -> void:
	if _virtual_target != null and is_instance_valid(_virtual_target):
		return
	var existing := get_node_or_null("VirtualTarget")
	if existing is Node3D:
		_virtual_target = existing
		return
	_virtual_target = Node3D.new()
	_virtual_target.name = "VirtualTarget"
	_virtual_target.top_level = true
	add_child(_virtual_target)


func _refresh_waypoints() -> void:
	_waypoints.clear()
	for child in get_children():
		if child is Node3D and child != _virtual_target:
			_waypoints.append(child)
	_rebuild_curve()


## Builds a Curve3D through every waypoint with auto-computed handles.
##
## For each interior point the in/out handles are set to
## ±tension × (neighbour_next - neighbour_prev) / 2 — this is the
## Catmull-Rom construction at `tension = 0.5`, producing a C¹ curve
## that passes exactly through every control point.  That "passes
## through" property is why we use it rather than bezier (brief §4.4).
func _rebuild_curve() -> void:
	_curve = null
	_waypoint_arc_lengths = PackedFloat32Array()
	_closed = loop_mode == LoopMode.LOOP and _waypoints.size() >= 2
	var n := _waypoints.size()
	if n < 2:
		return

	var curve := Curve3D.new()
	# Baked resolution — metres per baked segment.  Lower = smoother
	# sampling but more memory.  Default (0.2) is fine for drones.
	curve.bake_interval = 0.2

	# Number of control points on the curve.  Closed paths repeat
	# the first point at the end to close the loop smoothly.
	var point_count: int = n + (1 if _closed else 0)

	for i in range(point_count):
		var idx: int = i % n
		var pos: Vector3 = _waypoints[idx].global_position

		# Indices of the previous and next points used for handle
		# computation.  In closed mode we wrap; in open mode we
		# clamp to the endpoints (which makes end handles zero on
		# one side, producing natural end tangents).
		var prev_pos: Vector3
		var next_pos: Vector3
		if _closed:
			prev_pos = _waypoints[(idx - 1 + n) % n].global_position
			next_pos = _waypoints[(idx + 1) % n].global_position
		else:
			prev_pos = _waypoints[max(idx - 1, 0)].global_position
			next_pos = _waypoints[min(idx + 1, n - 1)].global_position

		# Catmull-Rom tangent for this point.
		var tangent: Vector3 = (next_pos - prev_pos) * 0.5 * curve_tension
		# Handles are relative to the point; `in` points backward,
		# `out` points forward.
		var handle_in: Vector3 = -tangent
		var handle_out: Vector3 = tangent

		# Open-path endpoints: zero the outward-facing handle so the
		# curve approaches/leaves the endpoint straight (rather than
		# looping away).  Without this, the single-sided tangent
		# computed above pulls the curve off the control point.
		if not _closed:
			if idx == 0:
				handle_in = Vector3.ZERO
			if idx == n - 1:
				handle_out = Vector3.ZERO

		curve.add_point(pos, handle_in, handle_out)

	_curve = curve

	# Record each authored waypoint's arc-length so we can fire
	# `waypoint_reached` at the right progress values.
	_waypoint_arc_lengths.append(0.0)
	for i in range(1, n):
		# `get_closest_offset` returns the baked arc-length of the
		# point on the curve closest to the given world position.
		# Because our control points *are* on the curve, this is
		# exactly the arc-length at that waypoint.
		_waypoint_arc_lengths.append(_curve.get_closest_offset(_waypoints[i].global_position))


## Project `world_pos` onto the curve and return its arc-length.
## Uses Curve3D.get_closest_offset (a global search, robust to
## self-intersection because we only use it for reporting — the
## chase-forward invariant is maintained separately via `chase_s`).
func _project_to_arc_length(world_pos: Vector3, hint_s: float, path_length: float) -> float:
	if _curve == null:
		return 0.0
	var s: float = _curve.get_closest_offset(world_pos)
	# `get_closest_offset` is global — on a self-crossing closed
	# path it can snap to "the other side" of the loop.  Keep the
	# chosen offset within a window of the previous one to enforce
	# forward progress, unless we've just wrapped.
	if _closed:
		# Compute the forward distance from hint to s (positive mod
		# path_length); if it's enormous assume we snapped the wrong
		# way and keep the hint.
		var forward: float = fposmod(s - hint_s, path_length)
		if forward > path_length * 0.5:
			return hint_s
		return s
	# Open path: monotonic forward.
	return max(s, hint_s - 0.5)


## Emit `waypoint_reached` as the chase point crosses each authored
## waypoint's baked arc-length.
func _handle_progress(chase_s: float) -> void:
	var next_index := _last_announced_index + 1
	while next_index < _waypoints.size():
		var wp_s: float = _waypoint_arc_lengths[next_index]
		if chase_s + 0.001 >= wp_s:
			_last_announced_index = next_index
			emit_signal("waypoint_reached", next_index, _waypoints[next_index])
			next_index += 1
		else:
			break


## Returns true if _physics_process should bail because the end-of-
## path branch has already taken over.
func _handle_end_of_path(drone: Node3D, chase_s: float, path_length: float) -> bool:
	if chase_s < path_length:
		return false

	match loop_mode:
		LoopMode.NONE:
			var final_wp: Node3D = _waypoints[_waypoints.size() - 1]
			_push_waypoint(final_wp)
			if not _completed_emitted:
				var dist := drone.global_position.distance_to(final_wp.global_position)
				if dist <= arrival_handoff_distance:
					_completed_emitted = true
					_active = false
					emit_signal("path_completed")
			return true
		LoopMode.LOOP:
			# Wrapping handled in-line in `_physics_process`.
			return false
		LoopMode.PING_PONG:
			_waypoints.reverse()
			_rebuild_curve()
			_s_drone = 0.0
			_s_target = 0.0
			_last_announced_index = -1
			_direction = -_direction
			emit_signal("path_looped")
			return false
	return false


func _push_waypoint(wp: Node3D) -> void:
	for drone in _bound_drones:
		if is_instance_valid(drone) and drone.has_method("set_waypoint"):
			drone.set_waypoint(wp)


func _resolve_drones() -> Array[Node]:
	var out: Array[Node] = []
	if drone_path != NodePath(""):
		var n := get_node_or_null(drone_path)
		if n != null:
			out.append(n)
		else:
			push_warning("WaypointPathCurve: drone_path '%s' did not resolve." % drone_path)
		return out
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
		if child is Node3D and child.name != "VirtualTarget":
			count += 1
	if count < 2:
		warnings.append("WaypointPathCurve needs at least 2 Node3D children to form a path.")
	return warnings
