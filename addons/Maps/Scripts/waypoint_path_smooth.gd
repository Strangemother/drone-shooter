@tool
extends Node3D
class_name WaypointPathSmooth

## Multi-waypoint path follower (Stage 2 — pure-pursuit lookahead).
##
## Same integration contract as `WaypointPath`:
##   • No changes to DroneScript or any FlightController.
##   • Consumes the drone's public `set_waypoint(Node3D)` API.
##
## The difference is *what* Node3D we hand the drone.  Instead of
## swapping between authored waypoints at `swap_radius`, this node
## owns a single invisible child — `VirtualTarget` — whose position
## is advanced smoothly along the polyline formed by the path's
## waypoint children every physics tick.  The drone chases that
## virtual target, and because the target sweeps continuously around
## corners the drone's path becomes smooth instead of jinking at
## each control point.
##
## This is "pure pursuit" in robotics terms.  See
## `docs/multi-waypoint-design-brief.md` §4.2 and §5 stage 2.
##
## ── Usage ───────────────────────────────────────────────────────────
##   1. Add this script to a Node3D in the scene.
##   2. Parent 2+ Node3D children under it — authored waypoints,
##      traversed in scene-tree order.
##   3. Either leave `drone_path` empty (auto-binds every drone in the
##      "drones" group) or drag a specific drone into it.
##   4. Press play.  Tweak `cruise_speed` and `lookahead_distance`.
##
## ── Tuning ──────────────────────────────────────────────────────────
##   • `cruise_speed` should be ≤ the flight controller's
##     `max_approach_speed` (default 6.0).  If it's higher the virtual
##     target outruns the drone — lookahead balloons and the drone
##     lags further behind each second.  The built-in lag clamp
##     (§7.3 of the brief) caps this but you'll still see the drone
##     straining.
##   • `lookahead_distance` ≈ `cruise_speed * 0.3s` feels natural.
##     Too short → drone hugs polyline & still jinks.
##     Too long  → drone cuts corners visibly.
##
## At the end of a non-looping path the drone is given the *final*
## waypoint directly (not the virtual target) so the flight
## controller can engage its arrival logic and hover cleanly.

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

## How fast the virtual target advances along the polyline (m/s).
## Keep ≤ your flight controller's max approach speed.
@export_range(0.1, 50.0, 0.1) var cruise_speed: float = 6.0

## How far *ahead* of the drone's current arc-length the virtual
## target sits (metres).  Core smoothing knob.
@export_range(0.1, 50.0, 0.1) var lookahead_distance: float = 2.0

## Maximum metres the virtual target is allowed to lead the drone
## along the path.  Without this clamp, if `cruise_speed` exceeds the
## drone's physical maximum the target runs away and never comes
## back (design brief §7.3, §8.2).  Set to 0 to disable the clamp.
@export_range(0.0, 100.0, 0.1) var max_lead_distance: float = 8.0

## Distance to the *final* path point at which the controller is
## handed the waypoint directly so its arrival logic engages.
@export_range(0.1, 20.0, 0.1) var arrival_handoff_distance: float = 2.0

## End-of-path behaviour.
enum LoopMode { NONE, LOOP, PING_PONG }
@export var loop_mode: LoopMode = LoopMode.NONE

## If true, binds & starts automatically in `_ready`.
@export var auto_start: bool = true

## Print arc-length / lookahead a few times a second for diagnosis.
@export var debug_print: bool = false


# ── internal state ────────────────────────────────────────────────

var _waypoints: Array[Node3D] = []
var _segment_lengths: PackedFloat32Array = PackedFloat32Array()
var _cumulative_lengths: PackedFloat32Array = PackedFloat32Array()
var _path_length: float = 0.0

## True when `loop_mode == LOOP` and we have ≥2 waypoints.  A closed
## path has an extra synthetic segment from the last waypoint back
## to the first so the virtual target can actually sweep the join —
## without it, LOOP mode wedges at the end of the polyline because
## the drone has nowhere physical to fly to.
var _closed: bool = false

## Which authored waypoints we've already announced via
## `waypoint_reached`.  Index into `_waypoints`.
var _last_announced_index: int = -1

## Virtual target — a bare Node3D whose `global_position` we rewrite
## each physics tick.  This is the single node the drone chases.
var _virtual_target: Node3D = null

## Arc-length progress of the *drone* along the polyline (metres).
var _s_drone: float = 0.0

## Arc-length progress of the *virtual target* along the polyline.
## Always ≥ `_s_drone`; kept within `max_lead_distance` of it.
var _s_target: float = 0.0

## Direction of travel for PING_PONG mode: +1 forward, -1 reverse.
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
		# Defer so drones have joined the "drones" group before we bind.
		call_deferred("start")


func _physics_process(delta: float) -> void:
	if Engine.is_editor_hint():
		return
	if not _active or _waypoints.is_empty() or _bound_drones.is_empty():
		return
	if _path_length <= 0.0:
		return

	var drone := _first_valid_drone()
	if drone == null:
		return

	# Update the drone's arc-length by projecting its current world
	# position onto the polyline.  In closed (LOOP) mode this is a
	# value in [0, _path_length); in open mode it's monotonic-ish
	# along the polyline.
	_s_drone = _project_to_arc_length(drone.global_position, _s_drone)

	# Advance the virtual target's arc-length.
	_s_target += cruise_speed * delta
	if _closed:
		_s_target = fposmod(_s_target, _path_length)

	# Enforce the "virtual target can't outrun the drone by more
	# than `max_lead_distance`" clamp.  In closed mode the distance
	# between two arc-lengths is computed modulo the path length —
	# otherwise a just-wrapped target looks like it's miles behind
	# the drone and we'd stop advancing it.
	if max_lead_distance > 0.0:
		var lead: float = _s_target - _s_drone
		if _closed:
			lead = fposmod(lead, _path_length)
		if lead > max_lead_distance:
			_s_target = _s_drone + max_lead_distance
			if _closed:
				_s_target = fposmod(_s_target, _path_length)

	# Where the drone should actually chase: `lookahead_distance`
	# along the path from wherever the drone currently is.  That
	# makes the pursuit speed-adaptive — slow drone → short lead;
	# fast drone → same fixed lead.
	var chase_s: float = _s_drone + lookahead_distance

	if _closed:
		# Detect a lap: chase_s has wrapped past the end of the path.
		if chase_s >= _path_length:
			_last_announced_index = -1
			emit_signal("path_looped")
		chase_s = fposmod(chase_s, _path_length)
		_handle_progress(chase_s)
	else:
		_handle_progress(chase_s)
		if _handle_end_of_path(drone, chase_s):
			return
		chase_s = clamp(chase_s, 0.0, _path_length)

	_virtual_target.global_position = _sample_polyline(chase_s)

	if debug_print:
		_debug_accum += delta
		if _debug_accum > 0.5:
			_debug_accum = 0.0
			print("[WaypointPathSmooth] s_drone=%.2f s_target=%.2f chase=%.2f len=%.2f" % [_s_drone, _s_target, chase_s, _path_length])


# ── public API ────────────────────────────────────────────────────

## Start (or restart) the path from its beginning.
func start() -> void:
	_ensure_virtual_target()
	_refresh_waypoints()
	if _waypoints.size() < 2:
		push_warning("WaypointPathSmooth '%s' needs at least 2 waypoint children." % name)
		return
	_bound_drones = _resolve_drones()
	if _bound_drones.is_empty():
		push_warning("WaypointPathSmooth '%s' could not find a drone to bind to." % name)
		return

	_s_drone = 0.0
	_s_target = 0.0
	_direction = 1
	_last_announced_index = -1
	_completed_emitted = false
	_active = true

	# Seed the virtual target on the first waypoint so the drone has
	# somewhere sensible to chase on frame 1.
	_virtual_target.global_position = _waypoints[0].global_position
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


## Re-scan children and recompute arc lengths.  Call after adding or
## removing waypoint nodes at runtime.
func refresh() -> void:
	_refresh_waypoints()


# ── internals ─────────────────────────────────────────────────────

func _ensure_virtual_target() -> void:
	if _virtual_target != null and is_instance_valid(_virtual_target):
		return
	# Look for an existing child named "VirtualTarget" (e.g. across
	# scene reloads) before creating a new one.
	var existing := get_node_or_null("VirtualTarget")
	if existing is Node3D:
		_virtual_target = existing
		return
	_virtual_target = Node3D.new()
	_virtual_target.name = "VirtualTarget"
	# Top-level so our node's transform can't drag the target around.
	_virtual_target.top_level = true
	add_child(_virtual_target)


func _refresh_waypoints() -> void:
	_waypoints.clear()
	for child in get_children():
		if child is Node3D and child != _virtual_target:
			_waypoints.append(child)
	_recompute_arc_lengths()


func _recompute_arc_lengths() -> void:
	_segment_lengths = PackedFloat32Array()
	_cumulative_lengths = PackedFloat32Array()
	_path_length = 0.0
	_closed = loop_mode == LoopMode.LOOP and _waypoints.size() >= 2
	if _waypoints.size() < 2:
		return
	_cumulative_lengths.append(0.0)
	for i in range(_waypoints.size() - 1):
		var a := _waypoints[i].global_position
		var b := _waypoints[i + 1].global_position
		var seg := a.distance_to(b)
		_segment_lengths.append(seg)
		_path_length += seg
		_cumulative_lengths.append(_path_length)
	# Closing segment (last → first) so LOOP mode has a continuous
	# path for the virtual target to sweep around.  Indexed as an
	# extra segment after the authored ones; `_sample_polyline`
	# handles it specially.
	if _closed:
		var a := _waypoints[_waypoints.size() - 1].global_position
		var b := _waypoints[0].global_position
		var seg := a.distance_to(b)
		_segment_lengths.append(seg)
		_path_length += seg
		_cumulative_lengths.append(_path_length)


## Returns the world-space point at arc-length `s` along the polyline.
## In closed (LOOP) mode `s` is taken modulo `_path_length` so callers
## can pass monotonic progress without wrapping.
func _sample_polyline(s: float) -> Vector3:
	if _waypoints.is_empty():
		return global_position
	if _waypoints.size() == 1:
		return _waypoints[0].global_position
	if _closed:
		s = fposmod(s, _path_length)
	else:
		s = clamp(s, 0.0, _path_length)
	var n := _waypoints.size()
	# Locate the segment containing `s`.  When closed the final
	# segment (index n-1) is the synthetic last→first closer.
	for i in range(_segment_lengths.size()):
		var s1: float = _cumulative_lengths[i + 1]
		if s <= s1 or i == _segment_lengths.size() - 1:
			var s0: float = _cumulative_lengths[i]
			var seg_len: float = _segment_lengths[i]
			var t: float = 0.0 if seg_len <= 0.0 else (s - s0) / seg_len
			var a: Vector3 = _waypoints[i].global_position
			var b: Vector3 = _waypoints[(i + 1) % n].global_position if _closed else _waypoints[i + 1].global_position
			return a.lerp(b, t)
	return _waypoints[n - 1].global_position


## Project `world_pos` onto the polyline and return its arc-length.
## `hint_s` biases the search to the segment around the drone's last
## known progress — keeps self-intersecting paths from snapping the
## drone's progress backward (or forward) to a different loop.
func _project_to_arc_length(world_pos: Vector3, hint_s: float) -> float:
	if _segment_lengths.is_empty():
		return 0.0
	var n := _waypoints.size()
	var first: int
	var last: int
	if _closed:
		# Closed paths are short; just search every segment.  Avoids
		# the "drone near the closing segment can't be found" bug a
		# hint-window search would have right after a lap.
		first = 0
		last = _segment_lengths.size() - 1
	else:
		# Open paths: search a window of ±2 segments around the hint
		# so self-intersecting layouts don't snap progress backward.
		var hint_seg := _segment_index_for_arc_length(hint_s)
		first = max(0, hint_seg - 2)
		last = min(_segment_lengths.size() - 1, hint_seg + 2)

	var best_s: float = hint_s
	var best_d2: float = INF
	for i in range(first, last + 1):
		var a: Vector3 = _waypoints[i].global_position
		var b: Vector3 = _waypoints[(i + 1) % n].global_position if _closed else _waypoints[i + 1].global_position
		var seg: Vector3 = b - a
		var seg_len2: float = seg.length_squared()
		var t: float = 0.0
		if seg_len2 > 0.0:
			t = clamp((world_pos - a).dot(seg) / seg_len2, 0.0, 1.0)
		var closest: Vector3 = a + seg * t
		var d2: float = world_pos.distance_squared_to(closest)
		if d2 < best_d2:
			best_d2 = d2
			best_s = _cumulative_lengths[i] + t * _segment_lengths[i]
	return best_s


func _segment_index_for_arc_length(s: float) -> int:
	if _segment_lengths.is_empty():
		return 0
	s = clamp(s, 0.0, _path_length)
	for i in range(_segment_lengths.size()):
		if s <= _cumulative_lengths[i + 1]:
			return i
	return _segment_lengths.size() - 1


## Emit `waypoint_reached` as the chase point crosses each authored
## waypoint's arc-length.  Index 0 fires as soon as the chase point
## starts moving along the path.
func _handle_progress(chase_s: float) -> void:
	var next_index := _last_announced_index + 1
	while next_index < _waypoints.size():
		var wp_s: float = _cumulative_lengths[next_index]
		if chase_s + 0.001 >= wp_s:
			_last_announced_index = next_index
			emit_signal("waypoint_reached", next_index, _waypoints[next_index])
			next_index += 1
		else:
			break


## Returns true if _physics_process should stop this tick because
## the end-of-path branch has already handled the drone.
func _handle_end_of_path(drone: Node3D, chase_s: float) -> bool:
	if chase_s < _path_length:
		return false

	match loop_mode:
		LoopMode.NONE:
			# Hand the *real* final waypoint to the flight controller
			# so its arrival radius / brake logic engages cleanly.
			var final_wp: Node3D = _waypoints[_waypoints.size() - 1]
			_push_waypoint(final_wp)
			# Declare "completed" once the drone is close enough that
			# the arrival logic is actually driving the stop.
			if not _completed_emitted:
				var dist := drone.global_position.distance_to(final_wp.global_position)
				if dist <= arrival_handoff_distance:
					_completed_emitted = true
					_active = false
					emit_signal("path_completed")
			return true
		LoopMode.LOOP:
			# Closed-path LOOP is handled inside `_physics_process`
			# (arc-length wrapping).  This branch is only reached if
			# `_closed` was false — shouldn't happen, but be safe.
			return false
		LoopMode.PING_PONG:
			# Flip the waypoint order and restart progress.  Cheap
			# and correct — reverses the polyline in-place.
			_waypoints.reverse()
			_recompute_arc_lengths()
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
			push_warning("WaypointPathSmooth: drone_path '%s' did not resolve." % drone_path)
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
		warnings.append("WaypointPathSmooth needs at least 2 Node3D children to form a path.")
	return warnings
