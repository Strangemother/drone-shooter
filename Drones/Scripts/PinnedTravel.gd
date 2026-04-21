extends Node3D
## Demo world for "pinned travel": the drone stays at (0,0,0) forever,
## while a finite grid of cubes is wrapped toroidally around it so that the
## player appears to fly through an infinite lattice.
##
## Core idea:
##   - The drone never translates. It only rotates.
##   - A single `virtual_position` accumulates what the drone *would* have
##     moved if it were free. This is the authoritative "where are we in the
##     universe" value.
##   - Each frame, every cube's displayed position is
##         base_grid_offset - virtual_position
##     wrapped per-axis into [-half_span, +half_span) using fposmod.
##     The wrap makes a cube that drifts off the back of the field reappear
##     at the front, at the correct lattice spot, with zero allocation.
##
## All cubes share one MultiMeshInstance3D, so 100+ cubes cost one draw call.

## Cubes per axis. Total cube count = grid_extent ** 3.
## 5 -> 125 cubes, 4 -> 64, 10 -> 1000. Defaults near the requested ~100.
@export var grid_extent: int = 5

## World-space distance between adjacent cubes in the lattice.
@export var grid_spacing: float = 20.0

## Optional custom mesh for each cube. Falls back to a unit BoxMesh.
@export var cube_mesh: Mesh

## Max translation speed along drone-local axes (m/s).
@export var move_speed: float = 40.0

## Rotation speed for yaw / pitch / roll input (rad/s).
@export var rotate_speed: float = 1.5

## Assign these in the editor (or rely on the default NodePaths below).
@export var multimesh_instance: MultiMeshInstance3D
@export var drone: Node3D
@export var hud_label: Label

## Accumulated virtual world position of the drone. The drone itself stays
## pinned at origin; this value is what the rest of the universe is offset by.
var virtual_position: Vector3 = Vector3.ZERO

var _instance_count: int = 0


func _ready() -> void:
	_setup_multimesh()


func _setup_multimesh() -> void:
	assert(multimesh_instance != null, "Assign multimesh_instance in the inspector.")
	_instance_count = grid_extent * grid_extent * grid_extent
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.use_colors = false
	mm.instance_count = _instance_count
	mm.mesh = cube_mesh if cube_mesh != null else BoxMesh.new()
	multimesh_instance.multimesh = mm


func _physics_process(dt: float) -> void:
	_handle_input(dt)
	_update_cubes()
	_update_hud()


func _handle_input(dt: float) -> void:
	# --- Rotation: keyboard arrows for yaw/pitch, Q/E for roll. ---
	var yaw := 0.0
	var pitch := 0.0
	var roll := 0.0
	if Input.is_key_pressed(KEY_LEFT):  yaw   += 1.0
	if Input.is_key_pressed(KEY_RIGHT): yaw   -= 1.0
	if Input.is_key_pressed(KEY_UP):    pitch += 1.0
	if Input.is_key_pressed(KEY_DOWN):  pitch -= 1.0
	if Input.is_key_pressed(KEY_Q):     roll  += 1.0
	if Input.is_key_pressed(KEY_E):     roll  -= 1.0

	# Yaw around world-up for stable flight feel; pitch/roll in local frame.
	drone.rotate_y(yaw * rotate_speed * dt)
	drone.rotate_object_local(Vector3.RIGHT,   pitch * rotate_speed * dt)
	drone.rotate_object_local(Vector3.FORWARD, roll  * rotate_speed * dt)

	# --- Translation intent: WASD + Space/Ctrl in drone-local axes. ---
	var local_input := Vector3.ZERO
	if Input.is_key_pressed(KEY_W):     local_input.z -= 1.0
	if Input.is_key_pressed(KEY_S):     local_input.z += 1.0
	if Input.is_key_pressed(KEY_A):     local_input.x -= 1.0
	if Input.is_key_pressed(KEY_D):     local_input.x += 1.0
	if Input.is_key_pressed(KEY_SPACE): local_input.y += 1.0
	if Input.is_key_pressed(KEY_SHIFT): local_input.y -= 1.0

	if local_input != Vector3.ZERO:
		local_input = local_input.normalized()

	# Transform local intent into world-space and accumulate.
	# The drone DOES NOT move -- only virtual_position does.
	var world_motion := drone.transform.basis * local_input
	virtual_position += world_motion * move_speed * dt


func _update_cubes() -> void:
	# Toroidal wrap: each cube is placed at (base - virtual) mod span,
	# centred around the drone. fposmod always returns a non-negative
	# result so shifting by +half before and -half after recentres the
	# wrap window to [-half, +half).
	var mm := multimesh_instance.multimesh
	var span := grid_spacing * float(grid_extent)
	var half := span * 0.5
	var i := 0
	for x in grid_extent:
		for y in grid_extent:
			for z in grid_extent:
				var base := Vector3(
					float(x) * grid_spacing,
					float(y) * grid_spacing,
					float(z) * grid_spacing,
				)
				var rel := base - virtual_position
				rel.x = fposmod(rel.x + half, span) - half
				rel.y = fposmod(rel.y + half, span) - half
				rel.z = fposmod(rel.z + half, span) - half
				mm.set_instance_transform(i, Transform3D(Basis.IDENTITY, rel))
				i += 1


func _update_hud() -> void:
	if hud_label == null:
		return
	hud_label.text = "virtual_position: (%.1f, %.1f, %.1f)\nspeed: %.1f m/s" % [
		virtual_position.x, virtual_position.y, virtual_position.z,
		(drone.transform.basis * _last_motion_sample()).length(),
	]


func _last_motion_sample() -> Vector3:
	# Approximate current commanded speed for HUD display only.
	var v := Vector3.ZERO
	if Input.is_key_pressed(KEY_W):     v.z -= 1.0
	if Input.is_key_pressed(KEY_S):     v.z += 1.0
	if Input.is_key_pressed(KEY_A):     v.x -= 1.0
	if Input.is_key_pressed(KEY_D):     v.x += 1.0
	if Input.is_key_pressed(KEY_SPACE): v.y += 1.0
	if Input.is_key_pressed(KEY_SHIFT): v.y -= 1.0
	if v != Vector3.ZERO:
		v = v.normalized() * move_speed
	return v
