extends Node3D
## Demo world for "pinned travel": the drone stays at (0,0,0) forever,
## while a finite grid of cubes is wrapped toroidally around it so that the
## player appears to fly through an infinite lattice.
##
## Core idea:
##   - The drone is a real RigidBody3D with its own flight controller. It
##     integrates normally, accumulating linear/angular velocity from thrust
##     and gravity just as it would in any other scene.
##   - Each physics tick, AFTER the physics step, we read the drone's drift
##     off origin, fold it into `virtual_position`, and snap the body back
##     to (0,0,0) via PhysicsServer3D.body_set_state(BODY_STATE_TRANSFORM).
##     Because a pure translation preserves linear_velocity and
##     angular_velocity exactly, the integrator never sees a disturbance.
##   - Every cube's displayed position is
##         base_grid_offset - virtual_position
##     wrapped per-axis into [-half_span, +half_span) via fposmod, so a
##     finite pool of cubes appears as an infinite lattice.
##
## All cubes share one MultiMeshInstance3D, so hundreds of cubes cost one
## draw call.

## Cubes per axis. Total cube count = grid_extent ** 3.
## 5 -> 125 cubes, 7 -> 343, 10 -> 1000.
@export var grid_extent: int = 7

## World-space distance between adjacent cubes in the lattice.
@export var grid_spacing: float = 80.0

## Optional custom mesh for each cube. Falls back to a unit BoxMesh.
@export var cube_mesh: Mesh

## The RigidBody3D drone to pin at origin. Assign in the inspector; typically
## an instance of ThrusterDroneScene.tscn placed as a child of this node.
@export var drone: RigidBody3D

## MultiMeshInstance3D that will host the cube lattice. Its MultiMesh is
## (re)created at runtime sized to grid_extent^3.
@export var multimesh_instance: MultiMeshInstance3D

## Optional label for a live HUD readout.
@export var hud_label: Label

## Accumulated virtual world position of the drone. The drone's RigidBody3D
## origin stays pinned at (0,0,0); this value is where it "would be" in the
## unpinned universe.
var virtual_position: Vector3 = Vector3.ZERO

var _instance_count: int = 0


func _ready() -> void:
	_setup_multimesh()
	_ensure_drone_at_origin()


func _setup_multimesh() -> void:
	assert(multimesh_instance != null, "Assign multimesh_instance in the inspector.")
	_instance_count = grid_extent * grid_extent * grid_extent
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.use_colors = false
	mm.instance_count = _instance_count
	mm.mesh = cube_mesh if cube_mesh != null else BoxMesh.new()
	multimesh_instance.multimesh = mm


func _ensure_drone_at_origin() -> void:
	# Start the drone exactly at the pinned position so the first physics
	# tick doesn't snap a large displacement. Any authored offset (e.g. a
	# spawn height) is folded into virtual_position as the starting point.
	if drone == null:
		return
	var xf := drone.global_transform
	virtual_position = xf.origin
	xf.origin = Vector3.ZERO
	PhysicsServer3D.body_set_state(
		drone.get_rid(),
		PhysicsServer3D.BODY_STATE_TRANSFORM,
		xf,
	)
	drone.reset_physics_interpolation()


func _physics_process(_dt: float) -> void:
	_pin_drone()
	_update_cubes()
	_update_hud()


## Snap the drone back to origin each physics tick, folding its drift into
## virtual_position. Rotation and velocities are preserved untouched.
func _pin_drone() -> void:
	if drone == null:
		return
	var xf := drone.global_transform
	var drift := xf.origin
	if drift == Vector3.ZERO:
		return
	virtual_position += drift
	xf.origin = Vector3.ZERO
	# Authoritative pose write: tells the physics server "this IS the body's
	# pose, do not treat the jump as motion." linear_velocity and
	# angular_velocity are deliberately left alone; pure translation is
	# velocity-invariant so they are already correct in the new frame.
	PhysicsServer3D.body_set_state(
		drone.get_rid(),
		PhysicsServer3D.BODY_STATE_TRANSFORM,
		xf,
	)
	# Kill visual interpolation across the teleport so the mesh does not
	# tween across the shift.
	drone.reset_physics_interpolation()


func _update_cubes() -> void:
	# Toroidal wrap: each cube is placed at (base - virtual) mod span,
	# centred around the drone. fposmod always returns a non-negative
	# result, so shifting by +half before and -half after recentres the
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
	if hud_label == null or drone == null:
		return
	var v := drone.linear_velocity
	hud_label.text = "virtual_position: (%.1f, %.1f, %.1f)\nspeed: %.1f m/s" % [
		virtual_position.x, virtual_position.y, virtual_position.z,
		v.length(),
	]
