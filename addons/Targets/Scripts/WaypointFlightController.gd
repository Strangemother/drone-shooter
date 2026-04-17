## Waypoint flight controller — flies the drone to a target Node3D position.
##
## Extends StabilisedFlightController: inherits PID attitude stabilisation
## and altitude hold, and adds XZ position tracking.  The position error
## (how far the drone is from the waypoint) is converted into target tilt
## angles that the existing pitch/roll PID loops fly to.
##
## Usage:
##   • Set waypoint_target in the inspector to a Node3D in the scene, OR
##   • Call set_waypoint(node) at runtime from any script.
##   • From anywhere: get_tree().call_group("drones", "set_waypoint", node)
##
## When no waypoint is set the controller behaves identically to the
## StabilisedFlightController (hover in place, respond to player input).
extends StabilisedFlightController

class_name WaypointFlightController


# ── waypoint target ───────────────────────────────────────────────

## Optional: drag a Node3D from the scene tree here in the inspector.
@export var waypoint_target_path: NodePath = NodePath("")

## The live waypoint node.  Set via inspector path or set_waypoint().
var waypoint_target: Node3D = null


# ── position PID gains (XZ plane) ────────────────────────────────

## Proportional: how aggressively the drone tilts toward the waypoint.
@export_range(0.0, 5.0, 0.001) var pos_p: float = 0.15

## Integral: corrects persistent positional drift (e.g. wind).
@export_range(0.0, 2.0, 0.001) var pos_i: float = 0.01

## Derivative: damps approach speed to prevent overshoot.
@export_range(0.0, 5.0, 0.001) var pos_d: float = 0.25


# ── limits ────────────────────────────────────────────────────────

## Maximum tilt angle the waypoint tracking can command (radians).
## Caps how fast the drone flies toward the waypoint.  ~0.35 rad ≈ 20°.
@export_range(0.0, 1.57, 0.01) var max_waypoint_tilt: float = 0.35

## Distance (metres) at which the drone is considered "arrived".
## Below this the position PID stops contributing.
@export_range(0.0, 5.0, 0.01) var arrival_radius: float = 0.3


# ── internal position PID state ──────────────────────────────────

var _pos_x_integral: float = 0.0
var _pos_x_prev_error: float = 0.0

var _pos_z_integral: float = 0.0
var _pos_z_prev_error: float = 0.0


func _ready() -> void:
	if waypoint_target_path != NodePath(""):
		var node := get_node_or_null(waypoint_target_path)
		if node is Node3D:
			waypoint_target = node


func set_waypoint(target: Node3D) -> void:
	waypoint_target = target
	# Reset position PID so old integral state doesn't cause a lurch.
	_pos_x_integral = 0.0
	_pos_x_prev_error = 0.0
	_pos_z_integral = 0.0
	_pos_z_prev_error = 0.0


func clear_waypoint() -> void:
	waypoint_target = null
	_pos_x_integral = 0.0
	_pos_x_prev_error = 0.0
	_pos_z_integral = 0.0
	_pos_z_prev_error = 0.0


func reset_state() -> void:
	super.reset_state()
	_pos_x_integral = 0.0
	_pos_x_prev_error = 0.0
	_pos_z_integral = 0.0
	_pos_z_prev_error = 0.0


func update_mix(body: RigidBody3D, thrusters: Array[Node]) -> void:
	# If no waypoint, fall back to normal stabilised hover + player input.
	if waypoint_target == null or not is_instance_valid(waypoint_target):
		super.update_mix(body, thrusters)
		return

	var total_max := get_total_max_force(thrusters)
	if total_max <= 0.0:
		return

	var dt := _get_delta()
	if dt <= 0.0:
		return

	# ── position error in world space ────────────────────────────
	var target_pos := waypoint_target.global_position
	var body_pos := body.global_position
	var error_world := target_pos - body_pos

	# Project onto the body's local XZ plane so tilt commands are
	# relative to the drone's current heading.
	var error_local := body.global_transform.basis.inverse() * error_world

	# ── if within arrival radius, just stabilise in place ────────
	var xz_distance := Vector2(error_local.x, error_local.z).length()
	var wp_pitch_target := 0.0
	var wp_roll_target := 0.0

	if xz_distance > arrival_radius:
		# Position PID on each horizontal axis → target tilt angle.
		# Forward (−Z in Godot) needs positive pitch to move there,
		# so we negate the Z error.
		var pitch_correction := _pid(
			0.0, error_local.z,  # error = how far ahead the target is
			pos_p, pos_i, pos_d, dt,
			_pos_z_integral, _pos_z_prev_error
		)
		_pos_z_integral = pitch_correction.y
		_pos_z_prev_error = pitch_correction.z
		# Negative because +Z error means target is behind → pitch forward (nose down).
		wp_pitch_target = clampf(-pitch_correction.x, -max_waypoint_tilt, max_waypoint_tilt)

		var roll_correction := _pid(
			0.0, -error_local.x,  # +X error means target is right → roll right
			pos_p, pos_i, pos_d, dt,
			_pos_x_integral, _pos_x_prev_error
		)
		_pos_x_integral = roll_correction.y
		_pos_x_prev_error = roll_correction.z
		wp_roll_target = clampf(-roll_correction.x, -max_waypoint_tilt, max_waypoint_tilt)

	# ── attitude: current tilt angles ────────────────────────────
	var local_up := body.global_transform.basis.y
	var current_pitch := atan2(-local_up.z, local_up.y)
	var current_roll  := atan2(local_up.x, local_up.y)

	# Layer player stick input on top of waypoint commands so the
	# player can nudge while tracking.
	var stick_pitch := get_axis_value(pitch_backward_action, pitch_forward_action) * max_tilt_angle
	var stick_roll  := get_axis_value(roll_left_action, roll_right_action) * max_tilt_angle

	var final_pitch_target := clampf(wp_pitch_target + stick_pitch, -max_tilt_angle, max_tilt_angle)
	var final_roll_target  := clampf(wp_roll_target + stick_roll, -max_tilt_angle, max_tilt_angle)

	# ── PID: pitch attitude ──────────────────────────────────────
	var pitch_correction := _pid(
		final_pitch_target, current_pitch,
		pitch_p, pitch_i, pitch_d, dt,
		_pitch_integral, _pitch_prev_error
	)
	_pitch_integral = pitch_correction.y
	_pitch_prev_error = pitch_correction.z
	var pitch_output: float = pitch_correction.x

	# ── PID: roll attitude ───────────────────────────────────────
	var roll_correction := _pid(
		final_roll_target, current_roll,
		roll_p, roll_i, roll_d, dt,
		_roll_integral, _roll_prev_error
	)
	_roll_integral = roll_correction.y
	_roll_prev_error = roll_correction.z
	var roll_output: float = roll_correction.x

	# ── collective: hover + altitude tracking to waypoint Y ──────
	var collective := _hover_throttle(body, total_max)
	collective += throttle_trim

	# Use the waypoint's Y as the altitude target.
	_target_altitude = target_pos.y

	# Layer player vertical input on top.
	var vert_input := get_axis_value(throttle_down_action, throttle_up_action)
	if absf(vert_input) > 0.01:
		_target_altitude += vert_input * collective_authority * dt * 10.0
		collective += vert_input * collective_authority

	var alt_correction := _pid(
		_target_altitude, body_pos.y,
		alt_p, alt_i, alt_d, dt,
		_alt_integral, _alt_prev_error
	)
	_alt_integral = alt_correction.y
	_alt_prev_error = alt_correction.z
	collective += alt_correction.x

	# ── mix per-thruster ─────────────────────────────────────────
	var max_arm_x := get_max_arm_length(body, thrusters, "x")
	var max_arm_z := get_max_arm_length(body, thrusters, "z")

	for thruster in thrusters:
		var offset := body.to_local(thruster.global_position)
		var pitch_mix := 0.0
		var roll_mix := 0.0

		if max_arm_z > 0.0:
			pitch_mix = (offset.z / max_arm_z) * pitch_output
		if max_arm_x > 0.0:
			roll_mix = (-offset.x / max_arm_x) * roll_output

		thruster.set_throttle(clampf(collective + pitch_mix + roll_mix, 0.0, 1.0))
