## Waypoint flight controller — flies the drone to a target Node3D position.
##
## Extends FlightStabilisedController: inherits PID attitude stabilisation
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
## FlightStabilisedController (hover in place, respond to player input).
extends FlightStabilisedController

class_name FlightWaypointController


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
@export_range(0.0, 5.0, 0.001) var pos_d: float = 0.325


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

	# ── position error, rotated into body frame ──────────────────
	# The attitude loop works in body frame, so the position error
	# must be too — otherwise pitch/roll commands become wrong as
	# soon as the drone yaws away from world-forward.
	var target_pos := waypoint_target.global_position
	var body_pos := body.global_position
	var error_world := target_pos - body_pos
	var basis := body.global_transform.basis
	var basis_t := basis.transposed()
	var error_body := basis_t * error_world

	# ── if within arrival radius, just stabilise in place ────────
	var xz_distance := Vector2(error_world.x, error_world.z).length()
	var wp_pitch_target := 0.0
	var wp_roll_target := 0.0

	if xz_distance > arrival_radius:
		# Position PID on each body axis → target tilt angle.
		# Body-forward in Godot is -Z, so a target ahead has
		# error_body.z < 0.  Nose-down (positive pitch target)
		# accelerates the drone forward (-Z), so we want
		# wp_pitch_target ∝ -error_body.z: feed error_body.z into
		# _pid(target=0, current=error_body.z) which returns an
		# output ∝ -error_body.z.
		var pitch_correction := _pid(
			0.0, error_body.z,
			pos_p, pos_i, pos_d, dt,
			_pos_z_integral, _pos_z_prev_error
		)
		_pos_z_integral = pitch_correction.y
		_pos_z_prev_error = pitch_correction.z
		wp_pitch_target = clampf(pitch_correction.x, -max_waypoint_tilt, max_waypoint_tilt)

		# Positive body-X error (target to the right) needs positive
		# roll (tilt right).  _pid(0, x) output ∝ −x, so negate.
		var roll_correction := _pid(
			0.0, error_body.x,
			pos_p, pos_i, pos_d, dt,
			_pos_x_integral, _pos_x_prev_error
		)
		_pos_x_integral = roll_correction.y
		_pos_x_prev_error = roll_correction.z
		wp_roll_target = clampf(-roll_correction.x, -max_waypoint_tilt, max_waypoint_tilt)

	# ── attitude: current tilt angles (body frame) ───────────────
	# Positive = nose-down / right-wing-down, matching target and mix.
	var world_up_body := basis_t * Vector3.UP
	var current_pitch := atan2(world_up_body.z, world_up_body.y)
	var current_roll  := atan2(world_up_body.x, world_up_body.y)
	var omega_body := basis_t * body.angular_velocity

	# Layer player stick input on top of waypoint commands so the
	# player can nudge while tracking.
	var stick_pitch := get_axis_value(pitch_backward_action, pitch_forward_action) * max_tilt_angle
	var stick_roll  := get_axis_value(roll_left_action, roll_right_action) * max_tilt_angle
	var yaw_stick   := get_axis_value(yaw_left_action, yaw_right_action)

	var final_pitch_target := clampf(wp_pitch_target + stick_pitch, -max_tilt_angle, max_tilt_angle)
	var final_roll_target  := clampf(wp_roll_target + stick_roll, -max_tilt_angle, max_tilt_angle)

	# ── PID: pitch attitude (derivative-on-measurement) ───────
	var pitch_correction := _pid_dom(
		final_pitch_target, current_pitch,
		pitch_p, pitch_i, pitch_d, dt,
		_pitch_integral, _pitch_prev_angle
	)
	_pitch_integral = pitch_correction.y
	_pitch_prev_angle = pitch_correction.z
	var pitch_output: float = clampf(pitch_correction.x, -attitude_authority, attitude_authority)

	# ── PID: roll attitude (derivative-on-measurement) ────────
	var roll_correction := _pid_dom(
		final_roll_target, current_roll,
		roll_p, roll_i, roll_d, dt,
		_roll_integral, _roll_prev_angle
	)
	_roll_integral = roll_correction.y
	_roll_prev_angle = roll_correction.z
	var roll_output: float = clampf(roll_correction.x, -attitude_authority, attitude_authority)

	# ── no yaw torque — let RigidBody3D.angular_damp handle residual spin ─
	var _ignore := yaw_stick

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
