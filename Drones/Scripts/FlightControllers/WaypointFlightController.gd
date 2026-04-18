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


# ── yaw-to-waypoint ──────────────────────────────────────────────

## If true, the waypoint controller actively yaws the drone so its nose
## points at the waypoint (horizontal plane only).  Without this the
## drone will happily fly tail-first or sideways to reach the target,
## which is kinematically valid but looks wrong.
##
## Note: the stabilised controller has no force-mixed yaw authority
## (all thrusters fire along body-UP, so differential thrust can't
## produce yaw torque).  To yaw we apply a direct torque to the body
## via `apply_torque`.  This is the one place we break the "no
## apply_torque" rule from the flight-controller guide — it's safe
## here because (a) the torque is small and damped by the body's
## `angular_damp`, and (b) no other control path touches the yaw axis,
## so there's nothing to fight with.
@export var yaw_to_waypoint: bool = true

## Proportional gain for the yaw-to-waypoint loop (N·m per radian of
## heading error).  Default tuned for ~0.2 kg·m² body inertia with
## `angular_damp ≈ 3`.  Lower this if the drone over-rotates.
@export_range(0.0, 10.0, 0.01) var yaw_p: float = 1.5

## Derivative gain on body-Y angular velocity — brakes rotation so
## the drone settles on heading without oscillating.
@export_range(0.0, 10.0, 0.01) var yaw_d: float = 0.4

## Heading error (radians) below which the pitch command is allowed
## to reach full strength.  Outside this, pitch is faded toward zero
## so the drone rotates first and only accelerates once it's roughly
## facing the waypoint.  ~0.7 rad ≈ 40°.
@export_range(0.0, 3.14, 0.01) var yaw_align_tolerance: float = 0.7


# ── debug ────────────────────────────────────────────────────────

## If true, prints body-frame error and commanded tilt ~2× a second.
## Use this to diagnose "drone flies away from waypoint" bugs — check
## that `error_body` points in the direction you expect, and that
## `wp_pitch_target` / `wp_roll_target` have the sign you'd expect
## given the drone's body-forward = -Z convention.
@export var debug_print: bool = false
var _debug_accum: float = 0.0


# ── internal position PID state ──────────────────────────────────

var _pos_x_integral: float = 0.0
var _pos_z_integral: float = 0.0


func _ready() -> void:
	if waypoint_target_path != NodePath(""):
		var node := get_node_or_null(waypoint_target_path)
		if node is Node3D:
			waypoint_target = node


func set_waypoint(target: Node3D) -> void:
	waypoint_target = target
	# Reset position PID so old integral state doesn't cause a lurch.
	_pos_x_integral = 0.0
	_pos_z_integral = 0.0


func clear_waypoint() -> void:
	waypoint_target = null
	_pos_x_integral = 0.0
	_pos_z_integral = 0.0


func reset_state() -> void:
	super.reset_state()
	_pos_x_integral = 0.0
	_pos_z_integral = 0.0


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

	# Body-frame linear velocity — used as the D-term rate, the same
	# way the attitude loop uses body angular velocity.  Finite
	# differencing of the position error produces catastrophic frame-
	# to-frame noise (the command can flip sign every tick on small
	# errors) AND provides no real damping of approach speed, so the
	# drone accelerates at a saturated tilt all the way to the
	# waypoint and smashes through it.  Using linear velocity fixes
	# both at once.
	var vel_body := basis_t * body.linear_velocity

	# ── if within arrival radius, just stabilise in place ────────
	var xz_distance := Vector2(error_world.x, error_world.z).length()
	var wp_pitch_target := 0.0
	var wp_roll_target := 0.0

	# Heading error: angle between body-forward (-Z in body = (0,0,-1))
	# and the horizontal direction to the waypoint, expressed in body
	# frame.  Computed from error_body.xz so it's yaw-only.
	#
	#   atan2(err_body.x, -err_body.z)
	#
	# returns 0 when the waypoint is directly ahead (err_body.z < 0,
	# err_body.x = 0), +π/2 when directly to the right, and ±π when
	# directly behind.  Positive = need to yaw right.
	var heading_error := 0.0
	if xz_distance > 0.01:
		heading_error = atan2(error_body.x, -error_body.z)

	# Pitch attenuation: if the drone isn't roughly facing the
	# waypoint, don't accelerate toward it — rotate first.  Scales
	# the pitch command smoothly from 1.0 (on-heading) to 0.0
	# (>yaw_align_tolerance off-heading).
	var align_factor := 1.0
	if yaw_to_waypoint and yaw_align_tolerance > 0.0:
		align_factor = clampf(
			1.0 - (absf(heading_error) / yaw_align_tolerance),
			0.0, 1.0
		)

	if xz_distance > arrival_radius:
		# PD on body-Z position → pitch command.
		#   Body-forward is -Z, so a waypoint ahead has err_body.z < 0
		#   and we want a positive pitch target (nose-down = fly
		#   forward in -Z).  → term:  −pos_p * err_body.z
		#   Damping:  as the drone gains −Z velocity it should back
		#   off pitch.  vel_body.z is negative when flying forward,
		#   so +pos_d * vel_body.z reduces the pitch command as
		#   forward speed builds.
		_pos_z_integral = clampf(
			_pos_z_integral + (-error_body.z) * dt,
			-integral_limit, integral_limit
		)
		var pitch_raw := -pos_p * error_body.z \
			+ pos_i * _pos_z_integral \
			+ pos_d * vel_body.z
		wp_pitch_target = clampf(pitch_raw, -max_waypoint_tilt, max_waypoint_tilt)

		# PD on body-X position → roll command.
		#   Waypoint to the right gives err_body.x > 0, and we want
		#   a positive roll target (right-wing-down).
		#   Damping: drone accelerating right has vel_body.x > 0,
		#   which should reduce the roll command.
		_pos_x_integral = clampf(
			_pos_x_integral + error_body.x * dt,
			-integral_limit, integral_limit
		)
		var roll_raw := pos_p * error_body.x \
			+ pos_i * _pos_x_integral \
			- pos_d * vel_body.x
		wp_roll_target = clampf(roll_raw, -max_waypoint_tilt, max_waypoint_tilt)

		# Fade pitch/roll to zero when we're not facing the waypoint
		# so the yaw loop can rotate us before we start flying.
		wp_pitch_target *= align_factor
		wp_roll_target *= align_factor
	else:
		# Inside arrival radius — bleed the position integrals so
		# they don't carry a stale bias into the next waypoint.
		_pos_x_integral = 0.0
		_pos_z_integral = 0.0

	# ── attitude: current tilt angles (body frame) ───────────────
	# Positive = nose-down / right-wing-down, matching target and mix.
	var world_up_body := basis_t * Vector3.UP
	var current_pitch := atan2(world_up_body.z, world_up_body.y)
	var current_roll  := atan2(-world_up_body.x, world_up_body.y)
	var omega_body := basis_t * body.angular_velocity

	# Layer player stick input on top of waypoint commands so the
	# player can nudge while tracking.
	var stick_pitch := get_axis_value(pitch_backward_action, pitch_forward_action) * max_tilt_angle
	var stick_roll  := get_axis_value(roll_left_action, roll_right_action) * max_tilt_angle
	var yaw_stick   := get_axis_value(yaw_left_action, yaw_right_action)

	var final_pitch_target := clampf(wp_pitch_target + stick_pitch, -max_tilt_angle, max_tilt_angle)
	var final_roll_target  := clampf(wp_roll_target + stick_roll, -max_tilt_angle, max_tilt_angle)

	# ── PID: pitch attitude (D on body angular velocity) ──────
	var pitch_correction := _pid_rate(
		final_pitch_target, current_pitch, -omega_body.x,
		pitch_p, pitch_i, pitch_d, dt,
		_pitch_integral
	)
	_pitch_integral = pitch_correction.y
	var pitch_output: float = clampf(pitch_correction.x, -attitude_authority, attitude_authority)

	# ── PID: roll attitude (D on body angular velocity) ───────
	var roll_correction := _pid_rate(
		final_roll_target, current_roll, -omega_body.z,
		roll_p, roll_i, roll_d, dt,
		_roll_integral
	)
	_roll_integral = roll_correction.y
	var roll_output: float = clampf(roll_correction.x, -attitude_authority, attitude_authority)

	# ── yaw: rotate to face the waypoint ─────────────────────────
	# The stabilised controller leaves yaw entirely to angular_damp
	# because thruster mixing can't produce yaw torque on this drone.
	# Waypoint following NEEDS yaw, however, or the drone flies
	# tail-first / sideways to reach targets behind or beside it.
	# We apply a small, damped torque directly to the body — see
	# the big block comment at `yaw_to_waypoint` above for why this
	# is the one safe place to use apply_torque on this stack.
	#
	# Player yaw stick is layered as an additive rate command
	# (multiplies heading-P error), so the player can still steer.
	if yaw_to_waypoint and xz_distance > arrival_radius:
		var yaw_torque_local := (-yaw_p * heading_error) - (yaw_d * omega_body.y)
		# Convert body-local torque vector (about body +Y) into world
		# space for apply_torque.
		var yaw_torque_world := basis * Vector3(0.0, yaw_torque_local, 0.0)
		body.apply_torque(yaw_torque_world)

	# Allow the player to nudge yaw (rate command around body +Y).
	if absf(yaw_stick) > 0.01:
		var yaw_stick_torque := basis * Vector3(0.0, -yaw_stick * yaw_p, 0.0)
		body.apply_torque(yaw_stick_torque)

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

	# ── mix per-thruster (airmode: attitude > altitude) ──────────
	_apply_mix(body, thrusters, pitch_output, roll_output, collective)

	# ── optional debug ───────────────────────────────────────────
	if debug_print:
		_debug_accum += dt
		if _debug_accum >= 0.5:
			_debug_accum = 0.0
			print("[waypoint] err_body=%s vel_body=%s heading_err=%.2f align=%.2f wp_pitch=%.3f wp_roll=%.3f cur_pitch=%.3f cur_roll=%.3f dist=%.2f" % [
				error_body, vel_body,
				heading_error, align_factor,
				wp_pitch_target, wp_roll_target,
				current_pitch, current_roll,
				xz_distance,
			])
