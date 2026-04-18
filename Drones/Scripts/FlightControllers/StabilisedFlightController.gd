## PID-stabilised "Angle" flight controller.
##
## Like FlightAngleController but with a feedback loop: the drone measures
## its actual pitch and roll every physics tick and drives them back toward
## level (or toward a player-commanded tilt angle).  It also damps vertical
## velocity so the drone holds a steady altitude rather than drifting.
##
## With no player input the drone will sit perfectly still in the air —
## no wobble, no drift.  Player stick input sets a *target tilt angle*
## rather than a raw throttle differential, and the PID loop flies to that
## angle.  This is the "angle mode" used by Betaflight / real quads.
extends FlightController

class_name FlightStabilisedController


# ── hover & collective ────────────────────────────────────────────

## Subtracted from project gravity when computing hover thrust.
@export_range(0.0, 40.0, 0.01) var anti_gravity: float = 0.0

## Fixed trim added to hover baseline.
@export_range(-1.0, 1.0, 0.001) var throttle_trim: float = 0.0

## How far player throttle input can push above/below hover.
@export_range(0.0, 1.0, 0.001) var collective_authority: float = 0.25


# ── PID gains for pitch axis ─────────────────────────────────────

## Proportional: how hard the correction reacts to current tilt error.
@export_range(0.0, 5.0, 0.001) var pitch_p: float = 0.6

## Integral: accumulates small persistent errors to eliminate steady-state
## offset (e.g. centre-of-mass asymmetry).  Keep low to avoid wind-up.
@export_range(0.0, 2.0, 0.001) var pitch_i: float = 0.05

## Derivative: damps oscillation by reacting to the rate of change of the
## error.  Higher = calmer settling, but too high = sluggish.
@export_range(0.0, 5.0, 0.001) var pitch_d: float = 0.3


# ── PID gains for roll axis ──────────────────────────────────────

@export_range(0.0, 5.0, 0.001) var roll_p: float = 0.6
@export_range(0.0, 2.0, 0.001) var roll_i: float = 0.05
@export_range(0.0, 5.0, 0.001) var roll_d: float = 0.3


# ── PID gains for vertical (altitude hold) ───────────────────────

@export_range(0.0, 5.0, 0.001) var alt_p: float = 0.4
@export_range(0.0, 2.0, 0.001) var alt_i: float = 0.02
@export_range(0.0, 5.0, 0.001) var alt_d: float = 0.5


# ── limits ────────────────────────────────────────────────────────

## Maximum tilt the player can command (radians).  ~0.52 rad ≈ 30°.
@export_range(0.0, 1.57, 0.01) var max_tilt_angle: float = 0.52

## Integral wind-up clamp — prevents the I term from growing without bound.
@export_range(0.0, 1.0, 0.01) var integral_limit: float = 0.3


# ── rate damping (inner loop) ─────────────────────────────────────
# A cascaded rate loop runs on top of the angle loop: angle error
# commands a desired angular rate, rate error commands a torque.
# This damps all three body axes and is what makes yaw recoverable
# even when the thruster geometry can't produce yaw torque via mixing.

## Converts angle error into a target angular rate (rad/s per rad).
## Higher = snappier angle tracking but risks overshoot.
@export_range(0.0, 20.0, 0.1) var angle_to_rate: float = 6.0

## Max body rate the angle loop may command (rad/s).
@export_range(0.0, 20.0, 0.1) var max_body_rate: float = 6.0

## Rate loop proportional gain (torque per rad/s of rate error).
## This is what produces the actual damping against spin.
@export_range(0.0, 50.0, 0.1) var rate_p: float = 8.0

## Rate loop derivative (damps rate-of-change of rate error).
@export_range(0.0, 10.0, 0.01) var rate_d: float = 0.5

## Player yaw input command range (rad/s).
@export_range(0.0, 10.0, 0.1) var max_yaw_rate: float = 3.0


# ── internal PID state ────────────────────────────────────────────

var _pitch_integral: float = 0.0
var _pitch_prev_error: float = 0.0

var _roll_integral: float = 0.0
var _roll_prev_error: float = 0.0

var _alt_integral: float = 0.0
var _alt_prev_error: float = 0.0

## Previous body-frame angular velocity — for rate-loop derivative.
var _prev_rate_err: Vector3 = Vector3.ZERO

## Captured on first update — the altitude the drone should hold when
## the player gives no vertical input.
var _target_altitude: float = NAN


func reset_state() -> void:
	_pitch_integral = 0.0
	_pitch_prev_error = 0.0
	_roll_integral = 0.0
	_roll_prev_error = 0.0
	_alt_integral = 0.0
	_alt_prev_error = 0.0
	_prev_rate_err = Vector3.ZERO
	_target_altitude = NAN


func update_mix(body: RigidBody3D, thrusters: Array[Node]) -> void:
	var total_max := get_total_max_force(thrusters)
	if total_max <= 0.0:
		return

	var dt := _get_delta()
	if dt <= 0.0:
		return

	# ── read current state from the rigid body ───────────────────
	var basis := body.global_transform.basis
	var basis_t := basis.transposed()  # orthonormal inverse — world→body

	# Measure tilt in the BODY frame so it stays consistent with the
	# thruster mix (which is also computed in body frame).  Using
	# world-frame tilt here causes pitch/roll to couple through yaw
	# and produces a runaway spin as soon as the drone isn't facing
	# world-forward.
	var world_up_body := basis_t * Vector3.UP
	var current_pitch := atan2(-world_up_body.z, world_up_body.y)  # rotation about body-X
	var current_roll  := atan2(world_up_body.x, world_up_body.y)   # rotation about body-Z

	# Body-frame angular velocity (rad/s about body X / Y / Z).
	var omega_body := basis_t * body.angular_velocity

	# ── target angles from player input ──────────────────────────
	var pitch_stick := get_axis_value(pitch_backward_action, pitch_forward_action)
	var roll_stick  := get_axis_value(roll_left_action, roll_right_action)
	var yaw_stick   := get_axis_value(yaw_left_action, yaw_right_action)

	var target_pitch := pitch_stick * max_tilt_angle
	var target_roll  := roll_stick * max_tilt_angle

	# ── PID: pitch (outer angle loop) ────────────────────────────
	var pitch_correction := _pid(
		target_pitch, current_pitch,
		pitch_p, pitch_i, pitch_d, dt,
		_pitch_integral, _pitch_prev_error
	)
	_pitch_integral = pitch_correction.y
	_pitch_prev_error = pitch_correction.z
	var pitch_output: float = pitch_correction.x

	# ── PID: roll (outer angle loop) ─────────────────────────────
	var roll_correction := _pid(
		target_roll, current_roll,
		roll_p, roll_i, roll_d, dt,
		_roll_integral, _roll_prev_error
	)
	_roll_integral = roll_correction.y
	_roll_prev_error = roll_correction.z
	var roll_output: float = roll_correction.x

	# ── inner rate loop: angle error → body-rate target → torque ─
	# The angle PID above drives thruster mixing; the rate loop
	# below applies a direct stabilising torque.  Together they form
	# a proper cascaded controller that stays stable regardless of
	# yaw orientation and also damps yaw spin (which thruster
	# mixing alone cannot).
	#
	# Target body rates (rad/s) about body X (pitch), Y (yaw), Z (roll).
	# Sign note: a positive "current_pitch" (nose-down tilt) is produced
	# by a negative rotation about body-X, so the pitch rate target is
	# negated here to agree with the body-X axis convention.
	var pitch_err := target_pitch - current_pitch
	var roll_err  := target_roll - current_roll
	var target_omega := Vector3(
		clampf(angle_to_rate * pitch_err, -max_body_rate, max_body_rate),
		yaw_stick * max_yaw_rate,
		clampf(angle_to_rate * roll_err, -max_body_rate, max_body_rate)
	)

	var rate_err := target_omega - omega_body
	var rate_deriv := (rate_err - _prev_rate_err) / dt
	_prev_rate_err = rate_err

	# Body-frame stabilising torque, then rotate to world for apply_torque.
	var torque_body := rate_p * rate_err + rate_d * rate_deriv
	body.apply_torque(basis * torque_body)

	# ── collective: hover baseline + altitude hold PID ───────────
	var collective := _hover_throttle(body, total_max)
	collective += throttle_trim

	# Player vertical input: if held, shift the target altitude.
	var vert_input := get_axis_value(throttle_down_action, throttle_up_action)
	if absf(vert_input) > 0.01:
		# While the player is actively commanding up/down, move the
		# target altitude and add direct collective.
		_target_altitude += vert_input * collective_authority * dt * 10.0
		collective += vert_input * collective_authority
	else:
		# No input — hold altitude with a PID loop on vertical position.
		if is_nan(_target_altitude):
			_target_altitude = body.global_position.y

		var alt_error_correction := _pid(
			_target_altitude, body.global_position.y,
			alt_p, alt_i, alt_d, dt,
			_alt_integral, _alt_prev_error
		)
		_alt_integral = alt_error_correction.y
		_alt_prev_error = alt_error_correction.z
		collective += alt_error_correction.x

	# ── mix per-thruster ─────────────────────────────────────────
	var max_arm_x := get_max_arm_length(body, thrusters, "x")
	var max_arm_z := get_max_arm_length(body, thrusters, "z")

	for thruster in thrusters:
		var offset := body.to_local(thruster.global_position)
		var pitch_mix := 0.0
		var roll_mix := 0.0

		# Pitch differential: rear engines push more to pitch nose-down.
		if max_arm_z > 0.0:
			pitch_mix = (offset.z / max_arm_z) * pitch_output

		# Roll differential: left engines push more to roll right.
		if max_arm_x > 0.0:
			roll_mix = (-offset.x / max_arm_x) * roll_output

		thruster.set_throttle(clampf(collective + pitch_mix + roll_mix, 0.0, 1.0))


# ── PID helper ────────────────────────────────────────────────────
# Returns Vector3(output, updated_integral, updated_prev_error)
# packed into one value to avoid extra state objects.

func _pid(
	target: float, current: float,
	kp: float, ki: float, kd: float, dt: float,
	integral: float, prev_error: float
) -> Vector3:
	var error := target - current

	# I: accumulate and clamp.
	integral += error * dt
	integral = clampf(integral, -integral_limit, integral_limit)

	# D: rate of change of error.
	var derivative := (error - prev_error) / dt

	var output := (kp * error) + (ki * integral) + (kd * derivative)
	return Vector3(output, integral, error)


func _hover_throttle(body: RigidBody3D, total_max: float) -> float:
	var g := maxf(float(ProjectSettings.get_setting("physics/3d/default_gravity")) - anti_gravity, 0.0)
	return clampf((body.mass * g) / total_max, 0.0, 1.0)


func _get_delta() -> float:
	return get_physics_process_delta_time()
