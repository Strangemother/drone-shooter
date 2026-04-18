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
## Tuned for ~0.5 kg, 4×7 N thrusters, 0.4 m arms (TWR ~5.7:1).
## Lower for snappier but unstable response; raise if drone is heavier.
@export_range(0.0, 5.0, 0.001) var pitch_p: float = 0.4

## Integral: accumulates small persistent errors to eliminate steady-state
## offset.  Keep zero on light drones to avoid wind-up.
@export_range(0.0, 2.0, 0.001) var pitch_i: float = 0.0

## Derivative: damps oscillation by opposing the measured body angular
## velocity directly (read from RigidBody3D.angular_velocity — no stick kick,
## no finite-difference noise).  This is the critical anti-flip gain:
## needs to be high enough to brake rotation before the drone tips past
## vertical.
@export_range(0.0, 5.0, 0.001) var pitch_d: float = 0.35


# ── PID gains for roll axis ──────────────────────────────────────

@export_range(0.0, 5.0, 0.001) var roll_p: float = 0.4
@export_range(0.0, 2.0, 0.001) var roll_i: float = 0.0
@export_range(0.0, 5.0, 0.001) var roll_d: float = 0.35


# ── PID gains for vertical (altitude hold) ───────────────────────

@export_range(0.0, 5.0, 0.001) var alt_p: float = 0.4
@export_range(0.0, 2.0, 0.001) var alt_i: float = 0.02
@export_range(0.0, 5.0, 0.001) var alt_d: float = 0.5


# ── limits ────────────────────────────────────────────────────────

## Maximum tilt the player can command (radians).  ~0.52 rad ≈ 30°.
@export_range(0.0, 1.57, 0.01) var max_tilt_angle: float = 0.52

## Integral wind-up clamp — prevents the I term from growing without bound.
@export_range(0.0, 1.0, 0.01) var integral_limit: float = 0.3

## Hard cap on per-thruster throttle adjustment from attitude PID.
## Bounds the absolute differential thrust.  Too low = PID can't brake
## hard rotations and the drone flips.  Too high = can overspeed recovery
## and oscillate.  Should be well above hover throttle.
@export_range(0.0, 1.0, 0.001) var attitude_authority: float = 0.6


# ── yaw damping ───────────────────────────────────────────────────
# Thruster force mixing handles pitch & roll through force differential.
# It cannot produce yaw torque (all thrusters fire along body-UP), so
# yaw is damped via a direct torque controller instead.

## Torque per rad/s of yaw-rate error.  Higher = stiffer yaw hold.
@export_range(0.0, 50.0, 0.1) var rate_p: float = 2.0

## Player yaw input command range (rad/s).
@export_range(0.0, 10.0, 0.1) var max_yaw_rate: float = 2.0


# ── internal PID state ────────────────────────────────────────────

var _pitch_integral: float = 0.0
var _roll_integral: float = 0.0

var _alt_integral: float = 0.0
var _alt_prev_error: float = 0.0

## Captured on first update — the altitude the drone should hold when
## the player gives no vertical input.
var _target_altitude: float = NAN


func reset_state() -> void:
	_pitch_integral = 0.0
	_roll_integral = 0.0
	_alt_integral = 0.0
	_alt_prev_error = 0.0
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
	# thruster mix (which is also computed in body frame).
	#
	# Sign convention: positive current_pitch = nose-DOWN (matches the
	# mix, where positive pitch_output = rear motors harder = nose-down)
	# and matches the player input (positive target = stick forward =
	# wants nose-down to fly forward).  Getting the sign here wrong
	# produces a runaway positive-feedback loop on the pitch axis.
	var world_up_body := basis_t * Vector3.UP
	var current_pitch := atan2(world_up_body.z, world_up_body.y)   # + = nose-down
	var current_roll  := atan2(world_up_body.x, world_up_body.y)   # + = right-wing-down

	# Body-frame angular velocity (rad/s about body X / Y / Z).
	var omega_body := basis_t * body.angular_velocity

	# ── target angles from player input ──────────────────────────
	var pitch_stick := get_axis_value(pitch_backward_action, pitch_forward_action)
	var roll_stick  := get_axis_value(roll_left_action, roll_right_action)
	var yaw_stick   := get_axis_value(yaw_left_action, yaw_right_action)

	var target_pitch := pitch_stick * max_tilt_angle
	var target_roll  := roll_stick * max_tilt_angle

	# ── PID: pitch ─────────────────────────────────────────────────
	# D term uses body-X angular velocity directly (positive = nose
	# pitching up, so we negate to oppose motion toward nose-down).
	var pitch_correction := _pid_rate(
		target_pitch, current_pitch, -omega_body.x,
		pitch_p, pitch_i, pitch_d, dt,
		_pitch_integral
	)
	_pitch_integral = pitch_correction.y
	var pitch_output: float = clampf(pitch_correction.x, -attitude_authority, attitude_authority)

	# ── PID: roll ──────────────────────────────────────────────────
	# D term uses body-Z angular velocity (positive = rolling right).
	var roll_correction := _pid_rate(
		target_roll, current_roll, omega_body.z,
		roll_p, roll_i, roll_d, dt,
		_roll_integral
	)
	_roll_integral = roll_correction.y
	var roll_output: float = clampf(roll_correction.x, -attitude_authority, attitude_authority)

	# ── yaw damping ───────────────────────────────────────────────────
	# Thrusters firing along body-UP cannot produce yaw torque by mixing.
	# Rather than fight the solver with apply_torque (which can go unstable
	# at low body inertia), set `angular_damp` to ~3.0 on the drone's
	# RigidBody3D in the inspector — the physics engine will damp any
	# residual spin for free.  Leave this controller silent on yaw until
	# a dedicated yaw actuator is present.
	var _ignore := yaw_stick  # keep the variable in scope for future use

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

	# ── mix per-thruster with saturation balancing ──────────────
	# Compute attitude contributions first, then fit the collective
	# into whatever throttle range remains.  Attitude is preserved at
	# the cost of altitude — this is "airmode" behaviour and is the
	# critical difference between a drone that recovers from hard
	# input and one that flips.  Motors can't produce negative thrust,
	# so if the front motors would need to go below zero to oppose a
	# nose-down rotation, the old code simply let the drone flip.
	_apply_mix(body, thrusters, pitch_output, roll_output, collective)


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


## Derivative-on-measurement PID.  Uses the rate of change of the
## *measured* value, not the error — so a step change in target
## (the player yanking the stick) does not cause a derivative kick.
## Returns Vector3(output, updated_integral, updated_prev_angle).
func _pid_dom(
	target: float, current: float,
	kp: float, ki: float, kd: float, dt: float,
	integral: float, prev_angle: float
) -> Vector3:
	var error := target - current

	integral += error * dt
	integral = clampf(integral, -integral_limit, integral_limit)

	# Note the sign: D acts against the direction of motion, so
	# subtract kd * (rate of change of measurement).
	var measurement_rate := (current - prev_angle) / dt
	var output := (kp * error) + (ki * integral) - (kd * measurement_rate)
	return Vector3(output, integral, current)


## PID with external rate input.  The D term comes from an explicitly-
## supplied rate (body angular velocity) rather than finite-differencing
## the angle, which avoids noise and angle-wrap problems.  Returns
## Vector2(output, updated_integral).
func _pid_rate(
	target: float, current: float, rate: float,
	kp: float, ki: float, kd: float, dt: float,
	integral: float
) -> Vector2:
	var error := target - current

	integral += error * dt
	integral = clampf(integral, -integral_limit, integral_limit)

	var output := (kp * error) + (ki * integral) - (kd * rate)
	return Vector2(output, integral)


func _hover_throttle(body: RigidBody3D, total_max: float) -> float:
	var g := maxf(float(ProjectSettings.get_setting("physics/3d/default_gravity")) - anti_gravity, 0.0)
	return clampf((body.mass * g) / total_max, 0.0, 1.0)


## Airmode-style thruster mix.
##
## Computes each motor's attitude contribution (pitch + roll differential)
## first, then fits the collective throttle into whatever [0,1] range
## remains — preserving the attitude differential at all costs.  If the
## attitude demand itself exceeds the full throttle range it is scaled
## down proportionally (so the drone doesn't spin up harder than
## physically possible) but the ratio between motors is maintained.
##
## The outcome: when the drone has to choose between "keep altitude" and
## "don't flip", it chooses "don't flip" every time.
func _apply_mix(
	body: RigidBody3D,
	thrusters: Array[Node],
	pitch_output: float,
	roll_output: float,
	collective: float
) -> void:
	var n := thrusters.size()
	if n == 0:
		return

	var max_arm_x := get_max_arm_length(body, thrusters, "x")
	var max_arm_z := get_max_arm_length(body, thrusters, "z")

	# 1) Per-motor attitude contribution.
	var atti := PackedFloat32Array()
	atti.resize(n)
	var max_a := -INF
	var min_a :=  INF
	for i in n:
		var offset := body.to_local(thrusters[i].global_position)
		var pm := 0.0
		var rm := 0.0
		if max_arm_z > 0.0:
			pm = (offset.z / max_arm_z) * pitch_output
		if max_arm_x > 0.0:
			rm = (-offset.x / max_arm_x) * roll_output
		var a := pm + rm
		atti[i] = a
		if a > max_a: max_a = a
		if a < min_a: min_a = a

	# 2) If the attitude span is wider than [0,1], scale it down
	# while preserving ratios between motors.
	var span := max_a - min_a
	if span > 1.0:
		var scale := 1.0 / span
		for i in n:
			atti[i] *= scale
		max_a *= scale
		min_a *= scale

	# 3) Clamp collective into the remaining throttle window so
	# that (collective + atti[i]) stays in [0,1] for every motor.
	# This sacrifices altitude authority to preserve attitude.
	var collective_effective := clampf(collective, -min_a, 1.0 - max_a)

	for i in n:
		thrusters[i].set_throttle(clampf(collective_effective + atti[i], 0.0, 1.0))


func _get_delta() -> float:
	return get_physics_process_delta_time()
