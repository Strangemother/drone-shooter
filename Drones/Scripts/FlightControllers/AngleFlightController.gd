## "Angle" (stabilised) flight controller.
## Auto-hovers at the throttle level needed to counteract gravity.
## Player input adjusts collective, pitch, and roll around that baseline.
## This is the DJI / consumer-drone feel.
extends FlightControllerBase

class_name FlightAngleController

## Subtracted from project gravity when computing hover thrust.
## At 0 the drone must fight full gravity; match project gravity to hover free.
@export_range(0.0, 20.0, 0.01) var anti_gravity: float = 0.0

## Fixed offset added to the computed hover throttle.
## Positive = tends to climb; negative = tends to sink.
@export_range(-1.0, 1.0, 0.001) var throttle_trim: float = 0.0

## How far the player's throttle input can push above/below hover.
@export_range(0.0, 1.0, 0.001) var collective_authority: float = 0.25

## How aggressively front-vs-back differential responds to pitch input.
@export_range(0.0, 1.0, 0.001) var pitch_authority: float = 0.18

## Same but for left-vs-right differential (roll).
@export_range(0.0, 1.0, 0.001) var roll_authority: float = 0.18


func update_mix(body: RigidBody3D, thrusters: Array[Node]) -> void:
	var total_max := get_total_max_force(thrusters)
	if total_max <= 0.0:
		return

	# --- COLLECTIVE ---
	# Start from the throttle that exactly counters gravity.
	var collective := _hover_throttle(body, total_max)
	collective += throttle_trim
	collective += get_axis_value(throttle_down_action, throttle_up_action) * collective_authority

	# --- DIFFERENTIAL ---
	var pitch_input := get_axis_value(pitch_backward_action, pitch_forward_action)
	var roll_input := get_axis_value(roll_left_action, roll_right_action)
	var max_arm_x := get_max_arm_length(body, thrusters, "x")
	var max_arm_z := get_max_arm_length(body, thrusters, "z")

	for thruster in thrusters:
		var offset := body.to_local(thruster.global_position)
		var pitch_mix := 0.0
		var roll_mix := 0.0

		if max_arm_z > 0.0:
			pitch_mix = (offset.z / max_arm_z) * pitch_input * pitch_authority

		if max_arm_x > 0.0:
			roll_mix = (-offset.x / max_arm_x) * roll_input * roll_authority

		thruster.set_throttle(clampf(collective + pitch_mix + roll_mix, 0.0, 1.0))


func _hover_throttle(body: RigidBody3D, total_max: float) -> float:
	var g := maxf(float(ProjectSettings.get_setting("physics/3d/default_gravity")) - anti_gravity, 0.0)
	return clampf((body.mass * g) / total_max, 0.0, 1.0)
