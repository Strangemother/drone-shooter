## "Acro" (rate / FPV racing) flight controller.
## No auto-hover — collective starts at zero and the player supplies all
## throttle.  Pitch and roll mix differentially the same way as Angle
## but without a gravity baseline.  Cut throttle and it drops.
extends DroneFlightController

class_name AcroFlightController

## Full range of collective the player can command (0..1).
## At 1.0 the player has access to the entire throttle sweep.
@export_range(0.0, 1.0, 0.001) var collective_authority: float = 1.0

## Pitch and roll differential authority — how much engine imbalance
## the stick input can create.  Higher = snappier flips.
@export_range(0.0, 1.0, 0.001) var pitch_authority: float = 0.35
@export_range(0.0, 1.0, 0.001) var roll_authority: float = 0.35


func update_mix(body: RigidBody3D, thrusters: Array[Node]) -> void:
	var total_max := get_total_max_force(thrusters)
	if total_max <= 0.0:
		return

	# No hover baseline — collective is purely player-driven.
	var collective := get_axis_value(throttle_down_action, throttle_up_action) * collective_authority

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
