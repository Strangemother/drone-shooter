extends Node3D

class_name FlightThruster

@export_node_path("RigidBody3D") var target_body_path: NodePath = NodePath("..")
@export_range(0.0, 500.0, 0.1) var max_force: float = 45.0

## Current *actual* throttle (∈ [0, 1]) — the value the thruster is
## applying to the physics body this tick.  Drifts toward
## `_commanded_throttle` via the PowerRamp low-pass filter (see
## `spool_up_time` / `spool_down_time` below).
##
## Normally read-only from outside; controllers should call
## `set_thrust` / `set_thrust_directed` / `set_throttle` to update
## the command.  Direct assignment to `throttle = x` *bypasses the
## ramp* and snaps both command and actual throttle to `x` — useful
## for tests or hard resets, but not for normal flight control.
@export_range(0.0, 1.0, 0.001) var throttle: float = 0.0:
	set(value):
		throttle = clampf(value, 0.0, 1.0)
		_commanded_throttle = throttle

## Thrust direction in the thruster's *local* frame.  The thruster's
## own `global_transform.basis` rotates this into world space each
## tick, so a force_axis of UP follows the thruster's orientation
## (and therefore the parent body's orientation) for free.
##
## A controller that wants to vector-thrust at runtime should call
## `set_thrust(Vector3)` rather than poking `force_axis` and
## `throttle` separately — see that method's docs.
@export var force_axis: Vector3 = Vector3.UP
@export var enabled: bool = true

## PowerRamp — spool-up time constant (seconds).  How long the motor
## takes to accelerate toward a higher commanded throttle.  Real
## brushless drone motors with props typically sit in the 0.05–0.15 s
## range; aircraft engines and turbines are 1–5 s.
##
## Implemented as a first-order low-pass filter:
##   α = 1 − exp(−Δt / τ)
##   throttle ← throttle + (commanded − throttle) · α
## so 63 % of the gap is closed after one τ, 95 % after 3 τ.
##
## Set to 0 to disable spool-up (instantaneous response).
@export_range(0.0, 5.0, 0.001) var spool_up_time: float = 0.08

## PowerRamp — spool-down time constant (seconds).  How long the
## motor takes to decelerate toward a lower commanded throttle.
## Usually *slower* than spool-up on a free-wheeling prop (motor
## stops being driven, prop inertia carries on for longer).
##
## Set equal to `spool_up_time` for a symmetric response, or set to
## 0 to disable spool-down (snaps instantly to lower values).
@export_range(0.0, 5.0, 0.001) var spool_down_time: float = 0.12

## Enable in-ground-effect (IGE) thrust augmentation.  When the
## downward-facing raycast (see `ground_ray_path`) sees a surface
## close enough, the thruster's output is boosted per the
## Cheeseman–Bennett model:
##
##   T_IGE / T_OGE = 1 / (1 − (R / 4h)²)
##
## where R is the effective rotor radius and h is the motor's height
## above the detected ground.  The effect is ~6 % at h = 2R, grows
## rapidly as h approaches R, and is capped by `ground_effect_max`.
##
## Requires a `RayCast3D` child (or a node at `ground_ray_path`)
## pointing along the thruster's local −Y with enough reach to see
## the ground at rest height.
@export var ground_effect_enabled: bool = true

## Path to the downward-pointing RayCast3D used to measure height
## above ground.  Defaults to a sibling or child called
## "FootRayCast3D" (as named in `thruster_collision_shape.tscn`).
## The ray's `target_position` sets the maximum detection distance;
## if the ray isn't colliding, ground effect is off (multiplier = 1).
@export_node_path("RayCast3D") var ground_ray_path: NodePath = NodePath("FootRayCast3D")

## Effective rotor radius R (metres).  Used as the scaling length in
## the Cheeseman–Bennett formula.  A good default is the visible
## prop radius — the cylinder mesh in `thruster_collision_shape.tscn`
## is 0.3 m, so 0.3 matches what the player sees.
##
## Smaller R → effect only kicks in very close to the ground.
## Larger R → effect felt further out, feels "floatier" on landing.
@export_range(0.05, 2.0, 0.01) var ground_effect_radius: float = 0.3

## Hard cap on the IGE thrust multiplier.  The raw formula diverges
## as h → R, so we clamp to prevent unphysical infinite lift when
## the drone is touching or below the rotor plane.  Real helicopters
## see roughly 1.3–1.5× thrust at very low hover heights; 1.5 is a
## safe, game-feel-friendly ceiling.  Raise this (together with
## `ground_effect_gain`) for an overdriven "development" cushion.
@export_range(1.0, 10.0, 0.01) var ground_effect_max: float = 1.5

## Gain applied to the *boost* portion of the IGE multiplier.
## Lets you exaggerate the cushion without changing rotor radius or
## ray length.  The Cheeseman–Bennett multiplier M ≥ 1 is rewritten
## as 1 + gain·(M − 1), so:
##   • gain = 1.0 → physically correct Cheeseman–Bennett.
##   • gain = 3.0 → three times the boost at every height.
##   • gain = 0.0 → ground effect fully disabled (same as flag off).
## The final value is still clamped to `ground_effect_max`, so push
## that cap up too if you want the overdriven boost to actually land.
@export_range(0.0, 10.0, 0.01) var ground_effect_gain: float = 1.0

## Print per-physics-tick diagnostics for this thruster's ground
## effect: ray-hit state, measured height, and computed multiplier.
## Leave off in shipped builds — it's noisy.  Useful for confirming
## the ray is actually hitting the ground layer during tuning.
@export var ground_effect_debug: bool = false

var target_body: RigidBody3D
var _ground_ray: RayCast3D

# Throttle the controller *asked* for — the target the ramped
# `throttle` chases every physics tick.  Written via set_thrust /
# set_thrust_directed / set_throttle; read only internally.
var _commanded_throttle: float = 0.0


func _enter_tree() -> void:
	add_to_group("vehicle_thrusters")


func _ready() -> void:
	target_body = _resolve_target_body()
	_ground_ray = get_node_or_null(ground_ray_path) as RayCast3D


func _physics_process(delta: float) -> void:
	if not enabled:
		return

	# Step the PowerRamp toward the current command.  Pick the
	# spool-up or spool-down time constant depending on which
	# direction we're moving — real motors are asymmetric.
	var tau: float
	if _commanded_throttle > throttle:
		tau = spool_up_time
	else:
		tau = spool_down_time

	if tau > 0.0:
		# First-order low-pass: α = 1 − exp(−Δt/τ).  Stable for any
		# Δt (never overshoots), reduces to linear for small Δt/τ.
		var alpha: float = 1.0 - exp(-delta / tau)
		throttle = throttle + (_commanded_throttle - throttle) * alpha
	else:
		# Zero time constant → snap instantly.
		throttle = _commanded_throttle

	# Nothing to push if the ramp has fully decayed to zero.
	if throttle <= 0.0001:
		return

	if target_body == null:
		target_body = _resolve_target_body()
		if target_body == null:
			return

	var normalized_axis := force_axis.normalized()
	if normalized_axis == Vector3.ZERO:
		return

	var ground_multiplier := _compute_ground_effect_multiplier()
	var force := global_transform.basis * normalized_axis * max_force * throttle * ground_multiplier
	var application_point := global_position - target_body.global_position
	target_body.sleeping = false
	target_body.apply_force(force, application_point)


## Compute the Cheeseman–Bennett in-ground-effect multiplier.
##
## Returns 1.0 (no effect) when ground effect is disabled, the ray
## is missing, or the ray isn't colliding.  Otherwise returns
## 1 / (1 − (R / 4h)²), clamped to [1, ground_effect_max].
##
## `h` is measured from the thruster's global position to the ray
## collision point (straight-line distance) — this is what a real
## rotor "feels", independent of body tilt, since the air cushion
## forms below the disc regardless of frame attitude.
func _compute_ground_effect_multiplier() -> float:
	if not ground_effect_enabled:
		return 1.0
	if _ground_ray == null:
		if ground_effect_debug:
			print("[GE %s] no RayCast3D at %s" % [name, ground_ray_path])
		return 1.0
	if not _ground_ray.is_colliding():
		if ground_effect_debug:
			print("[GE %s] ray not colliding (enabled=%s mask=%d)" % [
				name, _ground_ray.enabled, _ground_ray.collision_mask
			])
		return 1.0

	var h: float = global_position.distance_to(_ground_ray.get_collision_point())
	# Guard the singularity at h → 0 and stay well clear of it.
	# Treat anything closer than a quarter-radius as the cap.
	var h_min: float = ground_effect_radius * 0.25
	if h < h_min:
		if ground_effect_debug:
			print("[GE %s] h=%.3f < h_min=%.3f → max=%.2f" % [name, h, h_min, ground_effect_max])
		return ground_effect_max

	var ratio: float = ground_effect_radius / (4.0 * h)
	var denom: float = 1.0 - ratio * ratio
	if denom <= 0.0:
		if ground_effect_debug:
			print("[GE %s] denom<=0 at h=%.3f → max=%.2f" % [name, h, ground_effect_max])
		return ground_effect_max

	var raw: float = 1.0 / denom
	# Overdrive: scale just the boost (raw − 1), keeping the floor at 1.0
	# so "no effect" is still exactly 1.0 regardless of gain.
	var boosted: float = 1.0 + ground_effect_gain * (raw - 1.0)
	var result: float = clampf(boosted, 1.0, ground_effect_max)
	if ground_effect_debug:
		print("[GE %s] h=%.3f raw=%.3f boosted=%.3f mult=%.3f hit=%s" % [
			name, h, raw, boosted, result, _ground_ray.get_collider()
		])
	return result


## Set throttle in [0, 1].  Direction stays whatever `force_axis` was.
## Use this when you only want to vary power and the thruster's
## orientation is fixed (the common case).
##
## Updates the *command* — actual applied throttle will ramp toward
## this value over `spool_up_time` / `spool_down_time` seconds.
func set_throttle(value: float) -> void:
	_commanded_throttle = clampf(value, 0.0, 1.0)


## Set thrust as a single vector.  The vector's *length* is the
## commanded throttle (clamped to [0, 1]) and its *direction* is the
## local-frame axis the thrust is applied along.
##
## This is the recommended API for runtime controllers — the
## developer thinks in "I want this much push in this direction"
## terms, and the thruster handles converting that into a body
## force at the correct application point.  Body translation and
## torque from off-centre application happen automatically.
##
## Like `set_throttle`, this updates the *command*; the actual
## applied throttle will lag behind per the PowerRamp constants.
##
## Examples:
##   • `set_thrust(Vector3.UP * 0.5)` — half-throttle straight up.
##   • `set_thrust(Vector3.ZERO)` — engine off (will spool down).
##   • `set_thrust(Vector3(0, 0.7, -0.3).normalized() * 0.8)` —
##     80% throttle, vectored mostly up with a forward component.
##
## Note: like `force_axis`, the direction is in the thruster's
## *local* frame.  If you want world-space thrust, transform with
## `global_transform.basis.inverse() * world_vector` first.
func set_thrust(thrust: Vector3) -> void:
	var power := thrust.length()
	if power <= 0.0001:
		_commanded_throttle = 0.0
		return
	force_axis = thrust / power
	_commanded_throttle = clampf(power, 0.0, 1.0)


## Convenience: set direction and power independently in one call.
## Equivalent to `set_thrust(direction.normalized() * power)` but
## skips a normalize when the caller already has a unit vector.
##
## Updates the *command* — see `set_thrust` notes about PowerRamp.
func set_thrust_directed(direction: Vector3, power: float) -> void:
	if direction == Vector3.ZERO or power <= 0.0:
		_commanded_throttle = 0.0
		return
	force_axis = direction.normalized()
	_commanded_throttle = clampf(power, 0.0, 1.0)


func get_max_force() -> float:
	return max_force


## Returns the throttle value most recently *commanded* via set_thrust
## / set_thrust_directed / set_throttle.  The actual applied throttle
## (`throttle` property) lags this by the PowerRamp time constant.
##
## Useful for controllers that need to distinguish "what the pilot
## asked for" from "what the prop is actually doing right now" —
## e.g. a PID rate loop measuring tracking error.
func get_commanded_throttle() -> float:
	return _commanded_throttle


func _resolve_target_body() -> RigidBody3D:
	var node := get_node_or_null(target_body_path)
	if node is RigidBody3D:
		return node

	push_warning("Thruster target is not a RigidBody3D: %s" % [target_body_path])
	return null
