## Base class for drone flight controllers.
##
## Attach a subclass of this as a child of a `DroneScene` to control its
## thrusters.  If no controller is attached the drone runs in "raw"
## mode — no automatic mixing; external code sets each thruster's
## throttle directly via `set_throttle`.
##
## Subclasses must override `update_mix()`.  Optionally override
## `reset_state()` if the controller accumulates state (PID integrals,
## filters) that needs clearing after a teleport.
##
## ── Input convention ───────────────────────────────────────────────
## Input action names are exported so you can remap controls in the
## inspector without editing code.  An empty `StringName` (&"") means
## "this axis has no input bound"; the controller must tolerate that.
##
## ── Naming ─────────────────────────────────────────────────────────
## Class name is `FlightControllerBase` (the file of the same name).
## Subclasses should be named `Flight<Mode>Controller`, e.g.
## `FlightAngleController`, `FlightAcroController`.
extends Node

class_name FlightControllerBase

# Cache of action StringName → analog strength [0, 1] populated by
# MappedController stick signals.  Read by _action_strength before
# falling back to Input.get_action_strength.
var _controller_axes: Dictionary = {}


# ── input action bindings ─────────────────────────────────────────

## Input action names — must match entries in Project > Input Map.
## Change these to remap controls without editing code.  An empty
## StringName means "axis is unbound" and returns 0 from
## `get_axis_value`.
@export var throttle_up_action: StringName = &"jump"
@export var throttle_down_action: StringName = &"crouch"
@export var pitch_forward_action: StringName = &"moveForward"
@export var pitch_backward_action: StringName = &"moveBackward"
@export var roll_left_action: StringName = &"moveLeft"
@export var roll_right_action: StringName = &"moveRight"
@export var yaw_left_action: StringName = &""
@export var yaw_right_action: StringName = &""

## Optional MappedController whose stick signals drive `_action_strength`.
## Assign in the inspector or from code.  When set, controller stick values
## take priority over keyboard for any action name they cover.  When null,
## only Godot's InputMap / keyboard is used.
##
## Connection and disconnection are handled automatically by the setter;
## no _ready() call is required in this class or any subclass.
@export var controller: MappedController:
	set(value):
		_disconnect_controller()
		controller = value
		_connect_controller()


# ── virtual overrides ─────────────────────────────────────────────

## Called every physics tick by `DroneScript` with the vehicle body
## and its list of discovered thrusters.  Override in subclasses to
## implement your mixing strategy.  The base implementation is a
## no-op — a controller that doesn't override this does nothing.
func update_mix(_body: RigidBody3D, _thrusters: Array[Node]) -> void:
	pass


## Called after a position reset (teleport, respawn) to clear any
## accumulated internal state.  Override in subclasses that track
## state (e.g. PID integrals, velocity filters).
func reset_state() -> void:
	pass


# ── controller signal integration ────────────────────────────────

## Connects MappedController stick signals.  Called automatically by
## the `controller` property setter — do not call manually.
func _connect_controller() -> void:
	if controller == null:
		return
	if not controller.left_stick_changed.is_connected(_on_left_stick):
		controller.left_stick_changed.connect(_on_left_stick)
	if not controller.right_stick_changed.is_connected(_on_right_stick):
		controller.right_stick_changed.connect(_on_right_stick)
	if not controller.trigger_changed.is_connected(_on_trigger_changed):
		controller.trigger_changed.connect(_on_trigger_changed)


## Disconnects signals from the previously assigned controller and
## clears the axis cache so stale values do not persist.
func _disconnect_controller() -> void:
	if controller == null:
		return
	if controller.left_stick_changed.is_connected(_on_left_stick):
		controller.left_stick_changed.disconnect(_on_left_stick)
	if controller.right_stick_changed.is_connected(_on_right_stick):
		controller.right_stick_changed.disconnect(_on_right_stick)
	if controller.trigger_changed.is_connected(_on_trigger_changed):
		controller.trigger_changed.disconnect(_on_trigger_changed)
	_controller_axes.clear()


## Left stick handler — default layout Mode 2 (most common FPV):
##   X axis → yaw   (left = yaw_left,   right = yaw_right)
##   Y axis → throttle (up = throttle_up, down = throttle_down)
##
## MappedController signal convention: y positive = stick pushed DOWN.
## Negating Y means "stick up" produces a positive throttle_up value.
##
## When the active mapping has `use_trigger_for_throttle = true` the Y axis
## is skipped here — the right trigger drives throttle instead via
## _on_trigger_changed.  Only yaw is written from this stick in that case.
##
## Override in a subclass to remap axes to different action pairs
## (e.g. Mode 1, or to drive pitch/roll from the left stick instead).
func _on_left_stick(value: Vector2) -> void:
	_write_axis_pair(yaw_left_action, yaw_right_action, value.x)
	# Suppress throttle from left stick Y when the mapping uses trigger throttle.
	var use_trigger := (controller != null
		and controller.current_mapping != null
		and controller.current_mapping.use_trigger_for_throttle)
	if not use_trigger:
		_write_axis_pair(throttle_down_action, throttle_up_action, -value.y)


## Right stick handler — default layout Mode 2:
##   X axis → roll  (left = roll_left,   right = roll_right)
##   Y axis → pitch (up = pitch_forward, down = pitch_backward)
func _on_right_stick(value: Vector2) -> void:
	_write_axis_pair(roll_left_action,      roll_right_action,    value.x)
	_write_axis_pair(pitch_backward_action, pitch_forward_action, -value.y)


## Trigger handler.  Both trigger values arrive together in [0, 1].
## Only fires when the MappedController's mapping has trigger axes configured
## (i.e. trigger_right_axis or trigger_left_axis ≥ 0 in the mapping resource).
##
## Default: right trigger → throttle_up (no throttle_down contribution —
## triggers are unipolar so releasing the trigger reads as 0 throttle).
## Override in a subclass to add left-trigger behaviour (e.g. braking).
func _on_trigger_changed(left: float, _right_val: float) -> void:
	# Right trigger drives throttle up; releasing = 0 (no down input).
	_controller_axes[throttle_up_action]   = _right_val
	_controller_axes[throttle_down_action] = 0.0
	# Left trigger is unused in the base mapping.  Subclasses may override.
	_ = left


## Decomposes a signed axis value v ∈ [−1, 1] into positive and
## negative action strengths and writes them into the axis cache.
##
##   v > 0 → pos_action = v,    neg_action = 0
##   v < 0 → neg_action = |v|,  pos_action = 0
##
## This mirrors how `get_axis_value` consumes a positive/negative pair
## via _action_strength — the decomposition is the exact inverse:
##
##   get_axis_value(neg, pos) = _action_strength(pos) − _action_strength(neg)
##                            = v (when v > 0)  or  v (when v < 0).
func _write_axis_pair(neg_action: StringName, pos_action: StringName, v: float) -> void:
	_controller_axes[pos_action] = maxf(v, 0.0)
	_controller_axes[neg_action] = maxf(-v, 0.0)


# ── shared helpers available to every controller ──────────────────

## Sum of every thruster's `get_max_force()`.  Used to normalise
## per-thruster throttle commands.
func get_total_max_force(thrusters: Array[Node]) -> float:
	var total := 0.0
	for t in thrusters:
		total += t.get_max_force()
	return total


## Longest lever arm on `axis` ("x" or "z") — the distance from the
## body centre to the farthest thruster along that axis.  Used to
## convert "I want this much torque" into per-thruster throttle
## differentials.
func get_max_arm_length(body: RigidBody3D, thrusters: Array[Node], axis: String) -> float:
	var max_arm := 0.0
	for t in thrusters:
		var offset := body.to_local(t.global_position)
		var v := offset.x if axis == "x" else offset.z
		max_arm = maxf(max_arm, absf(v))
	return max_arm


## Returns a scalar in [-1, 1] for the axis formed by two input
## actions.  Unbound actions (empty StringName) contribute 0.
func get_axis_value(negative_action: StringName, positive_action: StringName) -> float:
	return _action_strength(positive_action) - _action_strength(negative_action)


## Betaflight-style expo / stick-shaping curve.  Input `x` is any
## scalar in [−1, 1] (typically a stick axis from `get_axis_value`),
## `amount` ∈ [0, 1] is the expo strength:
##
##   • 0.0 → output = input (perfectly linear, no shaping).
##   • 0.5 → gentle softening near centre; common default for
##           stabilised / "angle-mode" controllers.
##   • 0.7 → aggressive softening; typical FPV ACRO tune.
##   • 1.0 → pure cubic: out = x³.  Endpoints still at ±1 but the
##           centre is very flat — small stick deflections barely
##           register, full-stick still gives full output.
##
## Formula (Betaflight's `RC_EXPO`, exactly):
##
##     out = (1 − e)·x + e·x³
##
## Endpoints are preserved (|x|=1 → |out|=1 regardless of expo), so
## you never lose max authority — only the *slope* near x=0 is
## reduced.  This is the right shape for a quadratic-thrust plant
## (T ∝ Ω²), which already adds gain with throttle; expo flattens
## the stick's low-input region so small wiggles don't translate
## into twitchy motor commands.
##
## Safe for any `amount` outside [0, 1] — clamped internally — but
## values above 1 invert the slope (output moves *backwards* near
## centre), which is never what you want in a flight controller.
func apply_expo(x: float, amount: float) -> float:
	if amount <= 0.0:
		return x
	var e: float = clampf(amount, 0.0, 1.0)
	return (1.0 - e) * x + e * x * x * x


func _action_strength(action: StringName) -> float:
	if action == &"":
		return 0.0
	# Controller signal values take priority over keyboard/InputMap.
	# Only actions that have received at least one stick signal are cached;
	# actions not covered by any stick axis fall through to Input normally.
	if _controller_axes.has(action):
		return _controller_axes[action]
	if not InputMap.has_action(action):
		return 0.0
	print('Get Action: ', action)
	return Input.get_action_strength(action)
