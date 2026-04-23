class_name MappedController
extends Node
## Drop-in "pseudo-controller" node.
##
## Wraps a physical joypad plus a ControllerMapping resource and exposes its
## state through Godot signals. Instance this as a child of any scene that
## needs gamepad input — drones, menus, HUDs — and connect to its signals.
##
## Typical use:
##
##     @export var controller: MappedController
##
##     func _ready() -> void:
##         controller.left_stick_changed.connect(_on_left_stick)
##         controller.right_stick_changed.connect(_on_right_stick)
##         controller.button_pressed.connect(_on_button)
##
## A single MappedController tracks one physical device. Instance multiple
## copies for split-screen or AI-vs-player setups.

# --- Signals -----------------------------------------------------------------

## Emitted when a physical joypad becomes active on this controller.
signal controller_connected(device: int, joy_name: String)

## Emitted when the active joypad disconnects.
signal controller_disconnected(device: int)

## Emitted whenever a new ControllerMapping is applied, either via auto-match
## on connect, select_mapping(), or at startup.
signal mapping_applied(mapping: ControllerMapping)

## Emitted on any change of the left stick vector (x, y in [-1, 1], y positive = down).
signal left_stick_changed(value: Vector2)

## Emitted on any change of the right stick vector.
signal right_stick_changed(value: Vector2)

## Emitted on any joypad button press for the active device.
signal button_pressed(button: int)

## Emitted on any joypad button release for the active device.
signal button_released(button: int)

# --- Exports -----------------------------------------------------------------

## Mappings offered for selection. Leave empty to auto-load from mappings_dir.
@export var mappings: Array[ControllerMapping] = []

## Directory scanned for ControllerMapping .tres files when `mappings` is empty.
@export_dir var mappings_dir: String = "res://Controller/Mappings"

## Below this magnitude, stick vectors are snapped to zero (circular dead zone).
@export_range(0.0, 0.5, 0.01) var dead_zone: float = 0.1

## Smallest change in a stick axis that still triggers *_stick_changed.
## Prevents spamming subscribers with sub-pixel float noise.
@export_range(0.0, 0.1, 0.001) var change_epsilon: float = 0.002

# --- Runtime state -----------------------------------------------------------

## Device index of the active joypad, or -1 when none is connected.
var active_device: int = -1

## Currently applied mapping, or null.
var current_mapping: ControllerMapping

var _left_stick := Vector2.ZERO
var _right_stick := Vector2.ZERO


func _ready() -> void:
	if mappings.is_empty():
		_auto_load_mappings()

	Input.joy_connection_changed.connect(_on_joy_connection_changed)

	var already_connected := Input.get_connected_joypads()
	if already_connected.size() > 0:
		_connect_controller(already_connected[0])


func _process(_delta: float) -> void:
	if active_device == -1 or current_mapping == null:
		return

	_update_stick(
		current_mapping.left_axis_x, current_mapping.left_axis_y,
		current_mapping.left_invert_x, current_mapping.left_invert_y,
		true,
	)
	_update_stick(
		current_mapping.right_axis_x, current_mapping.right_axis_y,
		current_mapping.right_invert_x, current_mapping.right_invert_y,
		false,
	)


func _input(event: InputEvent) -> void:
	# Forward physical button presses/releases. Ignore events from other devices.
	if event is InputEventJoypadButton and event.device == active_device:
		if event.pressed:
			button_pressed.emit(event.button_index)
		else:
			button_released.emit(event.button_index)


# --- Public API --------------------------------------------------------------

## Snapshot of the current left-stick vector.
func get_left_stick() -> Vector2:
	return _left_stick


## Snapshot of the current right-stick vector.
func get_right_stick() -> Vector2:
	return _right_stick


## Apply the mapping at the given index in `mappings`. No-op if out of range.
func select_mapping(index: int) -> void:
	if index < 0 or index >= mappings.size():
		return
	_apply_mapping(mappings[index])


## Returns -1 if no mapping is currently applied.
func get_current_mapping_index() -> int:
	if current_mapping == null:
		return -1
	return mappings.find(current_mapping)


# --- Controller lifecycle ----------------------------------------------------

func _on_joy_connection_changed(device: int, connected: bool) -> void:
	if connected:
		if active_device == -1:
			# Deferred: Godot fires this signal before the name is registered.
			call_deferred("_connect_controller", device)
	elif device == active_device:
		_disconnect_controller()


func _connect_controller(device: int) -> void:
	active_device = device
	var joy_name := _resolve_joy_name(device)
	controller_connected.emit(device, joy_name)

	var auto_index := _find_mapping_index_for(joy_name)
	if auto_index != -1:
		_apply_mapping(mappings[auto_index])
	elif current_mapping == null and not mappings.is_empty():
		# Fall back to the first available mapping so there is always *something*.
		_apply_mapping(mappings[0])


func _disconnect_controller() -> void:
	var device := active_device
	active_device = -1
	_emit_left(Vector2.ZERO)
	_emit_right(Vector2.ZERO)
	controller_disconnected.emit(device)


# --- Stick polling -----------------------------------------------------------

func _update_stick(axis_x: int, axis_y: int, inv_x: bool, inv_y: bool, is_left: bool) -> void:
	var x := Input.get_joy_axis(active_device, axis_x)
	var y := Input.get_joy_axis(active_device, axis_y)
	if inv_x:
		x = -x
	if inv_y:
		y = -y

	var v := Vector2(x, y)
	if v.length() < dead_zone:
		v = Vector2.ZERO

	if is_left:
		_emit_left(v)
	else:
		_emit_right(v)


func _emit_left(v: Vector2) -> void:
	if _left_stick.distance_to(v) < change_epsilon and v != Vector2.ZERO:
		return
	if _left_stick == v:
		return
	_left_stick = v
	left_stick_changed.emit(v)


func _emit_right(v: Vector2) -> void:
	if _right_stick.distance_to(v) < change_epsilon and v != Vector2.ZERO:
		return
	if _right_stick == v:
		return
	_right_stick = v
	right_stick_changed.emit(v)


# --- Mapping handling --------------------------------------------------------

func _apply_mapping(mapping: ControllerMapping) -> void:
	current_mapping = mapping
	if mapping == null:
		return
	mapping_applied.emit(mapping)


func _find_mapping_index_for(joy_name: String) -> int:
	var lowered := joy_name.to_lower()
	for i in mappings.size():
		var m := mappings[i]
		if m == null or m.name_match == "":
			continue
		if lowered.find(m.name_match.to_lower()) != -1:
			return i
	return -1


func _auto_load_mappings() -> void:
	var dir := DirAccess.open(mappings_dir)
	if dir == null:
		push_warning("MappedController: mappings dir not found: %s" % mappings_dir)
		return

	dir.list_dir_begin()
	var file := dir.get_next()
	while file != "":
		if not dir.current_is_dir() and (file.ends_with(".tres") or file.ends_with(".res")):
			var res := load("%s/%s" % [mappings_dir, file])
			if res is ControllerMapping:
				mappings.append(res)
		file = dir.get_next()
	dir.list_dir_end()

	mappings.sort_custom(func(a, b): return a.display_name < b.display_name)


# --- Device name resolution --------------------------------------------------

## See controller_test_scene for rationale — Godot 4.4 on Windows XInput does
## not expose a product name, so fall back to xinput_index when available.
func _resolve_joy_name(device: int) -> String:
	var builtin := Input.get_joy_name(device)
	if builtin != "":
		return builtin

	var info := Input.get_joy_info(device)
	if info.has("xinput_index"):
		return "Xbox-compatible Controller (XInput #%d)" % info["xinput_index"]

	if Input.is_joy_known(device):
		return "Known Gamepad %d" % device

	return "Controller %d" % device
