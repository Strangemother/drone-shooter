extends Node2D
## Controller Input suite
##
## Listens for joypad connection/disconnection events.
## On connect: displays the controller name, auto-selects a matching
## ControllerMapping (if one is provided), and begins axis monitoring.
## On disconnect: resets the display and stops monitoring.
##
## The scene expects:
##   - A RichTextLabel named "ControllerName".
##   - Two Control nodes "JoystickLeft" / "JoystickRight" with joystick.gd.
##   - Optionally an OptionButton named "MappingPicker" — one is created at
##     runtime if it isn't present in the scene.

## ControllerMapping resources offered in the picker. Drop .tres files here
## in the inspector — the picker UI is built from this list.
@export var mappings: Array[ControllerMapping] = []

@onready var _name_field: RichTextLabel = $ControllerName
@onready var _joystick_left: Control = $JoystickLeft
@onready var _joystick_right: Control = $JoystickRight

## Picker UI — created lazily if the scene doesn't already provide one.
var _mapping_picker: OptionButton

## Currently applied mapping (null until one is selected).
var _current_mapping: ControllerMapping

## Device index of the active controller, or -1 when none is connected.
var _active_device: int = -1


func _ready() -> void:
	_name_field.text = "No Controller"
	_ensure_picker()
	_populate_picker()

	# Listen for future connect/disconnect events.
	Input.joy_connection_changed.connect(_on_joy_connection_changed)

	# Pick up any controller that was already connected before the scene loaded.
	var already_connected := Input.get_connected_joypads()
	if already_connected.size() > 0:
		_connect_controller(already_connected[0])


func _process(_delta: float) -> void:
	if _active_device == -1:
		return

	# Each joystick visual already knows its axes; poll reads from the device.
	if _joystick_left:
		_joystick_left.poll(_active_device)
	if _joystick_right:
		_joystick_right.poll(_active_device)


# --- Signal handlers ---------------------------------------------------------

func _on_joy_connection_changed(device: int, connected: bool) -> void:
	if connected:
		# Track the first controller that connects; ignore additional ones.
		# Deferred because Godot fires this signal before the name registers.
		if _active_device == -1:
			call_deferred("_connect_controller", device)
	else:
		if device == _active_device:
			_disconnect_controller()


func _on_mapping_selected(index: int) -> void:
	if index < 0 or index >= mappings.size():
		return
	_apply_mapping(mappings[index])


# --- Controller lifecycle ----------------------------------------------------

func _connect_controller(device: int) -> void:
	_active_device = device
	var joy_name := _resolve_joy_name(device)
	_name_field.text = "NAME: %s" % joy_name
	print("Controller connected [%d]: %s" % [device, joy_name])

	# Prefer an auto-match based on the reported device name.
	var auto_index := _find_mapping_index_for(joy_name)
	if auto_index != -1:
		_mapping_picker.select(auto_index)
		_apply_mapping(mappings[auto_index])


func _disconnect_controller() -> void:
	print("Controller disconnected [%d]" % _active_device)
	_active_device = -1
	_name_field.text = "No Controller"

	# Return both stick visuals to centre.
	if _joystick_left:
		_joystick_left.poll(-1)
	if _joystick_right:
		_joystick_right.poll(-1)


# --- Mapping handling --------------------------------------------------------

func _apply_mapping(mapping: ControllerMapping) -> void:
	_current_mapping = mapping
	if mapping == null:
		return

	if _joystick_left and _joystick_left.has_method("apply_axes"):
		_joystick_left.apply_axes(
			mapping.left_axis_x, mapping.left_axis_y,
			mapping.left_invert_x, mapping.left_invert_y,
		)
	if _joystick_right and _joystick_right.has_method("apply_axes"):
		_joystick_right.apply_axes(
			mapping.right_axis_x, mapping.right_axis_y,
			mapping.right_invert_x, mapping.right_invert_y,
		)
	print("Applied mapping: %s" % mapping.display_name)


## Returns the index of the first mapping whose name_match substring
## occurs in joy_name, or -1 if none match.
func _find_mapping_index_for(joy_name: String) -> int:
	var lowered := joy_name.to_lower()
	for i in mappings.size():
		var m := mappings[i]
		if m == null or m.name_match == "":
			continue
		if lowered.find(m.name_match.to_lower()) != -1:
			return i
	return -1


# --- Picker UI ---------------------------------------------------------------

func _ensure_picker() -> void:
	_mapping_picker = get_node_or_null("MappingPicker") as OptionButton
	if _mapping_picker != null:
		return

	# Scene didn't include one — create a minimal picker at runtime so the
	# feature works without requiring a .tscn edit.
	_mapping_picker = OptionButton.new()
	_mapping_picker.name = "MappingPicker"
	_mapping_picker.position = Vector2(6, 48)
	_mapping_picker.custom_minimum_size = Vector2(260, 28)
	add_child(_mapping_picker)


func _populate_picker() -> void:
	_mapping_picker.clear()
	for m in mappings:
		if m == null:
			continue
		_mapping_picker.add_item(m.display_name)

	# Connect the selection signal exactly once.
	if not _mapping_picker.item_selected.is_connected(_on_mapping_selected):
		_mapping_picker.item_selected.connect(_on_mapping_selected)

	# Apply the first entry by default, if we have any.
	if mappings.size() > 0 and mappings[0] != null:
		_mapping_picker.select(0)
		_apply_mapping(mappings[0])


# --- Controller name resolution ---------------------------------------------

## Returns the best available name for a joypad device.
##
## Godot 4.4 on Windows uses the native XInput driver, which does not expose
## a human-readable product name — Input.get_joy_name() is empty and
## Input.get_joy_info() only returns {"xinput_index": N}. XInput itself
## (the Windows API) does not provide a per-device product string.
##
## We do the best we can:
##   1. Use get_joy_name() if the platform/driver supplied one.
##   2. If get_joy_info() reports xinput_index, label it as Xbox-compatible.
##   3. If Godot recognises the device via its SDL mapping database,
##      mark it as a known gamepad.
##   4. Otherwise fall back to a generic "Controller N" label.
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
extends Node2D
## Controller Input suite
##
## Listens for joypad connection/disconnection events.
## On connect: displays the controller name, auto-selects a matching
## ControllerMapping (if one is provided), and begins axis monitoring.
## On disconnect: resets the display and stops monitoring.
##
## The scene expects:
##   - A RichTextLabel named "ControllerName".
##   - Two Control nodes "JoystickLeft" / "JoystickRight" with joystick.gd.
##   - Optionally an OptionButton named "MappingPicker" — one is created at
##     runtime if it isn't present in the scene.

## ControllerMapping resources offered in the picker. Drop .tres files here
## in the inspector — the picker UI is built from this list.
@export var mappings: Array[ControllerMapping] = []

@onready var _name_field: RichTextLabel = $ControllerName
@onready var _joystick_left: Control = $JoystickLeft
@onready var _joystick_right: Control = $JoystickRight

## Picker UI — created lazily if the scene doesn't already provide one.
var _mapping_picker: OptionButton

## Currently applied mapping (null until one is selected).
var _current_mapping: ControllerMapping

## Device index of the active controller, or -1 when none is connected.
var _active_device: int = -1


func _ready() -> void:
	_name_field.text = "No Controller"
	_ensure_picker()
	_populate_picker()

	# Listen for future connect/disconnect events.
	Input.joy_connection_changed.connect(_on_joy_connection_changed)

	# Pick up any controller that was already connected before the scene loaded.
	var already_connected := Input.get_connected_joypads()
	if already_connected.size() > 0:
		_connect_controller(already_connected[0])


func _process(_delta: float) -> void:
	if _active_device == -1:
		return

	# Each joystick visual already knows its axes; poll reads from the device.
	if _joystick_left:
		_joystick_left.poll(_active_device)
	if _joystick_right:
		_joystick_right.poll(_active_device)


# --- Signal handlers ---------------------------------------------------------

func _on_joy_connection_changed(device: int, connected: bool) -> void:
	if connected:
		# Track the first controller that connects; ignore additional ones.
		# Deferred because Godot fires this signal before the name registers.
		if _active_device == -1:
			call_deferred("_connect_controller", device)
	else:
		if device == _active_device:
			_disconnect_controller()


func _on_mapping_selected(index: int) -> void:
	if index < 0 or index >= mappings.size():
		return
	_apply_mapping(mappings[index])


# --- Controller lifecycle ----------------------------------------------------

func _connect_controller(device: int) -> void:
	_active_device = device
	var joy_name := _resolve_joy_name(device)
	_name_field.text = "NAME: %s" % joy_name
	print("Controller connected [%d]: %s" % [device, joy_name])

	# Prefer an auto-match based on the reported device name.
	var auto_index := _find_mapping_index_for(joy_name)
	if auto_index != -1:
		_mapping_picker.select(auto_index)
		_apply_mapping(mappings[auto_index])


func _disconnect_controller() -> void:
	print("Controller disconnected [%d]" % _active_device)
	_active_device = -1
	_name_field.text = "No Controller"

	# Return both stick visuals to centre.
	if _joystick_left:
		_joystick_left.poll(-1)
	if _joystick_right:
		_joystick_right.poll(-1)


# --- Mapping handling --------------------------------------------------------

func _apply_mapping(mapping: ControllerMapping) -> void:
	_current_mapping = mapping
	if mapping == null:
		return

	if _joystick_left and _joystick_left.has_method("apply_axes"):
		_joystick_left.apply_axes(
			mapping.left_axis_x, mapping.left_axis_y,
			mapping.left_invert_x, mapping.left_invert_y,
		)
	if _joystick_right and _joystick_right.has_method("apply_axes"):
		_joystick_right.apply_axes(
			mapping.right_axis_x, mapping.right_axis_y,
			mapping.right_invert_x, mapping.right_invert_y,
		)
	print("Applied mapping: %s" % mapping.display_name)


## Returns the index of the first mapping whose name_match substring
## occurs in joy_name, or -1 if none match.
func _find_mapping_index_for(joy_name: String) -> int:
	var lowered := joy_name.to_lower()
	for i in mappings.size():
		var m := mappings[i]
		if m == null or m.name_match == "":
			continue
		if lowered.find(m.name_match.to_lower()) != -1:
			return i
	return -1


# --- Picker UI ---------------------------------------------------------------

func _ensure_picker() -> void:
	_mapping_picker = get_node_or_null("MappingPicker") as OptionButton
	if _mapping_picker != null:
		return

	# Scene didn't include one — create a minimal picker at runtime so the
	# feature works without requiring a .tscn edit.
	_mapping_picker = OptionButton.new()
	_mapping_picker.name = "MappingPicker"
	_mapping_picker.position = Vector2(6, 48)
	_mapping_picker.custom_minimum_size = Vector2(260, 28)
	add_child(_mapping_picker)


func _populate_picker() -> void:
	_mapping_picker.clear()
	for m in mappings:
		if m == null:
			continue
		_mapping_picker.add_item(m.display_name)

	# Connect the selection signal exactly once.
	if not _mapping_picker.item_selected.is_connected(_on_mapping_selected):
		_mapping_picker.item_selected.connect(_on_mapping_selected)

	# Apply the first entry by default, if we have any.
	if mappings.size() > 0 and mappings[0] != null:
		_mapping_picker.select(0)
		_apply_mapping(mappings[0])


# --- Controller name resolution ---------------------------------------------

## Returns the best available name for a joypad device.
##
## Godot 4.4 on Windows uses the native XInput driver, which does not expose
## a human-readable product name — Input.get_joy_name() is empty and
## Input.get_joy_info() only returns {"xinput_index": N}. XInput itself
## (the Windows API) does not provide a per-device product string.
##
## We do the best we can:
##   1. Use get_joy_name() if the platform/driver supplied one.
##   2. If get_joy_info() reports xinput_index, label it as Xbox-compatible.
##   3. If Godot recognises the device via its SDL mapping database,
##      mark it as a known gamepad.
##   4. Otherwise fall back to a generic "Controller N" label.
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
extends Node2D
## Controller Input suite
##
## Listens for joypad connection/disconnection events.
## On connect: displays the controller name and begins axis monitoring.
## On disconnect: resets the display and stops monitoring.
##
## The scene expects:
##   - A RichTextLabel named "ControllerName"
##   - A Control node named "Control" with joystick.gd attached for the stick visual.

@onready var _name_field: RichTextLabel = $ControllerName

## The joystick visual — attach joystick.gd to the Control node in the editor.
@onready var _joystickLeft: Control = $JoystickLeft
@onready var _joystickRight: Control = $JoystickRight

## Device index of the active controller, or -1 when none is connected.
var _active_device: int = -1


func _ready() -> void:
	_name_field.text = "No Controller"

	# Listen for future connect/disconnect events.
	Input.joy_connection_changed.connect(_on_joy_connection_changed)

	# Pick up any controller that was already connected before the scene loaded.
	var already_connected := Input.get_connected_joypads()
	if already_connected.size() > 0:
		_connect_controller(already_connected[0])


func _process(_delta: float) -> void:
	if _active_device == -1:
		return

	# Each joystick visual knows which axes it monitors (set in the inspector).
	if _joystickLeft:
		_joystickLeft.poll(_active_device)
	if _joystickRight:
		_joystickRight.poll(_active_device)


# --- Signal handlers ---------------------------------------------------------

func _on_joy_connection_changed(device: int, connected: bool) -> void:
	if connected:
		# Track the first controller that connects; ignore additional ones for now.
		# Defer by one frame — Godot fires this signal before the device name is
		# fully registered, so get_joy_name() returns "" if called immediately.
		if _active_device == -1:
			call_deferred("_connect_controller", device)
	else:
		if device == _active_device:
			_disconnect_controller()


# --- Helpers -----------------------------------------------------------------

func _connect_controller(device: int) -> void:
	_active_device = device
	_name_field.text = "NAME: %s" % _resolve_joy_name(device)
	print("Controller connected [%d]: %s" % [device, _resolve_joy_name(device)])


## Returns the best available name for a joypad device.
##
## Godot 4.4 on Windows uses the native XInput driver, which does not expose
## a human-readable product name — Input.get_joy_name() is empty and
## Input.get_joy_info() only returns {"xinput_index": N}. XInput itself
## (the Windows API) does not provide a per-device product string.
##
## We do the best we can:
##   1. Use get_joy_name() if the platform/driver supplied one.
##   2. If get_joy_info() reports xinput_index, we know it is an XInput
##      (Xbox-compatible) pad — label it accordingly.
##   3. If Godot recognises the device via its SDL mapping database, mark it
##      as a known gamepad.
##   4. Otherwise fall back to a generic "Controller N" label.
func _resolve_joy_name(device: int) -> String:
	var builtin := Input.get_joy_name(device)
	if builtin != "":
		return builtin

	var info := Input.get_joy_info(device)
	if info.has("xinput_index"):
		# XInput-compatible = Xbox 360 / Xbox One / Xbox Series / XInput clone.
		return "Xbox-compatible Controller (XInput #%d)" % info["xinput_index"]

	if Input.is_joy_known(device):
		return "Known Gamepad %d" % device

	return "Controller %d" % device


func _disconnect_controller() -> void:
	print("Controller disconnected [%d]" % _active_device)
	_active_device = -1
	_name_field.text = "No Controller"

	# Return both stick visuals to centre.
	if _joystickLeft:
		_joystickLeft.poll(-1)
	if _joystickRight:
		_joystickRight.poll(-1)
