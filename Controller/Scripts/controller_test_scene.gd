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
## in the inspector — the picker UI is built from this list. If left empty,
## every ControllerMapping in `mappings_dir` is loaded automatically.
@export var mappings: Array[ControllerMapping] = []

## Directory scanned for ControllerMapping .tres files when `mappings` is empty.
@export_dir var mappings_dir: String = "res://Controller/Mappings"

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
	if mappings.is_empty():
		_auto_load_mappings()
	_populate_picker()

	Input.joy_connection_changed.connect(_on_joy_connection_changed)

	var already_connected := Input.get_connected_joypads()
	if already_connected.size() > 0:
		_connect_controller(already_connected[0])


func _process(_delta: float) -> void:
	if _active_device == -1:
		return

	if _joystick_left:
		_joystick_left.poll(_active_device)
	if _joystick_right:
		_joystick_right.poll(_active_device)


# --- Signal handlers ---------------------------------------------------------

func _on_joy_connection_changed(device: int, connected: bool) -> void:
	if connected:
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

	var auto_index := _find_mapping_index_for(joy_name)
	if auto_index != -1:
		_mapping_picker.select(auto_index)
		_apply_mapping(mappings[auto_index])


func _disconnect_controller() -> void:
	print("Controller disconnected [%d]" % _active_device)
	_active_device = -1
	_name_field.text = "No Controller"

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


## Load every ControllerMapping resource found directly inside mappings_dir.
## Used when the inspector-exposed `mappings` array is left empty.
func _auto_load_mappings() -> void:
	var dir := DirAccess.open(mappings_dir)
	if dir == null:
		push_warning("ControllerMapping dir not found: %s" % mappings_dir)
		return

	dir.list_dir_begin()
	var file := dir.get_next()
	while file != "":
		# Handle both uncompiled (.tres) and compiled-export (.res) forms.
		if not dir.current_is_dir() and (file.ends_with(".tres") or file.ends_with(".res")):
			var path := "%s/%s" % [mappings_dir, file]
			var res := load(path)
			if res is ControllerMapping:
				mappings.append(res)
			else:
				push_warning("Skipping non-ControllerMapping resource: %s" % path)
		file = dir.get_next()
	dir.list_dir_end()

	# Stable order so the picker is predictable across runs.
	mappings.sort_custom(func(a, b): return a.display_name < b.display_name)
	print("Auto-loaded %d controller mapping(s) from %s" % [mappings.size(), mappings_dir])


# --- Picker UI ---------------------------------------------------------------

func _ensure_picker() -> void:
	_mapping_picker = get_node_or_null("MappingPicker") as OptionButton
	if _mapping_picker != null:
		return

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

	if not _mapping_picker.item_selected.is_connected(_on_mapping_selected):
		_mapping_picker.item_selected.connect(_on_mapping_selected)

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
