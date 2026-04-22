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

	# Read the left stick axes each frame and forward them to the visual.
	var axis_x := Input.get_joy_axis(_active_device, JOY_AXIS_LEFT_X)
	var axis_y := Input.get_joy_axis(_active_device, JOY_AXIS_LEFT_Y)

	if _joystickLeft and _joystickLeft.has_method("set_axes"):
		_joystickLeft.set_axes(axis_x, axis_y)
	
	var axis2_x = Input.get_joy_axis(_active_device, JOY_AXIS_RIGHT_X)
	var axis2_y = Input.get_joy_axis(_active_device, JOY_AXIS_RIGHT_Y)

	if _joystickRight and _joystickRight.has_method("set_axes"):
		_joystickRight.set_axes(axis2_x, axis2_y)


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
## Godot 4 leaves get_joy_name() empty for XInput controllers on Windows,
## so we derive a label from get_joy_info()'s xinput_index instead.
func _resolve_joy_name(device: int) -> String:
	var builtin := Input.get_joy_name(device)
	if builtin != "":
		return builtin

	var info := Input.get_joy_info(device)
	if info.has("xinput_index"):
		return "XInput Controller %d" % info["xinput_index"]

	# Last resort — at least show something useful.
	return "Controller %d" % device


func _disconnect_controller() -> void:
	print("Controller disconnected [%d]" % _active_device)
	_active_device = -1
	_name_field.text = "No Controller"

	# Return the stick visual to centre.
	if _joystickLeft:
		_joystickLeft.set_axes(0.0, 0.0)
	if _joystickRight:
		_joystickRight.set_axes(0.0, 0.0)
