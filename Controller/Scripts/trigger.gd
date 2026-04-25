extends Control
## Joystick stick visual.
##
## Attach to a Control node that has:
##   - A child ColorRect named "Background" — the flat background pad.
##   - A child ColorRect named "Indicator"  — the movable dot indicator.
##
## Axis indices are exposed so you can wire this to any pair of joypad axes.
## On an Xbox-style pad the constants happen to line up; on generic devices
## (e.g. a TX16 radio transmitter) the raw axis numbers rarely match the
## JOY_AXIS_LEFT/RIGHT role constants — so just set them per instance in
## the inspector.

## Inputs with a magnitude below this threshold are snapped to zero.
const DEAD_ZONE := 0.1

## Which joypad axis drives horizontal motion (positive = right).
@export var axis_index: int = JOY_AXIS_LEFT_X
## Which joypad axis drives vertical motion (positive = down, matches screen).
#@export var axis_y_index: int = JOY_AXIS_LEFT_Y
## Invert the X axis if your hardware reports it reversed.
@export var invert: bool = false

@onready var _background: ColorRect = $Background
@onready var _dot: ColorRect = $Background/Indicator

signal axis_value_change(axis, input, output)


func _on_ready() -> void:
	$SpinBox.value = axis_index
	$InvertCheckBox.button_pressed = invert


func _on_spin_box_value_changed(value: float) -> void:
	axis_index = value

	
## Assign axis indices and inversion flags at runtime, typically from a
## ControllerMapping resource selected in the controller test scene.
func apply_axes(x_index: int, inv: bool = false) -> void:
	axis_index = x_index
	invert = inv

## Poll the configured axes from the given device and update the indicator.
## Pass a device of -1 to centre the indicator (no controller connected).
func poll(device: int) -> void:
	if device == -1:
		print('No device')
		set_axes(0.0, 0.0)
		return

	var x := Input.get_joy_axis(device, axis_index)
	if invert:
		x = -x
	set_value(x)

func set_value(axis_value):
	current_axis.y = -axis_value
	present_vector(current_axis)
	
var current_axis: Vector2 = Vector2.ZERO

## Move the dot to reflect the given axis values in [-1, 1] range.
func set_axes(x: float, y: float) -> void:
	current_axis = Vector2(x, y)
	present_vector(current_axis)

func present_vector(input: Vector2):
	# Centre position of the dot when axes are at zero.
	# max_travel is how far the dot's top-left corner can move from centre
	# in each direction while keeping the dot fully inside the background.
	var center_pos := (_background.size - _dot.size) 
	var max_travel := _background.size
	
	var output 
	var trigger_value = abs(input.y)
	if invert:
		max_travel *= -1
		trigger_value = 1 - trigger_value
		output = input * max_travel
		_dot.position = output
	else:
		output = input * max_travel
		_dot.position = center_pos + output
	axis_value_change.emit(axis_index, input, trigger_value)


func _on_invert_check_box_toggled(toggled_on: bool) -> void:
	invert = toggled_on
	present_vector(current_axis)
