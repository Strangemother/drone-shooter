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
@export var axis_x_index: int = JOY_AXIS_LEFT_X
## Which joypad axis drives vertical motion (positive = down, matches screen).
@export var axis_y_index: int = JOY_AXIS_LEFT_Y
## Invert the X axis if your hardware reports it reversed.
@export var invert_x: bool = false
## Invert the Y axis if your hardware reports it reversed.
@export var invert_y: bool = false

@onready var _background: ColorRect = $Background
@onready var _dot: ColorRect = $Indicator


## Poll the configured axes from the given device and update the indicator.
## Pass a device of -1 to centre the indicator (no controller connected).
func poll(device: int) -> void:
	if device == -1:
		set_axes(0.0, 0.0)
		return

	var x := Input.get_joy_axis(device, axis_x_index)
	var y := Input.get_joy_axis(device, axis_y_index)
	if invert_x:
		x = -x
	if invert_y:
		y = -y
	set_axes(x, y)


## Move the dot to reflect the given axis values in [-1, 1] range.
func set_axes(x: float, y: float) -> void:
	var input := Vector2(x, y)

	# Circular dead zone — cancel small stick drift.
	if input.length() < DEAD_ZONE:
		input = Vector2.ZERO

	# Centre position of the dot when axes are at zero.
	# max_travel is how far the dot's top-left corner can move from centre
	# in each direction while keeping the dot fully inside the background.
	var center_pos := (_background.size - _dot.size) / 2.0
	var max_travel := center_pos

	_dot.position = center_pos + input * max_travel
