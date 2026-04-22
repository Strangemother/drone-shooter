extends Control
## Joystick stick visual.
##
## Attach to a Control node that has:
##   - A child ColorRect named "ColorRect"  — the flat background pad.
##   - A grandchild ColorRect named "ColorRect2" — the movable dot indicator.
##
## Call set_axes(x, y) each frame (values in [-1, 1]) to move the dot.

## Inputs with a magnitude below this threshold are snapped to zero.
const DEAD_ZONE := 0.1

@onready var _background: ColorRect = $ColorRect
@onready var _dot: ColorRect = $ColorRect/ColorRect2


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
