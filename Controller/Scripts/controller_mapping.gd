@tool
class_name ControllerMapping
extends Resource
## Describes how a given physical controller's raw joypad axes map onto
## the game's logical "left stick" and "right stick" pair.
##
## Godot's JOY_AXIS_LEFT_* / JOY_AXIS_RIGHT_* constants assume an
## Xbox-style layout (axes 0..3). Other devices — such as a RadioMaster /
## TX16 radio transmitter over USB HID — expose their channels as raw
## axes in arbitrary order. A mapping resource captures that per-device
## translation as editable data rather than hard-coded numbers.
##
## Create new mappings as .tres files:
##   Right-click in FileSystem → New Resource → ControllerMapping.

## Human-readable name, shown in the picker UI.
@export var display_name: String = "Unnamed Controller"

## Optional substring of Input.get_joy_name(device) used to auto-select
## this mapping when a controller is connected. Leave empty to disable.
@export var name_match: String = ""

# --- Left stick --------------------------------------------------------------

@export_group("Left Stick")
@export var left_axis_x: int = JOY_AXIS_LEFT_X
@export var left_axis_y: int = JOY_AXIS_LEFT_Y
@export var left_invert_x: bool = false
@export var left_invert_y: bool = false

# --- Right stick -------------------------------------------------------------

@export_group("Right Stick")
@export var right_axis_x: int = JOY_AXIS_RIGHT_X
@export var right_axis_y: int = JOY_AXIS_RIGHT_Y
@export var right_invert_x: bool = false
@export var right_invert_y: bool = false
