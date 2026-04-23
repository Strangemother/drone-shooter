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

# --- Triggers ----------------------------------------------------------------

@export_group("Triggers")

## Axis index for the left trigger, or -1 to disable.
## On Xbox/PlayStation controllers this is JOY_AXIS_TRIGGER_LEFT (4).
@export var trigger_left_axis: int = -1

## Axis index for the right trigger, or -1 to disable.
## On Xbox/PlayStation controllers this is JOY_AXIS_TRIGGER_RIGHT (5).
@export var trigger_right_axis: int = -1

## When true, the right trigger drives throttle and the left stick Y axis
## is treated as yaw-only (its throttle write is suppressed in
## FlightControllerBase).  Leave false for controllers that use the
## left stick for throttle as normal.
@export var use_trigger_for_throttle: bool = false
