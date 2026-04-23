extends Node2D
## Controller test harness.
##
## This scene demonstrates how to consume a MappedController as a drop-in
## child node. It owns:
##   - A MappedController child (created at runtime if the .tscn does not
##     already include one).
##   - A name label, a mapping picker, and two joystick visuals that are
##     driven purely from the controller's signals.
##
## Copy the pattern below into any other scene that wants gamepad input:
##
##     @export var controller: MappedController
##     func _ready() -> void:
##         controller.left_stick_changed.connect(_on_left)
##         controller.right_stick_changed.connect(_on_right)
##         controller.button_pressed.connect(_on_button)

const MAPPED_CONTROLLER_SCENE := preload("res://Controller/Scenes/MappedController.tscn")

@onready var _name_field: RichTextLabel = $ControllerName
@onready var _joystick_left: Control = $JoystickLeft
@onready var _joystick_right: Control = $JoystickRight

var _controller: MappedController
var _mapping_picker: OptionButton


func _ready() -> void:
	_name_field.text = "No Controller"

	_ensure_controller()
	_ensure_picker()
	_populate_picker()
	_wire_controller_signals()

	# The controller may have already auto-connected by now; reflect that
	# in our UI.
	if _controller.active_device != -1:
		var info := Input.get_joy_info(_controller.active_device)
		_on_controller_connected(_controller.active_device, _name_field_text_for(info))
	if _controller.current_mapping != null:
		_on_mapping_applied(_controller.current_mapping)


# --- Setup -------------------------------------------------------------------

func _ensure_controller() -> void:
	_controller = get_node_or_null("MappedController") as MappedController
	if _controller == null:
		_controller = MAPPED_CONTROLLER_SCENE.instantiate()
		add_child(_controller)


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
	for m in _controller.mappings:
		if m == null:
			continue
		_mapping_picker.add_item(m.display_name)

	if not _mapping_picker.item_selected.is_connected(_on_picker_selected):
		_mapping_picker.item_selected.connect(_on_picker_selected)

	var idx := _controller.get_current_mapping_index()
	if idx != -1:
		_mapping_picker.select(idx)


func _wire_controller_signals() -> void:
	_controller.controller_connected.connect(_on_controller_connected)
	_controller.controller_disconnected.connect(_on_controller_disconnected)
	_controller.mapping_applied.connect(_on_mapping_applied)
	_controller.left_stick_changed.connect(_on_left_stick)
	_controller.right_stick_changed.connect(_on_right_stick)


# --- Signal handlers ---------------------------------------------------------

func _on_controller_connected(_device: int, joy_name: String) -> void:
	_name_field.text = "NAME: %s" % joy_name


func _on_controller_disconnected(_device: int) -> void:
	_name_field.text = "No Controller"
	if _joystick_left:
		_joystick_left.set_axes(0.0, 0.0)
	if _joystick_right:
		_joystick_right.set_axes(0.0, 0.0)


func _on_mapping_applied(_mapping: ControllerMapping) -> void:
	var idx := _controller.get_current_mapping_index()
	if idx != -1 and _mapping_picker.selected != idx:
		_mapping_picker.select(idx)


func _on_left_stick(value: Vector2) -> void:
	if _joystick_left:
		_joystick_left.set_axes(value.x, value.y)


func _on_right_stick(value: Vector2) -> void:
	if _joystick_right:
		_joystick_right.set_axes(value.x, value.y)


func _on_picker_selected(index: int) -> void:
	_controller.select_mapping(index)


# --- Helpers -----------------------------------------------------------------

## Best-effort name rendering for a late-bound device that was already
## connected when the scene came up.
func _name_field_text_for(_info: Dictionary) -> String:
	if _controller.active_device == -1:
		return "No Controller"
	return Input.get_joy_name(_controller.active_device)
