extends Node2D

@onready var _name_field: RichTextLabel = $ControllerName
#@onready var _joystick_left: Control = $JoystickLeft
#@onready var _joystick_right: Control = $JoystickRight

var action_event_map: Dictionary = {}
var controller_axis_list: Array = []

signal controller_axis_change(name, output)

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	$fps_label.set_text(str(Engine.get_frames_per_second()))

func _ready() -> void:
	_name_field.text = "No Controller"

	# The controller may have already auto-connected by now; reflect that
	# in our UI.
	var info := Input.get_joy_info(0)
	print('info: ', info)
	_on_controller_connected(info.xinput_index, _name_field_text_for(info))
	
	for _child in $Inputs.get_children():
		add_controller(_child)
		_child.connect('axis_value_change', _on_axis_value_change)
	#add_controller($Inputs/JoystickLeft)
	#add_controller($Inputs/JoystickRight)
	#add_controller($Inputs/TriggerLeft)


func _on_axis_value_change(axis_value, input, output):
	var found = false 
	for _child in $VBoxContainer.get_children():
		if _child.axis == axis_value:
			found = true 
			_child.set_value(output)
			controller_axis_change.emit(_child.label, output)
		if _child.set_mode == true:
			_child.check_set_value(output, axis_value)
			
	#if not found: 
		#print("No index _on_axis_value_change: ", axis_value, ' ', input, ' ', output)
		
	
func add_controller(_item:Control):
	"""Given a UI component, track its axis"""
	controller_axis_list.append(_item)

func _input(event):
	"""
	An Xbox controller will fire axis:

		InputEventJoypadMotion: axis=0, axis_value=0.04
		InputEventJoypadMotion: axis=1, axis_value=0.02
		InputEventJoypadMotion: axis=2, axis_value=-0.03
		InputEventJoypadMotion: axis=3, axis_value=0.02
		InputEventJoypadMotion: axis=4, axis_value=0.00
		InputEventJoypadMotion: axis=5, axis_value=0.00

	"""
	if event is InputEventJoypadMotion:
		# fiddle axis
		var mapping = action_event_map.get(event.axis)
		if mapping == null:
			print('Unknown axis: ', event.axis)
			_track_unknown_axis_from_event(event)
		_present_axis_from_event(event, mapping)
	#else: 
		#print(event)

func _present_axis_from_event(event, mapping):
	"""Given an event and mappings, present the change into the target controller.
	"""
	var axis: int = event.axis
	var found = false
	for _item in controller_axis_list:
		if 'axis_index' in _item and _item.axis_index == axis:
			_item.set_value(event.axis_value)
			found = true 
		if 'axis_x_index' in _item and _item.axis_x_index == axis:
			_item.set_x_value(event.axis_value)
			found = true 
		if 'axis_y_index' in _item and _item.axis_y_index == axis:
			_item.set_y_value(event.axis_value)
			found = true 
	
	if not found: 
		print('Unknown event: ', event)	
		
func _track_unknown_axis_from_event(event: InputEventJoypadMotion):
	"""A new axis event. Add it as a controller and 
	hook onto the active axis."""
	var record: Dictionary = { "axis": event.axis, 
								"value": event.axis_value,
								"device": event.device,
							}
	action_event_map.set(event.axis, record)
	
func _name_field_text_for(_info: Dictionary) -> String:
	var _name = Input.get_joy_name(_info.xinput_index)
	if len(_name) == 0:
		return "blank name"
	return _name
	
func _on_controller_connected(_device: int, joy_name: String) -> void:
	_name_field.text = "NAME: %s" % joy_name


func _on_controller_disconnected(_device: int) -> void:
	_name_field.text = "No Controller"
	#if _joystick_left:
		#_joystick_left.set_axes(0.0, 0.0)

#
#func _on_left_stick(value: Vector2) -> void:
	#if _joystick_left:
		#_joystick_left.set_axes(value.x, value.y)
