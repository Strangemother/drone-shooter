extends HBoxContainer

@export var label = 'Example'
@export var current_value: float = 1.0
@export var axis:int = -1

@export var set_mode = false 

func _ready() -> void:
	$Label.text = label 
	$Axis.value = axis
	set_value(current_value)

func set_value(_value):
	$Value.text = str(_value)
	
func check_set_value(_value, axis_value):
	if abs(_value) > .999:
		axis = axis_value
		set_mode = false
		$Axis.value = axis
	set_value(_value)
	
func _on_set_button_pressed() -> void:
	# Wait on input, map the index.
	set_mode = !set_mode
	
	
func _on_axis_value_changed(value: float) -> void:
	axis = value
