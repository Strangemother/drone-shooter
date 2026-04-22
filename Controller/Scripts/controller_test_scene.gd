extends Node2D

@onready var nameField : RichTextLabel = $ControllerName

func _ready() -> void:
	nameField.text = "No Controller"
# Setup name of pad
# Setup Joysticks
