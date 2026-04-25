@tool
extends Node2D

@export var radius := 50.0
@export var color := Color.WHITE

func _draw():
	draw_circle(Vector2.ZERO, radius, color)

func _ready():
	queue_redraw()
	
func _process(_delta):
	if Engine.is_editor_hint():
		queue_redraw()
