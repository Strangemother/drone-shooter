extends Node3D

func _notification(what):
	if what == NOTIFICATION_WM_CLOSE_REQUEST:
		get_tree().quit() # default behavior

func _input(event):
	if event.as_text() == 'Escape':
		get_tree().quit() # default behavior
