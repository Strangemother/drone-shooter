extends Node2D

var camera: Camera3D
var world_point: Vector3
var float_height: float = 0.44
var lifetime: float = 0.5 # seconds
var age: float = 0.0
var targetDamamge:float = 0.0
var presentedDamage:int = 0
#@onready var label = $Label

var display_damage: float = 0.0
var targetDamage: float = 0.0
#var ease_speed := 10.0
#var display_damage: float = 0.0
var target_damage: float = 0.0
var ease_speed: float = 3.0
var overshoot_scale: float = 1.01
var base_speed := 10.0
var start_color = Color(1.0, 0.482, 0.0)
var end_color = Color(1.0, 0.882, 0.30)

# The original hit.
var colliderPoint: Vector3 

func setup(p_camera: Camera3D, p_world_point: Vector3, p_normal: Vector3, damage: float) -> void:
	"""Called by HUDScript.shootManager_bullet_hit
	"""
	# store globals
	camera = p_camera
	world_point = p_world_point + p_normal * (0.05 + (randf() - 0.2))
	colliderPoint = p_world_point
	target_damage = damage * (calculate_distance(p_world_point) * .1)
	display_damage = 0.0
	
	# update initial display
	var lc = clamp(log(target_damage + 1.0), 0.0, 5.0)
	lifetime += randf() + ((lc+lc) * .3)
	update_label(0.0)
	$Label.add_theme_color_override("default_color", Color.RED)

func calculate_distance(world_point):
	
	#var camera : Camera3D = %Camera
	var span = camera.global_position.distance_to(world_point)
	return span

func update_color() -> void:
	var t = clamp(age / (lifetime * .3), 0.0, 1.4)
	if t > .1:
		var color = start_color.lerp(end_color, t - .2)
		$Label.add_theme_color_override("default_color", color)
	
	
func update_label(delta: float) -> void:
	var _scale: float = clamp(target_damage / 100.0, 0.5, 3.0)
	var speed: float = base_speed / _scale
	var overshoot_target: float = target_damage * overshoot_scale

	if display_damage < target_damage:
		display_damage = lerpf(display_damage, overshoot_target, 1.0 - exp(-speed * delta))
	else:
		display_damage = lerpf(display_damage, target_damage, 1.0 - exp(-(speed * 0.7) * delta))

	if abs(display_damage - target_damage) < 0.1:
		display_damage = target_damage
	
	#var default_yellow = Color(1.0, 0.482, 0.0)
	update_color()
	$Label.text = "+" + str(int(round(display_damage)))
	
func _process(delta: float) -> void:
	if camera == null:
		queue_free()
		return

	age += delta
	update_label(delta)
	# drift upward in world space
	var point := world_point + Vector3.UP * (float_height + age * 0.25)

	if camera.is_position_behind(point):
		visible = false
	else:
		visible = true
		position = camera.unproject_position(point)

	if age >= lifetime:
		queue_free()
