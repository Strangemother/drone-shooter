extends RayCast


# Declare member variables here. Examples:
# var a = 2
# var b = "text"
export var distance_vector:Vector3
export var distance:float

onready var indicator:Node = get_node('./RayIndicator')
#onready var padindicator:Node = get_node('./PressureIndicator')

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	var orB:Vector3 = get_global_transform().origin

	if is_colliding():
#		var obj = get_collider()
		var orA:Vector3 = get_collision_point()
		distance_vector = orB - orA
		distance = orB.y - orA.y
		#var normal = get_collision_normal()
		indicator.translation.y = -distance
	else:
		indicator.translation.y = cast_to.y

