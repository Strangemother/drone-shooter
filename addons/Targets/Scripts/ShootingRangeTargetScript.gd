extends CharacterBody3D

class_name ShootingRangeTarget

@export var health : float = 100.0
@export_range(0.0, 1.0, 0.01) var bullet_density : float = 0.3
var healthRef : float
var isDisabled : bool = false

@onready var animManager : AnimationPlayer = $AnimationPlayer

func _ready():
	healthRef = health
	animManager.play("idle")

func get_bullet_density() -> float:
	return bullet_density
	
func hitscanHit(damageVal : float, _hitscanDir : Vector3, _hitscanPos : Vector3):
	health -= damageVal
	
	print("Hitscan hit, damage : ", damageVal, ', target health: ', health)
	
	if health <= 0.0 and !isDisabled:
		isDisabled = true
		animManager.play("fall")
		
func projectileHit(damageVal : float, _hitscanDir : Vector3):
	health -= damageVal
	
	print("Projectile hit, target health : ", health)
	
	if health <= 0.0 and !isDisabled:
		isDisabled = true
		animManager.play("fall")
		
		
		
		
		
		
		
		
		
