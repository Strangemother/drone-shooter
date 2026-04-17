extends RigidBody3D

@export_range(0.0, 1.0, 0.01) var bullet_density : float = 1.0

@export var anti_gravity : float = 0.20

func _process(delta: float) -> void:
	self.linear_velocity.y += anti_gravity * delta

func get_bullet_density() -> float:
	return bullet_density

func hitscanHit(propulForce : float, propulDir: Vector3, propulPos : Vector3):
	var hitPos : Vector3 = propulPos - global_transform.origin#set the position to launch the object at
	if propulDir != Vector3.ZERO: apply_impulse(propulDir * propulForce, hitPos)
	
func projectileHit(propulForce : float, propulDir: Vector3):
	if propulDir != Vector3.ZERO: apply_central_force((global_transform.origin - propulDir) * propulForce)
	
