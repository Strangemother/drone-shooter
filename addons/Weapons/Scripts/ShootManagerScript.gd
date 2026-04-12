extends Node3D

signal bullet_hit
signal bullet_miss

const DEBUG_BULLET_ARC_COLOR = Color.CYAN

var cW #current weapon
var pointOfCollision : Vector3 = Vector3.ZERO
var rng : RandomNumberGenerator

@export_group("Debug")
@export var debug_draw_bullet_arc: bool = false
@export_range(0.05, 2.0, 0.05) var debug_bullet_arc_duration: float = 0.15

@onready var weaponManager : Node3D = %WeaponManager #weapon manager

func getCurrentWeapon(currWeap):
	#get current weapon resources
	cW = currWeap
	
func shoot():
	if !cW.isShooting and (
		# magazine isn't empty, and has >= ammo than the number of projectiles required for a shot
		(cW.totalAmmoInMag > 0 
			and cW.totalAmmoInMag >= cW.nbProjShotsAtSameTime
		)
		#has all ammos in the magazine, and number of ammo is positive
		or (cW.allAmmoInMag 
			and weaponManager.ammoManager.ammoDict[cW.ammoType] > 0 
			#has >= ammo than the number of projectiles required for a shot
			and weaponManager.ammoManager.ammoDict[cW.ammoType] >= cW.nbProjShotsAtSameTime)
	) and !cW.isReloading:
		cW.isShooting = true
		
		#number of successive shots (for example if 3, the weapon will shot 3 times in a row)
		for i in range(cW.nbProjShots):
			#same conditions has before, are checked before every shot
			if (
				(cW.totalAmmoInMag > 0 
					and cW.totalAmmoInMag >= cW.nbProjShotsAtSameTime) 
				or (cW.allAmmoInMag 
					and weaponManager.ammoManager.ammoDict[cW.ammoType] > 0
				) and  weaponManager.ammoManager.ammoDict[cW.ammoType] >= cW.nbProjShotsAtSameTime
			):	
				weaponManager.weaponSoundManagement(cW.shootSound, cW.shootSoundSpeed)
				
				if cW.shootAnimName != "":
					weaponManager.animManager.playAnimation("ShootAnim%s" % cW.weaponName, cW.shootAnimSpeed, true)
				else:
					print("%s doesn't have a shoot animation" % cW.weaponName)
					
				#number projectiles shots at the same time (for example, 
				#a shotgun shell is constituted of ~ 20 pellets that are spread across the target, 
				#so 20 projectiles shots at the same time)
				for j in range(0, cW.nbProjShotsAtSameTime):
					if cW.allAmmoInMag: weaponManager.ammoManager.ammoDict[cW.ammoType] -= 1
					else: cW.totalAmmoInMag -= 1
						
					#get the collision point
					pointOfCollision = getCameraPOV()
					
					var hit:bool = false 
					#call the fonction corresponding to the selected type
					if cW.type == cW.types.HITSCAN: 
						hit = hitscanShot(pointOfCollision)
					elif cW.type == cW.types.PROJECTILE: 
						projectileShot(pointOfCollision)
					
					if hit:
						print('Upper HIT!')
					else:
						print('Upper MISS')
				if cW.showMuzzleFlash: 
					weaponManager.displayMuzzleFlash()
				
				weaponManager.cameraRecoilHolder.setRecoilValues(cW.baseRotSpeed, cW.targetRotSpeed)
				weaponManager.cameraRecoilHolder.addRecoil(cW.recoilVal)
				
				await get_tree().create_timer(cW.timeBetweenShots).timeout
				
			else:
				print("Not enough ammunitions to shoot")
				
		cW.isShooting = false
		
func getCameraPOV():  
	var camera : Camera3D = %Camera
	var window : Window = get_window()
	var viewport : Vector2i
	
	#match viewport to window size, to ensure that the raycast goes in the right direction
	match window.content_scale_mode:
		window.CONTENT_SCALE_MODE_VIEWPORT:
			viewport = window.content_scale_size
		window.CONTENT_SCALE_MODE_CANVAS_ITEMS:
			viewport = window.content_scale_size
		window.CONTENT_SCALE_MODE_DISABLED:
			viewport = window.get_size()
			
	#Start raycast in camera position, and launch it in camera direction 
	var raycastStart = camera.project_ray_origin(viewport/2)
	var raycastEnd
	if cW.type == cW.types.HITSCAN: raycastEnd = raycastStart + camera.project_ray_normal(viewport/2) * cW.maxRange 
	if cW.type == cW.types.PROJECTILE: raycastEnd = raycastStart + camera.project_ray_normal(viewport/2) * 280
	
	#Create intersection space to contain possible collisions 
	var newIntersection = PhysicsRayQueryParameters3D.create(raycastStart, raycastEnd)
	var intersection = get_world_3d().direct_space_state.intersect_ray(newIntersection)
	
	#If the raycast has collide with something, return collision point transform properties
	if !intersection.is_empty():
		var collisionPoint = intersection.position
		return collisionPoint 
	#Else, return the end of the raycast (so nothing, because he hasn't collide with anything) 
	else:
		return raycastEnd 
		
func hitscanShotOrig(pointOfCollisionHitscan : Vector3):
	rng = RandomNumberGenerator.new()
	
	#set up weapon shot sprad 
	var spread = Vector3(
			rng.randf_range(cW.minSpread, cW.maxSpread), 
			rng.randf_range(cW.minSpread, cW.maxSpread), 
			rng.randf_range(cW.minSpread, cW.maxSpread)
		)
	
	#calculate direction of the hitscan bullet 
	var hitscanBulletDirection = (
			pointOfCollisionHitscan 
			- cW.weaponSlot.attackPoint.get_global_transform().origin
		).normalized()
	
	#create new intersection space to contain possibe collisions 
	var newIntersection = PhysicsRayQueryParameters3D.create(
			cW.weaponSlot.attackPoint.get_global_transform().origin, 
			pointOfCollisionHitscan + spread + hitscanBulletDirection * 2
		)
		
	newIntersection.collide_with_areas = true
	newIntersection.collide_with_bodies = true 
	var hitscanBulletCollision = get_world_3d().direct_space_state.intersect_ray(newIntersection)
	
	#if the raycast has collide
	if not hitscanBulletCollision: 
		print('Very MISS')
		bullet_miss.emit(null, null) # no collision point
		return false
		
	var collider = hitscanBulletCollision.collider
	var colliderPoint = hitscanBulletCollision.position
	var colliderNormal = hitscanBulletCollision.normal 
	var finalDamage : int
		
	if not collider.has_method("hitscanHit"):
		weaponManager.displayBulletHole(colliderPoint, colliderNormal)
		print('MISS')
		bullet_miss.emit(colliderPoint, colliderNormal)
		return false
		
	var maxDamage = cW.damagePerProj
	var sample = cW.damageDropoff.sample(pointOfCollisionHitscan.distance_to(global_position) / cW.maxRange)
	var groupName : String = 'urm'
	
	if collider.is_in_group("Enemies"):
		groupName = "Body"
		finalDamage = maxDamage * sample
		collider.hitscanHit(finalDamage, hitscanBulletDirection, hitscanBulletCollision.position)
		#print('bodyhit: ', finalDamage, ' accuracy: ', sample, ' ', maxDamage)
		#bullet_hit.emit(finalDamage, sample, maxDamage)
	elif collider.is_in_group("EnemiesHead"):
		groupName = "Head"
		maxDamage = cW.damagePerProj * cW.headshotDamageMult 
		finalDamage = maxDamage * sample
		collider.hitscanHit(finalDamage, hitscanBulletDirection, hitscanBulletCollision.position)
		#print('headshot: ', finalDamage, ' accuracy: ', sample, ' ', maxDamage)
		#bullet_hit.emit(damage, accuracy, maxDamage)
		#bullet_hit.emit(finalDamage, sample, maxDamage)
	elif collider.is_in_group("HitableObjects"): 
		groupName = "Object"
		finalDamage = maxDamage * sample
		collider.hitscanHit(finalDamage/6.0, hitscanBulletDirection, hitscanBulletCollision.position)
		weaponManager.displayBulletHole(colliderPoint, colliderNormal)
		#print('thing hit: ', finalDamage, ' accuracy: ', sample, ' ', maxDamage)
		#bullet_hit.emit(finalDamage, sample, maxDamage)
	else:
		weaponManager.displayBulletHole(colliderPoint, colliderNormal)
	
	bullet_hit.emit(colliderPoint, colliderNormal, groupName, finalDamage, sample, maxDamage)
	return true


func hitscanShot(pointOfCollisionHitscan: Vector3):
	rng = RandomNumberGenerator.new()

	var origin: Vector3 = cW.weaponSlot.attackPoint.global_transform.origin
	var debug_points = null
	if debug_draw_bullet_arc:
		debug_points = []

	var spread: Vector3 = Vector3(
		rng.randf_range(cW.minSpread, cW.maxSpread),
		rng.randf_range(cW.minSpread, cW.maxSpread),
		rng.randf_range(cW.minSpread, cW.maxSpread)
	)

	var aimed_point: Vector3 = pointOfCollisionHitscan + spread
	var initial_direction: Vector3 = (aimed_point - origin).normalized()
	var max_distance: float = cW.maxRange

	var hitscanBulletCollision: Dictionary = intersect_bullet_arc(
		origin,
		initial_direction,
		max_distance,
		300, # cW.bulletSpeed,
		20, # cW.bulletGravity,
		Vector3(0,2,-1.0), # cW.bulletBend,
		2.0,
		debug_points,
	)

	if debug_points != null:
		draw_debug_bullet_arc(debug_points)

	if hitscanBulletCollision.is_empty():
		print("Very MISS")
		bullet_miss.emit(null, null)
		return false

	var collider = hitscanBulletCollision.collider
	var colliderPoint: Vector3 = hitscanBulletCollision.position
	var colliderNormal: Vector3 = hitscanBulletCollision.normal
	var hitscanBulletDirection: Vector3 = hitscanBulletCollision["bullet_direction"]
	var finalDamage: int

	if not collider.has_method("hitscanHit"):
		weaponManager.displayBulletHole(colliderPoint, colliderNormal)
		print("MISS")
		bullet_miss.emit(colliderPoint, colliderNormal)
		return false

	var maxDamage = cW.damagePerProj
	var shot_distance: float = origin.distance_to(colliderPoint)
	var sample = cW.damageDropoff.sample(shot_distance / cW.maxRange)
	var groupName: String = "urm"

	if collider.is_in_group("Enemies"):
		groupName = "Body"
		finalDamage = maxDamage * sample
		collider.hitscanHit(finalDamage, hitscanBulletDirection, colliderPoint)

	elif collider.is_in_group("EnemiesHead"):
		groupName = "Head"
		maxDamage = cW.damagePerProj * cW.headshotDamageMult
		finalDamage = maxDamage * sample
		collider.hitscanHit(finalDamage, hitscanBulletDirection, colliderPoint)

	elif collider.is_in_group("HitableObjects"):
		groupName = "Object"
		finalDamage = maxDamage * sample
		collider.hitscanHit(finalDamage / 6.0, hitscanBulletDirection, colliderPoint)
		weaponManager.displayBulletHole(colliderPoint, colliderNormal)

	else:
		weaponManager.displayBulletHole(colliderPoint, colliderNormal)

	bullet_hit.emit(colliderPoint, colliderNormal, groupName, finalDamage, sample, maxDamage)
	return true
	

func apply_bullet_bend(velocity: Vector3, bend_rate: Vector3, roll_angle: float, delta: float) -> Vector3:
	var forward: Vector3 = velocity.normalized()

	if forward.length_squared() <= 0.000001:
		return velocity

	var ref_up: Vector3 = Vector3.UP
	if abs(forward.dot(ref_up)) > 0.98:
		ref_up = Vector3.RIGHT

	var right: Vector3 = forward.cross(ref_up).normalized()
	var up: Vector3 = right.cross(forward).normalized()

	if roll_angle != 0.0:
		right = right.rotated(forward, roll_angle)
		up = up.rotated(forward, roll_angle)

	var bent: Vector3 = forward

	# local pitch
	if bend_rate.x != 0.0:
		bent = bent.rotated(right, bend_rate.x * delta)

	# local yaw
	if bend_rate.y != 0.0:
		bent = bent.rotated(up, bend_rate.y * delta)

	return bent.normalized() * velocity.length()


func draw_debug_bullet_arc(points: Array):
	if points == null or points.size() < 2:
		return

	var line_mesh := ImmediateMesh.new()
	var line_material := StandardMaterial3D.new()
	line_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	line_material.albedo_color = DEBUG_BULLET_ARC_COLOR
	line_material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	line_material.no_depth_test = true

	line_mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP, line_material)
	for point in points:
		line_mesh.surface_add_vertex(point)
	line_mesh.surface_end()

	var line_instance := MeshInstance3D.new()
	line_instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	line_instance.mesh = line_mesh
	get_tree().get_root().add_child(line_instance)

	var cleanup_timer := get_tree().create_timer(debug_bullet_arc_duration)
	cleanup_timer.timeout.connect(line_instance.queue_free)


func intersect_bullet_arcOrig(
	origin: Vector3,
	initial_direction: Vector3,
	max_distance: float,
	bullet_speed: float,
	gravity: float,
	segment_length: float = 2.0
) -> Dictionary:
	var space_state = get_world_3d().direct_space_state

	var travelled: float = 0.0
	var current_pos: Vector3 = origin
	var velocity: Vector3 = initial_direction.normalized() * bullet_speed

	while travelled < max_distance:
		var dt: float = segment_length / bullet_speed
		var next_velocity: Vector3 = velocity + Vector3.DOWN * gravity * dt
		var next_pos: Vector3 = current_pos + velocity * dt

		var query := PhysicsRayQueryParameters3D.create(current_pos, next_pos)
		query.collide_with_areas = true
		query.collide_with_bodies = true

		var hit: Dictionary = space_state.intersect_ray(query)
		if not hit.is_empty():
			hit["bullet_direction"] = velocity.normalized()
			return hit

		travelled += current_pos.distance_to(next_pos)
		current_pos = next_pos
		velocity = next_velocity

	return {}


func intersect_bullet_arc(
	origin: Vector3,
	initial_direction: Vector3,
	max_distance: float,
	bullet_speed: float,
	gravity: float,
	bend_rate: Vector3,
	segment_length: float = 2.0,
	debug_points = null
) -> Dictionary:
	var space_state = get_world_3d().direct_space_state

	var travelled: float = 0.0
	var current_pos: Vector3 = origin
	var velocity: Vector3 = initial_direction.normalized() * bullet_speed
	var roll_angle: float = 0.0
	if debug_points != null:
		debug_points.append(current_pos)

	while travelled < max_distance:
		var dt: float = segment_length / bullet_speed
		roll_angle += bend_rate.z * dt

		# first apply bend
		velocity = apply_bullet_bend(velocity, bend_rate, roll_angle, dt)

		# then gravity
		velocity += Vector3.DOWN * gravity * dt

		var next_pos: Vector3 = current_pos + velocity * dt

		var query := PhysicsRayQueryParameters3D.create(current_pos, next_pos)
		query.collide_with_areas = true
		query.collide_with_bodies = true

		var hit: Dictionary = space_state.intersect_ray(query)
		if not hit.is_empty():
			if debug_points != null:
				debug_points.append(hit.position)
			hit["bullet_direction"] = velocity.normalized()
			return hit

		if debug_points != null:
			debug_points.append(next_pos)

		travelled += current_pos.distance_to(next_pos)
		current_pos = next_pos

	return {}	 
	
	
func projectileShot(pointOfCollisionProjectile : Vector3):
	rng = RandomNumberGenerator.new()
	
	#set up weapon shot sprad 
	var spread = Vector3(rng.randf_range(cW.minSpread, cW.maxSpread), rng.randf_range(cW.minSpread, cW.maxSpread), rng.randf_range(cW.minSpread, cW.maxSpread))
	
	#Calculate direction of the projectile
	var projectileDirection = ((pointOfCollisionProjectile - cW.weaponSlot.attackPoint.get_global_transform().origin).normalized() + spread)
	
	#Instantiate projectile
	var projInstance = cW.projRef.instantiate()
	
	#set projectile properties 
	projInstance.global_transform = cW.weaponSlot.attackPoint.global_transform
	projInstance.direction = projectileDirection
	projInstance.damage = cW.damagePerProj
	projInstance.timeBeforeVanish = cW.projTimeBeforeVanish
	projInstance.gravity_scale = cW.projGravityVal
	projInstance.isExplosive = cW.isProjExplosive
	
	get_tree().get_root().add_child(projInstance)
	
	projInstance.set_linear_velocity(projectileDirection * cW.projMoveSpeed)
