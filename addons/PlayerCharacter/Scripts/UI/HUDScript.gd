extends CanvasLayer

@onready var currStateLabelText = %CurrStateLabelText
@onready var currDirLabelText = %CurrDirectionLabelText
@onready var desiredMoveSpeedLabelText = %DesiredMoveSpeedLabelText
@onready var velocityLabelText = %VelocityLabelText
@onready var nbJumpsInAirAllowedLabelText = %NbJumpsInAirAllowedLabelText

@onready var weaponStackLabelText = %WeaponStackLabelText
@onready var weaponNameLabelText = %WeaponNameLabelText
@onready var totalAmmoInMagLabelText = %TotalAmmoInMagLabelText
@onready var totalAmmoLabelText = %TotalAmmoLabelText

@onready var shootManager = %ShootManager


const HitTextScene = preload("res://hud/scenes/hit_text.tscn")

func _ready() -> void:
	shootManager.bullet_hit.connect(shootManager_bullet_hit)
	shootManager.bullet_miss.connect(shootManager_bullet_miss)

func shootManager_bullet_hit(colliderPoint, colliderNormal, groupName, finalDamage, sample, maxDamage):
	print('shootManager_bullet_hit')

	var camera : Camera3D = %Camera
	var span = camera.global_position.distance_to(colliderPoint)
	
	print('span ', span)
	print(groupName, ': ', finalDamage, ' accuracy: ', sample, ' ', maxDamage)
	var hitText = HitTextScene.instantiate()
	hitText.setup(
		camera,
		colliderPoint,
		colliderNormal,
		finalDamage
	)
	# optional: set the number text if it has a Label
	#hit_text.get_node("Label").text = str(finalDamage)

	%HitPointsLayer.add_child(hitText)
	
	if not camera.is_position_behind(colliderPoint):
		var cup = camera.unproject_position(colliderPoint)
		hitText.position = cup
	
func shootManager_bullet_miss(colliderPoint, colliderNormal):
	print('shootManager_bullet_miss')

func displayCurrentState(currState : String):
	currStateLabelText.set_text(str(currState))
	
func displayCurrentDirection(currDir : Vector3):
	currDirLabelText.set_text(str(currDir))
	
func displayDesiredMoveSpeed(desMoveSpeed : float):
	desiredMoveSpeedLabelText.set_text(str(desMoveSpeed))
	
func displayVelocity(vel : float):
	velocityLabelText.set_text(str(vel))
	
func displayNbJumpsInAirAllowed(nbJumpsInAirAllowed : int):
	nbJumpsInAirAllowedLabelText.set_text(str(nbJumpsInAirAllowed))
	
#----------------------------------------------------------------------------
	
func displayWeaponStack(weaponStack : int):
	weaponStackLabelText.set_text(str(weaponStack))
	
func displayWeaponName(weaponName : String):
	weaponNameLabelText.set_text(str(weaponName))
	
func displayTotalAmmoInMag(totalAmmoInMag : int, nbProjShotsAtSameTime : int):
	totalAmmoInMagLabelText.set_text(str(totalAmmoInMag/nbProjShotsAtSameTime))
	
func displayTotalAmmo(totalAmmo : int, nbProjShotsAtSameTime : int):
	totalAmmoLabelText.set_text(str(totalAmmo/nbProjShotsAtSameTime))
	
	
	
	
	
