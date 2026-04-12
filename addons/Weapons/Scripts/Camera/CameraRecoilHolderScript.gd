extends Node3D

#Camera recoil variables
var currentRotation : Vector3
var targetRotation : Vector3 
var baseRotationSpeed : float
var targetRotationSpeed : float 

func _process(delta):
	handleRecoil(delta)
	breathSway(delta)
	
	
var breathStoreX : float
var breathStoreY : float

var speedX = .1
var widthX = .018

var speedY = .095
var widthY = .02 

func _input(event):
	if event.as_text() == 'Shift':
		
		if event.is_pressed():
			speedX = 0.01
			speedY = 0.01
			widthX = 0.03
			widthY = 0.03 
		else:
			speedX = 0.1
			widthX = .018
			speedY = .035
			widthY = .02
			
func breathSway(delta):
	
	breathStoreX += delta
	var aX = (1 + cos(2 * PI * speedX * breathStoreX + speedX)) * widthX
	var bX = (1.6 + cos(5 * PI * speedX * breathStoreX + speedX)) * widthX
	
	breathStoreX += delta
	var aY = (1 + sin(2 * PI * speedY * breathStoreX + speedY)) * widthY
	var bY = (1.6 + sin(5 * PI * speedY * breathStoreX + speedY)) * widthY
	
	rotation.x += (aX + bX) * .5
	rotation.y += (aY + bY) * .5
	

func handleRecoil(delta):
	#first phase, the camera will aim according the recoil values
	#second phase, the camera back down to her initial rotation value
	targetRotation = lerp(targetRotation, Vector3.ZERO, baseRotationSpeed * delta)
	currentRotation = lerp(currentRotation, targetRotation, targetRotationSpeed * delta)
	
	rotation = currentRotation

func setRecoilValues(baseRotSpeed : float, targRotSpeed : int):
	baseRotationSpeed = baseRotSpeed
	targetRotationSpeed = targRotSpeed
	
func addRecoil(recoilValue):
	targetRotation += Vector3(recoilValue.x, randf_range(-recoilValue.y, recoilValue.y), randf_range(-recoilValue.z, recoilValue.z))
