extends Position3D


func _physics_process(delta):
	if Input.is_action_pressed("rotate"):
		rotation_degrees += Vector3(0,0,1)
	if Input.is_action_pressed("rotate_n"):
		rotation_degrees += Vector3(0,0,-1)
