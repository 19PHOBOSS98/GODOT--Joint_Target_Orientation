extends MeshInstance


func _physics_process(delta):
	if Input.is_action_pressed("rotate"):
		rotation_degrees += Vector3(0,0,-0.5)
	if Input.is_action_pressed("rotate_n"):
		rotation_degrees += Vector3(0,0,0.5)

