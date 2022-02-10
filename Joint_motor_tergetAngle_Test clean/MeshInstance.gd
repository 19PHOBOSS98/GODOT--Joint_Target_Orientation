extends MeshInstance


# Declare member variables here. Examples:
# var a = 2
# var b = "text"


# Called when the node enters the scene tree for the first time.
func _ready():
	prints("degrot:")
	prints(rotation_degrees)
	prints("degrot txg:")
	prints(to_global(rotation_degrees))
	prints("degrot txl:")
	prints(to_local(rotation_degrees))


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	#$RichTextLabel.text = rotation_degrees
	pass
