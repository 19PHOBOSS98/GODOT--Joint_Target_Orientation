extends DirectionalLight


# Declare member variables here. Examples:
# var a = 2
# var b = "text"


# Called when the node enters the scene tree for the first time.
func _ready():
	var c = get_parent().get_node("LegD").get_children()
	for cc in c:
		prints(cc)


# Called every frame. 'delta' is the elapsed time since the previous frame.
#func _process(delta):
#	pass
