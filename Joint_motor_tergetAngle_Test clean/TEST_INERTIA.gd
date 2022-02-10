extends RigidBody


# Declare member variables here. Examples:
# var a = 2
# var b = "text"


# Called when the node enters the scene tree for the first time.
func _ready():
	var x = 20.0
	var y = 2.0
	var z = 2.0
	var Ixx = Vector3(1.0/12.0*mass*(y*y + z*z),0,0)
	var Iyy = Vector3(0,1.0/12.0*mass*(x*x + z*z),0)
	var Izz = Vector3(0,0,1.0/12.0*mass*(x*x + y*y))
	var I = Basis(Ixx,Iyy,Izz)
	prints("I: ",I.x,I.y,I.z)


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	var Ii = get_inverse_inertia_tensor().inverse()
	$RichTextLabel.text ="Test: "+ String(Ii.x)+ String(Ii.y)+ String(Ii.z)
