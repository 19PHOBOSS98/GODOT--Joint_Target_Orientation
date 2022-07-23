tool
extends Generic6DOFJoint


export(float,0,1) var stiffnessA = 0.0
export(float,0,1) var stiffnessB = 0.05
export(float,0,1) var dampingA = 0.0
export(float,0,1) var dampingB = 1.0

var body_b
var body_a

export(NodePath) var puppeteer_node
var baseBTOrigN

var baseBTOrig = Basis()

var massTotal
var massA
var massB

var hasBase = false

func add_Basis_by_Element(a,b):
	var ab = Basis(a.x+b.x,a.y+b.y,a.z+b.z)
	return ab
func div_Basis_by_Element(a,b):
	var ab = Basis(a.x/b.x,a.y/b.y,a.z/b.z)
	return ab
	
func _ready():
	if Engine.editor_hint:
		return
		Basis()
	self.set_flag_x(FLAG_ENABLE_LINEAR_LIMIT,true)
	self.set_flag_y(FLAG_ENABLE_LINEAR_LIMIT,true)
	self.set_flag_z(FLAG_ENABLE_LINEAR_LIMIT,true)
	self.set_flag_x(FLAG_ENABLE_ANGULAR_LIMIT,false)
	self.set_flag_y(FLAG_ENABLE_ANGULAR_LIMIT,false)
	self.set_flag_z(FLAG_ENABLE_ANGULAR_LIMIT,false)
	self.set_exclude_nodes_from_collision(false)
	
	body_a = get_node(get_node_a())
	body_b = get_node(get_node_b())
	set_body_a(body_a)
	set_body_b(body_b)

	set_base_transform_original(body_a,body_b)
	


func _physics_process(delta):
	if Engine.editor_hint:
		return
	if(hasBase):
		apply_rot_spring_quat(delta)


func set_body_a(A):
	if A == null:
		return
	if A.is_class("RigidBody"):
		body_a = A


func set_body_b(B):
	if B == null:
		return
	if B.is_class("RigidBody"):
		body_b = B


#base node B Transform Original (baseBTOrig) should be calculated instead of rellying on a 3D Position node on node A
#In short this has to go
func set_base_transform_original(A,B):
	if A == null or B == null:
		hasBase = false
		return
	baseBTOrig = A.global_transform.basis.inverse()*B.global_transform.basis
	
	hasBase = true

func SIGNAL_set_base_transform_original(B):# for dynamic node attachment in my game: IGNORE THIS
	if body_a == null or B == null:
		hasBase = false
		return
	baseBTOrig = body_a.global_transform.basis.inverse()*B.global_transform.basis

	hasBase = true

func calc_target_orientation_by_basis():
	var BTargetBasis = get_node(puppeteer_node).global_transform.basis #can be any node as long as it has it's own basis vector
	return Quat(BTargetBasis)
	

"""
Thanks to:
	DMGregory: https://gamedev.stackexchange.com/questions/182850/rotate-rigidbody-to-face-away-from-camera-with-addtorque/182873#182873
	The Step Event: https://youtu.be/vewwP8Od_7s
	and
	h4tt3n: https://www.gamedev.net/tutorials/programming/math-and-physics/towards-a-simpler-stiffer-and-more-stable-spring-r3227/
	For the calculations
"""
func apply_rot_spring_quat(delta):# apply spring rotation using quaternion
	if Engine.editor_hint:
		return
	var bAV = Vector3(0.0,0.0,0.0)# Node B Angular Velocity
	var aAV = Vector3(0.0,0.0,0.0)# Node A Angular Velocity
	var Ia_inv = Basis(Vector3(1,0,0),Vector3(0,1,0),Vector3(0,0,1))# Node B Inverse Inertia Tensor
	var Ib_inv = Basis(Vector3(1,0,0),Vector3(0,1,0),Vector3(0,0,1))# Node A Inverse Inertia Tensor

	if body_b.is_class("RigidBody"):
		bAV = body_b.angular_velocity
		Ib_inv = body_b.get_inverse_inertia_tensor()

	if body_a.is_class("RigidBody"):
		aAV = body_a.angular_velocity
		Ia_inv = body_a.get_inverse_inertia_tensor()

	var redInertia = add_Basis_by_Element(Ia_inv,Ib_inv).inverse()#reduced moment of inertia: https://www.gamedev.net/tutorials/programming/math-and-physics/towards-a-simpler-stiffer-and-more-stable-spring-r3227/
	var qBT = Quat(body_b.global_transform.basis)#Quaternion Node B Transform Basis
	var qTargetO = calc_target_orientation_by_basis()#Quaternion Target Orientation
	var rotChange = qTargetO * qBT.inverse() #rotation change quaternion
	
	var angle = 2.0 * acos(rotChange.w) #Turns to angle radians, the amount it turns around the quaternion axis

	#if node B's quat is already facing the same way as qTargetO the axis shoots to infinity
	#this is my sorry ass attempt to protect us from it
	if(is_nan(angle)):
		if body_b.is_class("RigidBody"):
			body_b.add_torque((redInertia)*-(bAV-aAV)*dampingB/delta)
		if body_a.is_class("RigidBody"):
			body_a.add_torque((redInertia)*-(aAV-bAV)*dampingA/delta)
		return

	var v = Vector3(rotChange.x,rotChange.y,rotChange.z)#rotation change quaternion "V" component
	var axis = v / sin(angle*0.5)#the quats axis

	if(angle>PI):
		angle -= 2.0*PI

	#as node B's quat faces the same way as qTargetO the angle nears 0
	#this slows it down to stop the axis from reaching infinity
	if(is_equal_approx(angle,0.0)):
		if body_b.is_class("RigidBody"):
			body_b.add_torque((redInertia)*-(bAV-aAV)*dampingB/delta)
		if body_a.is_class("RigidBody"):
			body_a.add_torque( (redInertia)*-(aAV-bAV)*dampingA/delta)
		return

	var targetAng = axis*angle
	var deltaSqr = delta*delta

	var tib_consolidated = (redInertia)*(targetAng*stiffnessB/delta - (bAV-aAV)*dampingB)#Consolidated Torque Impulse for node B
	var tia_consolidated = (redInertia)*(-targetAng*stiffnessA/delta - (aAV-bAV)*dampingA)#Consolidated Torque Impulse for node A
	
	
	#if(get_parent().name == "6DOF_G_DEMO1"):
		#$RichTextLabel.text= "Ib_inv: \n"+String(body_b.get_inverse_inertia_tensor())
		#$RichTextLabel2.text= "Ia_inv: \n"+String(body_a.get_inverse_inertia_tensor())
		#$RichTextLabel.text= "tb+ta: "+String(tb_consolidated+ta_consolidated)
		#$RichTextLabel.text= "tib: "+String(tib_consolidated)
		#$RichTextLabel2.text= "bAV: "+String(bAV)
		#$RichTextLabel2.text= "angle: "+String(angle*180/PI)
		#$RichTextLabel2.text= "Ib_inv: "+String(redInertia)
		#$RichTextLabel3.text= "ta2: "+String(ta_consolidated2)


	if body_b.is_class("RigidBody") and body_b != null:
		body_b.apply_torque_impulse(tib_consolidated)
		

	if body_a.is_class("RigidBody") and body_a != null:
		body_a.apply_torque_impulse(tia_consolidated)

