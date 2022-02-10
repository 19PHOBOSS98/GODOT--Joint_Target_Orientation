tool
extends Generic6DOFJoint


export(Vector3) var stiffnessA = Vector3(10.0,10.0,10.0)
export(Vector3) var stiffnessB = Vector3(10.0,10.0,10.0)
export(Vector3) var dampingA = Vector3(5.0,5.0,5.0)
export(Vector3) var dampingB = Vector3(5.0,5.0,5.0)

export(float,-360.0,360.0) var rest_angle_x = 0.0
export(float,-360.0,360.0) var rest_angle_y = 0.0
export(float,-360.0,360.0) var rest_angle_z = 0.0

var body_b
var body_a

#export(NodePath) var baseBTOrigNode
#var baseBTOrigN

var baseBTOrig = Basis()

var massTotal
var massA
var massB

var hasBase = false

func _ready():
	if Engine.editor_hint:
		return
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
	set_total_mass()
	set_base_transform_original(body_a,body_b)
	


func _physics_process(delta):
#func _integrate_forces(delta):
	if Engine.editor_hint:
		return
	if(hasBase):
		apply_rot_spring_quat(delta)


func set_body_a(A):
	if A == null:
		hasBase = false
		return
	if A.is_class("RigidBody"):
		body_a = A
		massA= A.mass
	else:
		massA = 999999.0

func set_body_b(B):
	if B == null:
		hasBase = false
		return
	if B.is_class("RigidBody"):
		body_b = B
		prints("name: ",self.name," B: ",body_b)
		massB = B.mass
	else:
		massB = 999999.0

func set_total_mass():
	if(massA == null || massB == null):
		hasBase = false
		massTotal = 0.0
		return
	massTotal = massA+massB

#######IGNORE THIS#######
func SIGNAL_set_body_a():
	if body_a == null:
		hasBase = false
		return
	if body_a.is_class("RigidBody"):
		massA = body_a.mass
	else:
		massA = 999999.0

func SIGNAL_set_body_b():
	if body_b == null:
		hasBase = false
		return
	if body_b.is_class("RigidBody"):
		massB = body_b.mass
	else:
		massB = 999999.0
#######IGNORE THIS#######

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


func calc_target_orientation(Abasis:Basis):

	var baseBTOActual = Abasis*baseBTOrig#the actual original base node B Transform following node A around in current global space

	var qx = Quat(Abasis.x,rest_angle_x*PI/180.0)
	var qy = Quat(Abasis.y,rest_angle_y*PI/180.0)
	var qz = Quat(Abasis.z,rest_angle_z*PI/180.0)
	
	var qBTargRo = qz*qy*qx# Quaternion node B Target Rotation
	var BTargetBasis = Basis()
	BTargetBasis.x =  qBTargRo*baseBTOActual.x
	BTargetBasis.y =  qBTargRo*baseBTOActual.y
	BTargetBasis.z =  qBTargRo*baseBTOActual.z

	return Quat(BTargetBasis)
	

"""
Thanks to:
	DMGregory: https://gamedev.stackexchange.com/questions/182850/rotate-rigidbody-to-face-away-from-camera-with-addtorque/182873#182873
	and
	The Step Event: https://youtu.be/vewwP8Od_7s
	For the calculations
"""
var c = 0
func apply_rot_spring_quat(delta):# apply spring rotation using quaternion
	if Engine.editor_hint:
		return
	var bAV = Vector3()# Node B ANgular Velocity
	var aAV = Vector3()# Node A ANgular Velocity
	
	var bI = Vector3()
	var aI = Vector3()
	if body_b.is_class("RigidBody"):
		bAV = body_b.angular_velocity
		#bI = body_b.get_inverse_inertia_tensor()
		bI.x = body_b.get_inverse_inertia_tensor().x.x
		bI.y = body_b.get_inverse_inertia_tensor().y.y
		bI.z = body_b.get_inverse_inertia_tensor().z.z
		bI = body_b.global_transform.basis.inverse()*bI
	else:
		bAV = Vector3(0.0,0.0,0.0)

	if body_a.is_class("RigidBody"):
		aAV = body_a.angular_velocity
		#aI = body_a.get_inverse_inertia_tensor()
		aI.x = body_a.get_inverse_inertia_tensor().x.x
		aI.y = body_a.get_inverse_inertia_tensor().y.y
		aI.z = body_a.get_inverse_inertia_tensor().z.z
		aI = body_a.global_transform.basis.inverse()*aI
	else:
		aAV = Vector3(0.0,0.0,0.0)
		


	var qBT = Quat(body_b.global_transform.basis)#Quaternion Node B Transform Basis
	#Quaternion Target Orientation
	var qTargetO = calc_target_orientation(body_a.global_transform.basis) #Does the same thing but without the Position3D node
	var rotChange = qTargetO * qBT.inverse() #rotation change quaternion
	
	var angle = 2.0 * acos(rotChange.w) #Turns to angle radians, the amount it turns around the quats axis

	#if node B's quat is already facing the same way as qTargetO the axis shoots to infinity
	#this is my sorry ass attempt to protect us from it
	
	if(is_nan(angle)):
		
		if body_b.is_class("RigidBody"):
			body_b.add_torque(-bAV)
		if body_a.is_class("RigidBody"):
			body_a.add_torque(-aAV)
		return

	var v = Vector3(rotChange.x,rotChange.y,rotChange.z)# rotation change quaternion "V" component
	var axis = v / sin(angle*0.5)# the quats axis

	if(angle>PI):
		angle -= 2.0*PI

	#as node B's quat faces the same way as qTargetO the angle nears 0
	#this slows it down to stop the axis from reaching infinity
	if(is_equal_approx(angle,0.0)):
		if body_b.is_class("RigidBody"):
			body_b.add_torque(-bAV)
		if body_a.is_class("RigidBody"):
			body_a.add_torque(-aAV)
		return

	var targetAngVel = axis*angle/delta
	
	#var tb_consolidated = (stiffnessB/massTotal)*targetAngVel - dampingB*(bAV-aAV)
	#var ta_consolidated = -(stiffnessA/massTotal)* targetAngVel - dampingA*(aAV - bAV)
	#var tb_consolidated = (stiffnessB/massB)*targetAngVel - dampingB*(bAV)
	#var ta_consolidated = -(stiffnessA/massA)* targetAngVel - dampingA*(aAV)
	#var tb_consolidated = (bI*stiffnessB)*targetAngVel - dampingB*(bAV)
	#var ta_consolidated = -(aI*stiffnessA)* targetAngVel - dampingA*(aAV)
	
	#var tb_consolidated = (stiffnessB/massB)*0.01*targetAngVel - stiffnessB*0.5*(bAV)
	#var ta_consolidated = -(stiffnessA/massA)*0.01* targetAngVel - stiffnessA*0.5*(aAV)
	#var tb_consolidated = (stiffnessB)*0.01*(bI*targetAngVel) - stiffnessB*0.5*(bAV)
	#if(get_parent().name == "Experiment_2"):
	#	$RichTextLabel.text = String(bI.x)+String(bI.y)+String(bI.z)
	#var ta_consolidated = -(stiffnessA)*0.01*(aI*targetAngVel) - stiffnessA*0.5*(aAV)
	if(get_parent().name == "Experiment_2"):
		var x = 20.0
		var y = 2.0
		var z = 2.0
		var Ixx = Vector3(1.0/12.0*body_b.mass*(y*y + z*z),0,0)
		var Iyy = Vector3(0,1.0/12.0*body_b.mass*(x*x + z*z),0)
		var Izz = Vector3(0,0,1.0/12.0*body_b.mass*(x*x + y*y))
		var I = Basis(Ixx,Iyy,Izz).inverse()
		#bI = I
		#$RichTextLabel3.text = "bI: "+String(body_b.global_transform.basis.inverse()*bI)
		#$RichTextLabel.text = String((bI*targetAngVel) - stiffnessB*0.3*(bAV))
		#if(!is_nan(bAV.z)):
		#if(c<10):
		#if(c<5):
		#if(c<3):
		#if(c<2):
			#c+=1
			#$RichTextLabel.text = "bAV: "+String(bAV)
			#$RichTextLabel2.text = "kx: "+String((stiffnessB)*(bI*targetAngVel))
			#$RichTextLabel3.text = "d: "+String(dampingA*(bAV))
			#$RichTextLabel2.text = "kx: "+String((stiffnessB/massB)*0.01*targetAngVel)
			#$RichTextLabel3.text = "d: "+String(stiffnessB*0.3*(bAV))
			#$RichTextLabel3.text = String((stiffnessB)*0.003*(bI*targetAngVel) - stiffnessB*0.3*(bAV))
	
	
			
	var tb_consolidated = (stiffnessB)*(bI*targetAngVel) - dampingB*(bAV)
	var ta_consolidated = -(stiffnessA)*(aI*targetAngVel) - dampingA*(aAV)
	#tb_consolidated = vector_clamp(tb_consolidated,-(stiffnessB*0.5),stiffnessB*0.5)
	#ta_consolidated = vector_clamp(ta_consolidated,-(stiffnessA*0.5),stiffnessA*0.5)
	#var tb_consolidated = (stiffnessB/massB)*0.01*targetAngVel - stiffnessB*0.5*(bAV)
	#var ta_consolidated = -(stiffnessA/massA)*0.01* targetAngVel - stiffnessA*0.5*(aAV)
	
	if body_b.is_class("RigidBody") and body_b != null:
		body_b.add_torque(tb_consolidated)
		#body_b.angular_velocity += tb_consolidated
		#body_b.angular_velocity = vector_clamp(body_b.angular_velocity,-(stiffnessB),stiffnessB)

	if body_a.is_class("RigidBody") and body_a != null:
		body_a.add_torque(ta_consolidated)
		#body_a.angular_velocity += ta_consolidated
		#body_a.angular_velocity = vector_clamp(body_a.angular_velocity,-(stiffnessA),stiffnessA)

func vector_clamp(v:Vector3,vmin:Vector3,vmax:Vector3):
	v.x = clamp(v.x,vmin.x,vmax.x)
	v.y = clamp(v.y,vmin.y,vmax.y)
	v.z = clamp(v.z,vmin.z,vmax.z)
	return v
