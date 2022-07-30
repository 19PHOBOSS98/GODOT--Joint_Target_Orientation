tool
extends Generic6DOFJoint


export(float,0,1) var stiffnessA = 1.0
export(float,0,1) var stiffnessB = 1.0
export(float,0,1) var dampingA = 1.0
export(float,0,1) var dampingB = 1.0
export var I_bias = 1.0

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
#	self.set_flag_x(FLAG_ENABLE_MOTOR,true)
#	self.set_flag_y(FLAG_ENABLE_MOTOR,true)
#	self.set_flag_z(FLAG_ENABLE_MOTOR,true)
#	self.set_param_x(PARAM_ANGULAR_MOTOR_FORCE_LIMIT,5000000)
#	self.set_param_y(PARAM_ANGULAR_MOTOR_FORCE_LIMIT,5000000)
#	self.set_param_z(PARAM_ANGULAR_MOTOR_FORCE_LIMIT,5000000)


	
	self.set_exclude_nodes_from_collision(false)
	
	
	body_a = get_node(get_node_a())
	body_b = get_node(get_node_b())
	set_body_a(body_a)
	set_body_b(body_b)
	set_total_mass()
	set_base_transform_original(body_a,body_b)
	


func _physics_process(delta):
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

func calc_target_orientation_by_basis():
	var BTargetBasis = get_node(puppeteer_node).global_transform.basis #can be any node as long as it has it's own basis vector
	return Quat(BTargetBasis)

func calc_target_position(isB:bool):
	var BTargetPosition
	if(isB):
		BTargetPosition = get_node(puppeteer_node).get_child(0).global_transform.origin
	else:
		BTargetPosition = get_node(puppeteer_node).get_parent().global_transform.origin
	return BTargetPosition

"""
Thanks to:
	DMGregory: https://gamedev.stackexchange.com/questions/182850/rotate-rigidbody-to-face-away-from-camera-with-addtorque/182873#182873
	The Step Event: https://youtu.be/vewwP8Od_7s
	and
	h4tt3n: https://www.gamedev.net/tutorials/programming/math-and-physics/towards-a-simpler-stiffer-and-more-stable-spring-r3227/
	For the calculations
"""
var prev_vel_diff_B = Vector3()
var prev_vel_diff_A = Vector3()

var derivative_init = false

func reset_derivative_init():
	derivative_init = false

var integ_stored = Vector3(0,0,0)

func apply_rot_spring_quat(delta):# apply spring rotation using quaternion
	if Engine.editor_hint:
		return
	var bAV = Vector3(0.0,0.0,0.0)# Node B ANgular Velocity
	var aAV = Vector3(0.0,0.0,0.0)# Node A ANgular Velocity
	var Ia_inv = Basis(Vector3(1,0,0),Vector3(0,1,0),Vector3(0,0,1))
	var Ib_inv = Basis(Vector3(1,0,0),Vector3(0,1,0),Vector3(0,0,1))
	var Ia_inv2
	if body_b.is_class("RigidBody"):
		bAV = body_b.angular_velocity
		Ib_inv = body_b.get_inverse_inertia_tensor()

	if body_a.is_class("RigidBody"):
		aAV = body_a.angular_velocity
		Ia_inv = body_a.get_inverse_inertia_tensor()#somehow, commenting this out works

	var redInertia = add_Basis_by_Element(Ia_inv,Ib_inv).inverse()#reduced moment of inertia: https://www.gamedev.net/tutorials/programming/math-and-physics/towards-a-simpler-stiffer-and-more-stable-spring-r3227/
	var qBT = Quat(body_b.global_transform.basis)#Quaternion Node B Transform Basis
	#Quaternion Target Orientation
	var qTargetO = calc_target_orientation_by_basis()#Does the same thing but without the Position3D node
	var rotChange = qTargetO * qBT.inverse() #rotation change quaternion
	
	var angle = 2.0 * acos(rotChange.w) #Turns to angle radians, the amount it turns around the quats axis

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

	bAV = body_b.angular_velocity
	aAV = body_a.angular_velocity
	
	var Pb = targetAng/delta
	var Pa = -Pb
	
	var vel_diff_B = bAV-aAV
	var vel_diff_A = -vel_diff_B
	
	var Db = vel_diff_B
	var Da = -Db

	

	prev_vel_diff_B = bAV-aAV
	prev_vel_diff_A = -prev_vel_diff_B
	integ_stored = integ_stored+(targetAng*delta)
	integ_stored = clamp_vector(integ_stored,-500000,500000)
	
	#var tb_consolidated = (redInertia)*(targetAng*stiffnessB/delta - (bAV-aAV)*dampingB)/delta
	var tb_consolidated = (redInertia)*(targetAng*stiffnessB/delta - (bAV-aAV)*dampingB)
	#var ta_consolidated = (redInertia)*(-targetAng*stiffnessA/delta - (aAV-bAV)*dampingA)
	
	#var tb_consolidated = (redInertia)*(Pb*stiffnessB - (Db)*dampingB + (integ_stored)*I_bias)
	#var tb_consolidated = (redInertia)*(Pb*stiffnessB - (Db)*dampingB)/delta
	
	##tb_consolidated = clamp_vector(tb_consolidated,-50000,50000)
	
	#tb_consolidated = clamp_vector(tb_consolidated,-50000000,50000000)
	#tb_consolidated *= delta
	
	
	#var ta_consolidated = (redInertia)*(Pa*stiffnessA - (Da)*dampingA)
	#var ta_consolidated = -tb_consolidated

	#var ang_v_b = targetAng*20
	#var ang_v_a = -targetAng*50
	#var ang_v_b = (Pb*stiffnessB - (Db)*dampingB)
	#var ang_v_a = -ang_v_b
	
	var tb_consolidated_v = (targetAng*stiffnessB/delta - (bAV-aAV)*dampingB)
	#var ang_v_b = targetAng*2
	var ang_v_b = targetAng*10
	#var torq_b = (redInertia)*(ang_v_b/delta)
	var torq_b = clamp_vector((redInertia)*(ang_v_b/delta),-500000,500000)
	ang_v_b = (redInertia).inverse()*torq_b*delta
	var ang_v_a = -targetAng*10
	
	var target_b = calc_target_position(true)
	var target_a = calc_target_position(false)
	
	var pos_b = body_b.global_transform.origin
	var pos_a = body_a.global_transform.origin
	
	var dist_b = target_b.distance_to(pos_b)
	var dist_a = target_a.distance_to(pos_a)
	
	var v_b = pos_b.direction_to(target_b) * dist_b * 50
	#var v_a = pos_a.direction_to(target_a) * dist_a * 50
	
	#if(get_parent().name == "LegG_REX_L"):
	if(get_parent().name == "Spider_LegG_SPINDLE1"):
		#$RichTextLabel.text= "Ib_inv: \n"+String(body_b.get_inverse_inertia_tensor())
		#$RichTextLabel2.text= "Ia_inv: \n"+String(body_a.get_inverse_inertia_tensor())
		#$RichTextLabel.text= "tb+ta: "+String(tb_consolidated+ta_consolidated)
		#$RichTextLabel.text= "tb: "+String(tb_consolidated)
		#$RichTextLabel.text= "targetAng: "+String(targetAng)
		$RichTextLabel.text= "ang_v_b: "+String(ang_v_b)
		$RichTextLabel2.text= "torq_b: "+String(torq_b)
		#$RichTextLabel2.text= "bAV: "+String(bAV)
		#$RichTextLabel2.text= "angle: "+String(angle*180/PI)
		#$RichTextLabel2.text= "Ib_inv: "+String(redInertia)
		$RichTextLabel3.text= "tb_consolidated_v: "+String(tb_consolidated_v)


	if body_b.is_class("RigidBody") and body_b != null:
		#body_b.apply_torque_impulse(tb_consolidated)
		body_b.angular_velocity = ang_v_b
		#self.set_param_x(PARAM_ANGULAR_MOTOR_TARGET_VELOCITY,ang_v_b.x)
		#self.set_param_y(PARAM_ANGULAR_MOTOR_TARGET_VELOCITY,ang_v_b.y)
		#self.set_param_z(PARAM_ANGULAR_MOTOR_TARGET_VELOCITY,ang_v_b.z)

		#body_b.angular_velocity = tb_consolidated_v
		#body_b.linear_velocity = v_b
		

	if body_a.is_class("RigidBody") and body_a != null:
		#body_a.apply_torque_impulse(ta_consolidated)
		body_a.angular_velocity = ang_v_a
		#body_a.linear_velocity = v_a
		pass
func clamp_vector(vec:Vector3,minimum:float,maximum:float):
	vec.x = clamp(vec.x,minimum,maximum)
	vec.y = clamp(vec.y,minimum,maximum)
	vec.z = clamp(vec.z,minimum,maximum)
	return vec
	
func round_vector(vec:Vector3,dec:float):
	vec.x = stepify(vec.x,dec)
	vec.y = stepify(vec.y,dec)
	vec.z = stepify(vec.z,dec)
	return vec
