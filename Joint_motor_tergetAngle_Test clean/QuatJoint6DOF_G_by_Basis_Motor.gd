tool
extends Generic6DOFJoint


export(float, 0,1) var stiffnessA = 1.0
export(float, 0,1) var stiffnessB = 0.05
export(float, 0,1) var dampingA = 1.0
export(float, 0,1) var dampingB = 1.0
export(float, 0,1) var I_bias = 1.0

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
#	self.set_param_x(PARAM_ANGULAR_MOTOR_FORCE_LIMIT,50000)
#	self.set_param_y(PARAM_ANGULAR_MOTOR_FORCE_LIMIT,50000)
#	self.set_param_z(PARAM_ANGULAR_MOTOR_FORCE_LIMIT,50000)
	#self.set_param_x(PARAM_ANGULAR_MOTOR_FORCE_LIMIT,300)
	#self.set_param_y(PARAM_ANGULAR_MOTOR_FORCE_LIMIT,300)
	#self.set_param_z(PARAM_ANGULAR_MOTOR_FORCE_LIMIT,300)


	
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
var prev_vel_diff_B = Vector3(0,0,0)
var prev_vel_diff_A = Vector3(0,0,0)

var derivative_init = false

func reset_derivative_init():
	derivative_init = false


export var torque_lim =5000000000
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
	
	if(is_nan(angle)):
		angle = 0

		

	
	#if node B's quat is already facing the same way as qTargetO the axis shoots to infinity
	#this is my sorry ass attempt to protect us from it
	
	var axis = Vector3(0,0,0)
	var v = Vector3(rotChange.x,rotChange.y,rotChange.z)#rotation change quaternion "V" component
	
	var axis_den = sin(angle*0.5)
	if(is_nan(axis_den)):
		axis = Vector3(0,0,0)
	elif(axis_den==0):
		axis = Vector3(0,0,0)
	else:
		axis = v / axis_den#the quats axis
	
	var axis_Mag = axis.length()
	if(is_nan(axis_Mag)||is_inf(axis_Mag)):
		axis = Vector3(0,0,0)

	#prev_quat_axis = axis
	if(angle>PI):
		angle -= 2.0*PI



	var targetAng = axis*angle
	
	#var tb_consolidated = redInertia.inverse()*(targetAng*stiffnessB - (err_D/delta)*dampingB)
	#var tb_consolidated = redInertia.inverse()*(targetAng*stiffnessB - (err_D/delta)*dampingB)/delta
	
	var P = targetAng*stiffnessB/delta
	var D = -(bAV-aAV)*dampingB # (cur_rotation-prev_rotation)/dt = angular velocity
	
	
	#integ_stored += targetAng*delta*I_clamp
	#I = integ_stored * I_bias


	

	var jtb_consolidated = redInertia*(P + D)
	#jtb_consolidated = clamp_vector(jtb_consolidated,lim_Min,lim_Max)
	
	var tb_consolidated = max_vector_magnitude(jtb_consolidated/delta,torque_lim)
	#var tb_consolidated = jtb_consolidated/delta
	var jtb_consolidated_clamped = tb_consolidated*delta
	#var jtb_consolidated_clamped = jtb_consolidated
#	var err_out_sign = clamp(sign(jtb_consolidated.length()) * sign(targetAng.length()),0,1)# 1,1:1 ; 1,-1:0 ;-1,-1:1, 0,0:0
#	I_clamp *= err_out_sign
#	I_clamp = 1-I_clamp
#	var jtb_consolidated_clamped = redInertia*clamp_vector(jtb_consolidated,-saturation_lim,saturation_lim)
#	var dif = (jtb_consolidated-jtb_consolidated_clamped)
#	#var dif = (Vector3(0.3,30,0.3)-Vector3(0.5,30,0.3))
#	dif = vec_step_rev(dif.abs(),Vector3(0,0,0))
#	var e_o_sign = vec_step_rev(jtb_consolidated * targetAng,Vector3(0,0,0))
#	#var e_o_sign = vec_step_rev(Vector3(0.3,30,0.3) * Vector3(-0.1,-30,0.3),Vector3(0,0,0))
#	I_clamp = dif * e_o_sign
#	I_clamp = Vector3(1,1,1) - I_clamp
	


	#if(get_parent().name == "6DOF_G_BASIS_DEMO"):
	#if(get_parent().name == "LegG_REX_L"):
	if(get_parent().name == "Spider_LegG_SPINDLE1"):
		#$RichTextLabel.text= "Ib_inv: \n"+String(body_b.get_inverse_inertia_tensor())
		#$RichTextLabel2.text= "Ia_inv: \n"+String(body_a.get_inverse_inertia_tensor())
		#$RichTextLabel.text= "tb+ta: "+String(tb_consolidated+ta_consolidated)
		#$RichTextLabel.text= "tb: "+String(tb_consolidated)
		#$RichTextLabel.text= "targetAng: "+String(targetAng*180/PI)
		#$RichTextLabel.text= "torq_b.length(): "+String(torq_b.length())
		#$RichTextLabel2.text= "torq_b: "+String(torq_b)
		#$RichTextLabel2.text= "bAV: "+String(bAV)
		#$RichTextLabel2.text= "angle: "+String(angle*180/PI)
		#$RichTextLabel2.text= "Ib_inv: "+String(redInertia)
		#$RichTextLabel2.text= "cur_rotation: "+String(cur_rotation*180/PI)
		#$RichTextLabel3.text= "P.length(): "+String(P.length())
		#$RichTextLabel3.text= "P: "+String(P)
		#$RichTextLabel2.text= "err_D: "+String(err_D)
		#$RichTextLabel.text= "ang_v_b.length(): "+String(ang_v_b.length())
		#$RichTextLabel2.text= "body_b.angular_velocity.length(): "+String(body_b.angular_velocity.length())
		$RichTextLabel.text= "tb_consolidated.length(): "+String(tb_consolidated.length())
		$RichTextLabel3.text= "tb_consolidated: "+String(tb_consolidated)
		#$RichTextLabel3.text= "tb_consolidated_v: "+String(tb_consolidated)
		#$RichTextLabel.text= "jtb_consolidated: "+String(jtb_consolidated)
		#$RichTextLabel3.text= "jtb_consolidated.length(): "+String(jtb_consolidated.length())
		#$RichTextLabel3.text= "D: "+String(D)
		#$RichTextLabel3.text= "deriveMeasure: "+String(deriveMeasure)
		#$RichTextLabel.text= "I: "+String(I)
		#var dM_dt = deriveMeasure/delta
		#$RichTextLabel.text= "bAV - deriveMeasure/delta: "+String(bAV - dM_dt)
		#$RichTextLabel3.text= "integ_stored: "+String(integ_stored)
		$RichTextLabel2.text= "targetAng.length(): "+String(targetAng.length())
		#$RichTextLabel3.text= "max_vector_magnitude(tb_consolidated,max_torque): "+String(max_vector_magnitude(tb_consolidated,max_torque).length())



	if body_b.is_class("RigidBody") and body_b != null:

		body_b.apply_torque_impulse(jtb_consolidated_clamped)

#		self.set_param_x(PARAM_ANGULAR_MOTOR_TARGET_VELOCITY,jtb_consolidated_clamped.x)	#Causes Jitters for some reason
#		self.set_param_y(PARAM_ANGULAR_MOTOR_TARGET_VELOCITY,jtb_consolidated_clamped.y)
#		self.set_param_z(PARAM_ANGULAR_MOTOR_TARGET_VELOCITY,jtb_consolidated_clamped.z)

		pass

	if body_a.is_class("RigidBody") and body_a != null:
		#body_a.apply_torque_impulse(-jtb_consolidated_clamped)
		#body_a.apply_torque_impulse(-PID)
		#body_a.apply_torque_impulse(-jtb_consolidated)
		#body_a.angular_velocity = ang_v_a
		#body_a.linear_velocity = v_a
		pass
func clamp_vector_by_elemepnt(vec:Vector3,minimum:Vector3,maximum:Vector3):
	vec.x = clamp(vec.x,minimum.x,maximum.x)
	vec.y = clamp(vec.y,minimum.y,maximum.y)
	vec.z = clamp(vec.z,minimum.z,maximum.z)
	return vec
func clamp_vector(vec:Vector3,minimum:float,maximum:float):
	vec.x = clamp(vec.x,minimum,maximum)
	vec.y = clamp(vec.y,minimum,maximum)
	vec.z = clamp(vec.z,minimum,maximum)
	return vec

func max_vector_magnitude(vec:Vector3,maximum:float):
	var vec_l = vec.length()
	var vec_n = vec.normalized()
	vec_l = min(vec_l,maximum)
	return vec_n * vec_l

func round_vector(vec:Vector3,dec:float):
	vec.x = stepify(vec.x,dec)
	vec.y = stepify(vec.y,dec)
	vec.z = stepify(vec.z,dec)
	return vec

func vec_step_rev(edge, x):
	var v = Vector3(0,0,0)
	v.x = 1 if x.x < edge.x else 0
	v.y = 1 if x.y < edge.y else 0
	v.z = 1 if x.z < edge.z else 0
	return v
func vec_step(edge, x): #dif, v(1,1,1)
	var v = Vector3(0,0,0)
	v.x = 0 if x.x < edge.x else 1
	v.y = 0 if x.y < edge.y else 1
	v.z = 0 if x.z < edge.z else 1
	return v
