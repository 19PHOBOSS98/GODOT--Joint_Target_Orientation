tool
extends Generic6DOFJoint

"""
Don't Forget to use Bullet Physics or something better
and set Physics step to 240
"""
"""
export(float) var stiffnessA_x = 10.0
export(float) var stiffnessB_x = 10.0
export(float) var dampingA_x = 5.0
export(float) var dampingB_x = 5.0

export(float) var stiffnessA_y = 10.0
export(float) var stiffnessB_y = 10.0
export(float) var dampingA_y = 5.0
export(float) var dampingB_y = 5.0

export(float) var stiffnessA_z = 10.0
export(float) var stiffnessB_z = 10.0
export(float) var dampingA_z = 5.0
export(float) var dampingB_z = 5.0
"""
export(Vector3) var stiffnessA = Vector3(10.0,10.0,10.0)
export(Vector3) var stiffnessB = Vector3(10.0,10.0,10.0)
export(Vector3) var dampingA = Vector3(5.0,5.0,5.0)
export(Vector3) var dampingB = Vector3(5.0,5.0,5.0)
#export(float,-360.0,360.0) var rest_angle = 0.0
export(float,-360.0,360.0) var rest_angle_x = 0.0
export(float,-360.0,360.0) var rest_angle_y = 0.0
export(float,-360.0,360.0) var rest_angle_z = 0.0
var rest_angle_rads = 0.0
var body_b
var body_a
var bI
var aI
var dampA
var dampB

var baseBTOrig
export(NodePath) var baseBTOrigPath
var damp

var massTotal

var massA
var massB
var hasBase = false
func _ready():
	if Engine.editor_hint:
		return
	"""
	if body_a.is_class("RigidBody"):
		massA = body_a.mass
		aI = body_a.get_inverse_inertia_tensor().inverse()
	else:
		massA = 999999.0
		aI = Basis()
		
	if body_b.is_class("RigidBody"):
		massB = body_b.mass
		bI = body_b.get_inverse_inertia_tensor().inverse()
	else:
		massB = 999999.0
		bI = Basis()
	"""
	#damp = 2000
	#damp = 2.0*sqrt(stiffness*(massB+massA))
	#damp = 2.0*sqrt(stiffness*(massB+massA))*0.05
	#damp = 2.0*sqrt(stiffness*(massB+massA))*0.005
	#dampA = 2.0*sqrt(stiffnessA*(massA))
	#dampB = 2.0*sqrt(stiffnessB*(massB))
	
	#prints("dampA: ")
	#prints(dampA)
	#prints("dampB: ")
	#prints(dampB)
	""""
	massTotal = massA+massB
	"""
	#baseBTOrig = body_b.global_transform.basis
	#baseBTOrig = body_a.get_node("Position3D")
	"""
	baseBTOrig = body_a.get_node(baseBTOrigPath)
	baseBTOrig.global_transform.basis = body_b.global_transform.basis
	"""
	body_a = get_node(get_node_a())
	body_b = get_node(get_node_b())
	set_body_a(body_a)
	set_body_b(body_b)
	set_total_mass()
	set_base_transform_original(body_a,body_b)
	
func set_total_mass():
	if(massA == null || massB == null):
		hasBase = false
		massTotal = 0.0
		return
	massTotal = massA+massB
	
func set_body_a(A):
	if A == null:
		hasBase = false
		return
	if A.is_class("RigidBody"):
		body_a = A
		massA= A.mass
	else:
		massA = 999999.0

func SIGNAL_set_body_a():
	if body_a == null:
		hasBase = false
		return
	if body_a.is_class("RigidBody"):
		massB = body_a.mass
	else:
		massB = 999999.0

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

func set_base_transform_original(A,B):
	if A == null or B == null:
		hasBase = false
		return
	baseBTOrig = A.get_node(baseBTOrigPath)
	baseBTOrig.global_transform.basis = B.global_transform.basis
	hasBase = true

func SIGNAL_set_base_transform_original(B):
	if body_a == null or B == null:
		hasBase = false
		return
	baseBTOrig = body_a.get_node(baseBTOrigPath)
	baseBTOrig.global_transform.basis = B.global_transform.basis
	prints("baseBTOrig.transform.basis: ",baseBTOrig.transform.basis.x,baseBTOrig.transform.basis.y,baseBTOrig.transform.basis.z)
	hasBase = true

func _physics_process(delta):
	
	if Engine.editor_hint:
		return
		
	self.set_flag_x(FLAG_ENABLE_LINEAR_LIMIT,true)
	self.set_flag_y(FLAG_ENABLE_LINEAR_LIMIT,true)
	self.set_flag_z(FLAG_ENABLE_LINEAR_LIMIT,true)
	self.set_flag_x(FLAG_ENABLE_ANGULAR_LIMIT,false)
	self.set_flag_y(FLAG_ENABLE_ANGULAR_LIMIT,false)
	self.set_flag_z(FLAG_ENABLE_ANGULAR_LIMIT,false)
	
	self.set_exclude_nodes_from_collision(false)
	if(hasBase):
		apply_rot_spring_quat(delta)
	#calc_target_orientation(body_a.global_transform.basis,baseBTOrig.global_transform.basis)
	#if Input.is_action_just_pressed("rotate"):
		#var gz = body_a.global_transform.basis.y
		
		#gz.y = -1
		#body_a.add_torque(gz)
		#rotation_degrees.y += 45.0
		#body_a.rotation_degrees.y += 45.0
		#body_b.rotation_degrees.y += 45.0

export(NodePath) var target_node_orientation_b 

func calc_target_orientation(A:Basis,BTO:Basis):
	var Abasis = A
	#var baseBTCurr = Abasis*baseBTOrig
	#var baseBTCurr = Abasis*BTO
	#baseBTCurr = Abasis*baseBTCurr
	#baseBTCurr = Abasis.inverse()*baseBTCurr

	#$RichTextLabel2.text = "baseBTCurr: "+String(baseBTCurr.x)+" "+String(baseBTCurr.y)+" "+String(baseBTCurr.z)
	#$RichTextLabel3.text = "BTO: "+String(BTO.x)+" "+String(BTO.y)+" "+String(BTO.z)
	#var BTC = Abasis * B
	#print("Abasis: ",Abasis.x,Abasis.y,Abasis.z)
	#print("baseBTCurr: ",baseBTCurr.x,baseBTCurr.y,baseBTCurr.z)
	#$RichTextLabel.text = "A: "+String(A.x)+" "+String(A.y)+" "+String(A.z)
	#$RichTextLabel2.text = "BTO: "+String(BTO.x)+" "+String(BTO.y)+" "+String(BTO.z)
	var qx = Quat(Abasis.x,rest_angle_x*PI/180.0)
	var qy = Quat(Abasis.y,rest_angle_y*PI/180.0)
	#var qz = Quat(Abasis.z,45.0*PI/180.0)
	#var qz = Quat(Abasis.z,45.0*PI/180.0)
	var qz = Quat(Abasis.z,rest_angle_z*PI/180.0)
	
	var qBTargRo = qz*qy*qx
	#print(" qBTargRo: ", qBTargRo)
	var BTargetBasis = Basis()
	BTargetBasis.x =  qBTargRo*BTO.x
	BTargetBasis.y =  qBTargRo*BTO.y
	BTargetBasis.z =  qBTargRo*BTO.z
	#$RichTextLabel3.text = "BTargetBasis: "+String(BTargetBasis.x)+" "+String(BTargetBasis.y)+" "+String(BTargetBasis.z)
	#print("BTargetBasis: ",BTargetBasis.x,BTargetBasis.y,BTargetBasis.z)
	return Quat(BTargetBasis)
	#$RichTextLabel.text = String(BTargetBasis.x)+" "+String(BTargetBasis.y)+" "+String(BTargetBasis.z)




func apply_rot_spring_quat(delta):
	if Engine.editor_hint:
		return
	var bAV = Vector3()
	var aAV = Vector3()
	if body_b.is_class("RigidBody"):
		bAV = body_b.angular_velocity
		#sbAV = body_b.get_inverse_inertia_tensor().inverse() *bAV
		
	else:
		bAV = Vector3(0.0,0.0,0.0)
		#bAV = body_b.get_inverse_inertia_tensor().inverse() *bAV
		
	if body_a.is_class("RigidBody"):
		aAV = body_a.angular_velocity
		#aAV = body_a.get_inverse_inertia_tensor().inverse() *aAV
		
	else:
		aAV = Vector3(0.0,0.0,0.0)
		#aAV = body_a.get_inverse_inertia_tensor().inverse() *aAV
	#bAV = bI*bAV
	#aAV = aI*aAV
	#$RichTextLabel.text = "body_b.angular_velocity: "+ String(global_transform.basis.inverse()*body_b.angular_velocity)
	var qBT = Quat(body_b.global_transform.basis)
	var qTargetO = calc_target_orientation(body_a.global_transform.basis,baseBTOrig.global_transform.basis)
	
	#$RichTextLabel3.text = "apprx: "+String((qTargetO-qBT).length())
	#$RichTextLabel3.text = "apprx: "+String(abs(qTargetO.dot(qBT)))
	#$RichTextLabel2.text = "qBT: "+String(qBT)
	#$RichTextLabel.text = "qTargetO: "+String(qTargetO)
	#if(qTargetO.is_equal_approx(qBT)):
	#if((qTargetO-qBT).length()<=0.001):
	#$RichTextLabel3.text = " "+String(2.0 * acos((qTargetO * qBT.inverse()).w))
	#if(abs(qTargetO.dot(qBT))>=0.9999):
	"""
	if(abs(qTargetO.dot(qBT))==1):
		if body_b.is_class("RigidBody"):
			body_b.add_torque(-body_b.angular_velocity)
		if body_a.is_class("RigidBody"):
			body_a.add_torque(-body_a.angular_velocity)
		return
	"""
	#if(qTargetO<=0.001):
	#$RichTextLabel3.text = "qTargetO qBT: "+String(qTargetO * qBT.inverse())
	#$RichTextLabel3.text = "abs(qTargetO.dot(qBT)): "+String(abs(qTargetO.dot(qBT)))
	#$RichTextLabel2.text = "qTargetO.is_equal_approx(qBT): "+String(qTargetO.is_equal_approx(qBT))
	#$RichTextLabel3.text = "apprx: "+String(is_equal_approx(qTargetO.w,qBT.w))+" "+String(is_equal_approx(qTargetO.x,qBT.x))+" "+String(is_equal_approx(qTargetO.y,qBT.y))+" "+String(is_equal_approx(qTargetO.z,qBT.z))
	#if(qTargetO.is_equal_approx(qBT)):
	#if(abs(qTargetO.dot(qBT))>=0.99):
		
		#body_b.add_torque(-body_b.angular_velocity)
		#body_a.add_torque(-body_a.angular_velocity)
		#return
		
	
	var rotChange = qTargetO * qBT.inverse()
	
	var angle = 2.0 * acos(rotChange.w)
	#var angleDeg = angle* 180.0/PI
	#$RichTextLabel.text = "angle: "+String(angle)
	#$RichTextLabel3.text = "is_equal_approx(angle,0.0): "+String(abs(angle)<=0.001)
	#$RichTextLabel3.text = String(body_b.get_inverse_inertia_tensor().inverse())
	
	
	#if(is_zero_approx(angle)):
	#if(is_equal_approx(angle,0.001)):
	#if(abs(angle)<=0.001):
	#if(abs(angleDeg)<=0.001):
		#body_b.angular_velocity =Vector3(0,0,0)
		#body_a.angular_velocity =Vector3(0,0,0)
		#body_b.add_torque(-body_b.angular_velocity)
		#body_a.add_torque(-body_a.angular_velocity)
		#return
	var v = Vector3(rotChange.x,rotChange.y,rotChange.z)
	var axis = v / sin(angle*0.5)
	if(is_nan(angle)):
		if body_b.is_class("RigidBody"):
			body_b.add_torque(-bAV)
		if body_a.is_class("RigidBody"):
			body_a.add_torque(-aAV)
		return
	#$RichTextLabel2.text = "axis: "+String(axis)
	
	#if(angle>180.0):
	#	angle -= 360.0
	#angle *= PI/180.0
	if(angle>PI):
		angle -= 2.0*PI
	if(is_equal_approx(angle,0.0)):
		if body_b.is_class("RigidBody"):
			body_b.add_torque(-bAV)
		if body_a.is_class("RigidBody"):
			body_a.add_torque(-aAV)
		return
	
	#var targetAngVel = axis*angle/delta
	var targetAngVelA = axis*angle/delta
	var targetAngVelB = axis*angle/delta
	#var catchUP = 0.30
	#var catchUP = 1.0
	#targetAngVel *= catchUP
	
	#var t = targetAngVel - 550.0*(body_b.angular_velocity-body_a.angular_velocity)
	#var t = targetAngVel - 550.0*(bAV-aAV)
	#var t = targetAngVel - 5.0*(bAV-aAV)
	#var t = stiffness*targetAngVel - damp*(bAV-aAV)
	
	#var tb = (stiffnessB/massB)*targetAngVelB - dampingB*(bAV-aAV)
	#var ta = -(stiffnessA/massA)*targetAngVelA - dampingA*(aAV - bAV)
	
	#var tb = (stiffnessB/massTotal)*targetAngVelB - dampingB*(bAV-aAV)
	#var ta = -(stiffnessA/massTotal)*targetAngVelA - dampingA*(aAV - bAV)
	var tb = Vector3()
	var ta = Vector3()
	tb.x = (stiffnessB.x/massTotal)*targetAngVelB.x - dampingB.x*(bAV.x-aAV.x)
	ta.x = -(stiffnessA.x/massTotal)*targetAngVelA.x - dampingA.x*(aAV.x - bAV.x)
	
	tb.y = (stiffnessB.y/massTotal)*targetAngVelB.y - dampingB.y*(bAV.y-aAV.y)
	ta.y = -(stiffnessA.y/massTotal)*targetAngVelA.y - dampingA.y*(aAV.y - bAV.y)
	
	tb.z = (stiffnessB.z/massTotal)*targetAngVelB.z - dampingB.z*(bAV.z-aAV.z)
	ta.z = -(stiffnessA.z/massTotal)*targetAngVelA.z - dampingA.z*(aAV.z - bAV.z)
	
	
	#var tb = stiffnessB*targetAngVel - dampingB*dampB*(bAV)
	#var ta = -stiffnessA*targetAngVel - dampingA*dampA*(aAV)
	#var tb = stiffnessB*targetAngVelB - dampingB*dampB*(bAV)
	#var ta = -stiffnessA*targetAngVelA - dampingA*dampA*(aAV)


	if body_b.is_class("RigidBody") and body_b != null:
		if(self.name == "Joint1"):
			#$RichTextLabel.text = String("bingus")
			#$RichTextLabel.text = body_b.name
			if Input.is_action_pressed("1"):
				$RichTextLabel.text = String(body_a.angular_velocity)
				body_a.add_torque(Vector3(0,0,5000))
		#body_b.add_torque(t)
		
		body_b.add_torque(tb)
		#body_b.angular_velocity = t
	if body_a.is_class("RigidBody") and body_a != null:
		#body_a.add_torque(-t)
		body_a.add_torque(ta)
		pass
	
	#if(self.name == "Joint1"):
		#$RichTextLabel.text = String(targetAngVelA)
		#$RichTextLabel.text = body_a.name
		#$RichTextLabel.text = body_b.name
		#$RichTextLabel.text = String(ta)
		#$RichTextLabel.text = String(tb)
		#$RichTextLabel.text = String(body_a.angular_velocity)
		#$RichTextLabel.text = String(body_b.angular_velocity)