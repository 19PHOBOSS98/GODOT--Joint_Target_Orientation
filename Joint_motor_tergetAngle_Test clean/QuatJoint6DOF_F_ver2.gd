tool
extends Generic6DOFJoint


export(Vector3) var stiffnessA = Vector3(10.0,10.0,10.0)
export(Vector3) var stiffnessB = Vector3(10.0,10.0,10.0)
export(Vector3) var dampingA = Vector3(1.0,1.0,1.0)
export(Vector3) var dampingB = Vector3(1.0,1.0,1.0)

export(float,-360.0,360.0) var rest_angle_x = 0.0
export(float,-360.0,360.0) var rest_angle_y = 0.0
export(float,-360.0,360.0) var rest_angle_z = 0.0

var body_b
var body_a

#export(NodePath) var baseBTOrigNode
#var baseBTOrigN



var massTotal
var massA
var massB

var hasBase = false

func _ready():
	if Engine.editor_hint:
		return
	prints("cos(thetaA): ",cos(0.0) )
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
		hasBase = false
		return
	if A.is_class("RigidBody"):
		body_a = A
		massA= A.mass
	else:
		massA = 999999.0
		#massA = 0.0

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
		#massB = 0.0

func set_total_mass():
	var mT
	if(massA == null || massB == null):
		hasBase = false
		return 
	#massTotal = massA+massB
	if(massA>=999999.0):
		mT = massB
	elif(massB>=999999.0):
		mT = massA
	else:
		mT = massA*massB/(massA+massB)
	return mT

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
var LaBTransO = Basis()
var LbATransO = Basis()
var AJo_len
var BJo_len
func set_base_transform_original(A,B):
	if A == null or B == null:
		hasBase = false
		return
	AJo_len = (global_transform.origin - body_a.global_transform.origin).length()
	BJo_len = (global_transform.origin - body_b.global_transform.origin).length()
	LaBTransO = body_a.global_transform.basis.inverse()*B.global_transform.basis
	LbATransO = body_b.global_transform.basis.inverse()*A.global_transform.basis
	
	hasBase = true

func SIGNAL_set_base_transform_original(B):# for dynamic node attachment in my game: IGNORE THIS
	if body_a == null or B == null:
		hasBase = false
		return
	AJo_len = (global_transform.origin - body_a.global_transform.origin).length()
	BJo_len = (global_transform.origin - B.global_transform.origin).length()
	LaBTransO = body_a.global_transform.basis.inverse()*B.global_transform.basis
	LbATransO = B.global_transform.basis.inverse()*body_a.global_transform.basis
	hasBase = true

var qBT = Quat()
var qAT = Quat()
var ax = Basis()
func calc_target_orientation(Abasis:Basis,Bbasis:Basis):

	var GBTransO = Abasis*LaBTransO#the actual original base node B Transform following node A around in current global space
	var GATransO = Bbasis*LbATransO
	
	
	var qx_b = Quat(Abasis.x,rest_angle_x*PI/180.0)
	var qy_b = Quat(Abasis.y,rest_angle_y*PI/180.0)
	var qz_b = Quat(Abasis.z,rest_angle_z*PI/180.0)
	
	var qBTargRo = qz_b*qy_b*qx_b# Quaternion node B Target Rotation
	var BTargetBasis = Basis()
	BTargetBasis.x =  qBTargRo*GBTransO.x
	BTargetBasis.y =  qBTargRo*GBTransO.y
	BTargetBasis.z =  qBTargRo*GBTransO.z
	
	#$B.global_transform.basis = BTargetBasis
	#$B.global_transform.origin = GBTransO

	qBT= Quat(BTargetBasis)
	#return BTargetBasis
	var qx_a = Quat(Bbasis.x,-rest_angle_x*PI/180.0)
	var qy_a = Quat(Bbasis.y,-rest_angle_y*PI/180.0)
	var qz_a = Quat(Bbasis.z,-rest_angle_z*PI/180.0)
	
	var qATargRo = qz_a*qy_a*qx_a# Quaternion node B Target Rotation
	var ATargetBasis = Basis()
	ATargetBasis.x =  qATargRo*GATransO.x
	ATargetBasis.y =  qATargRo*GATransO.y
	ATargetBasis.z =  qATargRo*GATransO.z
	
	#$A.global_transform.basis = ATargetBasis
	
	
	qAT= Quat(ATargetBasis)
	
	var axisX = BTargetBasis.x
	var axisZ = ATargetBasis.z
	ax.y = axisZ.cross(axisX)
	ax.x = ax.y.cross(axisZ)
	ax.z = axisX.cross(ax.y)

"""
babypandabear3: https://www.reddit.com/r/godot/comments/q85jgi/moving_a_rigidbody_to_a_specific_rotation_using/
"""
#func calc_angular_velocity(from_basis: Basis, to_basis: Basis,delta) -> Vector3:
var thetaA = 0.0
var thetaB = 0.0
func calc_angular_velocity(qfrom_basis: Quat, qto_basis: Quat,delta,isA:bool) -> Vector3:
	#var q1 = from_basis.get_rotation_quat()
	#var q2 = to_basis.get_rotation_quat()
	#var q1 = from_basis.get_rotation_quat()
	var q1 = qfrom_basis
	var q2 = qto_basis

	# Quaternion that transforms q1 into q2
	var qt = q2 * q1.inverse()

	# Angle from quaternion
	var angle = 2 * acos(qt.w)
	if(is_nan(angle)):
		if(isA):
			thetaA = 0.0
		else:
			thetaB = 0.0
		return Vector3.ZERO
	# There are two distinct quaternions for any orientation.
	# Ensure we use the representation with the smallest angle.
	if angle > PI:
		qt = -qt
		angle = TAU - angle

	# Prevent divide by zero
	#if angle < 0.0001:
	if(is_equal_approx(angle,0.0)):
		if(isA):
			thetaA = 0.0
		else:
			thetaB = 0.0
		return Vector3.ZERO
	
	if(isA):
		thetaA = angle
	else:
		thetaB = angle
	# Axis from quaternion
	var axis = Vector3(qt.x, qt.y, qt.z) / sqrt(1-qt.w*qt.w)

	return axis * angle/delta

"""
Thanks to:
	DMGregory: https://gamedev.stackexchange.com/questions/182850/rotate-rigidbody-to-face-away-from-camera-with-addtorque/182873#182873
	and
	The Step Event: https://youtu.be/vewwP8Od_7s
	For the calculations
"""

export(Vector3) var ks = Vector3(1.0,1.0,1.0)
export(Vector3) var kd = Vector3(1.0,1.0,1.0)
func apply_rot_spring_quat(delta):# apply spring rotation using quaternion
	if Engine.editor_hint:
		return
	var bW = Vector3()# Node B ANgular Velocity
	var aW = Vector3()# Node A ANgular Velocity
	var bI = Vector3()
	var aI = Vector3()
	if body_b.is_class("RigidBody"):
		bW = body_b.angular_velocity
		bI = body_b.get_inverse_inertia_tensor()
	else:
		bW = Vector3(0.0,0.0,0.0)
	
	

	if body_a.is_class("RigidBody"):
		aW = body_a.angular_velocity
		aI = body_a.get_inverse_inertia_tensor()
	else:
		aW = Vector3(0.0,0.0,0.0)
		
	#$RichTextLabel.text = "aW: "+String(aW)
	#$RichTextLabel2.text = "bW: "+String(bW)
	#$RichTextLabel.text = "a: "+String(body_a.global_transform.origin)
	#$RichTextLabel2.text = "b: "+String(body_b.global_transform.origin)

	
	var qA = Quat(body_a.global_transform.basis)
	var qB = Quat(body_b.global_transform.basis)
	
	calc_target_orientation(body_a.global_transform.basis,body_b.global_transform.basis)#qAT and qBT
	
	#var error_A = calc_angular_velocity(qA, qAT,delta,true)
	#$RichTextLabel.text = "error_A: "+String(error_A)
	var error_B = calc_angular_velocity(qB, qBT,delta,false)
	if(is_equal_approx(thetaA,0.001)|| is_equal_approx(thetaB,0.001)):
		body_a.add_torque(-aW)
		body_b.add_torque(-bW)
		return
	#$RichTextLabel.text = "thetaA: "+String(thetaA)
	#$RichTextLabel2.text = "thetaB: "+String(thetaB)
	#$RichTextLabel2.text = "error_B: "+String(error_B)
	#$RichTextLabel.text = "error_B: "+String(error_B)
	#$RichTextLabel2.text = "error_A: "+String(error_A)

	
	var rrA = 2.0 * (AJo_len*AJo_len) *(1.0 - cos(thetaA))
	var rrB = 2.0 * (BJo_len*BJo_len) *(1.0 - cos(thetaB))

		
	var mA = Vector3.ONE*body_a.mass
	var mB = Vector3.ONE*body_b.mass
	var aI_Inv_ax = aI*ax
	var aI_Inv_ax_mag = Vector3(aI_Inv_ax.x.length(),aI_Inv_ax.y.length(),aI_Inv_ax.z.length())
	
	var bI_Inv_ax = bI*ax
	var bI_Inv_ax_mag = Vector3(bI_Inv_ax.x.length(),bI_Inv_ax.y.length(),bI_Inv_ax.z.length())

	mA = mA *rrA + aI_Inv_ax_mag.inverse()
	mB = mB *rrB + bI_Inv_ax_mag.inverse()
	
	var mT = Vector3()
	if(body_a.mass>=999999.0):
		mT = mB
	elif(body_b.mass>=999999.0):
		mT = mA
	else:
		mT = mA*mB/(mA+mB)
	var angular_freq = Vector3(sqrt(ks.x*mT.x),sqrt(ks.y*mT.y),sqrt(ks.z*mT.z))
	
	if(0.25 < angular_freq.x*delta):
		ks.x = 16.0*mT.x/(delta*delta)
	if(0.25 < angular_freq.y*delta):
		ks.y = 16.0*mT.y/(delta*delta)
	if(0.25 < angular_freq.z*delta):
		ks.z = 16.0*mT.z/(delta*delta)
	
	if(kd.x*delta > mT.x):
		kd.x = mT.x/delta
	if(kd.y*delta > mT.y):
		kd.y = mT.y/delta
	if(kd.z*delta > mT.z):
		kd.z = mT.z/delta
		
	#var fsB = ks * error_B*delta
	#var fsA = ks * error_A*delta
	var fsB = ks * error_B*delta
	var fsA = ks * error_B*delta
	var fdB = kd*bW*delta
	var fdA = kd*aW*delta
	
	var fB = fsB +fdB
	var fA = fsA +fdA
	#var fA = -fsB +fdB


	
	
	var tb_consolidated = (ks)*error_B/mT - kd*(bW)
	var ta_consolidated = -(ks)*error_B/mT - kd*(aW)
	
	#$RichTextLabel.text = "aW: "+String(body_a.angular_velocity)
	#$RichTextLabel2.text = "bW: "+String(body_b.angular_velocity)
	
	
	var maxfB = Vector3()
	var maxfA = Vector3()
	maxfB.x = fB.x if fB.x<fdB.x else fdB.x
	maxfB.y = fB.y if fB.y<fdB.y else fdB.y
	maxfB.z = fB.z if fB.z<fdB.z else fdB.z
	
	maxfA.x = fA.x if fA.x<fdA.x else fdA.x
	maxfA.y = fA.y if fA.y<fdA.y else fdA.y
	maxfA.z = fA.z if fA.z<fdA.z else fdA.z
	
	var m_lowerLimit = Vector3()
	var m_upperLimit = Vector3()

	
	if body_b.is_class("RigidBody") and body_b != null:
		body_b.add_torque(tb_consolidated)
		#body_b.add_torque(fB)

	if body_a.is_class("RigidBody") and body_a != null:
		body_a.add_torque(ta_consolidated)
		#body_a.add_torque(fA)
