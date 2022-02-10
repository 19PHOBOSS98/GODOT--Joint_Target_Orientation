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
	#set_total_mass()
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
var baseBTOrig = Basis()
var baseATOrig = Basis()
func set_base_transform_original(A,B):
	if A == null or B == null:
		hasBase = false
		return
	baseBTOrig = A.global_transform.basis.inverse()*B.global_transform.basis
	baseATOrig = B.global_transform.basis.inverse()*A.global_transform.basis
	hasBase = true

func SIGNAL_set_base_transform_original(B):# for dynamic node attachment in my game: IGNORE THIS
	if body_a == null or B == null:
		hasBase = false
		return
	baseBTOrig = body_a.global_transform.basis.inverse()*B.global_transform.basis
	baseATOrig = B.global_transform.basis.inverse()*body_a.global_transform.basis

	hasBase = true


func calc_target_orientation_B(Abasis:Basis):

	var baseBTOActual = Abasis*baseBTOrig#the actual original base node B Transform following node A around in current global space

	var qx = Quat(Abasis.x,rest_angle_x*PI/180.0)
	var qy = Quat(Abasis.y,rest_angle_y*PI/180.0)
	var qz = Quat(Abasis.z,rest_angle_z*PI/180.0)
	
	var qBTargRo = qz*qy*qx# Quaternion node B Target Rotation
	var BTargetBasis = Basis()
	BTargetBasis.x =  qBTargRo*baseBTOActual.x
	BTargetBasis.y =  qBTargRo*baseBTOActual.y
	BTargetBasis.z =  qBTargRo*baseBTOActual.z

	#return Quat(BTargetBasis)
	return BTargetBasis
	
func calc_target_orientation_A(Bbasis:Basis):

	var baseATOActual = Bbasis*baseATOrig#the actual original base node B Transform following node A around in current global space

	var qx = Quat(Bbasis.x,-rest_angle_x*PI/180.0)
	var qy = Quat(Bbasis.y,-rest_angle_y*PI/180.0)
	var qz = Quat(Bbasis.z,-rest_angle_z*PI/180.0)
	
	var qATargRo = qz*qy*qx# Quaternion node B Target Rotation
	var ATargetBasis = Basis()
	ATargetBasis.x =  qATargRo*baseATOActual.x
	ATargetBasis.y =  qATargRo*baseATOActual.y
	ATargetBasis.z =  qATargRo*baseATOActual.z

	#return Quat(BTargetBasis)
	return ATargetBasis
	
"""
Thanks to:
	DMGregory: https://gamedev.stackexchange.com/questions/182850/rotate-rigidbody-to-face-away-from-camera-with-addtorque/182873#182873
	and
	The Step Event: https://youtu.be/vewwP8Od_7s
	For the calculations
"""
export(Vector3) var ks = Vector3(5.0,5.0,5.0)
export(Vector3) var kd = Vector3(5.0,5.0,5.0)
func apply_rot_spring_quat(delta):# apply spring rotation using quaternion
	if Engine.editor_hint:
		return
	var bAV = Vector3()# Node B ANgular Velocity
	var aAV = Vector3()# Node A ANgular Velocity
	var aI = Basis()
	var bI = Basis()
	if body_b.is_class("RigidBody"):
		bAV = body_b.angular_velocity
		bI = body_b.get_inverse_inertia_tensor()
	else:
		bAV = Vector3(0.0,0.0,0.0)

	if body_a.is_class("RigidBody"):
		aAV = body_a.angular_velocity
		bI = body_b.get_inverse_inertia_tensor()
	else:
		aAV = Vector3(0.0,0.0,0.0)


	var qB = Quat(body_b.global_transform.basis)
	var BT = calc_target_orientation_B(body_a.global_transform.basis)
	var AT = calc_target_orientation_B(body_b.global_transform.basis)
	
	var rotChangeB = Quat(BT) * qB.inverse()
	var angleB = 2.0 * acos(rotChangeB.w)
	if(is_nan(angleB)):
		if body_b.is_class("RigidBody"):
			body_b.add_torque(-bAV)
		if body_a.is_class("RigidBody"):
			body_a.add_torque(-aAV)
		return

	var v = Vector3(rotChangeB.x,rotChangeB.y,rotChangeB.z)# rotation change quaternion "V" component
	var axis = v / sin(angleB*0.5)# the quats axis

	if(angleB>PI):
		angleB -= 2.0*PI


	if(is_equal_approx(angleB,0.0)):
		if body_b.is_class("RigidBody"):
			body_b.add_torque(-bAV)
		if body_a.is_class("RigidBody"):
			body_a.add_torque(-aAV)
		return

	var error_B = axis*angleB/delta
	
	var axisX = BT.x
	var axisZ = AT.z
	var ax = Basis()
	ax.y = axisZ.cross(axisX)
	ax.x = ax.y.cross(axisZ)
	ax.z = axisX.cross(ax.y)
	
	var aI_inv_ax = aI * ax
	var bI_inv_ax = bI * ax
	
	var aI_inv_ax_mag = Vector3()
	var bI_inv_ax_mag
	
	var mA = body_a.mass
	var mB = body_b.mass
	if(mA>=999999.0):
		pass
	var mT = mA*mB/(mA+mB)
	
	#mT_inv = mT.inverse()
	var mT_inv = (Vector3.ONE * mT).inverse()
	var ang_freq = Vector3(sqrt(ks.x*mT_inv.x),sqrt(ks.y*mT_inv.y),sqrt(ks.z*mT_inv.z))
	if(0.25<ang_freq.x*delta):
		ks.x = 16.0*mT/(delta*delta)
	if(0.25<ang_freq.y*delta):
		ks.y = 16.0*mT/(delta*delta)
	if(0.25<ang_freq.z*delta):
		ks.z = 16.0*mT/(delta*delta)

	if(kd.x*delta>mT):
		kd.x = mT/delta
	if(kd.y*delta>mT):
		kd.y = mT/delta
	if(kd.z*delta>mT):
		kd.z = mT/delta
		
	#var tb_consolidated = (stiffnessB/mT)*error_B - dampingB*(bAV-aAV)
	#var ta_consolidated = -(stiffnessA/mT)*error_B - dampingA*(aAV - bAV)
	var tb_consolidated = (ks/mT)*error_B - kd*(bAV-aAV)
	var ta_consolidated = -(ks/mT)*error_B - kd*(aAV-bAV)

	if body_b.is_class("RigidBody") and body_b != null:
		body_b.add_torque(tb_consolidated)

	if body_a.is_class("RigidBody") and body_a != null:
		body_a.add_torque(ta_consolidated)

