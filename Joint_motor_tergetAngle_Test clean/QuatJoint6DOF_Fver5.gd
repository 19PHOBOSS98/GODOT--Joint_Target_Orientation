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

var JA_len
var JB_len
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
	JA_len = (A.global_transform.origin - global_transform.origin).length()
	JB_len = (B.global_transform.origin - global_transform.origin).length()
	hasBase = true

func SIGNAL_set_base_transform_original(B):# for dynamic node attachment in my game: IGNORE THIS
	if body_a == null or B == null:
		hasBase = false
		return
	baseBTOrig = body_a.global_transform.basis.inverse()*B.global_transform.basis
	baseATOrig = B.global_transform.basis.inverse()*body_a.global_transform.basis
	JA_len = (body_a.global_transform.origin - global_transform.origin).length()
	JB_len = (B.global_transform.origin - global_transform.origin).length()

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

	#var qx = Quat(Bbasis.x,(rest_angle_x+180.0)*PI/180.0)
	#var qy = Quat(Bbasis.y,(rest_angle_y+180.0)*PI/180.0)
	#var qz = Quat(Bbasis.z,(rest_angle_z+180.0)*PI/180.0)
	
	#var qx = Quat(Bbasis.x,(rest_angle_x-180.0)*PI/180.0)
	#var qy = Quat(Bbasis.y,(rest_angle_y-180.0)*PI/180.0)
	#var qz = Quat(Bbasis.z,(rest_angle_z-180.0)*PI/180.0)
	
	var qx = Quat(Bbasis.x,(-rest_angle_x)*PI/180.0)
	var qy = Quat(Bbasis.y,(-rest_angle_y)*PI/180.0)
	var qz = Quat(Bbasis.z,(-rest_angle_z)*PI/180.0)
	
	var qATargRo = qx*qy*qz# Quaternion node B Target Rotation; needs to be multiplied in the oppoiste oroder: qx*qy*qz
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
export(float) var zeta_kd = 1.0
var c = 0
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
	
	#if(c<1):
		#$RichTextLabel.text = "aAV: "+String(aAV)
		#$RichTextLabel2.text = "bAV: "+String(bAV)
		#c+=1
	
	var qB = Quat(body_b.global_transform.basis)
	var BT = calc_target_orientation_B(body_a.global_transform.basis)
	
	var qA = Quat(body_a.global_transform.basis)
	var AT = calc_target_orientation_A(body_b.global_transform.basis)
	#var AT2 = calc_target_orientation_A2(body_b.global_transform.basis)
	
	$A.global_transform.basis = AT
	#$A.global_transform.basis = AT2
	$B.global_transform.basis = BT
	var rotChangeB = Quat(BT) * qB.inverse()
	var angleB = 2.0 * acos(rotChangeB.w)
	if(is_nan(angleB)):
		if body_b.is_class("RigidBody"):
			#body_b.add_torque(-bAV)
			body_b.angular_velocity -= bAV
		if body_a.is_class("RigidBody"):
			#body_a.add_torque(-aAV)
			body_a.angular_velocity -= aAV
		return
	var vb = Vector3(rotChangeB.x,rotChangeB.y,rotChangeB.z)# rotation change quaternion "V" component
	var axisb = vb / sin(angleB*0.5)# the quats axis

	if(angleB>PI):
		angleB -= 2.0*PI


	var rotChangeA = Quat(AT) * qA.inverse()
	var angleA = 2.0 * acos(rotChangeA.w)
	if(is_nan(angleA)):
		if body_b.is_class("RigidBody"):
			#body_b.add_torque(-bAV)
			body_b.angular_velocity -= bAV
		if body_a.is_class("RigidBody"):
			#body_a.add_torque(-aAV)
			body_a.angular_velocity -= aAV
		return

	if(angleA>PI):
		angleA -= 2.0*PI

	if(is_equal_approx(angleA,0.0)):
		if body_b.is_class("RigidBody"):
			#body_b.add_torque(-bAV)
			body_b.angular_velocity -= bAV*0.5
		if body_a.is_class("RigidBody"):
			#body_a.add_torque(-aAV)
			body_a.angular_velocity -= aAV*0.5
		return


	if(is_equal_approx(angleB,0.0)):
		if body_b.is_class("RigidBody"):
			#body_b.add_torque(-bAV)
			body_b.angular_velocity -= bAV*0.5
		if body_a.is_class("RigidBody"):
			#body_a.add_torque(-aAV)
			body_a.angular_velocity -= aAV*0.5
		return

	var error_B = axisb*angleB/delta
	var va = Vector3(rotChangeA.x,rotChangeA.y,rotChangeA.z)# rotation change quaternion "V" component
	var axisa = va / sin(angleA*0.5)# the quats axis
	var error_A = axisa*angleA/delta
	
	var axisX = BT.x
	var axisZ = AT.z
	var ax = Basis()
	ax.y = axisZ.cross(axisX)
	ax.x = ax.y.cross(axisZ)
	ax.z = axisX.cross(ax.y)
	
	var aI_inv_ax = aI * ax
	var bI_inv_ax = bI * ax
	
	#$RichTextLabel.text = "aI * ax.x: "+String(aI * ax.x)
	#$RichTextLabel2.text = "(aI * ax).x: "+String((aI_inv_ax).x)
	#$RichTextLabel3.text = "(aI * ax).x - aI * ax.x: "+String((aI_inv_ax).x - aI * ax.x)
	
	var aI_inv_ax_mag = Vector3(aI_inv_ax.x.length(),aI_inv_ax.y.length(),aI_inv_ax.z.length())
	var bI_inv_ax_mag = Vector3(bI_inv_ax.x.length(),bI_inv_ax.y.length(),bI_inv_ax.z.length())

	#$RichTextLabel.text = "angleA: "+String(angleA)
	#$RichTextLabel2.text = "angleB: "+String(angleB)
	
	if body_a.is_class("RigidBody") && body_a.get_mode()== 0:
		massA = body_a.mass
	else:
		massA = 999999.0
	if body_b.is_class("RigidBody") && body_b.get_mode()== 0:
		massB = body_b.mass
	else:
		massB = 999999.0
	var mA = massA
	var mB = massB
	#$RichTextLabel.text = "mA: "+String(mA)
	#$RichTextLabel2.text = "mB: "+String(mB)
	#var mA = body_a.mass
	#var mB = body_b.mass


	var rrA = 2.0*JA_len*JA_len*(1-cos(angleA))
	#var rrA = 2.0*JA_len*JA_len*(1-cos(-angleB))
	var rrB = 2.0*JB_len*JB_len*(1-cos(angleB))
	#$RichTextLabel.text = "rrA: "+String(rrA)
	#$RichTextLabel2.text = "rrB: "+String(rrB)
	
	mA = (Vector3.ONE*mA)*rrA + aI_inv_ax_mag.inverse()
	mB = (Vector3.ONE*mB)*rrA + bI_inv_ax_mag.inverse()
	
	#$RichTextLabel.text = "mA: "+String(mA)
	#$RichTextLabel2.text = "mB: "+String(mB)
	#$RichTextLabel3.text = "mT: "+String(mT)
	
	var mT = Vector3()
	#if(body_a.mass>=999999.0):
	if(massB>=999999.0):
		mT = Vector3.ONE * mA
	#elif(body_b.mass>=999999.0):
	elif(massA>=999999.0):
		mT =Vector3.ONE * mB
	else:
		mT = Vector3.ONE * (mA*mB/(mA+mB))
	

	
	#mT_inv = mT.inverse()
	#var mT_inv = (Vector3.ONE * mT).inverse()
	var mT_inv = mT.inverse()
	var ang_freq = Vector3(sqrt(ks.x*mT_inv.x),sqrt(ks.y*mT_inv.y),sqrt(ks.z*mT_inv.z))
	
	if(0.25<ang_freq.x*delta):
		#ks.x = 16.0*mT.x/(delta*delta)
		ks.x = 1.0*mT.x/(delta*delta*16.0)
	if(0.25<ang_freq.y*delta):
		#ks.y = 16.0*mT.y/(delta*delta)
		ks.y = 1.0*mT.y/(delta*delta*16.0)
	if(0.25<ang_freq.z*delta):
		#ks.z = 16.0*mT.z/(delta*delta)
		ks.z = 1.0*mT.z/(delta*delta*16.0)

	var kd = Vector3()
	kd.x = zeta_kd*(2.0*sqrt(mT.x*ks.x))
	kd.y = zeta_kd*(2.0*sqrt(mT.y*ks.y))
	kd.z = zeta_kd*(2.0*sqrt(mT.z*ks.z))
	"""
	https://github.com/bulletphysics/bullet3/issues/345
	zeta > 1 -> overdamped
	zeta = 1 -> critically damped
	zeta < 1 -> underdamped
	"""
	
	if(kd.x*delta>mT.x):
		kd.x = mT.x/delta
	if(kd.y*delta>mT.y):
		kd.y = mT.y/delta
	if(kd.z*delta>mT.z):
		kd.z = mT.z/delta
		
	#$RichTextLabel.text = "kd: "+String(kd.z)
	#$RichTextLabel.text = "mT.z: "+String(mT.z)
	#$RichTextLabel2.text = "mT.z/delta: "+String(mT.z/delta)
	#$RichTextLabel2.text = "delta: "+String(delta)
	#$RichTextLabel3.text = "mT.z/delta diff: "+String(mT.z/delta - kd.z)
	$RichTextLabel3.text = "kd.z*delta>mT.z: "+String(kd.z*delta>mT.z)
	#if(c<2):
		#$RichTextLabel.text = "kd: "+String(kd*delta)
		#$RichTextLabel2.text = "ks: "+String(ks)
		#$RichTextLabel3.text = "error_B: "+String(error_B)
		#$RichTextLabel.text = "mA: "+String(mA)
		#$RichTextLabel2.text = "mB: "+String(mB)
		#$RichTextLabel3.text = "mT: "+String(mT)
		#c+=1
	#$RichTextLabel.text = "aAV: "+String(aAV)
	#$RichTextLabel2.text = "bAV: "+String(bAV)
	#var tb_consolidated = (stiffnessB/mT)*error_B - dampingB*(bAV-aAV)
	#var ta_consolidated = -(stiffnessA/mT)*error_B - dampingA*(aAV - bAV)

	#var tb_consolidated = (ks/mT)*error_B - kd*(bAV-aAV)
	#var ta_consolidated = -(ks/mT)*error_B - kd*(aAV-bAV)
	
	#var tb_consolidated = (ks/mT)*error_B*delta - kd*(bAV-aAV)*delta
	#var ta_consolidated = -(ks/mT)*error_B*delta - kd*(aAV-bAV)*delta
	#var ta_consolidated = (ks/mT)*error_A*delta + kd*(bAV - aAV)*delta
	
	var tb_consolidated = (ks)*error_B*delta - kd*(bAV-aAV)*delta
	#var ta_consolidated = -(ks)*error_B*delta - kd*(aAV-bAV)*delta
	#var ta_consolidated = (ks)*error_A*delta - kd*(aAV-bAV)*delta
	var ta_consolidated = (ks)*error_A*delta - kd*(bAV-aAV)*delta
	
	#var tb_consolidated = (ks/mT)*error_B - kd*(bAV-aAV)
	#var ta_consolidated = (ks/mT)*error_A - kd*(aAV-bAV)
	#var tb_consolidated = ((ks)*error_B + kd*(bAV-aAV))/mT
	#var ta_consolidated = ((ks)*error_A + kd*(bAV-aAV))/mT
	#$RichTextLabel.text = "tb_consolidated: "+String(tb_consolidated)
	#$RichTextLabel2.text = "ta_consolidated: "+String(ta_consolidated)
	"""
	if(c<2):
		$RichTextLabel.text = "tb_consolidated: "+String(tb_consolidated)
		$RichTextLabel2.text = "ta_consolidated: "+String(ta_consolidated)
		#$RichTextLabel3.text = "error_B: "+String(error_B)
		$RichTextLabel3.text = "mT: "+String(mT)
		c+=1
	"""
	
	if body_b.is_class("RigidBody") and body_b != null:
		#body_b.add_torque(tb_consolidated/mT)
		body_b.add_torque(tb_consolidated)
		#body_b.angular_velocity += tb_consolidated/mT

	if body_a.is_class("RigidBody") and body_a != null:
		#body_a.add_torque(ta_consolidated/mT)
		body_a.add_torque(ta_consolidated)
		#body_a.angular_velocity += ta_consolidated/mT

