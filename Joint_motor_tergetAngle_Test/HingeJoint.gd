tool
extends HingeJoint


# Declare member variables here. Examples:
# var a = 2
# var b = "text"

export(float) var stiffness = 1.0
export(float) var damping = 1.0
export(float) var rest_angle = 0.0
var rest_angle_rads = 0.0
var body_b
var body_a
var bI
var aI
var dampA
var dampB

var baseBTOrig
# Called when the node enters the scene tree for the first time.
func _ready():
	if Engine.editor_hint:
		return
	body_a = get_node(get_node_a())
	body_b = get_node(get_node_b())
	"""
	aI = 0.0#body_a.get_inverse_inertia_tensor()
	bI = body_b.get_inverse_inertia_tensor().inverse()
	"""
	#rest_angle_rads = normailse_angle(rest_angle*PI/180.0)
	rest_angle_rads = normailse_angle(rest_angle)
	var massA
	var massB
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
	#damp = 2000
	#damp = 2.0*sqrt(stiffness*(massB+massA))
	#damp = 2.0*sqrt(stiffness*(massB+massA))*0.05
	#damp = 2.0*sqrt(stiffness*(massB+massA))*0.005
	dampA = 2.0*sqrt(stiffness*(massA))
	dampB = 2.0*sqrt(stiffness*(massB))
	
	prints("dampA: ")
	prints(dampA)
	prints("dampB: ")
	prints(dampB)
	
	baseBTOrig = body_b.global_transform.basis
	
	prints(shortest_angle_to( 45.0, 0.0))
	prints(shortest_angle_to( 45.0, 90.0))
	prints(shortest_angle_to( 45.0, 180.0))
	prints(shortest_angle_to( 45.0, 360.0))
	"""
	prints(normailse_angle(179.999))
	prints(normailse_angle(180.0))
	prints(normailse_angle(-179.999))
	prints(normailse_angle(-180.0))
	prints(normailse_angle(90.0))
	prints(normailse_angle(-90.0))
	prints(normailse_angle(45.0))
	prints(normailse_angle(-45.0))
	prints(normailse_angle(135.0))
	prints(normailse_angle(-135.0))
	prints(normailse_angle(181.0))
	prints(normailse_angle(-181.0))
	prints(normailse_angle(270.0))
	prints(normailse_angle(-270.0))
	prints(normailse_angle(360.0))
	prints(normailse_angle(-360.0))
	prints(normailse_angle(0.0))
	prints(normailse_angle(-0.0))
	prints(normailse_angle(630.0))
	prints(normailse_angle(-630.0))
	"""


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta):
	apply_rot_spring()

func step(edge,value):
	return 1.0 - max(sign(edge - value),0.0)

func normailse_angle(ang):
	prints("ang: "+String (ang))
	var absosule_angle = abs(ang)
	var mod360 = fmod(absosule_angle,360.0)
	if(mod360 == 180.0):
		return ang
	var sign_angle = sign(ang)
	#var overstep = (max(sign(180.0 - mod360),0.0) - 1.0)*(-1.0)
	var overstep = step(180.0,mod360)
	#prints("overstep: "+String(overstep))
	return fmod(mod360,180.0) * sign_angle - 180.0 * overstep * sign_angle


func shortest_angle_to( to, from ):
	var diff = fmod(( to - from + 180.0 ),360.0) - 180.0
	var overstep = 1.0 - step(-180.0,diff)
	return diff + 360.0*overstep
	#var diff = ( to - from + 180 ) % 360 - 180
	#return diff < -180 ? diff + 360 : diff

func v3_step(edge,value):
	##return 1.0 - max(sign(edge - value),0.0)
	var comp = Vector3()
	comp.x = 1.0 - max(sign(edge - value.x),0.0)
	comp.z = 1.0 - max(sign(edge - value.y),0.0)
	comp.z = 1.0 - max(sign(edge - value.z),0.0)
	return comp
	 
func v3_shortest_angle_to( to, from ):
	var diff = Vector3()
	diff.x = fmod(( to.x - from.x + 180.0 ),360.0) - 180.0
	diff.y = fmod(( to.y - from.y + 180.0 ),360.0) - 180.0
	diff.z = fmod(( to.z - from.z + 180.0 ),360.0) - 180.0
	var overstep = Vector3(1.0,1.0,1.0) - v3_step(-180.0,diff)
	return diff + 360.0*overstep
	#var diff = ( to - from + 180 ) % 360 - 180
	#return diff < -180 ? diff + 360 : diff

func apply_rot_spring():
	if Engine.editor_hint:
		return
	var k = stiffness
	var d = damping
	#var d = damp
	var l = rest_angle_rads
	var bRot
	var aRot
	if body_b.is_class("RigidBody"):
		bRot = body_b.rotation_degrees
		#bRot = to_local(body_b.rotation_degrees)
	else:
		bRot = Vector3(0.0,0.0,0.0)
	if body_a.is_class("RigidBody"):
		aRot = body_a.rotation_degrees
		#aRot = to_local(body_a.rotation_degrees)
	else:
		aRot = Vector3(0.0,0.0,0.0)
	$RichTextLabel.text = String(bRot)
	
	
	var bAV = Vector3()
	var aAV = Vector3()
	
	if body_b.is_class("RigidBody"):
		bAV = body_b.angular_velocity
		#bAVx. = body_b.angular_velocity
		
	else:
		bAV = Vector3(0.0,0.0,0.0)
	if body_a.is_class("RigidBody"):
		aAV = body_a.angular_velocity
		#aAV = to_local(body_a.angular_velocity)
	else:
		aAV = Vector3(0.0,0.0,0.0)
	
	var Ta = aI*aAV
	var Tb = bI*bAV
	#$RichTextLabel.text = String(u.z)
	
	#var zA = shortest_angle_to(bRot.z,aRot.z)
	#var zB = shortest_angle_to(aRot.z,bRot.z)
	
	var zA = v3_shortest_angle_to(bRot,aRot)
	var zB = v3_shortest_angle_to(aRot,bRot)
	
	#var l_to_zA = shortest_angle_to(zA, l) * (PI/180.0)
	#var l_to_zB = shortest_angle_to(zB, l) * (PI/180.0)
	
	#var l_to_z = shortest_angle_to(zA, l) * (PI/180.0)
	#var global_l = self.transform.basis.xform(Vector3(0,0,l))#transform local/global?
	#var l_to_z = v3_shortest_angle_to(zA,global_l) * (PI/180.0)
	var l_to_z = v3_shortest_angle_to(zA,Vector3(0,0,l)) * (PI/180.0)

	#var torque_a = - k * Vector3(0,0,l_to_zA) - Ta
	#var torque_b = - k * Vector3(0,0,l_to_zB) - Tb
	
	#var kz = - k*Vector3(0,0,l_to_z)
	var kz = - k*l_to_z
	
	#var torque_a = kz - Ta
	#var torque_b = kz - Tb
	#$RichTextLabel.text = String(Tb)+"/n"+String(Ta)
	var torque_a = kz - d*(bAV - aAV)
	#var torque_a = to_global(kz - d*(Tb - Ta))
	
	#var torque_b =kz - dampB*Tb
	#var torque_a =kz - dampA*Ta
	
	
	if body_a.is_class("RigidBody"):
		#body_a.add_torque(Vector3(0,0,-torque_a.z))
		body_a.add_torque(-torque_a)
		#body_a.add_torque(Vector3(0,0,-torque_a.z))
	if body_b.is_class("RigidBody"):
		#body_b.add_torque(Vector3(0,0,torque_a.z))
		body_b.add_torque(torque_a)
		#body_b.add_torque(Vector3(0,0,-torque_b.z))
	"""
	var u = bAV - aAV
	#$RichTextLabel.text = String(u.z)
	var x = shortest_angle_to(bRot.z,aRot.z)
	var l_to_x = shortest_angle_to(x, l)
	l_to_x *= (PI/180.0)
	#var torque = - k * Vector3(0,0,l_to_x) - (d * u)
	var torque = - k * Vector3(0,0,l_to_x) - d * u
	if body_a.is_class("RigidBody"):
		body_a.add_torque(Vector3(0,0,-torque.z))
	if body_b.is_class("RigidBody"):
		body_b.add_torque(Vector3(0,0,torque.z))
	"""
	
	
func apply_rot_spring_z():
	if Engine.editor_hint:
		return
	var k = stiffness
	var d = damping
	#var d = damp
	var l = rest_angle_rads
	var bRot
	var aRot
	if body_b.is_class("RigidBody"):
		bRot = body_b.rotation_degrees
		#bRot = to_local(body_b.rotation_degrees)
	else:
		bRot = Vector3(0.0,0.0,0.0)
	if body_a.is_class("RigidBody"):
		aRot = body_a.rotation_degrees
		#aRot = to_local(body_a.rotation_degrees)
	else:
		aRot = Vector3(0.0,0.0,0.0)
	$RichTextLabel.text = String(bRot)
	
	
	var bAV = Vector3()
	var aAV = Vector3()
	
	if body_b.is_class("RigidBody"):
		bAV = body_b.angular_velocity
		#bAVx. = body_b.angular_velocity
		
	else:
		bAV = Vector3(0.0,0.0,0.0)
	if body_a.is_class("RigidBody"):
		aAV = body_a.angular_velocity
		#aAV = to_local(body_a.angular_velocity)
	else:
		aAV = Vector3(0.0,0.0,0.0)
	
	var Ta = aI*aAV
	var Tb = bI*bAV
	#$RichTextLabel.text = String(u.z)
	
	#var zA = shortest_angle_to(bRot.z,aRot.z)
	#var zB = shortest_angle_to(aRot.z,bRot.z)
	
	var zA = v3_shortest_angle_to(bRot,aRot)
	var zB = v3_shortest_angle_to(aRot,bRot)
	
	#var l_to_zA = shortest_angle_to(zA, l) * (PI/180.0)
	#var l_to_zB = shortest_angle_to(zB, l) * (PI/180.0)
	
	#var l_to_z = shortest_angle_to(zA, l) * (PI/180.0)
	#var global_l = self.transform.basis.xform(Vector3(0,0,l))#transform local/global?
	#var l_to_z = v3_shortest_angle_to(zA,global_l) * (PI/180.0)
	var l_to_z = v3_shortest_angle_to(zA,Vector3(0,0,l)) * (PI/180.0)

	#var torque_a = - k * Vector3(0,0,l_to_zA) - Ta
	#var torque_b = - k * Vector3(0,0,l_to_zB) - Tb
	
	#var kz = - k*Vector3(0,0,l_to_z)
	var kz = - k*l_to_z
	
	#var torque_a = kz - Ta
	#var torque_b = kz - Tb
	#$RichTextLabel.text = String(Tb)+"/n"+String(Ta)
	var torque_a = kz - d*(bAV - aAV)
	#var torque_a = to_global(kz - d*(Tb - Ta))
	
	#var torque_b =kz - dampB*Tb
	#var torque_a =kz - dampA*Ta
	
	
	if body_a.is_class("RigidBody"):
		#body_a.add_torque(Vector3(0,0,-torque_a.z))
		body_a.add_torque(-torque_a)
		#body_a.add_torque(Vector3(0,0,-torque_a.z))
	if body_b.is_class("RigidBody"):
		#body_b.add_torque(Vector3(0,0,torque_a.z))
		body_b.add_torque(torque_a)
		#body_b.add_torque(Vector3(0,0,-torque_b.z))
	
	
func calc_diff_to_targ(A:Basis,B:Basis):
	var Abasis = A
	var baseBTCurr = Abasis*baseBTOrig
	var BTC = Abasis * B
	#print("Abasis: ",Abasis.x,Abasis.y,Abasis.z)
	#print("baseBTCurr: ",baseBTCurr.x,baseBTCurr.y,baseBTCurr.z)
	
	var qx = Quat(Abasis.x,0.0*PI/180.0)
	var qy = Quat(Abasis.y,90.0*PI/180.0)
	var qz = Quat(Abasis.z,0.0*PI/180.0)
	var qBTargRo = qz*qy*qx
	#print(" qBTargRo: ", qBTargRo)
	var BTargetBasis = Basis()
	BTargetBasis.x =  qBTargRo*baseBTCurr.x
	BTargetBasis.y =  qBTargRo*baseBTCurr.y
	BTargetBasis.z =  qBTargRo*baseBTCurr.z
	#print("BTargetBasis: ",BTargetBasis.x,BTargetBasis.y,BTargetBasis.z)
	var qBTargetBasis = Quat(BTargetBasis)
	var qBTC = Quat(BTC)
	var thetaDiff = qBTC.angle_to(qBTargetBasis)*180.0/PI
	#prints("thetaDiff: ",qBTC.angle_to(qBTargetBasis)*180.0/PI)
	$RichTextLabel.text = String(BTargetBasis.x)+" "+String(BTargetBasis.y)+" "+String(BTargetBasis.z)
	$RichTextLabel2.text = String(thetaDiff)



func apply_rot_spring_quat():
	if Engine.editor_hint:
		return
	var k = stiffness
	var d = damping
	#var d = damp
	var l = rest_angle_rads
	var bRot
	var aRot
	if body_b.is_class("RigidBody"):
		bRot = body_b.rotation_degrees
	else:
		bRot = Vector3(0.0,0.0,0.0)
	if body_a.is_class("RigidBody"):
		aRot = body_a.rotation_degrees
	else:
		aRot = Vector3(0.0,0.0,0.0)
	
	
	var bAV = Vector3()
	var aAV = Vector3()
	
	if body_b.is_class("RigidBody"):
		bAV = body_b.angular_velocity
		
	else:
		bAV = Vector3(0.0,0.0,0.0)
	if body_a.is_class("RigidBody"):
		aAV = body_a.angular_velocity
	else:
		aAV = Vector3(0.0,0.0,0.0)
	
	var Ta = aI*aAV
	var Tb = bI*bAV

	var zA = v3_shortest_angle_to(bRot,aRot)

	#var l_to_z = v3_shortest_angle_to(zA,Vector3(0,0,l)) * (PI/180.0)

	#var kz = - k*l_to_z
	var A = body_a.global_transform.basis
	var B = body_b.global_transform.basis
	var l_to_z = calc_diff_to_targ(A,B)
	var kz = - k*l_to_z
	
	var torque_a = kz - d*(bAV - aAV)

	
	
	if body_a.is_class("RigidBody"):
		body_a.add_torque(-torque_a)
	if body_b.is_class("RigidBody"):
		body_b.add_torque(torque_a)
	
