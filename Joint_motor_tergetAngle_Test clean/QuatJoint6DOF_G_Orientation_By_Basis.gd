tool
extends Generic6DOFJoint


export(Vector3) var stiffnessA = Vector3(10.0,10.0,10.0)
export(Vector3) var stiffnessB = Vector3(10.0,10.0,10.0)
export(Vector3) var dampingA = Vector3(5.0,5.0,5.0)
export(Vector3) var dampingB = Vector3(5.0,5.0,5.0)

export(NodePath) var puppeteer_node #use global_transfrom.basis directly to construct quaternion target orientation

var body_b
var body_a

var massTotal
var massA
var massB

var hasBase = true

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


func calc_target_orientation():#Don't need to calculate rotations if you already know where the target orientation should end up
	var BTargetBasis2 = get_node(puppeteer_node).global_transform.basis #can be any node as long as it has it's own basis vector
	return Quat(BTargetBasis2)
	

"""
Thanks to:
	DMGregory: https://gamedev.stackexchange.com/questions/182850/rotate-rigidbody-to-face-away-from-camera-with-addtorque/182873#182873
	and
	The Step Event: https://youtu.be/vewwP8Od_7s
	For the calculations
"""
func apply_rot_spring_quat(delta):# apply spring rotation using quaternion
	if Engine.editor_hint:
		return
	var bAV = Vector3()# Node B ANgular Velocity
	var aAV = Vector3()# Node A ANgular Velocity
	if body_b.is_class("RigidBody"):
		bAV = body_b.angular_velocity
	else:
		bAV = Vector3(0.0,0.0,0.0)

	if body_a.is_class("RigidBody"):
		aAV = body_a.angular_velocity
	else:
		aAV = Vector3(0.0,0.0,0.0)

	var qBT = Quat(body_b.global_transform.basis)#Quaternion Node B Transform Basis
	#Quaternion Target Orientation
	var qTargetO = calc_target_orientation() #Uses a Puppeteer target node's transform instead of rotation angles
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

	var tb_consolidated = (stiffnessB/massTotal)*targetAngVel - dampingB*(bAV-aAV)
	var ta_consolidated = -(stiffnessA/massTotal)* targetAngVel - dampingA*(aAV - bAV)

	if body_b.is_class("RigidBody") and body_b != null:
		body_b.add_torque(tb_consolidated)

	if body_a.is_class("RigidBody") and body_a != null:
		body_a.add_torque(ta_consolidated)
