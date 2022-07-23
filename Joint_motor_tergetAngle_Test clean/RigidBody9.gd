extends RigidBody

var baseBTOrig
var rest_angle_x = 0
var rest_angle_y = 0
var rest_angle_z = 0
export(NodePath) var puppeteer_node

func _ready():
	baseBTOrig = global_transform.basis
	prints("baseBTOrig: ",baseBTOrig)
	prints(Basis(Vector3(1,0,0),Vector3(0,1,0),Vector3(0,0,1)))
	pass

func calc_target_orientation_basis():#Don't need to calculate rotations if you already know where the target orientation should end up
	var BTargetBasis2 = get_node(puppeteer_node).global_transform.basis #can be any node as long as it has it's own basis vector
	return Quat(BTargetBasis2)

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
	
	
func _physics_process(delta):
	var I_inv = get_inverse_inertia_tensor()
	var aV = angular_velocity
	
	var T = Vector3()
	
	var qBT = Quat(global_transform.basis)#Quaternion Node B Transform Basis
	#Quaternion Target Orientation
	#var qTargetO = calc_target_orientation(Basis(Vector3(1,0,0),Vector3(0,1,0),Vector3(0,0,1))) #Does the same thing but without the Position3D node
	var qTargetO = calc_target_orientation_basis()

	var rotChange = qTargetO * qBT.inverse() #rotation change quaternion
	
	var angle = 2.0 * acos(rotChange.w) #Turns to angle radians, the amount it turns around the quats axis

	#if node B's quat is already facing the same way as qTargetO the axis shoots to infinity
	#this is my sorry ass attempt to protect us from it
#	var v = Vector3()
#	var axis = Vector3()

#	if(is_nan(angle)):
#		axis = Vector3()
#	else:
#		v = Vector3(rotChange.x,rotChange.y,rotChange.z)# rotation change quaternion "V" component
#		axis = v / sin(angle*0.5)# the quats axis
#
#	if(angle>PI):
#		angle -= 2.0*PI
#	var targetAng = Vector3()
#	if(!is_equal_approx(angle,0.0)):
#		targetAng = axis*angle
#
#	var deltaSqr = delta*delta


	if(is_nan(angle)):
		add_torque(I_inv * -angular_velocity/delta)
		return

	var v = Vector3(rotChange.x,rotChange.y,rotChange.z)# rotation change quaternion "V" component
	var axis = v / sin(angle*0.5)# the quats axis
#
	if(angle>PI):
		angle -= 2.0*PI
#
#	#as node B's quat faces the same way as qTargetO the angle nears 0
#	#this slows it down to stop the axis from reaching infinity
	if(is_equal_approx(angle,0.0)):
		add_torque(I_inv *-angular_velocity/delta)
		return


	var targetAng = axis*angle
	var deltaSqr = delta*delta
	
	if Input.is_action_pressed("rotate"):
		T = I_inv * -targetAng/deltaSqr - I_inv * angular_velocity/delta
		self.add_torque(T)
	$RichTextLabel.text = "targetAng: "+String(targetAng)+"\na_v: \n"+String(self.angular_velocity)
