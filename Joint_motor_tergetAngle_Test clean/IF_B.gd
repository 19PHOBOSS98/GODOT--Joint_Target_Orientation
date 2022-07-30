extends RigidBody

export var w = Vector3(0,0,0)
export var J_trq = Vector3(0,0,0)
export(NodePath) var body_a
export(NodePath) var ppt
export(float, 0,1) var stiffnessB = 0.05
export(float, 0,1) var dampingB = 1.0
export var torque_lim = 50000.0

func add_Basis_by_Element(a,b):
	var ab = Basis(a.x+b.x,a.y+b.y,a.z+b.z)
	return ab
	
func calc_target_orientation_by_basis():
	var BTargetBasis = get_node(ppt).global_transform.basis #can be any node as long as it has it's own basis vector
	return Quat(BTargetBasis)
	
func max_vector_magnitude(vec:Vector3,maximum:float):
	var vec_l = vec.length()
	var vec_n = vec.normalized()
	vec_l = min(vec_l,maximum)
	return vec_n * vec_l
	
func _integrate_forces(state):
#	var step = state.get_step()
#
#	var Ia_inv = body_a.get_inverse_inertia_tensor()
#	var Ib_inv = get_inverse_inertia_tensor()
#	var redInertia = add_Basis_by_Element(Ia_inv,Ib_inv).inverse()
#	var qBT = Quat(global_transform.basis)#Quaternion Node B Transform Basis
#	#Quaternion Target Orientation
#	var qTargetO = calc_target_orientation_by_basis()#Does the same thing but without the Position3D node
#	var rotChange = qTargetO * qBT.inverse() #rotation change quaternion
#
#	var angle = 2.0 * acos(rotChange.w) #Turns to angle radians, the amount it turns around the quats axis
#
#	if(is_nan(angle)):
#		angle = 0
#
#	var axis = Vector3(0,0,0)
#	var v = Vector3(rotChange.x,rotChange.y,rotChange.z)#rotation change quaternion "V" component
#
#	var axis_den = sin(angle*0.5)
#	if(is_nan(axis_den)):
#		axis = Vector3(0,0,0)
#	elif(axis_den==0):
#		axis = Vector3(0,0,0)
#	else:
#		axis = v / axis_den#the quats axis
#
#	var axis_Mag = axis.length()
#	if(is_nan(axis_Mag)||is_inf(axis_Mag)):
#		axis = Vector3(0,0,0)
#
#	#prev_quat_axis = axis
#	if(angle>PI):
#		angle -= 2.0*PI
#
#
#
#	var targetAng = axis*angle
#
#	var bAV = state.get_angular_velocity()
#	var aAV = body_a.get_angular_velocity()
#	var trq_joint_b = redInertia*(targetAng*stiffnessB/step - (bAV-aAV)*dampingB)/step
#	trq_joint_b = max_vector_magnitude(trq_joint_b,torque_lim)
#	var j_trq_joint_b = trq_joint_b*step
#	var w_joint_b = redInertia.inverse()*(trq_joint_b)*step
#
#	var lv = state.get_linear_velocity()
#	#var g = state.get_total_gravity()
#	var g = Vector3(0,-1,0)*9.8
#	lv += g * step
#
#	#var  av = state.get_angular_velocity() + w_joint_b
#	#var  av = state.get_angular_velocity()
#	#var  av = w_joint_b
#
#	var  av = targetAng*50
#
#
#	var linear_damp = 1.0 - step * state.get_total_linear_damp()
#
#	if (linear_damp < 0):
#		linear_damp = 0
#
#
#	var angular_damp = 1.0 - step * state.get_total_angular_damp()
#
#	if (angular_damp < 0):
#		angular_damp = 0
#
#	lv *= linear_damp
#	av *= angular_damp
#
#	state.set_linear_velocity(lv)
#	state.set_angular_velocity(av)
	#state.apply_torque_impulse(j_trq_joint_b)
	#state.add_torque(trq_joint_b)
	pass
