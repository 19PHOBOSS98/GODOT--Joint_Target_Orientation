extends RigidBody


# Declare member variables here. Examples:
# var a = 2
# var b = "text"


# Called when the node enters the scene tree for the first time.
var anchor
var anchor_ang

var baseBTOrig

func _ready():
	prints("degrot:",rotation_degrees)
	prints("degrot txg:",to_global(rotation_degrees))
	prints("degrot txl:",to_local(rotation_degrees))
	#anchor = get_parent().get_node("Position3D").global_transform.basis.inverse()
	#anchor_ang = get_parent().get_node("Position3D").rotation_degrees
	#prints("anchor_ang:")
	#anchor = get_parent().get_node("Position3D").global_transform.basis.inverse()
	baseBTOrig = self.global_transform.basis
	prints("baseBTOrig: ",baseBTOrig.x,baseBTOrig.y,baseBTOrig.z)
	calc_targ_orientation()
	#prints(anchor_ang)

func v3_step(edge,value):
	##return 1.0 - max(sign(edge - value),0.0)
	var comp = Vector3()
	comp.x = 1.0 - max(sign(edge.x - value.x),0.0)
	comp.z = 1.0 - max(sign(edge.y - value.y),0.0)
	comp.z = 1.0 - max(sign(edge.z - value.z),0.0)
	return comp
	 

func v3_shortest_angle_to( to, from ):
	var diff = Vector3()
	diff.x = fmod(( to.x - from.x + 180.0 ),360.0) - 180.0
	diff.y = fmod(( to.y - from.y + 180.0 ),360.0) - 180.0
	diff.z = fmod(( to.z - from.z + 180.0 ),360.0) - 180.0
	var overstep = Vector3(1.0,1.0,1.0) - v3_step(Vector3(-180.0,-180.0,-180.0),diff)
	#var overstep = v3_step(diff,Vector3(-180.0,-180.0,-180.0))
	var ret = diff + 360.0*overstep
	return ret #if ret<360.0 else 0.0
	#var diff = ( to - from + 180 ) % 360 - 180
	#return diff < -180 ? diff + 360 : diff

var gz = Vector3()

func calc_targ_orientation():
	var Abasis = get_parent().get_node("Position3D").global_transform.basis
	#var baseBTCurr = Abasis.inverse()*baseBTOrig
	#var baseBTCurr = baseBTOrig
	print("Abasis: ",Abasis.x,Abasis.y,Abasis.z)
	print("baseBTOrig: ",baseBTOrig.x,baseBTOrig.y,baseBTOrig.z)
	#print("baseBTCurr: ",baseBTCurr.x,baseBTCurr.y,baseBTCurr.z)
	
	
	var qx = Quat(Abasis.x,0.0*PI/180.0)
	var qy = Quat(Abasis.y,0.0*PI/180.0)
	var qz = Quat(Abasis.z,90.0*PI/180.0)
	#var qx = Quat(baseBTOrig.x,0.0*PI/180.0)
	#var qy = Quat(baseBTOrig.y,0.0*PI/180.0)
	#var qz = Quat(baseBTOrig.z,90.0*PI/180.0)
	var qBTargRo = qz*qy*qx
	#print(" qBTargRo*Vector3(0,-1,1): ", qBTargRo*Vector3(0,-1,1))
	#print(" Vector3(0,-1,1): ", Vector3(0,-1,1))
	var BTargetBasis = Basis()
	#BTargetBasis.x =  qBTargRo*baseBTCurr.x
	#BTargetBasis.y =  qBTargRo*baseBTCurr.y
	#BTargetBasis.z =  qBTargRo*baseBTCurr.z
	BTargetBasis.x =  qBTargRo*baseBTOrig.x
	BTargetBasis.y =  qBTargRo*baseBTOrig.y
	BTargetBasis.z =  qBTargRo*baseBTOrig.z
	#BTargetBasis = Abasis*BTargetBasis
	print("BTargetBasis: ",BTargetBasis.x,BTargetBasis.y,BTargetBasis.z)
	#$MeshInstance2.global_transform.basis = BTargetBasis
	var qBTargetBasis = Quat(BTargetBasis)
	#var qbaseBTC = Quat(baseBTCurr)
	#prints("thetaDiff: ",qbaseBTC.angle_to(qBTargetBasis)*180.0/PI)
	#var qBTC = Quat(baseBTCurr)
	#prints("thetaDiff: ",qBTC.angle_to(qBTargetBasis)*180.0/PI)
	
	
	
	#var mTBasis = $MeshInstance.global_transform.basis
	#mTBasis.x = qz * mTBasis.x
	#mTBasis.y = qz * mTBasis.y
	#mTBasis.z = qz * mTBasis.z
	#$MeshInstance.global_transform.basis = mTBasis
	
	var s = Quat(Vector3(0,0,1),0*PI/180)*Vector3(1,0,0)#targetO
	var e = Quat(Vector3(0,0,1),(90)*PI/180)*Vector3(1,0,0)
	var d = s.dot(e)
	prints("dot: ",d)
	

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


func _process(delta):
	#var A = get_parent().get_node("Position3D").global_transform.basis
	#var B = global_transform.basis
	#calc_diff_to_targ(A,B)
	#$RichTextLabel.text = String(rotation_degrees)
	
	if Input.is_action_pressed("rotate"):
		gz = global_transform.basis.x*1
		
		#gz.y = -1
		add_torque(gz)
		
	$RichTextLabel.text = String(global_transform.basis.inverse()*angular_velocity)
	#var diff = v3_shortest_angle_to( rotation_degrees,anchor_ang )
	#$RichTextLabel.text = String(diff)
	#$RichTextLabel2.text = String(rotation_degrees- anchor)

