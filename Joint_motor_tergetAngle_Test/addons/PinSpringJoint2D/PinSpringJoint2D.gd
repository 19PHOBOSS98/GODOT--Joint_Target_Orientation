tool
extends PinJoint2D

#should be anywhere between 0 <= damping <= 10
# for most desired behavior
#careful not to set damping too high, otherwise
# it will act like super strong friction, and barely
# even act like a spring. slowly increase damping 
# to approach desired level

export(float) var Stiffness = 100
export(float) var Damping = 1
export(float) var RestAngleDeg = 0
export(bool) var UseInitialAngle = true

var stiffness
var damping
var rest_angle_rads

var body_a
var body_b

var debug = false

func normalize_ang(ang):
		while ang > PI:
			ang -= PI*2
		while ang < -PI:
			ang += PI*2
		return ang
			
# shortest ang between 2 normal angles -PI <= ang <= PI
func shortest_ang_to(to, from):
	if to < from:
		var _to = to + PI*2
		var a = _to-from
		var b = from-to
		if abs(a) < abs(b):
			return a
		else:
			return -b
	else:
		var _from = from + PI*2
		var a = _from-to
		var b = to-from
		if abs(a) < abs(b):
			return -a
		else:
			return b

func _ready():
	if Engine.editor_hint:
		return
	
	body_a = get_node(node_a)
	body_b = get_node(node_b)
	
	if false:#node_b == "../../r_arm":
		debug = true
	
	damping = Damping * 1000
	stiffness = Stiffness
	
	# looks glitchy without a little slack
	if self.softness < 0.5:
		self.softness = 0.5
	
	if UseInitialAngle and body_a != null and body_b != null:
		rest_angle_rads = body_b.rotation - body_a.rotation
	else:
		rest_angle_rads = normalize_ang(RestAngleDeg * PI / 180)

func get_float(obj, prop):
	var v = obj.get(prop)
	if v == null:
		return 0.0
	return float(v)
			
func apply_rot_spring():
	var k = stiffness
	var d = damping
	var l = rest_angle_rads
	var x = shortest_ang_to(body_b.rotation, body_a.rotation)
	var u = get_float(body_b,"angular_velocity") - get_float(body_a,"angular_velocity")
	
	var l_to_x = shortest_ang_to(x, l)
	
	var torque = - k * l_to_x - (d * u)
	if debug:
		print(str(l_to_x) + ", " + str(u))
	
	if body_a.get("applied_torque") != null:
		body_a.applied_torque -= torque
	if body_b.get("applied_torque") != null:
		body_b.applied_torque += torque

func _physics_process(delta):
	if body_a == null or body_b == null or Engine.editor_hint:
		return
	
	apply_rot_spring()
	#OS.delay_msec(10)
