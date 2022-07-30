extends Spatial

const chassisJointConnectorInstance = preload("res://Chassis_Joint_Connector_Rex.tscn")
func _ready():
	
	#var legs = []
	#var legs_base = []
	var platform = get_node("Platform")
	for c in get_parent().get_children():
		if c.is_in_group("LEG"):
			#legs.append(c)
			var c_base = c.get_node("Base")
			#legs_base.append(c_base)
			#var puppeteer_base = c.get_node("Puppeteer_base")
			
			var joint_Connector = chassisJointConnectorInstance.instance()
			self.add_child(joint_Connector)
			#joint_Connector.puppeteer_node = puppeteer_base.get_path()
			#prints(joint_Connector.puppeteer_node)
			joint_Connector.global_transform = c_base.global_transform
			joint_Connector.set_node_a(platform.get_path())
			joint_Connector.set_body_a(platform)
			joint_Connector.set_node_b(c_base.get_path())
			joint_Connector.set_body_b(c_base)
			joint_Connector.SIGNAL_set_base_transform_original(c_base)
			
			
	"""
	#prints(get_parent().get_node("LegD1").get_node("Base").get_path())
	var leg1Base = get_parent().get_node("Spider_LegG_SPINDLE1").get_node("Base")
	#prints("leg1Base: ")
	#aprints(leg1Base)
	var leg2Base = get_parent().get_node("Spider_LegG_SPINDLE2").get_node("Base")
	var leg3Base = get_parent().get_node("Spider_LegG_SPINDLE3").get_node("Base")
	var leg4Base = get_parent().get_node("Spider_LegG_SPINDLE4").get_node("Base")
	
	get_node("Joint1").SIGNAL_set_body_a()
	get_node("Joint1").set_node_b(leg1Base.get_path())
	get_node("Joint1").set_body_b(leg1Base)
	get_node("Joint1").SIGNAL_set_base_transform_original(leg1Base)
	
	get_node("Joint2").SIGNAL_set_body_a()
	get_node("Joint2").set_node_b(leg2Base.get_path())
	get_node("Joint2").set_body_b(leg2Base)
	get_node("Joint2").SIGNAL_set_base_transform_original(leg2Base)
	
	get_node("Joint3").SIGNAL_set_body_a()
	get_node("Joint3").set_node_b(leg3Base.get_path())
	get_node("Joint3").set_body_b(leg3Base)
	get_node("Joint3").SIGNAL_set_base_transform_original(leg3Base)
	
	get_node("Joint4").SIGNAL_set_body_a()
	get_node("Joint4").set_node_b(leg4Base.get_path())
	get_node("Joint4").set_body_b(leg4Base)
	get_node("Joint4").SIGNAL_set_base_transform_original(leg4Base)
	"""
	
