extends Spatial


func _ready():
	"""
	var legs = []
	for c in get_parent().get_children():
		if c.is_in_group("LEG"):
			legs.append(c)
	"""
	#prints(get_parent().get_node("LegD1").get_node("Base").get_path())
	var leg1Base = get_parent().get_node("LegD1").get_node("Base")
	#prints("leg1Base: ")
	#aprints(leg1Base)
	var leg2Base = get_parent().get_node("LegD2").get_node("Base")
	var leg3Base = get_parent().get_node("LegD3").get_node("Base")
	var leg4Base = get_parent().get_node("LegD4").get_node("Base")
	
	get_node("Joint1").SIGNAL_set_body_a()
	get_node("Joint1").set_node_b(leg1Base.get_path())
	get_node("Joint1").set_body_b(leg1Base)
	get_node("Joint1").set_total_mass()
	get_node("Joint1").SIGNAL_set_base_transform_original(leg1Base)
	
	get_node("Joint2").SIGNAL_set_body_a()
	get_node("Joint2").set_node_b(leg2Base.get_path())
	get_node("Joint2").set_body_b(leg2Base)
	get_node("Joint2").set_total_mass()
	get_node("Joint2").SIGNAL_set_base_transform_original(leg2Base)
	
	get_node("Joint3").SIGNAL_set_body_a()
	get_node("Joint3").set_node_b(leg3Base.get_path())
	get_node("Joint3").set_body_b(leg3Base)
	get_node("Joint3").set_total_mass()
	get_node("Joint3").SIGNAL_set_base_transform_original(leg3Base)
	
	get_node("Joint4").SIGNAL_set_body_a()
	get_node("Joint4").set_node_b(leg4Base.get_path())
	get_node("Joint4").set_body_b(leg4Base)
	get_node("Joint4").set_total_mass()
	get_node("Joint4").SIGNAL_set_base_transform_original(leg4Base)
	
	
