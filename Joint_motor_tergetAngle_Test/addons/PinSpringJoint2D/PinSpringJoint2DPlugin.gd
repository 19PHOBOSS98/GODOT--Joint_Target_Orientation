tool
extends EditorPlugin

func _enter_tree():
    # Initialization of the plugin goes here
    # Add the new type with a name, a parent type, a script and an icon
    add_custom_type("PinSpringJoint2D", "PinJoint2D", preload("PinSpringJoint2D.gd"), preload("icon_pin_spring_joint.svg"))

func _exit_tree():
    # Clean-up of the plugin goes here
    # Always remember to remove it from the engine when deactivated
    remove_custom_type("PinSpringJoint2D")
