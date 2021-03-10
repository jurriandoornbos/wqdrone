import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
        Node(
	    package = "lora",
	    namespace = "lr",
	    executable = "cmd_send",
	    output = "screen"),
		
	Node(
	    package = "teleop_twist_keyboard",
	    namespace = "ttk",
	    executable = "teleop_twist_keyboard",
	    remappings = [("cmd_vel", "teleop_lora"),],
	    output = "screen",
	    prefix = "xterm -e",
	    name = "teleop"),
	    	
	    	

	    

	]) 
