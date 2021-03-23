import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([

	Node(
	    package = "lora",
	    namespace = "lr",
	    executable = "wq_rec"),
	    
	Node(
	    package = "lora",
	    namespace = "lr",
	    executable = "sonar_rec"),
	    
	Node(
	    package = "lora",
	    namespace = "lr",
	    executable = "wq_gps_rec"),

	    
	launch.actions.ExecuteProcess(
		cmd = ["ros2", "bag","record","-o", "Mar15_lora", "/lr/wq_lora", "/lr/sonar_lora", "/lr/gps_lora_teensy"],
		output = "screen")
	
	]) 
