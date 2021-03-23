import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		
	Node(
	    package = "teensy",
	    namespace = "ty",
	    executable = "teensy_pub"),
	    	
	    	
	Node(
	    package = "kogger_sonar",
	    namespace = "ks",
	    executable = "rec_distance"),
	    
	Node(
	    package = "lora",
	    namespace = "lr",
	    executable = "wq_send"),
	    
	Node(
	    package = "lora",
	    namespace = "lr",
	    executable = "sonar_send"),
	    
	Node(
	    package = "lora",
	    namespace = "lr",
	    executable = "wq_gps_send"),
	Node(
	    package = "lora",
	    executable = "cmd_receive"),
	

	    
	launch.actions.ExecuteProcess(
		cmd = ["ros2", "bag","record","-a" ,"-o", "RPi_Mar26"],
		output = "screen")
	
	]) 
