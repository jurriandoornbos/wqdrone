import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		
	Node(
	    package = "teensy",
	    namespace = "/ty/",
	    executable = "serial_pub",
	    parameters = [{"device": "/dev/ttyACM0"}]
	    ),
	    	
	Node(
	    package = "teensy",
	    namespace = "/ty/",
	    executable = "gps",),
	Node(
	    package = "teensy",
	    namespace = "/ty/",
	    executable = "sensor",),
	    
	    	    	
	Node(
	    package = "kogger_sonar",
	    namespace = "ks",
	    executable = "rec_distance",
	    parameters = [{"device": "/dev/ttySonar"}]
	    ),
	    
  
	#Node(
	#    package = "lora",
	#    namespace = "lr",
	#    executable = "sonar_send"),
	    
	#Node(
	#    package = "lora",
	#    namespace = "lr",
	#    executable = "wq_gps_send"),
	Node(
	    package = "lora",
	    executable = "cmd_receive",
	    parameters = [{"device": "/dev/ttyLoRa"}]
	    ),
	

	    
	launch.actions.ExecuteProcess(
		cmd = ["ros2", "bag","record","-a" ,"-o", "RPi_April28"],
		output = "screen")
	
	]) 
