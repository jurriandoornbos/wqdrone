import launch
from launch import LaunchDescription
def generate_launch_description():
	return LaunchDescription([
	launch.actions.ExecuteProcess(
		cmd = ["ros2", "bag","record","-o", "test5_May14", "/ty/wq_sensors", "/ty/teensy_serial", "/ty/teensy_fix"],
		output = "screen")
	
	]) 
