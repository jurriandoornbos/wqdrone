import launch
from launch import LaunchDescription
def generate_launch_description():
	return LaunchDescription([
	launch.actions.ExecuteProcess(
		cmd = ["ros2", "bag","record","-a" ,"-o", "test2_April24"],
		output = "screen")
	
	]) 
