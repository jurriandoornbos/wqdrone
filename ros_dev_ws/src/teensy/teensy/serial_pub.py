import rclpy
import serial

import time
from rclpy.node import Node

from std_msgs.msg import String

from rclpy.executors import SingleThreadedExecutor
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class SerialPublisher(Node):

    def __init__(self):
        super().__init__('serial_publisher')
        self.serial_publisher_ = self.create_publisher(String, 'teensy_serial', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #in init
        self.declare_parameter("device", "/dev/ttyTeensy")

        device = self.get_parameter('device').get_parameter_value().string_value
        self.ser = serial.Serial(device,baudrate =115200)
        time.sleep(3)
    
    def timer_callback(self):
        msg = String()
        try:
            line = self.ser.readline()
            line = line.decode("utf-8")
            msg.data = line
            self.serial_publisher_.publish(msg)
        except:
            pass
        
        

        
        
def main(args=None):
    rclpy.init(args=args)
    try:
        serial_publisher = SerialPublisher()
        
        executor = SingleThreadedExecutor()
        
        executor.add_node(serial_publisher)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            serial_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
