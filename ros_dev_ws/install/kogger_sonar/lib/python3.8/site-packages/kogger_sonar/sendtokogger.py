import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import serial

ser = serial.Serial("/dev/ttyUSB0", 115200)
cmd  = "bb5500030200050d"
send = bytes.fromhex(cmd)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'sonar_send', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        msg = String()
        msg.data = cmd
        #Sending the information to the initialized serial device up top!
        ser.write(send)
        self.publisher_.publish(msg)
        self.get_logger().info('Sending to sonar: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
