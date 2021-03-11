import rclpy
import serial
import time

from rclpy.node import Node

from geometry_msgs.msg import Twist

device = "/dev/ttyLoRa"
ser = serial.Serial(device, 9600)
time.sleep(3)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('lora_cmd_rec')
        self.publisher_ = self.create_publisher(Twist, 'cmd_lora', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        ser = serial.Serial(device, 9600)
        s = ser.readline()
        s = s.decode("ascii")
        if len(s)>4 and s[0:12] == "teleop_lora":
                l = s.split()
                msg.linear.x = float(l[3])
                msg.angular.z =float(l[5])
                self.publisher_.publish(msg)
                self.get_logger().info('Lora Received: lx: %f , az: %f' % (msg.linear.x, msg.angular.z))
        ser.close()
            
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
