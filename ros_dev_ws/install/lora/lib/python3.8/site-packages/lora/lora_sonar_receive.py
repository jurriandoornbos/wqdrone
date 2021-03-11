import rclpy
import serial
import time

from rclpy.node import Node

from std_msgs.msg import UInt32


device = "/dev/ttyLoRa"
ser = serial.Serial(device, 9600)
time.sleep(3)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('lora_sonar_rec')
        self.publisher_ = self.create_publisher(UInt32, 'sonar_lora', 10)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = UInt32()
        ser = serial.Serial(device, 9600)
        s = ser.readline()
        s = s.decode("ascii")
        if len(s)>4 and s[0:11] == "sonar_send":
                u = int(s.split()[1])
                msg.data = l
                self.publisher_.publish(msg)
                self.get_logger().info('Lora Received: "%s"' % msg.data)
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
