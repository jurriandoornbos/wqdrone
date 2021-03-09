import rclpy
import serial
import time

from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
device = "/dev/ttyLoRa"
ser = serial.Serial(device, 9600)
time.sleep(3)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('lora_gps_teensy_rec')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_lora_teensy', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = NavSatFix()
        s = ser.readline()
        s = s.decode("ascii")
        if len(s)>4 and s[0:10] == "teensy_gps":
                l = s.split()
                msg.latitude =float(l[3])
                msg.longitude =float(l[5])
                msg.altitude = 1.0
                self.publisher_.publish(msg)
                self.get_logger().info('Lora Received: "%s"' % s)
            
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
