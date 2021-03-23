import rclpy
import serial
import time

from rclpy.node import Node

from std_msgs.msg import String


device = "/dev/ttyLoRa"
ser = serial.Serial(device, 115200)
time.sleep(3)

class LoRaPublisher(Node):

    def __init__(self):
        super().__init__('lora_rec')
        self.publisher_ = self.create_publisher(String, 'lora_str', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        s = ser.readline()
        s = s.decode("ascii")
        if len(s)>10:
            try:
                msg.data = s
                self.publisher_.publish(msg)
                self.get_logger().info('Lora Received: "%s"' % msg.data)
            except:
                pass
def main(args=None):
    rclpy.init(args=args)
    lora_publisher = LoRaPublisher()
    rclpy.spin(lora_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lora_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
