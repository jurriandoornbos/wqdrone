import rclpy
import serial
import time

from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

device = "/dev/ttyLoRa"
ser = serial.Serial(device, baudrate = 115200,timeout=0.1,write_timeout=0.1)

time.sleep(3)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('lora_wq_send')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/ty/wq_sensors',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        '''
        The idea is that for every topic, there is a LoRa Comms module available
        send + receive for each one. End of a line is hardcoded in the Arduino module as "$"
    	'''
        ser = serial.Serial(device, baudrate = 9600,timeout=0.1,write_timeout=0.1)
        s = "wq_sensors: " + str(msg.data) + "$"
        ser.write(s.encode("ascii"))
        self.get_logger().info('Sent LoRa msg: %s' % s)
        ser.close()

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()