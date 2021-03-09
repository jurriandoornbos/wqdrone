import rclpy
import serial
import time

from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, String
device = "/dev/ttyLoRa"
ser = serial.Serial(device, 9600)
time.sleep(3)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('lora_wq_rec')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'wq_lora', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64MultiArray()
        s = ser.readline()
        s = s.decode("ascii")
        if len(s)>4 and s[0:10] == "wq_sensors":
                l = s.split(",")[1:-2]
                l[0] = l[0][2:]
                l[-1] = l[-1][0:-2]
                l = [float(i) for i in l] 
                msg.data = l
                self.publisher_.publish(msg)
                self.get_logger().info('Lora Received: "%s"' % msg.data)
            
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
