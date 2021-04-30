import rclpy
import re
from rclpy.node import Node

from std_msgs.msg import String, Float64MultiArray


class WQSubscriber(Node):

    def __init__(self):
        super().__init__('wq_sensor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'teensy_serial',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.sensor_publisher = self.create_publisher(Float64MultiArray, "wq_sensors", 10)

    def listener_callback(self, msg):
        regex = r'([\-0-9\.]+).*\s([\-0-9\.]+).*Turbidity:\s([\-0-9\.]+).*\s([\-0-9\.]+).*\s([\-0-9\.]+).*\/\s([\-0-9\.]+).'
        line = msg.data
        try:
            begin = line[0:3]
            if begin == "<Te":
                try:
                    t = re.findall(regex,line)[0]
                    
                    t = [float(item)for item in t]           
                    msg = Float64MultiArray()
                    msg.data = t
                    self.sensor_publisher.publish(msg)
                    self.get_logger().info(str(msg.data))
                except:
                    pass
        except:
            pass
    


def main(args=None):
    rclpy.init(args=args)

    wq_subscriber = WQSubscriber()

    rclpy.spin(wq_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wq_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
