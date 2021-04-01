import rclpy
import serial
import time

from rclpy.node import Node

from std_msgs.msg import UInt32, String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('lora_sonar_rec')
        self.subscription = self.create_subscription(String,"/lr/lora_str",self.listener_callback,10)
        self.subscription
        self.publisher_ = self.create_publisher(UInt32, 'sonar_lora', 10)
        
    def listener_callback(self,msg):
        pub = UInt32()
        s = msg.data
        if s[0:11] == "sonar_send":
            try:
                u = int(s.split()[1])
                pub.data = u
                self.publisher_.publish(pub)

            except:
                pass
        
            
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
