import rclpy
import serial
import time

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('lora_wq_rec')
        self.subscription = self.create_subscription(String,"/lr/lora_str",self.listener_callback,10)
        self.subscription
        self.publisher_ = self.create_publisher(Float64MultiArray, 'wq_lora', 10)

    def listener_callback(self,msg):
        pub = Float64MultiArray()
        s = msg.data
        if s[0:10] == "wq_sensors":
            try:   	        
                l = s.split(",")[1:-2]
                l[0] = l[0][2:]
                l[-1] = l[-1][0:-2]
                l = [float(i) for i in l] 
                pub.data = l
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
