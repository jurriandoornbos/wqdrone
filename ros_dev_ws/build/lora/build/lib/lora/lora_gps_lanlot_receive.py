import rclpy
import serial
import time

from rclpy.node import Node

from sensor_msgs.msg import NavSatFix

'''
When troubleshooting, please take out the try statement. Doublecheck the get_logger info as well.
'''

class GPS_Receive(Node):

    def __init__(self):
        super().__init__('lora_gps_teensy_rec')
        self.subscription = self.create_subscription(String,"/lr/lora_str",self.listener_callback,10)
        self.subscription
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_lora_lanlot', 10)


    def listener_callback(self,msg):
        pub = NavSatFix()
        s = msg.data
        if s[0:11] == "gps_lanlot":
            try:
                l = s.split()
                pub.latitude =float(l[2])
                pub.longitude =float(l[4])
                pub.altitude = 1.0
                self.publisher_.publish(pub)
                            
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    gps_receive = GPS_Receive()
    rclpy.spin(gps_receive)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_receive.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
