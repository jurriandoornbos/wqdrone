import rclpy
import serial
import time

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

'''
When troubleshooting, please take out the try statement. Doublecheck the get_logger info as well.
'''

class LoRaGPS(Node):

    def __init__(self):
        super().__init__('lora_gps_teensy_rec')
        self.subscription = self.create_subscription(String,"/lr/lora_str",self.listener_callback,10)
        self.subscription
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_lora_teensy', 10)
       
    def listener_callback(self,msg):
        if ser.in_waiting>30:
            
            pub = NavSatFix()
            s = msg.data
            if s[0:10] == "teensy_gps":
                l = s.split()
                try:
                    pub.latitude =float(l[2])
                    pub.longitude =float(l[4])
                    pub.altitude = 1.0
                    self.publisher_.publish(pub)
                    
                except:
                    pass
                
        
           
def main(args=None):
    rclpy.init(args=args)
    lora_gps = LoRaGPS()
    rclpy.spin(lora_gps)
         
    lora_gps.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
