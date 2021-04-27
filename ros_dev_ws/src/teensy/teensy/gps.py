import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import re
import time


class GPSSubscriber(Node):

    def __init__(self):
        super().__init__('wq_sensor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'teensy_serial',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.gps_publisher = self.create_publisher(NavSatFix, "teensy_fix",10)
        
        
    def gps_parse(self,line):
        
        def splitter(item):
            begin  = str(item[:-7])
            end = str(item[-7:])
            s = "0"+begin + "." +end
            f = float(s)
            return f
            
        items = line[6:].split(",")
        utc = items[0]
        date = items[1]
        lat = splitter(items[2])
        lon = splitter(items[3])
        hdg = items[4]
        spd = items[5]
        alt = 1.0
        sats = items[7]
        rxok = items[8]
        rxerr = items[9]
        rxchars = items[10]
            
        return [lat,lon,alt,sats]

    def listener_callback(self, msg):
        line = msg.data
    
        begin = line[0:3]    
        if begin == "GPS":
            payload = self.gps_parse(line)  
            nav_msg = NavSatFix()
            nav_msg.latitude = payload[0]
            nav_msg.longitude = payload[1]
            nav_msg.altitude = payload[2]
            self.gps_publisher_.publish(nav_msg)
            self.get_logger().info("Lat: %f"%nav_msg.latitude +" Lon: %f" % nav_msg.longitude)
    


def main(args=None):
    rclpy.init(args=args)

    gps_subscriber = GPSSubscriber()

    rclpy.spin(gps_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
