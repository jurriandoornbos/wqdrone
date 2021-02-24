import rclpy
import serial
import re
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from sensor_msgs.msg import NavSatFix

from rclpy.executors import SingleThreadedExecutor

port = "/dev/ttyACM1"

ser = serial.Serial(port, 115200)

class GPSPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')
        self.gps_publisher_ = self.create_publisher(NavSatFix, 'teensy_fix', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    
    def gps_parse(self,line):
        
        def splitter(item):
            begin  = item[:-7]
            end = item[-7:]
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
        alt = float(items[6])
        sats = int(items[7])
        rxok = items[8]
        rxerr = items[9]
        rxchars = items[10]
            
        return [lat,lon,alt,sats]

    def timer_callback(self):
        line = ser.readline()
        line = line.decode("utf-8")
        begin = line[0:3]    
       
        
        if begin == "GPS":
            payload = self.gps_parse(line)  
            nav_msg = NavSatFix()
            nav_msg.latitude = payload[0]
            nav_msg.longitude = payload[1]
            nav_msg.altitude = payload[2]
            self.gps_publisher_.publish(nav_msg)
            self.get_logger().info("Lat: %f"%nav_msg.latitude +" Lon: %f" % nav_msg.longitude)

'''
The sensor publisher acquires the data and presents a list with the following order:
[Temperature, TDS, Turbidity, Turbidity voltage, Acidity, Acidity voltage]
'''
class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')
        self.sensor_publisher_ = self.create_publisher(Float64MultiArray, 'wq_sensors', 10)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
 
    def timer_callback(self):
        line = ser.readline()
        line = line.decode("utf-8")
        regex = r'([\-0-9\.]+).*\s([\-0-9\.]+).*Turbidity:\s([\-0-9\.]+).*\s([\-0-9\.]+).*\s([\-0-9\.]+).*\/\s([\-0-9\.]+).'

        begin = line[0:3]
        if begin == "<Te":
            t = re.findall(regex,line)[0]
          
            t = [float(item)for item in t]           
            msg = Float64MultiArray()
            msg.data = t
            self.sensor_publisher_.publish(msg)
            self.get_logger().info(str(msg.data))
        
        
def main(args=None):
    rclpy.init(args=args)
    try:
        gps_publisher = GPSPublisher()
        sensor_publisher = SensorPublisher()
        
        executor = SingleThreadedExecutor()
        
        executor.add_node(gps_publisher)
        executor.add_node(sensor_publisher)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            gps_publisher.destroy_node()
            sensor_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
