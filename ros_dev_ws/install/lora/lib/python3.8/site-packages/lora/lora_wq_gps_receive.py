import rclpy
import serial
import time

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

device = "/dev/ttyLoRa"
ser = serial.Serial(device, 115200)
time.sleep(3)

class LoRaGPS(Node):

    def __init__(self):
        super().__init__('lora_gps_teensy_rec')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_lora_teensy', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if ser.in_waiting>30:
            
            msg = NavSatFix()
            s = ser.readline()
            s = s.decode("ascii")
            if len(s)>4 and s[0:10] == "teensy_gps":
                l = s.split()
                try:
                    msg.latitude =float(l[2])
                    msg.longitude =float(l[4])
                    msg.altitude = 1.0
                    self.publisher_.publish(msg)
                    self.get_logger().info('Lora Received: "lat: " "%f" "lon: " %f' % (msg.latitude, msg.longitude))
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
