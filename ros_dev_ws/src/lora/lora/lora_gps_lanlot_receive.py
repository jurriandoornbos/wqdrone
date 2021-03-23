import rclpy
import serial
import time

from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
device = "/dev/ttyLoRa"
ser = serial.Serial(device, 115200)
time.sleep(3)
'''
When troubleshooting, please take out the try statement. Doublecheck the get_logger info as well.
'''

class GPS_Receive(Node):

    def __init__(self):
        super().__init__('lora_gps_teensy_rec')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_lora_teensy', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
	    if ser.in_waiting>30:
                msg = NavSatFix()
                s = ser.readline()
                s = s.decode("ascii")
                if len(s)>4 and s[0:11] == "gps_lanlot":
                        try:
                            l = s.split()
                            msg.latitude =float(l[2])
                            msg.longitude =float(l[4])
                            msg.altitude = 1.0
                            self.publisher_.publish(msg)
                            self.get_logger().info('Lora Received: "%s"' % s)
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
