import rclpy
import serial
import time

from rclpy.node import Node

from sensor_msgs.msg import NavSatFix

device = "/dev/ttyLoRa"
ser = serial.Serial(device, baudrate = 115200,timeout=0.1,write_timeout=0.1)

time.sleep(3)
'''
When troubleshooting, please take out the try statement. Doublecheck the get_logger info as well.
'''

class GPS_Send(Node):

    def __init__(self):
        super().__init__('lora_gps_teensy_sender')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ty/teensy_fix',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        '''
        The idea is that for every topic, there is a LoRa Comms module available
        send + receive for each one. End of a line is hardcoded in the Arduino module as "$"
    	'''
        ser = serial.Serial(device, baudrate = 9600,timeout=0.1,write_timeout=0.1)
        s = "teensy_gps: " + "lat: " + str(msg.latitude)+ " "+ "lon: " + str(msg.longitude)+ " $"
        ser.write(s.encode("ascii"))
        self.get_logger().info('Sent LoRa msg: %s' % s)
        ser.close()

def main(args=None):
    rclpy.init(args=args)

    gps_send = GPS_Send()

    rclpy.spin(gps_send)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_send.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
