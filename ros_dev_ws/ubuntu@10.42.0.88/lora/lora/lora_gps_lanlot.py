import rclpy
import serial
import time

from rclpy.node import Node

from sensor_msgs.msg import NavSatFix

device = "/dev/ttyLoRa"
ser = serial.Serial(device, baudrate = 115200,timeout=0.1,write_timeout=0.1)

time.sleep(3)

class GPS_LanLot_Send(Node):

    def __init__(self):
        super().__init__('lora_gps_lanlot_sender')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps_lanlot',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        '''
        The idea is that for every topic, there is a LoRa Comms module available
        send + receive for each one. End of a line is hardcoded in the Arduino module as "$"
    	'''
        s = "gps_lanlot: " + "lat: " + str(msg.latitude) + "lon: " + str(msgs.longitude)+ " $"
        ser.write(s.encode("ascii"))
        self.get_logger().info('Sent LoRa msg: %s' % s)

def main(args=None):
    rclpy.init(args=args)

    gps_lanlot = GPS_LanLot_Send()

    rclpy.spin(gps_lanlot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_lanlot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
