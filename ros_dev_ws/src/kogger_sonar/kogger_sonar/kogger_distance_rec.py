import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt32
import serial

ser = serial.Serial("/dev/ttyUSB0", 115200)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('sonar_distance_receiver')
        self.publisher_ = self.create_publisher(UInt32, 'sonar_dist', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sonar_serial = self.sonar_serial()
                
             
    def sonar_serial(self):  
        def U4converter(data):
            
            end = ""
            l = [data[i:i+2] for i in range(0,len(data),2)]
            l = l[::-1]
            for item in l:
                end+=item   
            return int(end, base=16)    
                
        payload = ""
        payloadmax = -1
        while True:
            data_raw = ser.read()
            hexs = data_raw.hex()
        
            payload+=hexs
            if len(payload)==12:
                length = int(payload[-2:], 16)
                payloadmax = len(payload)+length*2+4
            
            if len(payload)==payloadmax:
                data = payload[12:payloadmax-4]
                distance = U4converter(data)
                return(distance)	

    
    def timer_callback(self):
    
        msg = UInt32()
        msg.data = self.sonar_serial
        self.publisher_.publish(msg)
        self.get_logger().info('Received distance from Sonar: "%i"' % msg.data)
    
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
