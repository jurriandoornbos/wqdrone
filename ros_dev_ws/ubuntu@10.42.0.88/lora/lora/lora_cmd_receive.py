import rclpy
import serial
import time

from rclpy.node import Node

from geometry_msgs.msg import Twist

device = "/dev/ttyLoRa"
ser = serial.Serial(device, 115200)
time.sleep(3)

'''
When troubleshooting, please take out the try statement. Doublecheck the get_logger info as well.
'''

class VelReceive(Node):

    def __init__(self):
        super().__init__('lora_cmd_rec')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1# seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        s = ser.readline()
        r = s.decode("ascii")
        if len(r)>4 and r[0:11] == "teleop_lora":
                try:
                        l = r.split()
                        msg.linear.x = float(l[2])
                        msg.linear.y =0.0
                        msg.linear.z = 0.0
                        msg.angular.x =0.0
                        msg.angular.y= 0.0
                        msg.angular.z =float(l[4])
                        self.publisher_.publish(msg)
                        self.get_logger().info('Lora Received: lx: %f , az: %f' % (msg.linear.x, msg.angular.z))
                except:
                        pass

        
            
def main(args=None):
    rclpy.init(args=args)
    vel_receive = VelReceive()
    rclpy.spin(vel_receive)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vel_receive.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
