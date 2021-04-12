import rclpy
import serial
import re
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray

ser =serial.Serial("/dev/ttyUSB0",115200)
class PowerPublisher(Node):

    def __init__(self):
        super().__init__('power_publisher')
        self.ardui_ = self.create_publisher(String, 'ardu_string', 10)
        self.is_water_ = self.create_publisher(String, 'is_water', 10)
        self.motorL_ = self.create_publisher(Float32MultiArray, 'motorL', 10)
        self.motorR_ = self.create_publisher(Float32MultiArray, 'motorR', 10)
        self.box_ = self.create_publisher(Float32MultiArray, 'box_mon', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        '''
        Example msg over serial:
        PowerSensingDuino Temperature: -127.00C | Voltage0: 0.00V | Voltage1: 0.00V | Voltage2: 0.00V | Ampere0: 10.17A | Ampere1: 10.17A | Ampere2: 10.17A | Water0: 1 | Water1: 1 | 
        '''
        data = ser.readline()
        data = data.decode("utf-8")
        if len(data)>10:
            ardu_msg = String()           
            water_msg = String()
            motorl_msg = Float32MultiArray()
            motorr_msg = Float32MultiArray()
            box_msg = Float32MultiArray()
            l = data.split("|")
            regex = r'([\-0-9\.]+)'
            result = [re.findall(regex,item) for item in l]
            temp = float(result[0][0])
            v0 = float(result[1][1])
            v1 = float(result[2][1])
            v2 = float(result[3][1])
            a0 = float(result[4][1])
            a1 = float(result[5][1])
            a2 = float(result[6][1])
            w0 = l[7]
            w1 = l[8]
            
            #make sure this is correctly wired in the box
            box = [temp,v2,a2]
            motorl = [v0,a0]
            motorr = [v1,a1]
            # 00 is water in both, 01 or 10 is water in either, 11 is nothing            
            water_msg.data = w0+w1            
            
            motorr_msg.data = motorr
            motorl_msg.data = motorl
            box_msg.data = box
            #and the simple string of everything
            ardu_msg.data = data
            
            self.ardui_.publish(ardu_msg)
            self.motorL_.publish(motorl_msg)
            self.motorR_.publish(motorr_msg)
            self.box_.publish(box_msg)
            self.is_water_.publish(water_msg)



def main(args=None):
    rclpy.init(args=args)

    power_publisher = PowerPublisher()

    rclpy.spin(power_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    power_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
