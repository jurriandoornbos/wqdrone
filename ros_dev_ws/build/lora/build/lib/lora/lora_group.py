import rclpy
import serial
import time
from rclpy.callback_groups import CallbackGroup

device = "/dev/ttyLoRa"
ser = serial.Serial(device, 9600)
time.sleep(3)

class LoRaGroup(CallbackGroup):

    def __init__(self,node):
        super().__init__()
   
                
    def can_execute(self,entity):

        return ser.in_waiting>5
    
    
    def beginning_execution(self, entity):
        if ser.in_waiting>5:
            return True
        
        return False
    
    def ending_execution(self,entity):
        pass

    

