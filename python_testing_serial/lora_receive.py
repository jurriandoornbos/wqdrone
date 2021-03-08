import serial
import time
ser = serial.Serial("COM3", baudrate = 9600,write_timeout=0.1)
time.sleep(2)

while True:
    result = ser.readline()
    print(result)

# 64 bytes max, with timeout between each 64 bytes works!