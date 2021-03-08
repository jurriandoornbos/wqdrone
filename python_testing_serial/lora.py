import serial
import time
ser = serial.Serial("COM3", baudrate = 9600,timeout=1,write_timeout=0.1)
time.sleep(2)


s = '1.1234567$'

f = 1.123456
for i in range(0,100):
    ser.write(s.encode("ascii"))
    time.sleep(0.1)

# 64 bytes max, with timeout between each 64 bytes works!