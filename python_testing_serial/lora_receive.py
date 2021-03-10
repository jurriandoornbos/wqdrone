import serial
import time
device = "/dev/ttyUSB0"
ser = serial.Serial(device, baudrate = 9600,write_timeout=0.1)
time.sleep(2)

while True:
    result = ser.readline().decode("ascii")
    print(result)

# 64 bytes max, with timeout between each 64 bytes works!
