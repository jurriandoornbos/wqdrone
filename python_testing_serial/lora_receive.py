import serial
import time
device = "/dev/ttyUSB0"
ser = serial.Serial(device, baudrate = 115200,write_timeout=0.1)
time.sleep(2)

while True:
    result = ser.readline()
    s = result.decode("ascii")
    print(s)
    print(len(s))

# 64 bytes max, with timeout between each 64 bytes works!
