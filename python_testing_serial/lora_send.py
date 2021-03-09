import serial
import time
ser = serial.Serial("/dev/ttyLoRa", baudrate = 9600,timeout=1,write_timeout=0.1)
time.sleep(2)


while True:

    for i in range(0,5):
        s = "Bericht %i" % i
        ser.write(s.encode("ascii"))
        time.sleep(0.1)
    time.sleep(10)

# 64 bytes max, with timeout between each 64 bytes works!
