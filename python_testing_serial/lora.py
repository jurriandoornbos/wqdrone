import serial

ser = serial.Serial("COM3", 9600,timeout=0.5)

s = "/teensy_fix: LAT1: 53.230000,LON1:4.551412$\n"

byt = s.encode("ascii")

if ser.isOpen():
    ser.write(byt)
    ser.close()
    print(byt)
