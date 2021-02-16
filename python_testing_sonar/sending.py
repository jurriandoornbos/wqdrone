import serial

ser = serial.Serial("/dev/ttyUSB0", 115200)

def fletcher(line):
	n=2
	ch1 = 0
	ch2 = 0
	
	l = [int(line[i:i+n], base =16) for i in range(0, len(line), n)]

	for item in l:
		ch1+=item % 255
		ch2+=ch1
	ch1 = ch1 & 0b11111111
	ch2 = ch2 & 0b11111111
	ch1 ="{:02x}".format(ch1)
	ch2 ="{:02x}".format(ch2)
	
	return (ch1,ch2)

sync1 = "bb"
sync2 = "55"
route = "00"
mode = "12" #could be different (host-device)
cid = "02"
length = "00"
payload = ""

#swift manual constructing mode : response, GETTING 0 HOST → DEVICE
mode = "10000011"
mode = "03"

#for a given payload, calculate the checksum line = input range to be calculated
checksumrange = route+mode+cid + length + payload
ch1,ch2 =fletcher(checksumrange)

#construct the message
msg = sync1 + sync2 + route +  mode + cid + length + payload + ch1 + ch2 
bmsg=bytes.fromhex(msg)
print(msg)
ser.write(bmsg)

