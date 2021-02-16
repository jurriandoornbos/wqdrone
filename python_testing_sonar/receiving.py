import serial
# Just simple distance acquisition

def U4converter(data):
	end = ""
	l = [data[i:i+2] for i in range(0,len(data),2)]
	l = l[::-1]
	
	for item in l:
		end+=item
	
	return int(end, base=16)

ser = serial.Serial('/dev/ttyUSB0', 115200)

payload = ""

payloadmax = -1

while True:
	data_raw = ser.read()
	hexs = data_raw.hex()
	payload+=hexs

	if len(payload)==12:
		length = int(payload[-2:], 16)
		payloadmax = len(payload)+length*2+4
		
	if len(payload)==payloadmax:
		
		
		data = payload[12:payloadmax-4]
		
		distance = U4converter(data)	
		print(distance)
		payload= ""
		payloadmax=-1
