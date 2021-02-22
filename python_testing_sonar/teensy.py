

def gps_parse(line):
	def splitter(item):
		begin  = item[:-7]
		end = item[-7:]
		s = begin + "." +end
		f = float(s)
		return f
		
	items = line[6:].split(",")
	
	utc = items[0]
	date = items[1]
	lat = splitter(items[2])
	lon = splitter(items[3])
	hdg = items[4]
	spd = items[5]
	alt = items[6]
	sats = int(items[7])
	rxok = items[8]
	rxerr = items[9]
	rxchars = items[10]
	
	return([utc,date,lat,lon,alt,sats])
	
def sensor_parse(line):
	print(line[1:-1])

import serial

ser = serial.Serial('/dev/ttyACM0',115200)

while True:
	line = ser.readline()
	line = line.decode("utf-8")
	begin = line[0:3]
	if begin == "GPS":
		print(gps_parse(line))
	elif begin == "<Te":
		sensor_parse(line)


