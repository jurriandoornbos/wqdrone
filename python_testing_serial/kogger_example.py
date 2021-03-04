import serial
import matplotlib.pyplot as plt

## U numbers larger than 1 need to be reordered in reverse order of the hex value (per byte (2 strings))

## possible commands KS can send

cmds = {"01":"timestamp",
	"02": "distance",
	"03": "chart",
	"04": "attitude",
	"05": "temp",
	"10": "dataset",
	"11": "distance setup",
	"12": "chart setup",
	"13": "dsp",
	"14": "transceiver settings",
	"15": "speed of sound",
	"16": "pin functions",
	"17": "bus settings",
	"18": "uart settings",
	"19": "I2C settings",
	"1A": "can settings",
	"1B": "imu settings",
	"20": "version",
	"21": "mark of cont. work",
	"22": "diagnostics",
	"23": "flash",
	"24": "boot device",
	"25": "update",
	"64": "navigation"}


##Parse the SYNC frame

sync = {'bb': "SYNC1",'55':"SYNC2"}

## Parse the ROUTE frame
route = {'00':'Route address at 0x0'}

## Parse all the modes frame //Kogger SB Protocol
modes = {
	"type": {"00":"Reserved","01":"Content","10":"Setting","11":"Getting"},
	"reserved": {"0":"Reserved","1":"Reserved"},
	"version": "Payload Data Version",
	"mark": {"0": "reset", "1": "active"},
	"response": {"0":"device-host","1":"host-device"}	
	}
	
def mode_parser(modehex):
	bitfield = format(int(modehex,base=16), "#010b")[2:]
	mode_type = bitfield[-2:]
	mode_reserved = bitfield[-3]
	mode_version = bitfield[-6:-3]
	mode_mark = bitfield[-7]
	mode_response = bitfield[-8]
	
	typed =modes["type"][mode_type]
	reserved = modes["reserved"][mode_reserved]
	versioned = mode_version
	marked = modes["mark"][mode_mark]
	responsed = modes["response"][mode_response]
	
	print(typed + " " + reserved + " " + versioned + " " + marked + " " + responsed)
	
def payload_parser(payload):
	
	payloadmax=len(payload)
	cid = payload[8:10]
	data = payload[12:payloadmax-4]
	
	checksum1 = payload[payloadmax-4:payloadmax-2]
	checksum2 = payload[payloadmax-2:]		
	
	return(cid,data)

def fletcher(line):
	n=2
	ch1 = 0
	ch2 = 0
	
	l = [int(line[i:i+n], base =16) for i in range(0, len(line), n)]

	for item in l:
		ch1+=item%255
		ch2+=ch1
	ch1 = ch1 & 0b11111111
	ch2 = ch2 & 0b11111111
	ch1 ="{:02x}".format(ch1)
	ch2 ="{:02x}".format(ch2)
	
	return (ch1,ch2)
	
	
def chart_parser(payload):


	def reorder(line):
				
		l = [line[i:i+2]for i in range(0, len(line), 2)]
		
		return(l[-1]+l[0])

	seq_offset = payload[0:4]
	sample_res  = payload[4:8]
	abs_offset = payload[8:12]
	chart = payload[12:]
	
	seq_offset = int(reorder(seq_offset), base=16)
	sample_res = int(reorder(sample_res), base=16)
	abs_offset = int(reorder(abs_offset), base=16)
	
	reflection = [int(chart[i:i+2], base=16) for i in range(0,len(chart),2)]
	print(len(reflection))
	fig, ax  = plt.subplots()
	t = list(range(len(reflection)))
	t2 =[element*(sample_res/100)*5 for element in t]
	
	ax.plot(t2, reflection)
	
	ax.set(xlabel = "meters",ylabel = "reflection" )
	ax.grid()
	plt.show()
	
	
##main code##	
ser = serial.Serial('/dev/ttyUSB0', 115200)

payload = ""

payloadmax = -1

while True:
	data_raw = ser.read()
	hexs = data_raw.hex()
	payload+=hexs
	print(payload)
	if len(payload)==12:
		length = int(payload[-2:], 16)
		payloadmax = len(payload)+length*2+4
		
	if len(payload)==payloadmax:
		
		checksum1 = payload[payloadmax-4:payloadmax-2]
		checksum2 = payload[payloadmax-2:]

		
		cid, info = payload_parser(payload)
		
		c1,c2 = fletcher(payload[4:-4])

		
		print("Main C: ",c1,checksum1,c2,checksum2)
		
		mhex = payload[6:8]
		mode_parser(mhex)
		command = cmds[cid]
		print("My length is: ",payloadmax)
		if payloadmax>40 and command == "chart":
			chart = chart_parser(info)
		print(command)
		print("____")
		payload = ""
  		

