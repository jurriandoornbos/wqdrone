#include <NMEAGPS.h>

//-------------------------------------------------------------------------
//  The GPSport.h include file tries to choose a default serial port
//  for the GPS device.  If you know which serial port you want to use,
//  edit the GPSport.h file.
 
#include <GPSport.h>

//------------------------------------------------------------
// For the NeoGPS example programs, "Streamers" is common set
//   of printing and formatting routines for GPS data, in a
//   Comma-Separated Values text format (aka CSV).  The CSV
//   data will be printed to the "debug output device".
// If you don't need these formatters, simply delete this section.

#include <Streamers.h>

//------------------------------------------------------------
// This object parses received characters
//   into the gps.fix() data structure

static NMEAGPS  gps;

//------------------------------------------------------------
//  Define a set of GPS fix information.  It will
//  hold on to the various pieces as they are received from
//  an RMC sentence.  It can be used anywhere in your sketch.

static gps_fix  fix;


//-------------------------------------------------------------
//Setup for the WQ sensor throughput from the arduino nano
const byte numChars = 128;

char receivedChars[numChars];

boolean newData = false;

//setup power sensing pins
#define V0Pin A0
#define Amp0Pin A1

// setup temperature stuff
// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS 4
// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);  

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    gpsPort.begin( 9600 );//serial3
    sensors.begin();  //initialization
    while (!Serial){
      ;
      }
}

void loop() {
    recvWithStartEndMarkers();
    showNewData();
    GPSloop();
    powerSensing();
}


//----------------------------------------------------------------
//  This function gets called about once per second, during the GPS
//  quiet time.  It's the best place to do anything that might take
//  a while: print a bunch of things, write to SD, send an SMS, etc.
//
//  By doing the "hard" work during the quiet time, the CPU can get back to
//  reading the GPS chars as they come in, so that no chars are lost.

static void doSomeWork()
{
  // Print all the things!
  //Printed things: status, UTC, Date/Time, Lat, Lon, Hdg, Spd, Alt, Sats, Rx ok, Rx err, RX chars
  Serial.print("GPS///");
  trace_all( DEBUG_PORT, gps, fix );

} 


//  This is the main GPS parsing loop.

static void GPSloop()
{
  while (gps.available( gpsPort )) {
    fix = gps.read();
    doSomeWork();
  }

} 


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        Serial.print("<");
        Serial.print(receivedChars);
        Serial.println(">");
        newData = false;
    }
}

void powerSensing() {
    //read the temp sensor
    sensors.requestTemperatures(); 

    
      // Read the Current Sensor ACS712
    unsigned int x=0;
    float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,ampere0=0.0;

    for (int x = 0; x < 20; x++){ //Get 20 samples
    AcsValue = analogRead(Amp0Pin);     //Read current sensor values   
    Samples = Samples + AcsValue;  //Add samples together
    delay (2); // let ADC settle before next sample 3ms
    }
  
    AvgAcs=Samples/20.0;//Taking Average of Samples

    //((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
    //2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
    //out to be 2.5 which is out offset. If your arduino is working on different voltage than 
    //you must change the offset according to the input voltage)
    //0.185v(185mV) is rise in output voltage when 1A current flows at input Version5A
    //0.100v '' Version20A
    //0.066v '''Version30A
    ampere0 = (1.5 - (AvgAcs * (3.0 / 4096.0)) )/0.185;
  

    int sensorValue0 = analogRead(V0Pin);
    float voltage0 = sensorValue0 * (3.0 / 4096.0) * 5;
    
    Serial.print("<PowerSensing ");
    Serial.print("Temperature: ");
    Serial.print(sensors.getTempCByIndex(0));
    Serial.print("C | ");
     //print the voltage readings
    Serial.print("Voltage0: ");
    Serial.print(voltage0);
    Serial.print("V | ");
      //print the current sensors
    Serial.print("Ampere0: ");
    Serial.print(ampere0);
    Serial.print("A > ");
    
}
}
