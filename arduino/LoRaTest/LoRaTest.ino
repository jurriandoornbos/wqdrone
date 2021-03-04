#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); //TX, RX
// (Send and Receive)

void setup() {
  Serial.begin(9600);
  while (!Serial){
  ;
  }
  mySerial.begin(9600);
}

void loop() {
  
 char inByte = ' ';  
  if(Serial.available() > 0){//Read from serial monitor and send over LoRa wireless module
    char inByte = Serial.read();
    mySerial.println(inByte);    
  }
 char outByte = ' ';
  if(mySerial.available() > 0){//Read from LoRa wireless module and send to serial monitor
    char outByte = mySerial.read();
    Serial.println(outByte);    
  }
  delay(10);
}
