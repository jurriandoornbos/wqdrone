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
  
  
  if(Serial.available() > 0){//Read from serial monitor and send over LoRa wireless module
    String input = Serial.readStringUntil('$');
    input.trim();
    mySerial.print(input);
    mySerial.println(); 
  }
 
  if(mySerial.available() > 1){//Read from LoRa wireless module and send to serial monitor
    String input = mySerial.readStringUntil('$');
    input.trim();
    Serial.print(input);
    Serial.println();
  }
}
