#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 2); //TX, RX
// (Send and Receive)
//String message = "Bericht $";
int i = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial){
  ;
  }
  mySerial.begin(9600);
}

void loop() {

  //mySerial.println(message + i);
  //delay(1000);
  
  if(Serial.available() > 15){//Read from serial monitor and send over LoRa wireless module
    String input = Serial.readStringUntil('$');
 //   //input.trim();
    mySerial.print(input);
    mySerial.println('$'); 
  }
 
  if(mySerial.available() > 30){//Read from LoRa wireless module and send to serial monitor
   String input = mySerial.readStringUntil('$');
   //input.trim();
   Serial.println(input);
  }
  //i+=1;
  //Serial.println(i);
}
