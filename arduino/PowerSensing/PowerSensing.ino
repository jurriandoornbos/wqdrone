//BuzzerMeme-ing
const int c = 261;
const int d = 294;
const int e = 329;
const int f = 349;
const int g = 391;
const int gS = 415;
const int a = 440;
const int aS = 455;
const int b = 466;
const int cH = 523;
const int cSH = 554;
const int dH = 587;
const int dSH = 622;
const int eH = 659;
const int fH = 698;
const int fSH = 740;
const int gH = 784;
const int gSH = 830;
const int aH = 880;

int counter =0;

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS 4


#define SCOUNT  30           // sum of sample point

#define V2Pin A2
#define V1Pin A1
#define V0Pin A0

#define Amp0Pin A3
#define Amp1Pin A4
#define Amp2Pin A5
int BuzPin = 5;
int Water0Pin = 2;
int Water1Pin = 3;

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);  

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);


void setup(){
    Serial.begin(115200);
    sensors.begin();  //initialization
    pinMode(Water0Pin, INPUT);
    pinMode(Water1Pin, INPUT);
    pinMode(BuzPin, OUTPUT);
}

void loop(){
  //Read the temp sensor DS18_20
  sensors.requestTemperatures(); 

  // Read the voltage sensors
  int sensorValue0 = analogRead(V0Pin);
  int sensorValue1 = analogRead(V1Pin);
  int sensorValue2 = analogRead(V2Pin);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage0 = sensorValue0 * (5.0 / 1023.0) * 5;
  float voltage1 = sensorValue1 * (5.0 / 1023.0) * 5;
  float voltage2 = sensorValue2 * (5.0 / 1023.0) * 5;


  // Read the Current Sensor ACS712
  unsigned int x=0;
  float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,ampere0=0.0;

  for (int x = 0; x < 5; x++){ //Get 20 samples
  AcsValue = analogRead(Amp0Pin);     //Read current sensor values   
  Samples = Samples + AcsValue;  //Add samples together
  delay (1); // let ADC settle before next sample 3ms
  }
  
  AvgAcs=Samples/5.0;//Taking Average of Samples

  //((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
  //2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
  //out to be 2.5 which is out offset. If your arduino is working on different voltage than 
  //you must change the offset according to the input voltage)
  //0.185v(185mV) is rise in output voltage when 1A current flows at input Version5A
  //0.100v '' Version20A
  //0.066v '''Version30A
  ampere0 = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.100;

  //Rinse, repeat for Amp1
  x=0;
  AcsValue=0.0,Samples=0.0,AvgAcs=0.0;
  float ampere1=0.0;
  for (int x = 0; x < 5; x++){ //Get 20 samples
  AcsValue = analogRead(Amp1Pin);     //Read current sensor values   
  Samples = Samples + AcsValue;  //Add samples together
  delay (1); // let ADC settle before next sample 2ms
  }
  
  AvgAcs=Samples/5.0;//Taking Average of Samples

  //((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
  //2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
  //out to be 2.5 which is out offset. If your arduino is working on different voltage than 
  //you must change the offset according to the input voltage)
  //0.185v(185mV) is rise in output voltage when 1A current flows at input Version5A
  //0.100v '' Version20A
  //0.066v '''Version30A
  ampere1 = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.100;

    //Rinse, repeat vor Amp2Pin
  x=0;
  AcsValue=0.0,Samples=0.0,AvgAcs=0.0;
  float ampere2=0.0;
  for (int x = 0; x < 5; x++){ //Get 20 samples
  AcsValue = analogRead(Amp2Pin);     //Read current sensor values   
  Samples = Samples + AcsValue;  //Add samples together
  delay (1); // let ADC settle before next sample 3ms
  }
  
  AvgAcs=Samples/5.0;//Taking Average of Samples

  //((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
  //2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
  //out to be 2.5 which is out offset. If your arduino is working on different voltage than 
  //you must change the offset according to the input voltage)
  //0.185v(185mV) is rise in output voltage when 1A current flows at input Version5A
  //0.100v '' Version20A
  //0.066v '''Version30A
  ampere2 = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.100;

  // Read the water sensors
  int WaterValue0 = digitalRead(Water0Pin);
  int WaterValue1 = digitalRead(Water1Pin);


  // Write the water sensor value to the buzzer; if water -> beep Vedar Theme
  //if (WaterValue0 ==  LOW && WaterValue1 == LOW || WaterValue0 == LOW || WaterValue1 == LOW){
  //  firstSection();
  //}
  //else{
  //  digitalWrite(BuzPin,LOW);
  //'}
 
  Serial.print("PowerSensingDuino ");
    //print the temperature in Celsius
  Serial.print("Temperature: ");
  Serial.print(sensors.getTempCByIndex(0));
  Serial.print("C | ");

  //print the voltage readings
  Serial.print("Voltage0: ");
  Serial.print(voltage0);
  Serial.print("V | ");
  Serial.print("Voltage1: ");
  Serial.print(voltage1);
  Serial.print("V | ");

  Serial.print("Voltage2: ");
  Serial.print(voltage2);
  Serial.print("V | ");
  
  //print the current sensors
  Serial.print("Ampere0: ");
  Serial.print(ampere0);
  Serial.print("A | ");
  //print the current sensors
  Serial.print("Ampere1: ");
  Serial.print(ampere1);
  Serial.print("A | ");
  //print the current sensors
  Serial.print("Ampere2: ");
  Serial.print(ampere2);
  Serial.print("A | ");


  //print the water sensors
  Serial.print("Water0: ");
  Serial.print(WaterValue0);
  Serial.print(" | ");
  Serial.print("Water1: ");
  Serial.print(WaterValue1);
  Serial.println(" | ");


  delay(10);
}


// More star wars memeage
void beep(int note, int duration)
{
  //Play tone on buzzerPin
  tone(BuzPin, note, duration);
 
  //Play different LED depending on value of 'counter'
  if(counter % 2 == 0)
  {

    delay(duration);

  }else
  {

    delay(duration);

  }
 
  //Stop tone on buzzerPin
  noTone(BuzPin);
 
  delay(01);
 
  //Increment counter
  counter++;
}


//Star Wars theme
void firstSection()
{
  beep(a, 500);
  beep(a, 500);    
  beep(a, 500);
  beep(f, 350);
  beep(cH, 150);  
  beep(a, 500);
  beep(f, 350);
  beep(cH, 150);
  beep(a, 650);
 
  delay(500);
 
  beep(eH, 500);
  beep(eH, 500);
  beep(eH, 500);  
  beep(fH, 350);
  beep(cH, 150);
  beep(gS, 500);
  beep(f, 350);
  beep(cH, 150);
  beep(a, 650);
 
  delay(500);
}
