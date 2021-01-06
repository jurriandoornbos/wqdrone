#include <EEPROM.h>
#include <GravityTDS.h>
#include <OneWire.h>

#define SCOUNT  30           // sum of sample point

#define TdsSensorPin A1
#define TurbPin A0
#define PhPin A2
#define Offset 0.00

//TDS chip i/o
GravityTDS gravityTds;

// store the sensor values in the array
float analogBufferTDS[SCOUNT], analogBufferTurb[SCOUNT], analogBufferTemp[SCOUNT], analogBufferPH[SCOUNT], analogBufferTurb_v[SCOUNT], analogBufferPH_v[SCOUNT];  
float temperature = 18.0,tdsValue = 0.0, medianTDS = 0.0,  medianTemp = 0.0, medianPH = 0.0, medianPH_v = 0.0;
int analogBufferIndex = 0,copyIndex = 0,DS18S20_Pin = 2;  

//PH Calibration 
//m = (ph7 - ph4) / (Vph7 - Vph4)
const float m = -5.268;

//Turb calibration
// tb = (turbhi-turblo)/(vTurbhi - vTurblo)
const float tb = 303;
     
//Temperature chip i/o
OneWire ds(DS18S20_Pin);


void setup()
{
    Serial.begin(115200);
    gravityTds.setPin(TdsSensorPin);
    gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
    gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
    gravityTds.begin();  //initialization
}

void loop() 
{
   static unsigned long analogSampleTimepoint = millis();
   static float turbVoltage, turbNTU, medianTurb, medianTurb_v;
   static int turbValue;
      
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     //Temperature sampling, temperature is also used in TDS adjustment
     temperature = getTemp();  //add your temperature sensor and read it
    
     //TDS sampling
     gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
     gravityTds.update();  //sample and calculate
     tdsValue = gravityTds.getTdsValue();  // then get the value

     //Turbidity sampling
     turbValue = analogRead(TurbPin);
     turbVoltage = turbValue * (5.0/1024.0);
     turbNTU = -2172.82 + (631.9 * turbVoltage);

     //pH Sampling
     int phSensorValue = analogRead(PhPin);
     float phVoltage = phSensorValue * (5.0/1024.0);
     float phValue = 7.78-(2.5 - phVoltage) * m;

     analogSampleTimepoint = millis();
     
     analogBufferTDS[analogBufferIndex] = tdsValue; //read the calibrated value and store into the buffer
     analogBufferTurb[analogBufferIndex] = turbNTU;//read the analog voltage and store into the buffer
     analogBufferTemp[analogBufferIndex] = temperature;//read the digital temperature and store into the buffer
     analogBufferPH[analogBufferIndex] = phValue;//read the analog offsetted Temperature value
     analogBufferPH_v[analogBufferIndex] = phVoltage;
     analogBufferTurb_v[analogBufferIndex] = turbVoltage;
     
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }

   static unsigned long printTimepoint = millis();
   
   if(millis()-printTimepoint > 800U)
   {
      //Temperature filtering and printing
      printTimepoint = millis();
      medianTemp = getMedianNum(analogBufferTemp, SCOUNT);

      Serial.print("Temperature: ");
      Serial.print(medianTemp,3);
      Serial.print(" Celsius; ");

      
      //TDS printing and filtering
      printTimepoint = millis();
      medianTDS = getMedianNum(analogBufferTDS,SCOUNT);

      Serial.print("TDS: ");          
      Serial.print(medianTDS,0);
      Serial.print(" ppm; ");

      
      //Turb filtering and printing
      printTimepoint = millis();
      medianTurb = getMedianNum(analogBufferTurb, SCOUNT);
      medianTurb_v = getMedianNum(analogBufferTurb_v,SCOUNT);
      
      Serial.print("Turbidity: ");
      Serial.print(medianTurb);
      Serial.print(" NTU; ");


      //pH filtering and printing
      printTimepoint = millis();
      medianPH = getMedianNum(analogBufferPH, SCOUNT);
      medianPH_v = getMedianNum(analogBufferPH_v, SCOUNT);
      Serial.print("Acidity: ");
      Serial.print(medianPH);
      Serial.println(" PH; ");
   }

}


//Find the median in an array:
float getMedianNum(float bArray[], int iFilterLen) 
{
      float bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j;
      float bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}


float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
  
}
