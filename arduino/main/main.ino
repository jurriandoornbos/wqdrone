#include <EEPROM.h>
#include <GravityTDS.h>

#define SCOUNT  30           // sum of sample point

#define TdsSensorPin A1
#define TurbPin A0


GravityTDS gravityTds;

int analogBufferTDS[SCOUNT], analogBufferTurb[SCOUNT];    // store the analog value in the array, read from ADC
float temperature = 18,tdsValue = 0, medianTDS = 0, medianTurb = 0;
int analogBufferIndex = 0,copyIndex = 0;


//Find the median in an array:
int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
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
   
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
    
     //temperature = readTemperature();  //add your temperature sensor and read it

     //TDS sampling
     gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
     gravityTds.update();  //sample and calculate
     tdsValue = gravityTds.getTdsValue();  // then get the value

     //Turbidity sampling
     int turbValue = analogRead(TurbPin);
     float turbVoltage = turbValue * (5.0/1024.0);

     
     analogSampleTimepoint = millis();
     analogBufferTDS[analogBufferIndex] = tdsValue; //read the calibrated value and store into the buffer
     analogBufferTurb[analogBufferIndex] = turbVoltage; //read the analog voltage and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }

   static unsigned long printTimepoint = millis();
   
   if(millis()-printTimepoint > 800U)
   {
      //TDS printing and filtering
      printTimepoint = millis();
      medianTDS = getMedianNum(analogBufferTDS,SCOUNT);

      Serial.print("TDS: ");          
      Serial.print(medianTDS,0);
      Serial.println(" ppm");
      
      //Turb filtering and printing
      printTimepoint = millis();
      medianTurb = getMedianNum(analogBufferTurb, SCOUNT);
      
      Serial.print("Turbidity Voltage: ");
      Serial.print(medianTurb);
      Serial.println(" v");
   }

}
