/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/Ted/Documents/IoT/capstone-project/Capstone_Main/NO2_Get_VladderOffsetVal/src/NO2_Get_VladderOffsetVal.ino"
/*
 * Project NO2_Get_VladderOffsetVal
 * Description:
 * Author: Ted Fites
 * Date: 8/31/20
 */
// HEADER section
void setup();
void loop();
#line 8 "c:/Users/Ted/Documents/IoT/capstone-project/Capstone_Main/NO2_Get_VladderOffsetVal/src/NO2_Get_VladderOffsetVal.ino"
const int SMPL_SIZE = 256;
const int analogInPinD15=D15;
long int offSetVal;

// end HEADER section
// setup() runs once, when the device is first turned on.
SYSTEM_MODE(SEMI_AUTOMATIC);
void setup() 
{
  Serial.begin(9600);
  while(!Serial);
  delay(1000);
  Serial.printf("Leson NO2_Get_VladderOffsetVal\n");
  offSetVal = 0;
  for (int i = 0; i < SMPL_SIZE; i++) 
  {
    offSetVal = analogRead(analogInPinD15); // source code with analog read
    Serial.printf(" LOOP: offSetVal >%i\n", offSetVal);
    delay(3);   // needs 2 ms for the analog-to-digital converter to settle after the last reading
  }
  Serial.printf(" TOTAL: offSetVal >%i\n", offSetVal);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() 
{

}