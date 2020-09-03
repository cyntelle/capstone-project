/*
 * Project NO2_Get_VladderOffsetVal
 * Description:
 * This program is used for refining program NO2_GasCollection by finding a valid 
 * voltage ladder value to be used as an offset value. This value is to be sutracted 
 * from the inpout sensor value obtained from the analog read on the NO2 gas sensor 
 * pin. We attach the pin from an unused ADC Argon pin and connect it to a point within
 * the voltage ladder we are using the offset for. This ladder provides a new base zero
 * point for low voltage readings, where we wish to accurately capture an analog reading
 * within the voltage ladder. This value will be our offset
 * Results range from 360-380, so use 375.
 * Author: Ted Fites
 * Date: 8/31/20
 */
// HEADER section
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