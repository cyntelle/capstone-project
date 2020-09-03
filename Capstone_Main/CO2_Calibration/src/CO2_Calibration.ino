/*
 * Project CO2_Calibration
 * Description:
 * Author:
 * Date:
 */
// HEADER section
#include "MQ135.h"
const int inPinD15=D15;
long int sensorValue;
long int nbrReadings=0;
float sensorValueAvg=0.0;
float totSensor=0.0;
float rZero;
float CO2_PPM;
MQ135 gasSensor = MQ135(inPinD15);

// end HEADER section
SYSTEM_MODE(SEMI_AUTOMATIC);
void setup() 
{
  Serial.begin(9600);
  while(!Serial);
  delay(1000);
  pinMode(inPinD15,INPUT);
  Serial.printf("Program CO2_Calibration\n");
//  Calibrate_sensor();
//  Get_PPM_Conc();
}

void loop() 
{
//  Calibrate_sensor();
  Get_PPM_Conc();
}
void Calibrate_sensor()
{
  rZero=gasSensor.getRZero();
  totSensor+=rZero;
  nbrReadings++;
  sensorValueAvg= (float) totSensor/ (float) nbrReadings;
  // Print the current sensor reading, number readings & running average
  Serial.printf("Sensor reading >%0.2f/ Number readings >%i/ Running Average >%0.2f\n",rZero, nbrReadings, sensorValueAvg); 
  delay(1000); 
  return;
}

void Get_PPM_Conc()
{
  CO2_PPM = gasSensor.getPPM();
  Serial.printf("PPM >%0.2f\n",CO2_PPM);
  delay(1000); 
}
