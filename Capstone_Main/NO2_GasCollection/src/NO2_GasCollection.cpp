/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/Ted/Documents/IoT/capstone-project/Capstone_Main/NO2_GasCollection/src/NO2_GasCollection.ino"
/*
 * Project NO2_GasCollection.ino
 *  This program provides tested code for the main driver gas collection program (Capstone_Main.ino).
 * This program consists of two functionS: 
 * M01_GetGasConcentration_MakerIO
 * M02_GetGasConcentration_MakerIO_TEST
 * Function M01 contains the original code from David Peasley's CO gas collection program from website MAKer.IO.
 * This is the SOURCE function whose calculated gas concentration values will be compared against with
 * the test function M02. Code in M02 incorporates certain program refinements recommended by BR in order
 * to make the calculation more readily comprehensible, and to more easily incorporate additional calculation
 * corrections should they be required. When the calculated gas concentrations from the test function
 * M02_GetGasConcentration_MakerIO-TEST match those calculations using the same test input values from the
 * M01 source function, then final program changes will be included and tested in production function:
 * M03_GetGasConcentration_MakerIO-FINAL 
 *  This last function, along with required constants & variables in the HEADER section, will 
 * be included into the driver program Capstone_Main.ino for production gas collection.
 * In lieu of actual gas concentration data, the program will provide input test values for functions M01 & M02
 * in order to create calculation values for comparison between functions.
 *  These functions can be run a single time in SETUP() in  order to better compare their results.
 * 
 * Author: Ted Fites
 * Date: 8/29/20
 */
// *** M01 & M02 _GetGasConcentration_MakerIO: program CONSTANTS
//const float Vref = 1.1;  //ORIGINAL VALUE: This is the voltage of the internal reference sourced from Maker.IO
void setup();
void loop();
void M01_GetGasConcentration_MakerIO();
void M02_GetGasConcentration_MakerIO_TEST();
void M03_GetGasConcentration_MakerIO_FINAL();
#line 26 "c:/Users/Ted/Documents/IoT/capstone-project/Capstone_Main/NO2_GasCollection/src/NO2_GasCollection.ino"
const float VOLTS_LADDER_REF = 1.0;  // TEST internal voltage used while determining final internal voltage value
const float V_REF = 3.3; // Per BR, adjusted internal voltage set to Argon input voltage. will triple nanoAmp 
// calculation compared to TEST VOLTS_LADDER_REF. 
const float S_F = 1.4; // sensitivity in nA/ppm. This is roughly about 1/2 the sensitivity of the barcode on the
//sensor (2.78::see Spec Sensor 3SP_NO2_5F-P-Package-110-507.pdf). A number less than 2 will inflate the #PPMs 

const int R1_VALUE = 9700;  // Value of 10kOhm resistor (falls within +/- 5% range)
const int SMPL_SIZE = 256; //Number of samples averaged, like adding 8 bits to ADC
const int VLTG_LADDER_OFF = 375; // replaces source C_OFF for accurate voltage ladder offset from sesnor reading
const int analogInPinD14 = D14;  // Analog input pin that the sensor is attached to
const long int TEST_SENSOR_VALUE=10000; // provides intial TEST value for program,replacing analogRead() values
const long int C_OFF = 68286; //DEFUNCT: 286mV offset due to resistor ladder. Try taking the average of a long
//measurement of the counts without a sensor in place. This should give a good Zero.

// END M01 & M02 _GetGasConcentration_MakerIO: CONSTANTS

// *** M03_GetGasConcentration_MakerIO: program CONSTANTS
/*
* Input resolution on Argon mc is 2 12th power. This is 4 times more refined than the resolution used in M01 & 
* M02 functions. 
*/
const float CONTROLLER_RESOLUTION = 4096; 
const float NANO_AMPS = 1000000000.0;
// END M03_GetGasConcentration_MakerIO: program CONSTANTS

long int sensorValue = 0;        // ORIGINAL value read from the sensor
float PPMconc=0.0;

// setup() runs once, when the device is first turned on.
SYSTEM_MODE(SEMI_AUTOMATIC);
void setup() 
{
  Serial.begin(9600);
  while(!Serial);
  delay(1000);
  Serial.printf("Start NO2_GasCollection\n");

//  M01_GetGasConcentration_MakerIO();
//  M02_GetGasConcentration_MakerIO_TEST();
//  M03_GetGasConcentration_MakerIO_FINAL();
}

void loop() 
{
/*
******************************** M A I N  L O O P **********************************************
*/
  M03_GetGasConcentration_MakerIO_FINAL();
/*
******************************** E N D   L O O P  **********************************************
*/
}

void M01_GetGasConcentration_MakerIO()
{
  Serial.printf("M01_GetGasConcentration_MakerIO\n");
// 1) read the test analog in values:
  sensorValue = 0;
  for (int i = 0; i < SMPL_SIZE; i++) 
  {
 //   sensorValue = analogRead(analogInPin) + sensorValue; // source code with analog read
    sensorValue+= TEST_SENSOR_VALUE; // replaces read value with constant value each time in loop
    delay(3);   // needs 2 ms for the analog-to-digital converter to settle after the last reading
  }
  Serial.printf(" TOTAL: sensorValue >%i\n", sensorValue);

 //2) subtract the offset of the resistor ladder * 256
 sensorValue = sensorValue - C_OFF; 
Serial.printf(" TOTAL: sensorValue following subtraction >%i\n", sensorValue);

  // 3) print the PPM results to the serial monitor:
  Serial.print("PPM = ");
  Serial.println( ((float) sensorValue / SMPL_SIZE / 1024 * VOLTS_LADDER_REF / R1_VALUE * 1000000000) / S_F);
 //Trying to make each loop 1 second
  delay(218);  //1 second – 3ms*256ms (each adc read)-14ms (for printing)= 218ms
  return;
}

void M02_GetGasConcentration_MakerIO_TEST()
{
  float rawInput=0.0;
  float calc_nA=0.0;

  Serial.printf("\nM02_GetGasConcentration_MakerIO-TEST\n");
// 1) read the test analog in values:
  sensorValue = 0;
  for (int i = 0; i < SMPL_SIZE; i++) 
  {
 //   sensorValue = analogRead(analogInPin) + sensorValue; // source code with analog read
    sensorValue+= TEST_SENSOR_VALUE; // replaces read value with constant value each time in loop
    delay(3);   // needs 2 ms for the analog-to-digital converter to settle after the last reading
  }
  Serial.printf(" TOTAL: sensorValue >%i\n", sensorValue);

//2) subtract the offset of the resistor ladder * 256
 sensorValue = sensorValue - C_OFF; 
Serial.printf(" TOTAL: sensorValue following subtraction >%i\n", sensorValue);

 // 3) print the PPM results to the serial monitor:
  rawInput = (float) sensorValue / (float) SMPL_SIZE / 1024.0;
  calc_nA =  VOLTS_LADDER_REF/ (float) R1_VALUE * 1000000000.0;
  PPMconc = (float) rawInput * calc_nA / S_F;
  Serial.printf("PPM >%0.2f\n",PPMconc);
   
 //Trying to make each loop 1 second
  delay(218);  //1 second – 3ms*256ms (each adc read)-14ms (for printing)= 218ms
  return;
}

void M03_GetGasConcentration_MakerIO_FINAL()
{
  float rawInput=0.0;
  float calc_nA=0.0;

  Serial.printf("\nM03_GetGasConcentration_MakerIO-FINAL\n");
// 1) read and accumulate the test analog in values:
  sensorValue = 0;
  for (int i = 0; i < SMPL_SIZE; i++) 
  {
    /*
    * 8/31/20 TF. Per BR, relocate voltage ladder offset to INSIDE sample size loop to more
    * accurately reduce input sensor reading dur to ladder. This will make low sensor readings
    * more accurate.
    */
    sensorValue += analogRead(analogInPinD14) - VLTG_LADDER_OFF; // source code with analog read
    delay(3);   // needs 2 ms for the analog-to-digital converter to settle after the last reading
  }
  Serial.printf(" TOTAL: sensorValue >%i\n", sensorValue);

 // 2) print the PPM results to the serial monitor:
  rawInput = (float) sensorValue / (float) SMPL_SIZE / CONTROLLER_RESOLUTION;
  calc_nA =  V_REF/ (float) R1_VALUE * NANO_AMPS; // I=V/R * nanoAmps
  PPMconc = (float) rawInput * calc_nA / S_F;
  Serial.printf("PPM >%0.2f\n",PPMconc);
   
 //Trying to make each loop 1 second
  delay(218);  //1 second – 3ms*256ms (each adc read)-14ms (for printing)= 218ms
  return;
}