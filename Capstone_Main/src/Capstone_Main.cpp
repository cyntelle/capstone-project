/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/cyntelle/Documents/IoT/capstone/Capstone_Main/src/Capstone_Main.ino"
/*
 * Project: Capstone Driver Program
 * Description: The purpose of this capstone project is to establish a cost-effective detection mechanism 
 * for two major organizations: City of Albuquerque and Fuse Makerspace. This project focuses on the 
 * detection and data analysis of airborne pollutants emitted from ABQ city- and Fuse-owned equipment.
 * Components/Sensors included:
 * - Particle Argon Microcontroller
 * - NeoPixel Ring/Strip 
 * - Particulate Matter Sensor
 * - CO (& Methane) - MQ9 I2C
 * - CO2 - I2C 
 * - NO2 - Analog
 * - VOC/Alcohol - Analog MQ3

 * Author: Cyntelle Renteria & Ted Fites
 * Date: 8/23/20
 * Modifications:
 * 8/31/20  CR Added function M02_get_MQ131_data to capture ozone gas emissions + Json function to post CO & O3 data to Particle Console
 * 8/31/20  TF Added function M03_GetGasConcentration_MakerIO_FINAL to calculate NO2 gas concentrations
 * 8/31/20  CR modified + tested MQ9 sensor function in clean air (working)
 * 8/23/20  CR Created program + added MQ9 function (not tested)
 * 
 */

// HEADER section ********************************************************

//Header Files
#include "JsonParserGeneratorRK.h"

// Constants & variables

//******* M01_get_MQ9_data constants and variables for function **********//
void setup();
void loop();
void M01_get_MQ9_data();
void M02_get_MQ131_data();
void M03_GetGasConcentration_MakerIO_FINAL();
void  createEventPayLoad(float COppm, float O3ppm);
#line 33 "/Users/cyntelle/Documents/IoT/capstone/Capstone_Main/src/Capstone_Main.ino"
const int MQ9_Addr = 0x50; // Address for MQ9 I2C CO Sensor
unsigned int MQ9_data[2];
int MQ9_raw_adc = 0;
float COppm = 0.0;
// END  Variables for M01_get_MQ9_data function **********//

//******* M02_get_MQ131_data constants and variables for function **********//
const int MQ131_Addr = 0x51; // Address for MQ131 I2C Ozone Sensor
unsigned int MQ131_data[2];
int MQ131_raw_adc = 0;
float O3ppm = 0.0;
// END  Variables for M02_get_MQ131_data function **********//

// *** M03_GetGasConcentration_MakerIO-FINAL: program CONSTANTS & VARIABLES
/*
* Input resolution on Argon mc is 2 12th power. This is 4 times more refined than the resolution used in M01 & 
* M02 functions (used in source microcontroller Arduino Uno from source program).
*/
const float V_REF = 3.3; // Per BR, adjusted internal voltage set to Argon input voltage. will triple nanoAmp 
// calculation compared to TEST VOLTS_LADDER_REF. 
const float S_F = 1.4; // sensitivity in nA/ppm. This is roughly about 1/2 the sensitivity of the barcode on the
//sensor (2.78::see Spec Sensor 3SP_NO2_5F-P-Package-110-507.pdf). A number less than 2 will inflate the #PPMs 

const int analogInPinD14 = D14;  // Analog input pin that the sensor is attached to
const int R1_VALUE = 9700;  // Value of 10kOhm resistor (falls within +/- 5% range)
const int SMPL_SIZE = 256; //Number of samples averaged, like adding 8 bits to ADC
const int VLTG_LADDER_OFF = 375; // replaces source C_OFF for accurate voltage ladder offset from sesnor reading

const float CONTROLLER_RESOLUTION = 4096; 
const float NANO_AMPS = 1000000000.0;

long int NO2SensorValue = 0;        // ORIGINAL value read from the sensor
float NO2PPMconc=0.0;  // Stores calculated NO2 gas concentration from sensor readings
// END M03_GetGasConcentration_MakerIO: program CONSTANTS & VARIABLES



void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MQ9_Addr);
  Wire.beginTransmission(MQ131_Addr);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() 
/************************************************************************************
**************************************  MAIN LOOP  **********************************
*************************************************************************************/
{
    M01_get_MQ9_data();
    M02_get_MQ131_data();
    // M03_GetGasConcentration_MakerIO_FINAL();
    createEventPayLoad(COppm,O3ppm);
} //*********************************** END LOOP  *************************************

void M01_get_MQ9_data()
{
  // Start I2C transmission
  Wire.beginTransmission(MQ9_Addr);
  Wire.write(0x00);
  Wire.endTransmission(false);
  // Request 2 bytes of data
  Wire.requestFrom(MQ9_Addr, 2, true);
  // Read 2 bytes of data: raw_adc msb, raw_adc lsb
  MQ9_data[0] = Wire.read();
  MQ9_data[1] = Wire.read();
  delay(300);
  // Convert the data to 12-bits
  MQ9_raw_adc = ((MQ9_data[0] & 0x0F) * 256) + MQ9_data[1];
  COppm = (1000.0 / 4096.0) * MQ9_raw_adc + 10.0;
  Serial.printf("Carbon Monoxide: %0.2fppm\n", COppm);
}

void M02_get_MQ131_data()
{
  // Start I2C transmission
  Wire.beginTransmission(MQ131_Addr);
  Wire.write(0x00);
  Wire.endTransmission(false);
  // Request 2 bytes of data
  Wire.requestFrom(MQ131_Addr, 2, true);
  // Read 2 bytes of data
  // raw_adc msb, raw_adc lsb
    MQ131_data[0] = Wire.read();
    MQ131_data[1] = Wire.read();
  delay(300);
  // Convert the data to 12-bits
  MQ131_raw_adc = ((MQ131_data[0] & 0x0F) * 256) + MQ131_data[1];
  O3ppm = (1.99 / 4095.0) * MQ131_raw_adc + 0.01;
  Serial.printf("Ozone: %0.2f ppm\n", O3ppm);
}

void M03_GetGasConcentration_MakerIO_FINAL()
{
  float rawInput=0.0;
  float calc_nA=0.0;

  Serial.printf("\nM03_GetGasConcentration_MakerIO-FINAL\n");
// 1) read and accumulate the test analog in values:
  NO2SensorValue = 0;
  for (int i = 0; i < SMPL_SIZE; i++) 
  {
    /*
    * 8/31/20 TF. Per BR, relocate voltage ladder offset to INSIDE sample size loop to more
    * accurately reduce input sensor reading dur to ladder. This will make low sensor readings
    * more accurate.
    */
    NO2SensorValue += analogRead(analogInPinD14) - VLTG_LADDER_OFF; // source code with analog read
    delay(3);   // needs 2 ms for the analog-to-digital converter to settle after the last reading
  }
  Serial.printf(" TOTAL: NO2SensorValue >%i\n", NO2SensorValue);

 // 2) print the PPM results to the serial monitor:
  rawInput = (float) NO2SensorValue / (float) SMPL_SIZE / CONTROLLER_RESOLUTION;
  calc_nA =  V_REF/ (float) R1_VALUE * NANO_AMPS; // I=V/R * nanoAmps
  NO2PPMconc = (float) rawInput * calc_nA / S_F;
  Serial.printf("PPM >%0.2f\n",NO2PPMconc);
   
 //Trying to make each loop 1 second
  delay(218);  //1 second – 3ms*256ms (each adc read)-14ms (for printing)= 218ms
  return;
}

void  createEventPayLoad(float COppm, float O3ppm)
{
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("Ozone", O3ppm);
    jw.insertKeyValue("Carbon Monoxide", COppm);
  }
  Particle.publish("gas-emissions", jw.getBuffer(), PRIVATE);
}