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

 * Author: Ted Fites & Cyntelle Renteria
 * Date: 8/23/20
 * Modifications:
 * 8/23/20  CR Created program + added MQ9 function (not tested)
 * 8/31/20  CR modified + tested MQ9 sensor function in clean air (working)
 * 
 */

// HEADER section ********************************************************

// Constants & variables

//******* Variables for M01_get_MQ9_data function **********//
void setup();
void loop();
void M01_get_MQ9_data();
#line 28 "/Users/cyntelle/Documents/IoT/capstone/Capstone_Main/src/Capstone_Main.ino"
const int MQ9_Addr = 0x50; // Address for MQ9 I2C CO Sensor
unsigned int data[2];
int raw_adc = 0;
float COppm = 0.0;


void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MQ9_Addr);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() 
//************************************************************************************
//*************************************  MAIN LOOP  **********************************
//************************************************************************************
{
    M01_get_MQ9_data();

} // end MAIN loop  ********************************************************************

void M01_get_MQ9_data()
{
  // Start I2C transmission
  Wire.beginTransmission(MQ9_Addr);
  Wire.write(0x00);
  Wire.endTransmission(false);
  // Request 2 bytes of data
  Wire.requestFrom(MQ9_Addr, 2, true);
  // Read 2 bytes of data: raw_adc msb, raw_adc lsb
  data[0] = Wire.read();
  data[1] = Wire.read();
  delay(300);
  // Convert the data to 12-bits
  raw_adc = ((data[0] & 0x0F) * 256) + data[1];
  COppm = (1000.0 / 4096.0) * raw_adc + 10.0;
  Serial.printf("Carbon Monoxide: %0.2fppm\n", COppm);
}