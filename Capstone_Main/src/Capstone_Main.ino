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
 * 
 */

// HEADER section ********************************************************

// Constants & variables

//******* Variables for M01_get_MQ9_data function **********//
const int MPU_ADDR = 0x00; // Address for MQ9 I2C CO Sensor
int16_t rawADCval;
int COppm;


void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x6B);
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
  Wire.beginTransmission(MPU_ADDR);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  rawADCval = (Wire.read() << 8) | Wire.read();
  COppm = ((1000/4096)*rawADCval)+10;
  Serial.printf("Carbon Monoxide: %i\n", COppm)
}