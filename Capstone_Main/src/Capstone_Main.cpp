/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/Ted/Documents/IoT/capstone-project/Capstone_Main/src/Capstone_Main.ino"
/*
 * Project: Capstone Driver Program
 * Description: The purpose of this capstone project is to establish a cost-effective detection mechanism 
 * for two major organizations: City of Albuquerque and Fuse Makerspace. This project focuses on the 
 * detection and data analysis of airborne pollutants emitted from ABQ city- and Fuse-owned equipment.
 * Components/Sensors included:
 * - Particle Argon Microcontroller
 * - NeoPixel Ring/Strip 
 * - Particulate Matter Sensor (HM3301)
 * - Ozone - (MQ131)
 * - CO - (MQ9 I2C)
 * - CO2 - (MQ135)
 * - NO2 - Analog

 * Author: Cyntelle Renteria & Ted Fites
 * Date: 8/23/20
 * Modifications:
 * 9/2/20 CR added function M04_get_HM3301_data to capture particulate matter data + neopixel 
 *        to visualize good/medium/poor air quality + Adafruit IO dashboard to publish data to cloud
 * 8/31/20  CR Added function M02_get_MQ131_data to capture ozone gas emissions + Json function 
 *          to post CO & O3 data to Particle Console
 * 8/31/20  TF Added function M03_GetGasConcentration_MakerIO_FINAL to calculate NO2 gas concentrations
 * 8/31/20  CR modified + tested MQ9 sensor function in clean air (working)
 * 8/23/20  CR Created program + added MQ9 function (not tested)
 * 
 */

// HEADER section ********************************************************

//Header Files
#include <secrets.h>
#include <Adafruit_MQTT.h>
#include "JsonParserGeneratorRK.h"
<<<<<<< HEAD
#include "MQ135.h"
=======
#include "neopixel.h"
>>>>>>> 2981049f078de13ec83d582d3dcf62cf2993101d

#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "Adafruit_MQTT/Adafruit_MQTT.h" 

/************************* Adafruit.io Setup *********************************/ 
void setup();
void loop();
void M01_get_MQ9_data();
void M02_get_MQ131_data();
void M03_GetGasConcentration_MakerIO_FINAL();
<<<<<<< HEAD
void  createEventPayLoad(float COppm, float O3ppm);
#line 34 "c:/Users/Ted/Documents/IoT/capstone-project/Capstone_Main/src/Capstone_Main.ino"
=======
void M04_get_HM3301_data();
void  light_MQ9_Pixel();
void  light_MQ131_Pixel();
void  light_HM3301_Pixel();
void MQTT_connect();
#line 41 "/Users/cyntelle/Documents/IoT/capstone/Capstone_Main/src/Capstone_Main.ino"
#define AIO_SERVER      "io.adafruit.com" 
#define AIO_SERVERPORT  1883                   // use 8883 for SSL 

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
Adafruit_MQTT_Publish CO = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Carbon Monoxide");
Adafruit_MQTT_Publish O3 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Ozone");
Adafruit_MQTT_Publish PM = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Particulate Matter");

// Constants & variables

//******* Variables for MQTT **********//
unsigned long last;
unsigned long lastMinute;
// END  Variables for MQTT **********//

//******* Variables for NeoPixel **********//
int neo_pin = D3;
int PixelON = 0xC0C0C0; // first pixel in silver
int GoodAQ = 0x00FFFF; // pixel color cyan
int MedAQ = 0xFFFF00; // pixel color yellow
int PoorAQ = 0x8B0000; // pixel color red
int ErrorAQ = 0x9932CC; // pixel color purple

#define PIXEL_PIN D3
#define PIXEL_COUNT 12
#define PIXEL_TYPE WS2812B

Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);
// END  Variables for lightNeoPixel functions **********//


//******* M01_get_MQ9_data constants and variables for function **********//
>>>>>>> 2981049f078de13ec83d582d3dcf62cf2993101d
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

//******* Variables for M04_get_HM3301_data function **********//
const int HM3301_Addr = 0x40; // Address for HM3301 I2C Dust Sensor
uint8_t HM3301_data[30];
uint16_t HM3301_adc;
uint8_t *pointer;
uint16_t HM3301_data2;
// END  Variables for M04_get_HM3301_data function **********//

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MQ9_Addr);
  Wire.beginTransmission(MQ131_Addr);
  Wire.write(0);
  Wire.endTransmission(true);

  pixel.begin();
  pixel.show();
}

void loop() 
/************************************************************************************
**************************************  MAIN LOOP  **********************************
*************************************************************************************/
{
  pixel.setPixelColor(0, PixelON);
  pixel.setBrightness(80);
  pixel.show();

  MQTT_connect();

  if ((millis()-last)>60000) 
  {
    Serial.printf("Pinging MQTT \n");
    if(! mqtt.ping()) 
    {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
    last = millis();
  }

  if(millis()-lastMinute > 30000) 
  {
    M01_get_MQ9_data();
    M02_get_MQ131_data();
    M04_get_HM3301_data();
    if(mqtt.Update()) 
    {
      CO.publish(COppm);
      O3.publish(O3ppm);
      PM.publish(HM3301_data2);
    }
    lastMinute = millis();
  } 
  light_MQ9_Pixel();
  light_MQ131_Pixel();
  light_HM3301_Pixel();
    // M03_GetGasConcentration_MakerIO_FINAL();
    // createEventPayLoad(COppm,O3ppm);
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

void M04_get_HM3301_data()
{
  // Start I2C transmission
  Wire.beginTransmission(HM3301_Addr);
  // Wire.write(0x81);
  Wire.endTransmission(false);
  // Request 2 bytes of data
  Wire.requestFrom(HM3301_Addr, 29, true);
  // Read 2 bytes of data: raw_adc msb, raw_adc lsb
  for(int i=0;i<29;i++)
  {
      HM3301_data[i] = Wire.read();
  }
  // Convert the data to 12-bits
  for(int n=0;n<29;n=n+2)
  {
    HM3301_adc = (HM3301_data[n] << 8) | HM3301_data[n+1];
    // Serial.printf("Data: %i  Particulate Matter: %i\n", n+1, HM3301_adc);
  }
  pointer = &HM3301_data[13];
  HM3301_data2 = *pointer;
  Serial.printf("Particulate Matter: %i\n", HM3301_data2);
}

void  light_MQ9_Pixel()
{
  // float bri;

  if(COppm < 480.0)
  {
    // bri = map(COppm, 10.0, 480.0, 10.0, 160.0);
    pixel.setPixelColor(2, GoodAQ);
    pixel.setBrightness(80);
    pixel.show();
  }

  if(COppm > 480.0 && COppm < 650.0)
  {
    // bri = map(COppm, 480.0, 550.0, 10.0, 160.0);
    pixel.setPixelColor(2, MedAQ);
    pixel.setBrightness(80);
    pixel.show();
  }

  if(COppm > 650.0)
  {
    // bri = map(COppm, 650.0, 750.0, 10.0, 160.0);
    pixel.setPixelColor(2, PoorAQ);
    pixel.setBrightness(80);
    pixel.show();
  }
}

void  light_MQ131_Pixel()
{
  // float bri;

  if(O3ppm < 1.0)
  {
    // bri = map(O3ppm, 0.0, 1.0, 10.0, 160.0);
    pixel.setPixelColor(4, GoodAQ);
    pixel.setBrightness(80);
    pixel.show();
  }

    if(O3ppm > 1.0 && O3ppm < 5.0)
  {
    // bri = map(O3ppm, 1.0, 5.0, 10.0, 160.0);
    pixel.setPixelColor(4, MedAQ);
    pixel.setBrightness(80);
    pixel.show();
  }

  if(O3ppm > 5.0)
  {
    // bri = map(O3ppm, 5.0, 10.0, 10.0, 160.0);
    pixel.setPixelColor(4, PoorAQ);
    pixel.setBrightness(80);
    pixel.show();
  }
}

void  light_HM3301_Pixel()
{
  // int bri;

  if(HM3301_data2 < 10)
  {
    // bri = map(HM3301_data2, 0, 10, 10, 160);
    pixel.setPixelColor(6, GoodAQ);
    pixel.setBrightness(80);
    pixel.show();
  }

    if(HM3301_data2 > 10 && HM3301_data2 < 20)
  {
    // bri = map(HM3301_data2, 10, 20, 10, 160);
    pixel.setPixelColor(6, MedAQ);
    pixel.setBrightness(80);
    pixel.show();
  }

  if(HM3301_data2 > 20)
  {
    // bri = map(HM3301_data2, 20, 1000, 10, 160);
    pixel.setPixelColor(6, PoorAQ);
    pixel.setBrightness(80);
    pixel.show();
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() 
{
  int8_t ret;
 
  if (mqtt.connected()) 
  {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) 
  { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}

// void  createEventPayLoad(float COppm, float O3ppm)
// {
//   JsonWriterStatic<256> jw;
//   {
//     JsonWriterAutoObject obj(&jw);
//     jw.insertKeyValue("Ozone", O3ppm);
//     jw.insertKeyValue("Carbon Monoxide", COppm);
//   }
//   Particle.publish("gas-emissions", jw.getBuffer(), PRIVATE);
// }