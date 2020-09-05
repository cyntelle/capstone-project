/*
 * Project: Capstone Driver Program
 * Description: The purpose of this capstone project is to establish a cost-effective detection mechanism 
 * for two major organizations: City of Albuquerque and Fuse Makerspace. This project focuses on the 
 * detection and data analysis of airborne pollutants emitted from ABQ city- and Fuse-owned equipment.
 * Components/Sensors included:
 * - Particle Argon Microcontroller
 * - Gas Sensor: CO (Carbon Monixide) MQ-9 I2C
 * - Gas Sensor: O3 (Ozone) MQ-131 I2C
 * - Gas Sensor: PM2.5 (Particulate Matter) HM3301 I2C
 * - Gas Sensor: CO2 (Carbon Dioxide) MG-811 analog
 * - OLED Display I2C
 * - Button
 * - NeoPixel Ring

 * Author: Cyntelle Renteria & Ted Fites
 * Date: 8/23/20
 * Modifications:
 * 9/5/20 CR cleaned up code and added documentation (comments) throughout main driver program
 * 9/4/20 CR integrated CO2 functions per BR for MG-811 sensor to capture CO2 emissions (working)
 * 9/4/20 CR added OLED with button to switch menus, and updated MQ131 code (working)
 * 9/3/20 CR modified code per CC notes (still need to work on neo pixel + proper documentation of code)
 * 9/2/20 CR added function M04_get_HM3301_data to capture particulate matter data + neopixel 
 *        to visualize good/medium/poor air quality + Adafruit IO dashboard to publish data to cloud
 * 8/31/20  CR Added function M02_get_MQ131_data to capture ozone gas emissions + Json function 
 *          to post CO & O3 data to Particle Console
 * 8/31/20  TF Added function M03_GetGasConcentration_MakerIO_FINAL to calculate NO2 gas concentrations
 * 8/31/20  CR modified + tested MQ9 sensor function in clean air (working)
 * 8/23/20  CR Created program + added MQ9 function (not tested)
 */

// HEADER section ********************************************************

//Header Files
#include <secrets.h>
#include <math.h>
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_SSD1306.h"
#include "neopixel.h"

/************************* Adafruit.io Setup *********************************/ 
#define AIO_SERVER      "io.adafruit.com" 
#define AIO_SERVERPORT  1883                   // use 8883 for SSL 

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
/************************* END Adafruit.io Setup *********************************/ 

/****************************** Adafruit.io Feeds ***************************************/ 
Adafruit_MQTT_Publish CO = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Carbon Monoxide");
Adafruit_MQTT_Publish O3 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Ozone");
Adafruit_MQTT_Publish PM = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Particulate Matter");
Adafruit_MQTT_Publish CO2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Carbon Dioxide");
/****************************** END Adafruit.io Feeds ***************************************/ 

// Constants & variables ********************************************************

//******* M01_get_MQ9_data constants and variables for function **********//
const int MQ9_Addr = 0x50; // Address for MQ9 I2C CO Sensor
unsigned int MQ9_data[2];
int MQ9_raw_adc = 0;
float COppm = 0.0;
// END  Variables for M01_get_MQ9_data function **********//

//******* M02_get_MQ131_data constants and variables for function **********//
const int MQ131_Addr = 0x51; // Address for MQ131 I2C Ozone Sensor
unsigned int MQ131_data[2];
int MQ131_raw_adc = 0;
float Vacd1;
float RsRo;
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

//******* Variables for M05_get_MG811_data function **********//
int mgPin = A5;
float raw_MG;
float CO2ppm;

#define DC_GAIN (8.5) //define the DC gain of amplifier
#define ZERO_POINT_VOLTAGE (0.3176) //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define REACTION_VOLTAGE (0.030) //define the voltage drop of the sensor when move the sensor from air into 1000PPM CO2
#define READ_SAMPLE_INTERVAL (50) //define how many samples you are going to take in normal operation
#define READ_SAMPLE_TIMES (5) //define the time interval(in milisecond) between each samples in 

float CO2Curve[3] = {2.602, ZERO_POINT_VOLTAGE, (REACTION_VOLTAGE/(2.602-3))};
// END  Variables for M05_get_MG811_data function **********//

//******* Variables for OLED **********//
#define OLED_RESET A0
Adafruit_SSD1306 display(OLED_RESET);
char currentDateTime[25], currentTime[9];
// END  Variables for OLED **********//

//******* Variables for Button **********//
int buttonPin = D2;
bool buttonState;
bool menuSwitch = false;
//******* END Variables for Button **********//

//******* Variables for NeoPixel **********//
const int neo_pin = D3;
const int PixelON = 0xFF00FF; // first pixel in magenta
const int GoodAQ = 0x00FFFF; // pixel color cyan
const int MedAQ = 0xFFFF00; // pixel color yellow
const int PoorAQ = 0x8B0000; // pixel color red
const int ErrorAQ = 0x9932CC; // pixel color purple

#define PIXEL_PIN D3
#define PIXEL_COUNT 12
#define PIXEL_TYPE WS2812B

Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);
// END  Variables for lightNeoPixel functions **********//

//******* Variables for MQTT **********//
unsigned long last;
unsigned long lastMinute;
// END  Variables for MQTT **********//

// END  Constants & variables ********************************************************

// END  HEADER section ********************************************************


void setup() 
{
  pinMode(buttonPin, INPUT_PULLDOWN); //pin mode for button
  pinMode(mgPin, INPUT); //pin mode for CO2 MG-811 sensor analog read
  Serial.begin(9600);

  //initialize I2C transmission
  Wire.begin();
  Wire.beginTransmission(MQ9_Addr);
  Wire.beginTransmission(MQ131_Addr);
  Wire.beginTransmission(HM3301_Addr);
  Wire.write(0);
  Wire.endTransmission(true);

  //initialize OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(1000);
  display.clearDisplay();
  display.setRotation(0);
  display.display();

  //request time sync from Particle Cloud for OLED display
  Particle.syncTime();
  waitUntil(Particle.syncTimeDone);
  Time.zone(-6); // set to mountain
  
  //removes latency from button click (enables quick switch from menu 1 to menu 2)
  attachInterrupt(buttonPin, enableButton, RISING);

  //initialize neopixels
  pixel.begin();
  pixel.show();
}

void loop() 
/************************************************************************************
**************************************  MAIN LOOP  **********************************
*************************************************************************************/
{
  //light pixel 1 to indicate program running
  pixel.setPixelColor(0, PixelON);
  pixel.setBrightness(60);
  pixel.show();
  
  //switch between two OLED menus with button
  buttonState = digitalRead(buttonPin);
  if(buttonState)
  {//switches from menu 1 to menu 2
    menuSwitch = !menuSwitch;
  }

  //display neopixel color key (menu 1) + gas concentration data (menu 2)
  display_Data_OLED();

  //connect to Adafruit.io
  MQTT_connect();
  MQTT_ping();

  //read and publish data to Adafruit.io "Gas Emissions" dashboard every 30 seconds
   if(millis()-lastMinute > 30000) 
   {//read sensors to detect gas concentration 
    light_Read_Sensors_Pixel();
    M01_get_MQ9_data();
    M02_get_MQ131_data();
    M04_get_HM3301_data();
    M05_get_MG811_data();
    if(mqtt.Update()) 
    {//publish gas concentration data to Adafruit feeds and dashboard
      CO.publish(COppm);
      O3.publish(O3ppm);
      PM.publish(HM3301_data2);
      CO2.publish(CO2ppm);
    }
    lastMinute = millis();
  } 

  //light neo pixels to visual gas concentrations (see color key on OLED)
  light_CO_MQ9_Pixel();
  light_O3_MQ131_Pixel();
  light_PM_HM3301_Pixel();
  light_CO2_MG811_Pixel();
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
  //Formula to convert raw data to PPM (parts per million)
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
  //convert to 12 bits - combine bytes
  MQ131_raw_adc = ((MQ131_data[0] & 0x0F) * 256) + MQ131_data[1];
  //voltage conversion
  Vacd1 = MQ131_raw_adc*(3.3/4096);
  // Formulas to convert raw data from combined bytes to PPM (parts per million)
  RsRo = (20.0/0.18)*(3.3-Vacd1)/Vacd1;
  O3ppm = pow(10,-log(RsRo)+1.48);
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
  Wire.endTransmission(false);
  Wire.requestFrom(HM3301_Addr, 29, true);
  //stores all data from 29 bytes into array
  for(int i=0;i<29;i++)
  {
      HM3301_data[i] = Wire.read();
  }
  //bit shifting to combine bytes
  for(int n=0;n<29;n=n+2)
  {
    HM3301_adc = (HM3301_data[n] << 8) | HM3301_data[n+1];
    // Serial.printf("Data: %i  Particulate Matter: %i\n", n+1, HM3301_adc);
  }
  //use pointer to access single byte to get PM2.5 data from sensor 
  pointer = &HM3301_data[13];
  HM3301_data2 = *pointer;
  Serial.printf("Particulate Matter: %i\n", HM3301_data2);
}

void  M05_get_MG811_data()
{
  //read sensor data and get sensor voltage for base 0 CO2 level 
  raw_MG = MGRead(mgPin);
  //convert raw data to PPM
  CO2ppm = MGGetPercentage(raw_MG,CO2Curve);
  Serial.printf("CO2 conc = %0.2f, Voltage = %0.2f \n", CO2ppm, raw_MG);
}

//read sensor data and get sensor voltage for base 0 CO2 level 
float MGRead(int mgPin) 
{
  float v = 0;
  for(int i=0; i<READ_SAMPLE_TIMES; i++)
  {
    v += analogRead(mgPin);
    delay(READ_SAMPLE_INTERVAL);
  }
  v = (v/READ_SAMPLE_TIMES)*(3.3/4096);
  return v;
}

//convert raw data from MG-811 CO2 sensor to PPM
int MGGetPercentage(float volts, float *pcurve)
{
  if((volts/DC_GAIN) >= ZERO_POINT_VOLTAGE)
  {
    return -1;
  }
  else
  {
    return pow(10, ((volts/DC_GAIN) - pcurve[1]) / pcurve[2] + pcurve[0]);
  }
}

//blink neopixel to indicate reading gas sensors
void  light_Read_Sensors_Pixel()
{
  for(int i=0; i<4; i++)
  {
    pixel.setPixelColor(0, PixelON);
    pixel.show();
    delay(120);
    pixel.clear();
    pixel.show();
    delay(120);
  }
}

void  light_CO_MQ9_Pixel()
{
  if(COppm < 480.0)
  {
    pixel.setPixelColor(3, GoodAQ);
    pixel.show();
  }

  if(COppm > 480.0 && COppm < 650.0)
  {
    pixel.setPixelColor(3, MedAQ);
    pixel.show();
  }

  if(COppm > 650.0)
  {
    pixel.setPixelColor(3, PoorAQ);
    pixel.show();
  }
}

void  light_O3_MQ131_Pixel()
{
  if(O3ppm < 1.0)
  {
    pixel.setPixelColor(5, GoodAQ);
    pixel.show();
  }

    if(O3ppm > 1.0 && O3ppm < 5.0)
  {
    pixel.setPixelColor(5, MedAQ);
    pixel.show();
  }

  if(O3ppm > 5.0)
  {
    pixel.setPixelColor(5, PoorAQ);
    pixel.show();
  }
}

void  light_PM_HM3301_Pixel()
{
  if(HM3301_data2 < 10)
  {
    pixel.setPixelColor(7, GoodAQ);
    pixel.show();
  }

    if(HM3301_data2 > 10 && HM3301_data2 < 20)
  {
    pixel.setPixelColor(7, MedAQ);
    pixel.show();
  }

  if(HM3301_data2 > 20)
  {
    pixel.setPixelColor(7, PoorAQ);
    pixel.show();
  }
}

void  light_CO2_MG811_Pixel()
{
  if(CO2ppm < 450)
  {
    pixel.setPixelColor(9, GoodAQ);
    pixel.show();
  }

    if(CO2ppm > 450 && CO2ppm < 1000)
  {
    pixel.setPixelColor(9, MedAQ);
    pixel.show();
  }

  if(CO2ppm > 1000)
  {
    pixel.setPixelColor(9, PoorAQ);
    pixel.show();
  }

}

//OLED menu 1 displays color key for neopixels, OLED menu 2 displays timestamp and gas concentrations
void  display_Data_OLED()
{
  getTime();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  displayMenu();
}

void  getTime()
{
  String DateTime, TimeOnly;

  // get current time
  DateTime = Time.timeStr();
  TimeOnly = DateTime.substring(11,19);

  // using time with formatted print statements
  DateTime.toCharArray(currentDateTime,25);
  TimeOnly.toCharArray(currentTime,9);
  // Serial.printf("Date and Time is %s\n", currentDateTime);
  // Serial.printf("Time is %s\n", currentTime);
}

//button switches menu on OLED
void  displayMenu()
{
  if(menuSwitch)
  {
    display.clearDisplay();
    display.printf("Time: %s\n", currentTime);
    display.printf("\n");
    display.printf("clockwise from pink:\n");
    display.printf("1- reading sensors..\n");
    display.printf("2- CO: %0.2fppm\n", COppm);
    display.printf("3- O3: %0.2fppm\n", O3ppm);
    display.printf("4- PM2.5: %i\n", HM3301_data2);
    display.printf("5- CO2: %0.2fppm\n", CO2ppm);
    display.display();
  }
  else if(!menuSwitch)
  {
      display.clearDisplay();
      display.printf("Air Quality Color Key\n");
      display.printf("\n");
      display.printf("blue:    GOOD\n");
      display.printf("\n");
      display.printf("yellow:  MODERATE\n");
      display.printf("\n");
      display.printf("red:     HAZARDOUS\n");
      display.display();  
  }
    // Serial.printf("button state: %i\n", buttonState);
    // Serial.printf("menu: %i\n", menuSwitch);
}

//function used in interrupt in setup to remove latency from button functionality
void  enableButton()
{
  buttonState = true;
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

//function pings to communicate to Adafruit.io
void MQTT_ping()
{
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
}