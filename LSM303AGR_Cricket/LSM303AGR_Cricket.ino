/*
/* 09/23/2017 Copyright Tlera Corporation

 Created by Kris Winer
    
 The LSM303AGR is an ultra-low-power high-performance system-in-package featuring a 3D digital linear acceleration 
 sensor and a 3D digital magnetic sensor.The LSM303AGR has linear acceleration full scales of ±2g/±4g/±8g/16g and 
 a magnetic field dynamic range of ±50 gauss. The LSM303AGR includes an I2C serial bus interface that supports standard, 
 fast mode, fast mode plus, and high-speed (100 kHz, 400 kHz, 1 MHz, and 3.4 MHz) and an SPI serial standard interface.
 The system can be configured to generate an interrupt signal for free-fall, motion detection and magnetic field detection. 
 The magnetic and accelerometer blocks can be enabled or put into power-down mode separately.

 Here we are treating the accelerometer and magnetometer as separate sensors, an LSM303AGR accelerometer and an LIS2MDL magnetometer,
 event though they are embedded in the same device package. In reality, they are separate devices, so this makes sense as well as
 being convenient.

 https://www.st.com/content/ccc/resource/technical/document/datasheet/74/c4/19/54/62/c5/46/13/DM00177685.pdf/files/DM00177685.pdf/jcr:content/translations/en.DM00177685.pdf

 Library may be used freely and without limit with attribution.

*/
#include "LSM303AGR.h"
#include "LIS2MDL.h"
#include <STM32L0.h>
#include <RTC.h>
#include "I2Cdev.h"

#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 10

//LSM303AGR accel definitions
#define LSM303AGR_intPin1 A4  // interrupt1 pin definitions, data ready
#define LSM303AGR_intPin2 3   // interrupt2 pin definitions, significant motion

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      AODR_1Hz, AODR_10Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_400Hz 
*/ 
uint8_t Ascale = AFS_2G, AODR = AODR_100Hz; // assuming normal mode operation

float aRes;              // scale resolutions per LSB for the accel sensor
float accelBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel  
int16_t accelData[3], accTempData;  // Stores the 10-bit signed accel output if normal mode
float   accTemperature;             // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az;                   // variables to hold latest accel data values 

volatile bool newLSM303AGRData = false; // used for data ready interrupt handling
volatile bool newLSM303AGRactivity  = false; // used for activity interrupt handling

LSM303AGR LSM303AGR(&i2c_0); // instantiate LSM303AGR accel class


//LIS2MDL magnetometer definitions
#define LIS2MDL_intPin  A2 // interrupt for magnetometer data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are: MODR_10Hz, MOIDR_20Hz, MODR_50 Hz and MODR_100Hz
*/ 
uint8_t MODR = MODR_10Hz;

float mRes = 0.0015f;            // mag sensitivity
float magBias[3] = {0.0f, 0.0f, 0.0f}, magScale[3]  = {0.0f, 0.0f, 0.0f}; // Bias corrections for magnetometer
int16_t magData[4];              // Stores the 16-bit signed sensor output
float magTemperature;            // Stores the real internal chip temperature in degrees Celsius
float mx, my, mz;                // variables to hold latest mag data values 
uint8_t LIS2MDLstatus;

volatile bool newLIS2MDLData = false; // LIS2MDL magnetometer interrupt flag

LIS2MDL LIS2MDL(&i2c_0); // instantiate LIS2MDL class


// RTC set time using STM32L4 natve RTC class
/* Change these values to set the current initial time */
uint8_t seconds = 0;
uint8_t minutes = 15;
uint8_t hours = 11;

/* Change these values to set the current initial date */
uint8_t day = 15;
uint8_t month = 1;
uint8_t year = 18;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;

bool alarmFlag = false; // for RTC alarm interrupt



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(4000);

  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led off, active LOW

  pinMode(LIS2MDL_intPin, INPUT);    // set up interrupt pins
  pinMode(LSM303AGR_intPin1, INPUT);
  pinMode(LSM303AGR_intPin2, INPUT);
    
  I2C_BUS.begin();                                      // Set master mode, default on SDA/SCL for STM32L4
  delay(1000);
  I2C_BUS.setClock(400000);                             // I2C frequency at 400 kHz
  delay(1000);
 
  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                                      // should detect LIS2MDL at 0x1E and LSM303AGR at 0x19
  delay(1000);

  // Read the LSM303AGR Chip ID register, this is a good test of communication
  Serial.println("LSM303AGR accel/gyro...");
  byte c = LSM303AGR.getChipID();  // Read CHIP_ID register for LSM303AGR
  Serial.print("LSM303AGR "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x33, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the LIS2MDL Chip ID register, this is a good test of communication
  Serial.println("LIS2MDL mag...");
  byte d = LIS2MDL.getChipID();  // Read CHIP_ID register for LSM303AGR
  Serial.print("LIS2MDL "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x40, HEX);
  Serial.println(" ");
  delay(1000); 
  
  if(c == 0x33 && d == 0x40) // check if all I2C sensors have acknowledged
  {
   Serial.println("LSM303AGR and LIS2MDL are online..."); Serial.println(" ");
   
   digitalWrite(myLed, LOW); // turn on led during device initialization
 
   aRes = LSM303AGR.getAres(Ascale); // get sensor resolution, only need to do this once
   LSM303AGR.selfTest();
   LSM303AGR.reset();
   LSM303AGR.init(Ascale, AODR);

   LSM303AGR.offsetBias(accelBias);
   Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
   delay(1000); 

   mRes = 0.0015f;  // fixed sensitivity and full scale (+/- 49.152 Gauss); 
   LIS2MDL.selfTest();
   LIS2MDL.reset(); // software reset LIS2MDL to default registers  
   LIS2MDL.init(MODR);

   LIS2MDL.offsetBias(magBias, magScale);
   Serial.println("mag biases (mG)"); Serial.println(1000.0f * magBias[0]); Serial.println(1000.0f * magBias[1]); Serial.println(1000.0f * magBias[2]); 
   Serial.println("mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
   delay(2000); // add delay to see results before serial spew of data

   digitalWrite(myLed, HIGH); // turn off led when configuration successfully completed
   
  }
  else 
  {
  if(c != 0x33) Serial.println(" LSM303AGR not functioning!"); // otherwise there is a problem somewhere
  if(d != 0x40) Serial.println(" LIS2MDL not functioning!");    
  while(1){};
  }

 // Set the time
  SetDefaultRTC();
  
  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  attachInterrupt(LSM303AGR_intPin1, myinthandler1, RISING);  // define data ready interrupt for intPin1 output of LSM303AGR
  attachInterrupt(LSM303AGR_intPin2, myinthandler2, RISING);  // define no-motion activity interrupt for intPin2 output of LSM303AGR
  attachInterrupt(LIS2MDL_intPin ,   myinthandler3, RISING);  // define data ready interrupt for intPin  output of LIS2MDL

  LSM303AGR.readAccData(accelData); // INT1 cleared on any read
  LIS2MDL.readData(magData);  // read data register to clear interrupt before main loop
}
/* End of setup */


void loop() {

  if(newLSM303AGRactivity)
  {
    newLSM303AGRactivity = false;
    Serial.print("no motion activity detected!");
  }


   // If intPin goes high, either all data registers have new data
   if(newLSM303AGRData == true) {   // on interrupt, read data
      newLSM303AGRData = false;     // reset newData flag

     LSM303AGR.readAccData(accelData); // INT1 cleared on any read
   
   // Now we'll calculate the accleration value into actual g's
     ax = (float)accelData[0]*aRes;// - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)accelData[1]*aRes;// - accelBias[1];   
     az = (float)accelData[2]*aRes;// - accelBias[2];  
   }


    // If intPin goes high, either all data registers have new data
    if(newLIS2MDLData == true) {   // On interrupt, read data
      newLIS2MDLData = false;     // reset newData flag

     LIS2MDLstatus = LIS2MDL.status();
     
     if(LIS2MDLstatus & 0x08) // if all axes have new data ready
     {
      LIS2MDL.readData(magData);  
   
   // Now we'll calculate the accleration value into actual G's
     mx = (float)magData[0]*mRes - magBias[0];  // get actual G value 
     my = (float)magData[1]*mRes - magBias[1];   
     mz = (float)magData[2]*mRes - magBias[2]; 
     mx *= magScale[0];
     my *= magScale[1];
     mz *= magScale[2];  
     }
   }// end sensor interrupt handling


    if(alarmFlag) { // update RTC output (serial display) whenever the RTC alarm condition is achieved and the MPU9250 is awake
       alarmFlag = false;

    // Read RTC
    if(SerialDebug){
    Serial.println("RTC:");
    Day = RTC.getDay();
    Month = RTC.getMonth();
    Year = RTC.getYear();
    Seconds = RTC.getSeconds();
    Minutes = RTC.getMinutes();
    Hours   = RTC.getHours();     
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");
    }
    
    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("mx = "); Serial.print((int)1000*mx);  
    Serial.print(" my = "); Serial.print((int)1000*my); 
    Serial.print(" mz = "); Serial.print((int)1000*mz); Serial.println(" mG");
    }

    accTempData = LSM303AGR.readAccTempData();
    accTemperature = ((float) accTempData) + 25.0f; // Accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Accel temperature is ");  Serial.print(accTemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    digitalWrite(myLed, LOW); delay(10); digitalWrite(myLed, HIGH);   // blink led at the end of RTC alarm handling
    } // end of RTC alarm handling

    STM32L0.stop(); // wait in low power mode for an interrupt
}

/*  End of main loop */


void myinthandler1()
{
  newLSM303AGRData = true;
  STM32L0.wakeup();
}

void myinthandler2()
{
  newLSM303AGRactivity = true;
  STM32L0.wakeup();
}


void myinthandler3()
{
  newLIS2MDLData = true;
  STM32L0.wakeup();
}

void alarmMatch()
{
  alarmFlag = true; // STM32L0 automatically wakes up on RTC alarm
}

void SetDefaultRTC()             // Function sets the RTC to the FW build date-time...
{
  char Build_mo[3];
  String build_mo = "";

  Build_mo[0] = build_date[0];   // Convert month string to integer
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];
  for(uint8_t i=0; i<3; i++)
  {
    build_mo += Build_mo[i];
  }
  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;                                                                                       // Default to January if something goes wrong...
  }
  if(build_date[4] != 32)                                                                            // If the first digit of the date string is not a space
  {
    day   = (build_date[4] - 48)*10 + build_date[5]  - 48;                                           // Convert ASCII strings to integers; ASCII "0" = 48
  } else
  {
    day   = build_date[5]  - 48;
  }
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1]  - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4]  - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7]  - 48;
  RTC.setDay(day);                                                                                   // Set the date/time
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}

