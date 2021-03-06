/*
  Send UBX binary commands to enable RTCM sentences on Ublox NEO-M8P-2 module
  By: Nathan Seidle
  SparkFun Electronics
  Date: September 7th, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example does all steps to configure and enable a NEO-M8P-2 as a base station:
    Begin Survey-In
    Once we've achieved 2m accuracy and 300s have passed, survey is complete
    Enable four RTCM messages
    Begin outputting RTCM bytes

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  Plug a SerLCD onto the Qwiic bus
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Watch the output on the LCD or open the serial monitor at 115200 baud to see the output
*/

#define STAT_LED 13

#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GPS myGPS;

#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
SerLCD lcd; // Initialize the library with default I2C address 0x72

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("Ublox GPS I2C Test");

  Wire.begin();

  pinMode(STAT_LED, OUTPUT);
  digitalWrite(STAT_LED, LOW);

  lcd.begin(Wire); //Set up the LCD for Serial communication at 9600bps
  lcd.setBacklight(0x4B0082); //indigo, a kind of dark purplish blue
  lcd.clear();
  lcd.print(F("LCD Ready"));

  myGPS.begin(Wire);
  if (myGPS.isConnected() == false)
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    lcd.setCursor(0, 1);
    lcd.print(F("No GPS detected"));
    while (1);
  }

  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  lcd.setCursor(0, 1);
  lcd.print("GPS Detected");

  //Check if Survey is in Progress before initiating one
  boolean response;
  response = myGPS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (request can take a long time)
  if (response == false)
  {
    Serial.println(F("Failed to get Survey In status"));
    while (1); //Freeze
  }

  if (myGPS.svin.active == true)
  {
    Serial.print(F("Survey already in progress."));
    lcd.setCursor(0, 2);
    lcd.print(F("Survey already going"));
  }
  else
  {
    //Start survey
    response = myGPS.enableSurveyMode(300, 2.000); //Enable Survey in, 300 seconds, 2.0m
    if (response == false)
    {
      Serial.println(F("Survey start failed"));
      lcd.setCursor(0, 3);
      lcd.print(F("Survey start failed"));
      while (1);
    }
    Serial.println(F("Survey started. This will run until 300s has passed and less than 2m accuracy is achieved."));
  }

  while (Serial.available()) Serial.read(); //Clear buffer

  lcd.clear();
  lcd.print(F("Survey in progress"));

  //Begin waiting for survey to complete
  while (myGPS.svin.valid == false)
  {
    if (Serial.available())
    {
      byte incoming = Serial.read();
      if (incoming == 'x')
      {
        //Stop survey mode
        response = myGPS.disableSurveyMode(); //Disable survey
        Serial.println(F("Survey stopped"));
        break;
      }
    }

    response = myGPS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (req can take a long time)
    if (response == true)
    {
      Serial.print(F("Press x to end survey - "));
      Serial.print(F("Time elapsed: "));
      Serial.print((String)myGPS.svin.observationTime);

      lcd.setCursor(0, 1);
      lcd.print(F("Elapsed: "));
      lcd.print((String)myGPS.svin.observationTime);

      Serial.print(F(" Accuracy: "));
      Serial.print((String)myGPS.svin.meanAccuracy);
      Serial.println();

      lcd.setCursor(0, 2);
      lcd.print(F("Accuracy: "));
      lcd.print((String)myGPS.svin.meanAccuracy);
    }
    else
    {
      Serial.println(F("SVIN request failed"));
    }

    delay(1000);
  }
  Serial.println(F("Survey valid!"));

  response = true;
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1005, UBX_RTCM_I2C_PORT, 1); //Enable message 1005 to output through I2C port, message every second
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1077, UBX_RTCM_I2C_PORT, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1087, UBX_RTCM_I2C_PORT, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1230, UBX_RTCM_I2C_PORT, 10); //Enable message every 10 seconds

  if (response == true)
  {
    Serial.println(F("RTCM messages enabled"));
  }
  else
  {
    Serial.println(F("RTCM failed to enable. Are you sure you have an NEO-M8P?"));
    while (1); //Freeze
  }

  Serial.println(F("Base survey complete! RTCM now broadcasting."));
  lcd.clear();
  lcd.print(F("Transmitting RTCM"));
}

void loop()
{
  myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

  //Do anything you want. Call checkUblox() every second. NEO-M8P-2 has TX buffer of 4k bytes.
  
  delay(250); //Don't pound too hard on the I2C bus
}

//This function gets called from the SparkFun Ublox Arduino Library.
//As each RTCM byte comes in you can specify what to do with it
//Useful for passing the RTCM correction data to a radio, Ntrip broadcaster, etc.
void SFE_UBLOX_GPS::processRTCM(uint8_t incoming)
{
  //Let's just pretty-print the HEX values for now
  if (myGPS.rtcmFrameCounter % 16 == 0) Serial.println();
  Serial.print(" ");
  if (incoming < 0x10) Serial.print("0");
  Serial.print(incoming, HEX);
}
