/*
  Reading the Information texts of a Ublox module
  By: mayopan
  https://github.com/mayopan
  Date: May 4th, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to get Ublox module information in text.
  It's for UART connection to GPS.
  If you want to connect GPS through i2c, include Wire.h


  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

//#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS

SFE_UBLOX_GPS myGPS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println("SparkFun Ublox Example");

  Serial2.begin(115200);
  delay(50);
  if (myGPS.begin(Serial2) == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("115200 baud connection is failed. Trying at 9600 baud."));
    Serial2.begin(9600);
    delay(50);
  }

    if (myGPS.begin(Serial2) == false) //Connect to the Ublox module using Wire port
    {
      Serial.println(F("Ublox GPS not detected. Please check wiring. Freezing."));
      while (1)
        ;
    }

    myGPS.getModuleInfo();
    Serial.println("Module Info : ");
    Serial.print("Soft version: ");
    Serial.println(myGPS.minfo.swVersion);
    Serial.print("Hard version: ");
    Serial.println(myGPS.minfo.hwVersion);
    Serial.println("Extensions:");
    for (int i = 0; i < myGPS.minfo.extensionNo; i++)
    {
      Serial.print("  ");
      Serial.println(myGPS.minfo.extension[i]);
    }
    Serial.println();
    Serial.println("Done!");
  }

void loop()
{
  //Do nothing
}