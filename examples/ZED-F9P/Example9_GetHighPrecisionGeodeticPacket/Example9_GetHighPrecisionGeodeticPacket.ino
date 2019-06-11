/*
  Get the high precision geodetic solution
  By: Nathan Seidle
  Modified by: Steven Rowland
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to inspect the accuracy of the high-precision
  positional solution.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal

  Wire.begin();

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  //myGPS.enableDebugging(Serial);

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.setNavigationFrequency(20); //Set output to 20 times a second

  byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
  Serial.print("Current update rate:");
  Serial.println(rate);
  
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}

void loop()
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    Serial.print("HP Lat: ");
    int32_t latitude = myGPS.getHighResLatitude();
    Serial.print(latitude);
    Serial.print(", HP Lon: ");

    int32_t longitude = myGPS.getHighResLongitude();
    Serial.print(longitude);
    Serial.print(", 2D Accuracy(MM): ");

    uint32_t accuracy = myGPS.getHorizontalAccuracy();
    Serial.println(accuracy);
    Serial.print(", Vertical Accuracy(MM): ");

    uint32_t verticalAccuracy = myGPS.getVerticalAccuracy();
    Serial.println(verticalAccuracy);
    Serial.print(", Elipsoid(MM): ");
    
    int32_t elipsoid = myGPS.getElipsoid();
    Serial.println(elipsoid);
    Serial.print(", Mean Sea Level(MM): ");
    
    int32_t meanSeaLevel = myGPS.getMeanSeaLevel();
    Serial.println(meanSeaLevel);
    Serial.print(", Geoid Separation(MM): ");
    
    int32_t geoidSeparation = myGPS.getGeoidSeparation();
    Serial.println(geoidSeparation);
    Serial.print(", Time of Week(Millis): ");
    
    uint32_t timeOfWeek = myGPS.getTimeOfWeek();
    Serial.println(timeOfWeek);
    Serial.print(",");
    
  }

}
