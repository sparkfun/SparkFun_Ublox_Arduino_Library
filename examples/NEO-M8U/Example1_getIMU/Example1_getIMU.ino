
/*
  Some Description
  By: Elias Santistevan
  SparkFun Electronics
  Date: February
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.


  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  NEO-M8U: https://www.sparkfun.com/products/15136
  ZED-F9R: https://www.sparkfun.com/products/15136

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GPS

#include <SparkFun_Ublox_Arduino_Library.h> //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun Ublox Example");

  Wire.begin();

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  if (myGPS.getUdrStatus())
  {
    Serial.print("Mode: ");
    Serial.println(myGPS.imuMetric.fusionMode);  
    Serial.print("Number of Sensors: ");
    Serial.println(myGPS.imuMetric.numSens);  
  }
}

void loop()
{
  if (myGPS.getUdrStatus())
    Serial.println(myGPS.imuMetric.fusionMode);  

  if (myGPS.getInsInfo())
  {
    Serial.print("X validity: ");
    Serial.println(myGPS.imuMetric.xAngRateVald);  
    Serial.print("X: ");
    Serial.println(myGPS.imuMetric.xAngRate);  
    Serial.print("Y validity: ");
    Serial.println(myGPS.imuMetric.yAngRateVald);  
    Serial.print("Y: ");
    Serial.println(myGPS.imuMetric.yAngRate);  
    Serial.print("Z validity: ");
    Serial.println(myGPS.imuMetric.zAngRateVald);  
    Serial.print("Z: ");
    Serial.println(myGPS.imuMetric.zAngRate);  
  }
  delay(250);
}

