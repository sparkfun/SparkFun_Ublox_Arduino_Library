/*
  By: Elias Santistevan
  SparkFun Electronics
  Date: May, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  NEO-M8U: https://www.sparkfun.com/products/16329
  ZED-F9R: https://www.sparkfun.com/products/16344  

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a Redboard Qwiic
  If you don't have a platform with a Qwiic connection use the 
	SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output

	After calibrating the module, also known as "Fusion Mode", you can get
	data directly from the IMU. This data is integrated directly into the GNSS
	output, but is provided by the module as well. 

*/

#include <Wire.h> //Needed for I2C to GPS

#include <SparkFun_Ublox_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GPS
SFE_UBLOX_GPS myGPS;

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();

  if (myGPS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  if (myGPS.getEsfInfo()){

    Serial.print(F("Fusion Mode: "));  
    Serial.println(myGPS.imuMeas.fusionMode);  

    if (myGPS.imuMeas.fusionMode == 1){
      Serial.println(F("Fusion Mode is Initialized!"));  
		}
		else {
      Serial.println(F("Fusion Mode is either disabled or not initialized - Freezing!"));  
			Serial.println(F("Please see Example 1 description at top for more information."));
		}
  }
}

void loop()
{

  if (myGPS.getEsfIns())
  {
    Serial.print(F("X: "));
    Serial.println(myGPS.imuMeas.xAngRate);  
    Serial.print(F("Y: "));
    Serial.println(myGPS.imuMeas.yAngRate);  
    Serial.print(F("Z: "));
    Serial.println(myGPS.imuMeas.zAngRate);  
    Serial.print(F("X Acceleration: "));
    Serial.println(myGPS.imuMeas.xAccel);  
    Serial.print(F("Y Acceleration: "));
    Serial.println(myGPS.imuMeas.yAccel);  
    Serial.print(F("Z Acceleration: "));
    Serial.println(myGPS.imuMeas.zAccel);  
		// These values also have "validity checks" that can be provided by the
		// ublox library, add "Vald" to values: e.g. xAngRateVald or xAccelVald. 
  }

  delay(250);
}

