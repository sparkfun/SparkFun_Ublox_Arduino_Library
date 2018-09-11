/*
  Reading CO2, humidity and temperature from the SCD30
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14751

  This example prints the current CO2 level, relative humidity, and temperature in C.

  Hardware Connections:
  If needed, attach a Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the device into an available Qwiic port
  Open the serial monitor at 9600 baud to see the output
*/

//Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include "SparkFun_Laser_Arduino_Library.h"

Laser myLaser;

void setup()
{
  Wire.begin();
  Wire.setClock(400000); //Use 400kHz I2C communication
  
  Serial.begin(9600); //We have to start the serial port before we call begin()

  myLaser.begin();
}

void loop()
{
  byte response = myLaser.readRegister(0xAA);

  Serial.println("Command sent"); //For Uno

  delay(1000);
}
