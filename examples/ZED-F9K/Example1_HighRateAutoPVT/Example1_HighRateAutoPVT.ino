/*
  Configuring the GPS to automatically send position reports over I2C at high frequency
  The ZED-F9K can handle rates up to 30Hz
  By: Paul Clark (PaulZC)
  Date: May 11th, 2020

  Based on an earlier example:
  By: Nathan Seidle and Thorsten von Eicken
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the U-Blox GPS the send navigation reports automatically
  and retrieving the latest one via getPVT. This eliminates the blocking in getPVT while the GPS
  produces a fresh navigation solution at the expense of returning a slighly old solution.

  This can be used over serial or over I2C, this example shows the I2C use. With serial the GPS
  simply outputs the UBX_NAV_PVT packet. With I2C it queues it into its internal I2C buffer (4KB in
  size?) where it can be retrieved in the next I2C poll.

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

#include <SparkFun_Ublox_Arduino_Library.h> //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

uint16_t old_millis = 0; // Store the last millis reading
uint16_t new_millis = 0; // Store the new millis reading

void setup()
{
  Serial.begin(115200); // You may need to increase this for very high rates
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("SparkFun Ublox Example"));

  Wire.begin();

#if defined(ARDUINO_ARCH_APOLLO3)
  // For the Artemis, we need to disable the internal I2C pull-ups
  // otherwise we see many additional I2C bus gremlins.
  // If you have disabled the pull-up resistors on your GNSS board,
  // you _may_ need to comment the next line:
  Wire.setPullups(0); // Disable Artemis I2C pull-ups

  // Alternatives values are:
  //Wire.setPullups(1); // Set Artemis I2C pull-ups to 1.5K (default)
  //Wire.setPullups(6); // Set Artemis I2C pull-ups to 6K
  //Wire.setPullups(12); // Set Artemis I2C pull-ups to 12K
  //Wire.setPullups(24); // Set Artemis I2C pull-ups to 24K
#endif

  myGPS.enableDebugging(Serial, true); // Print limited debug messages only

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  // For the ZED family, we can limit the GNSS Satellite Systems used to GPS only to
  // allow faster navigation rates
  myGPS.newCfgValset8(0x10310021, 0, VAL_LAYER_RAM); // Disable GAL in RAM
  myGPS.addCfgValset8(0x10310022, 0); // Disable BDS (in RAM)
  myGPS.addCfgValset8(0x10310024, 0); // Disable QZSS (in RAM)
  myGPS.sendCfgValset8(0x10310025, 0); // Disable GLO (in RAM)

  delay(3100); // Wait for the GNSS to reconfigure
  
  myGPS.setNavigationFrequency(30); //Produce 30 (!) solutions per second (only valid on devices like the ZED-F9K)
  myGPS.setAutoPVT(true); //Tell the GPS to "send" each solution
  //myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}

void loop()
{
  // Only print GNSS millis otherwise we will probably overrun the Serial buffer
  new_millis = myGPS.getMillisecond(); // Get the latest millis (using AutoPVT)
  if (new_millis != old_millis) // Only print millis when it changes
  {
    if (new_millis < 10) Serial.print(F("0")); // Pad zeros
    if (new_millis < 100) Serial.print(F("0")); // Pad zeros
    Serial.println(new_millis);
    old_millis = new_millis; // Update old_millis
  }
  delay(5); // Let's limit how often we call checkUbloxInternal via getPVT
}
