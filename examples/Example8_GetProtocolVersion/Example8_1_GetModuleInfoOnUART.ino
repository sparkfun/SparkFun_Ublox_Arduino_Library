/*
  Reading the Information texts of a Ublox module
  By: mayopan
  https://github.com/mayopan
  Date: May 4th, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to get Ublox module information in text.
  It's for UART connection to GPS.
  Standard adruino board using ATmega328 like Uno, nano, mini has only one UART port.
  So this example uses SoftwareSerial library that has limitation on the serial baud rate.
  At 38400 or higher baud, the packets will be lost and you can't get data correctly.
  If so, you have to change serial rate to lower one refer to example12.
  If your board has multiple hardware UART port, use them instead of SoftwareSerial, of course! 

  Hardware Connections:
  Connect the U-Blox serial TX pin to Uno pin 2
  Connect the U-Blox serial RX pin to Uno pin 3
    Open the serial monitor at 115200 baud to see the output
*/

#include <SoftwareSerial.h>
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS

SFE_UBLOX_GPS myGPS;

void setup()
{
  int32_t baud = 9600;
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun Ublox Example");
  SoftwareSerial Serial2(2,3); //If your board has multiple UART ports, comment out this line to use hardware serial.

  Serial2.begin(baud);
  Serial.print("Connecting GPS at 9600 baud");

  while (!myGPS.begin(Serial2))// Typically, default baud rate is 9600, But if any other rate, it's displayed and stopped.
  {
    Serial.println("... is failed.");

    if (baud == 38400)
    {
      baud = 57600;
    }
    else if(baud > 115200)
    {
      baud = 9600;
    }
    else
    {
      baud *= 2;
    }
    Serial.print("Connecting GPS at ");
    Serial.print(baud);
    Serial.print(" baud");
    Serial2.begin(baud);
  }
  Serial.println(" is success!");
  if(baud > 19200)
  {
    Serial.println("Change baud lower than 38400 first! Freezing..");
    while(1)
      ;
  }

//  myGPS.setAutoPVT(false);
  Serial.print("Polling module info ");
  while (myGPS.getModuleInfo() == false)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println();
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