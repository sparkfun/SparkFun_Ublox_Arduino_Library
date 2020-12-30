/*
  Send UBX binary commands to enable RTCM sentences on Ublox ZED-F9P module
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 9th, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to query the module for RELPOS information in the NED frame.
  It assumes you already have RTCM correction data being fed to the receiver.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a RedBoard Qwiic or BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GPS myGPS;

//#define USE_SERIAL1 // Uncomment this line to push the RTCM data from Serial1 to the module via I2C

size_t numBytes = 0; // Record the number os bytes received from Serial1

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("Ublox Base station example");

#ifdef USE_SERIAL1
  // If our board supports it, we can receive the RTCM data on Serial1
  Serial1.begin(115200);
#endif

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  //myGPS.factoryDefault(); delay(5000);

#ifdef USE_SERIAL1
  Serial.print(F("Enabling UBX and RTCM input on I2C. Result: "));
  Serial.print(myGPS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_RTCM3)); //Enable UBX and RTCM input on I2C
  myGPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save the communications port settings to flash and BBR
#endif
}

void loop()
{
  if (myGPS.getRELPOSNED() == true)
  {
    Serial.print("relPosN: ");
    Serial.println(myGPS.relPosInfo.relPosN, 4);
    Serial.print("relPosE: ");
    Serial.println(myGPS.relPosInfo.relPosE, 4);
    Serial.print("relPosD: ");
    Serial.println(myGPS.relPosInfo.relPosD, 4);

    Serial.print("relPosLength: ");
    Serial.println(myGPS.relPosInfo.relPosLength);
    Serial.print("relPosHeading: ");
    Serial.println(myGPS.relPosInfo.relPosHeading);

    Serial.print("relPosHPN: ");
    Serial.println(myGPS.relPosInfo.relPosHPN);
    Serial.print("relPosHPE: ");
    Serial.println(myGPS.relPosInfo.relPosHPE);
    Serial.print("relPosHPD: ");
    Serial.println(myGPS.relPosInfo.relPosHPD);
    Serial.print("relPosHPLength: ");
    Serial.println(myGPS.relPosInfo.relPosHPLength);

    Serial.print("accN: ");
    Serial.println(myGPS.relPosInfo.accN, 4);
    Serial.print("accE: ");
    Serial.println(myGPS.relPosInfo.accE, 4);
    Serial.print("accD: ");
    Serial.println(myGPS.relPosInfo.accD, 4);

    Serial.print("gnssFixOk: ");
    if (myGPS.relPosInfo.gnssFixOk == true)
      Serial.println("x");
    else
      Serial.println("");

    Serial.print("diffSolution: ");
    if (myGPS.relPosInfo.diffSoln == true)
      Serial.println("x");
    else
      Serial.println("");

    Serial.print("relPosValid: ");
    if (myGPS.relPosInfo.relPosValid == true)
      Serial.println("x");
    else
      Serial.println("");

    Serial.print("carrier Solution Type: ");
    if (myGPS.relPosInfo.carrSoln == 0)
      Serial.println("None");
    else if (myGPS.relPosInfo.carrSoln == 1)
      Serial.println("Float");
    else if (myGPS.relPosInfo.carrSoln == 2)
      Serial.println("Fixed");

    Serial.print("isMoving: ");
    if (myGPS.relPosInfo.isMoving == true)
      Serial.println("x");
    else
      Serial.println("");

    Serial.print("refPosMiss: ");
    if (myGPS.relPosInfo.refPosMiss == true)
      Serial.println("x");
    else
      Serial.println("");

    Serial.print("refObsMiss: ");
    if (myGPS.relPosInfo.refObsMiss == true)
      Serial.println("x");
    else
      Serial.println("");
  }
  else
    Serial.println("RELPOS request failed");

  for (int i = 0; i < 500; i++)
  {
#ifdef USE_SERIAL1
    uint8_t store[256];
    while ((Serial1.available()) && (numBytes < 256)) // Check if data has been received
    {
      store[numBytes++] = Serial1.read(); // Read a byte from Serial1 and store it
    }
    if (numBytes > 0) // Check if data was received
    {
      //Serial.print("Pushing ");
      //Serial.print(numBytes);
      //Serial.println(" bytes via I2C");
      myGPS.pushRawData(((uint8_t *)&store), numBytes); // Push the RTCM data via I2C
      numBytes = 0; // Reset numBytes
    }
#endif
    delay(10);
  }
}
