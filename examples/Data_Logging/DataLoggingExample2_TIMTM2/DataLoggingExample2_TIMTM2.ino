/*
  Configuring the GNSS to automatically send TIM TM2 reports over I2C and log them to file on SD card
  By: Paul Clark
  SparkFun Electronics
  Date: December 30th, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the u-blox GNSS to send TIM TM2 reports automatically
  and log the data to SD card in UBX format.

  This code is intended to be run on the MicroMod Data Logging Carrier Board using the Artemis Processor
  but can be adapted by changing the chip select pin and SPI allocations:
  https://www.sparkfun.com/products/16829
  https://www.sparkfun.com/products/16401

  Hardware Connections:
  Please see: https://learn.sparkfun.com/tutorials/micromod-data-logging-carrier-board-hookup-guide
  Insert the Artemis Processor into the MicroMod Data Logging Carrier Board and secure with the screw.
  Connect your GNSS breakout to the Carrier Board using a Qwiic cable.
  Connect an antenna to your GNSS board if required.
  Insert a formatted micro-SD card into the socket on the Carrier Board.
  Connect the Carrier Board to your computer using a USB-C cable.
  Ensure you have the SparkFun Apollo3 boards installed: http://boardsmanager/All#SparkFun_Apollo3
  This code has been tested using version 1.2.1 of the Apollo3 boards.
  Select "SparkFun Artemis MicroMod" as the board type.
  Press upload to upload the code onto the Artemis.
  Open the Serial Monitor at 115200 baud to see the output.

  Connecting the PPS (Pulse Per Second) breakout pin to the INT (Interrupt) pin with a jumper wire
  will cause a TIM TM2 message to be produced once per second. You can then study the timing of the
  pulse edges with nanosecond resolution!

  Data is logged in u-blox UBX format. You can replay and analyze the data using u-center:
  https://www.u-blox.com/en/product/u-center

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_Ublox_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GPS myGPS;

File myFile; //File that all GNSS data is written to

// Callback: printTIMTM2data will be called when new TIM TM2 data arrives
// See u-blox_structs.h for the full definition of UBX_TIM_TM2_data_t *packetUBXTIMTM2copy
void printTIMTM2data()
{
  Serial.println();

  Serial.print(F("newFallingEdge: ")); // 1 if a new falling edge was detected
  Serial.print(myGPS.packetUBXTIMTM2copy->flags.bits.newFallingEdge);
  
  Serial.print(F(" newRisingEdge: ")); // 1 if a new rising edge was detected
  Serial.print(myGPS.packetUBXTIMTM2copy->flags.bits.newRisingEdge);
  
  Serial.print(F(" Rising Edge Counter: ")); // Rising edge counter
  Serial.print(myGPS.packetUBXTIMTM2copy->count);

  Serial.print(F(" towMsR: ")); // Time Of Week of rising edge (ms)
  Serial.print(myGPS.packetUBXTIMTM2copy->towMsR);

  Serial.print(F(" towSubMsR: ")); // Millisecond fraction of Time Of Week of rising edge in nanoseconds
  Serial.print(myGPS.packetUBXTIMTM2copy->towSubMsR);

  Serial.print(F(" towMsF: ")); // Time Of Week of falling edge (ms)
  Serial.print(myGPS.packetUBXTIMTM2copy->towMsF);

  Serial.print(F(" towSubMsF: ")); // Millisecond fraction of Time Of Week of falling edge in nanoseconds
  Serial.println(myGPS.packetUBXTIMTM2copy->towSubMsF);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  while (Serial.available()) // Make sure the Serial buffer is empty
  {
    Serial.read();
  }

  Serial.println(F("Press any key to start logging."));

  while (!Serial.available()) // Wait for the user to press a key
  {
    ; // Do nothing
  }

  delay(100); // Wait, just in case multiple characters were sent
  
  while (Serial.available()) // Empty the Serial buffer
  {
    Serial.read();
  }

  Serial.print("Initializing SD card...");

  // See if the card is present and can be initialized:
  if (!SD.begin(CS)) //Primary SPI Chip Select is CS for Artemis MicroMod. Adjust for your processor if necessary.
  {
    Serial.println("Card failed, or not present. Freezing...");
    // don't do anything more:
    while (1);
  }
  Serial.println("SD card initialized.");

  myFile = SD.open("TIM_TM2.ubx", FILE_WRITE); // Create or open a file called "TIM_TM2.ubx" on the SD card
  if(!myFile)
  {
    Serial.println(F("Failed to create UBX data file! Freezing..."));
    while (1);
  }

  //myGPS.enableDebugging(); // Uncomment this line to enable helpful GNSS debug messages on Serial

  // TIM TM2 contains 28 bytes of data.
  // In this example, the data will arrive no faster than one message per second.
  // So, setting the file buffer size to 256 bytes should be more than adequate.
  myGPS.setFileBufferSize(256); // setFileBufferSize must be called _before_ .begin

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing..."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  
  myGPS.setNavigationFrequency(1); //Produce one navigation solution per second

  myGPS.setAutoTIMTM2callback(printTIMTM2data); // Enable automatic TIM TM2 messages with callback to printTIMTM2data
  
  myGPS.logTIMTM2(); // Enable TIM TM2 data logging

  Serial.println(F("Press any key to stop logging."));
}

void loop()
{
  myGPS.checkUblox(); // Check for the arrival of new data and process it.
  myGPS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  if (myGPS.fileBufferAvailable() >= 28) // Check to see if a new 28-byte TIM TM2 message has been stored
  {
    char myBuffer[28]; // Create our own buffer to hold the data while we write it to SD card

    uint16_t numBytesExtracted = myGPS.extractFileBufferData((uint8_t *)&myBuffer, 28); // Extract exactly 28 bytes from the UBX file buffer and put them into myBuffer

    if (numBytesExtracted != 28) // Check that we did extract exactly 28 bytes
    {
      Serial.println(F("PANIC! numBytesExtracted is not 28! Something really bad has happened! Freezing..."));
      while (1);      
    }

    uint16_t numBytesWritten = myFile.write(myBuffer, 28); // Write exactly 28 bytes from myBuffer to the ubxDataFile on the SD card
    
    if (numBytesWritten != 28) // Check that we did write exactly 28 bytes
    {
      Serial.println(F("PANIC! numBytesWritten is not 28! Something really bad has happened! Freezing..."));
      while (1);      
    }

    Serial.println(F("Wrote 28 bytes of data to ubxDataFile.ubx"));
  }

  if (Serial.available()) // Check if the user wants to stop logging
  {
    myFile.close(); // Close the data file
    Serial.println(F("Logging stopped. Freezing..."));
    while(1); // Do nothing more
  }
  
  Serial.print(".");
  delay(50);
}
