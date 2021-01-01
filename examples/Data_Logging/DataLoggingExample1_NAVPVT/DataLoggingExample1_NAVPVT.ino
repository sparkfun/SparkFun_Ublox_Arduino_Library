/*
  Configuring the GNSS to automatically send NAV PVT reports over I2C and log them to file on SD card
  By: Paul Clark
  SparkFun Electronics
  Date: December 30th, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the u-blox GNSS to send NAV PVT reports automatically
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

// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t *packetUBXNAVPVTcopy
void printPVTdata()
{
    Serial.println();

    Serial.print(F("Time: ")); // Print the time
    uint8_t hms = myGPS.packetUBXNAVPVTcopy->hour; // Print the hours
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = myGPS.packetUBXNAVPVTcopy->min; // Print the minutes
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = myGPS.packetUBXNAVPVTcopy->sec; // Print the seconds
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F("."));
    unsigned long millisecs = myGPS.packetUBXNAVPVTcopy->iTOW % 1000; // Print the milliseconds
    if (millisecs < 100) Serial.print(F("0")); // Print the trailing zeros correctly
    if (millisecs < 10) Serial.print(F("0"));
    Serial.print(millisecs);
    
    long latitude = myGPS.packetUBXNAVPVTcopy->lat; // Print the latitude
    Serial.print(F(" Lat: "));
    Serial.print(latitude);

    long longitude = myGPS.packetUBXNAVPVTcopy->lon; // Print the longitude
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));
    
    long altitude = myGPS.packetUBXNAVPVTcopy->hMSL; // Print the height above mean sea level
    Serial.print(F(" Height above MSL: "));
    Serial.print(altitude);
    Serial.println(F(" (mm)"));  
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

  myFile = SD.open("NAV_PVT.ubx", FILE_WRITE); // Create or open a file called "NAV_PVT.ubx" on the SD card
  if(!myFile)
  {
    Serial.println(F("Failed to create UBX data file! Freezing..."));
    while (1);
  }

  //myGPS.enableDebugging(); // Uncomment this line to enable helpful GNSS debug messages on Serial

  //myGPS.disableUBX7Fcheck(); // Advanced users only: uncomment this line to disable the "7F" check in checkUbloxI2C

  // NAV PVT contains 92 bytes of data.
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

  myGPS.setAutoPVTcallback(printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
  
  myGPS.logNAVPVT(); // Enable NAV PVT data logging

  Serial.println(F("Press any key to stop logging."));
}

void loop()
{
  myGPS.checkUblox(); // Check for the arrival of new data and process it.
  myGPS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  if (myGPS.fileBufferAvailable() >= 92) // Check to see if a new 92-byte NAV PVT message has been stored
  {
    char myBuffer[92]; // Create our own buffer to hold the data while we write it to SD card

    uint16_t numBytesExtracted = myGPS.extractFileBufferData((uint8_t *)&myBuffer, 92); // Extract exactly 92 bytes from the UBX file buffer and put them into myBuffer

    if (numBytesExtracted != 92) // Check that we did extract exactly 92 bytes
    {
      Serial.println(F("PANIC! numBytesExtracted is not 92! Something really bad has happened! Freezing..."));
      while (1);      
    }

    uint16_t numBytesWritten = myFile.write(myBuffer, 92); // Write exactly 92 bytes from myBuffer to the ubxDataFile on the SD card
    
    if (numBytesWritten != 92) // Check that we did write exactly 92 bytes
    {
      Serial.println(F("PANIC! numBytesWritten is not 92! Something really bad has happened! Freezing..."));
      while (1);      
    }

    Serial.println(F("Wrote 92 bytes of data to NAV_PVT.ubx"));
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
