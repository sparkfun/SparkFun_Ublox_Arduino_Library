/*
  sendCustomCommand
  By: Paul Clark (PaulZC)
  Date: April 18th, 2020

  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how you can create and send a custom UBX packet
  using the SparkFun u-blox library.

  Previously it was possible to create and send a custom packet
  through the library but it would always appear to timeout as
  some of the internal functions referred to the internal private
  struct packetCfg.
  The most recent version of the library allows sendCustomCommand to
  use a custom packet as if it were packetCfg and so:
  - sendCustomCommand will return a sfe_ublox_status_e enum as if
    sendCommand had been called from within the library
  - the custom packet will be updated with data returned by the module
    (previously this was not possible)

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
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println("SparkFun Ublox Example");

  Wire.begin();

  //myGPS.enableDebugging(); // Uncomment this line to enable debug messages

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  // Let's configure the module's dynamic platform model as if we were using setDynamicModel
  // Possible values are:
  // 0 (PORTABLE),   2 (STATIONARY), 3 (PEDESTRIAN), 4 (AUTOMOTIVE), 5 (SEA),
  // 6 (AIRBORNE1g), 7 (AIRBORNE2g), 8 (AIRBORNE4g), 9 (WRIST),     10 (BIKE)

  // Let's create our custom packet
  uint8_t customPayload[MAX_PAYLOAD_SIZE]; // This array holds the payload data bytes
  // The next line creates and initialises the packet information which wraps around the payload
  ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

  // The structure of ubxPacket is:
  // uint8_t cls           : The message Class
  // uint8_t id            : The message ID
  // uint16_t len          : Length of the payload. Does not include cls, id, or checksum bytes
  // uint16_t counter      : Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
  // uint16_t startingSpot : The counter value needed to go past before we begin recording into payload array
  // uint8_t *payload      : The payload
  // uint8_t checksumA     : Given to us by the module. Checked against the rolling calculated A/B checksums.
  // uint8_t checksumB
  // sfe_ublox_packet_validity_e valid            : Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
  // sfe_ublox_packet_validity_e classAndIDmatch  : Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID

  // sendCustomCommand will return:
  // SFE_UBLOX_STATUS_DATA_RECEIVED if the data we requested was read / polled successfully
  // SFE_UBLOX_STATUS_DATA_SENT     if the data we sent was writted successfully (ACK'd)
  // Other values indicate errors. Please see the sfe_ublox_status_e enum for further details.
  // If you see a failure you can of course simply try sending the same command again.

  // Referring to the u-blox M8 Receiver Description and Protocol Specification we see that
  // the dynamic model is configured using the UBX-CFG-NAV5 message. So let's load our
  // custom packet with the correct information so we can read (poll) the current settings.

  customCfg.cls = UBX_CLASS_CFG; // This is the message Class
  customCfg.id = UBX_CFG_NAV5; // This is the message ID
  customCfg.len = 0; // Setting the len (length) to zero let's us poll the current settings
  customCfg.startingSpot = 0; // Always set the startingSpot to zero (unless you really know what you are doing)

  // We also need to tell sendCustomCommand how long it should wait for a reply
  uint16_t maxWait = 250; // Wait for up to 250ms (Serial may need a lot longer e.g. 1100)

  // Now let's read the current navigation model settings. The results will be loaded into customCfg.
  if (myGPS.sendCustomCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
  {
    Serial.println(F("sendCustomCommand (poll) failed! Trying again..."));
    // We need to reset the packet before we try again as the values could have changed
    customCfg.cls = UBX_CLASS_CFG;
    customCfg.id = UBX_CFG_NAV5;
    customCfg.len = 0;
    customCfg.startingSpot = 0;
    if (myGPS.sendCustomCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    {
      Serial.println(F("sendCustomCommand (poll) failed again! Freezing."));
      while (1)
        ;
    }
  }

  // Referring to the message definition for UBX-CFG-NAV5 we see that we need to change
  // byte 2 to update the dynamic platform model.

  // Print the current dynamic model
  Serial.print(F("The current dynamic model is: "));
  Serial.print(customPayload[2]);

  // Let's change it
  if (customPayload[2] != 0x04) // If it is current not 4, change it to 4
  {
    Serial.println(F(". Changing it to 4."));
    customPayload[2] = 0x04;
  }
  else // If it is already 4, change it to 2
  {
    Serial.println(F(". Changing it to 2."));
    customPayload[2] = 0x02;
  }

  // We don't need to update customCfg.len as it will have been set to 36 (0x24)
  // when sendCustomCommand read the data

  // Now we write the custom packet back again to change the setting
  if (myGPS.sendCustomCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_SENT) // This time we are only expecting an ACK
  {
    Serial.println(F("sendCustomCommand (set) failed! Freezing."));
    while (1)
      ;
  }
  else
  {
    Serial.println(F("Dynamic platform model updated."));
  }

  //myGPS.saveConfigSelective(VAL_CFG_SUBSEC_NAVCONF); //Uncomment this line to save only the NAV settings to flash and BBR
}

void loop()
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer

    long latitude = myGPS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGPS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGPS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    Serial.println();
  }
}
