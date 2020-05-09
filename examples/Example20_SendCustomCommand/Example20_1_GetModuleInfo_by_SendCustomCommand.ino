/*
  Send Custom Command
  By: Paul Clark (PaulZC)
  Date: April 20th, 2020

  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how you can create and send a custom UBX packet
  using the SparkFun u-blox library.

  Previously it was possible to create and send a custom packet
  through the library but it would always appear to timeout as
  some of the internal functions referred to the internal private
  struct packetCfg.
  The most recent version of the library allows sendCommand to
  use a custom packet as if it were packetCfg and so:
  - sendCommand will return a sfe_ublox_status_e enum as if
    it had been called from within the library
  - the custom packet will be updated with data returned by the module
    (previously this was not possible from outside the library)

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

#include <SoftwareSerial.h>

//#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS

class SFE_UBLOX_GPS_ADD : public SFE_UBLOX_GPS
{
public:
  boolean getModuleInfo(uint16_t maxWait = 500); //Queries module, texts

  struct minfoStructure
  {
    char swVersion[30];
    char hwVersion[10];
    uint8_t extensionNo =0;
    char extension[10][30];
  } minfo;
};

SFE_UBLOX_GPS_ADD myGPS;

void setup()
{
  SoftwareSerial Serial2(2, 3);
  uint32_t baud = 9600;

  Serial.begin(115200); // You may need to increase this for high navigation rates!
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println("SparkFun Ublox Example");

  Serial2.begin(baud);
  Serial.print("Connecting GPS at 9600 baud");

  while (!myGPS.begin(Serial2)) // Typically, default baud rate is 9600, But if any other rate, it's displayed and stopped.
  {
    Serial.println("... is failed.");

    if (baud == 38400)
    {
      baud = 57600;
    }
    else if (baud > 115200)
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
  if (baud > 19200)
  {
    Serial.println("Change baud lower than 19200 first! Freezing..");
    while (1)
      ;
  }
  myGPS.setAutoPVT(false);
//  delay(200);
  myGPS.enableDebugging();
  Serial.print("Polling module info ");
  while (myGPS.getModuleInfo(1000) == false)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println();
  Serial.println("Module Info : ");
  Serial.print("Soft version: ");
  Serial.println(myGPS.minfo.swVersion);
  Serial.print("Hard version: ");
  Serial.println(myGPS.minfo.hwVersion);
  Serial.print("Extensions:");
  Serial.println(myGPS.minfo.extensionNo);
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

}

boolean SFE_UBLOX_GPS_ADD::getModuleInfo(uint16_t maxWait)
{
  myGPS.minfo.hwVersion[0] = NULL;
  myGPS.minfo.swVersion[0] = NULL;
  for (int i = 0; i < 10; i++)
    myGPS.minfo.extension[i][0] = NULL;

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

  // sendCommand will return:
  // SFE_UBLOX_STATUS_DATA_RECEIVED if the data we requested was read / polled successfully
  // SFE_UBLOX_STATUS_DATA_SENT     if the data we sent was writted successfully (ACK'd)
  // Other values indicate errors. Please see the sfe_ublox_status_e enum for further details.

  // Referring to the u-blox M8 Receiver Description and Protocol Specification we see that
  // the navigation rate is configured using the UBX-CFG-RATE message. So let's load our
  // custom packet with the correct information so we can read (poll / get) the current settings.

  customCfg.cls = UBX_CLASS_MON; // This is the message Class
  customCfg.id = UBX_MON_VER;    // This is the message ID
  customCfg.len = 0;             // Setting the len (length) to zero let's us poll the current settings
  customCfg.startingSpot = 0;    // Always set the startingSpot to zero (unless you really know what you are doing)

  // We also need to tell sendCommand how long it should wait for a reply
//  uint16_t maxWait = 250; // Wait for up to 250ms (Serial may need a lot longer e.g. 1100)

  if (sendCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (false); //If command send fails then bail

  uint16_t position = 0;
  for (int i = 0; i < 30; i++)
  {
    minfo.swVersion[i] = customPayload[position];
    position++;
  }
  for (int i = 0; i < 10; i++)
  {
    minfo.hwVersion[i] = customPayload[position];
    position++;
  }

  while (customCfg.len >= position + 30)
  {
    for (int i = 0; i < 30; i++)
    {
      minfo.extension[minfo.extensionNo][i] = customPayload[position];
      position++;
    }
    minfo.extensionNo++;
    if(minfo.extensionNo>9)
      break;
  }

  return (true); //We failed
}