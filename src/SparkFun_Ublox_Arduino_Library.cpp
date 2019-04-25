/*
  This is a library written for the Ublox NEO-M8P-2
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14586

  Written by Nathan Seidle @ SparkFun Electronics, September 6th, 2018

  The NEO-M8P-2 is a powerful GPS receiver capable of calculating correction data
  to achieve 2cm accuracy.

  This library handles the configuration of 'survey-in', RTCM messages, and to output
  the RTCM messages to the user's selected stream

  https://github.com/sparkfun/SparkFun_RTK_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_Ublox_Arduino_Library.h"

SFE_UBLOX_GPS::SFE_UBLOX_GPS(void)
{
  // Constructor
}

//Initialize the Serial port
boolean SFE_UBLOX_GPS::begin(TwoWire &wirePort, uint8_t deviceAddress)
{
  commType = COMM_TYPE_I2C;
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  //We expect caller to begin their I2C port, with the speed of their choice external to the library
  //But if they forget, we start the hardware here.

  //We're moving away from the practice of starting Wire hardware in a library. This is to avoid cross platform issues.
  //ie, there are some platforms that don't handle multiple starts to the wire hardware. Also, every time you start the wire
  //hardware the clock speed reverts back to 100kHz regardless of previous Wire.setClocks().
  //_i2cPort->begin();

  _gpsI2Caddress = deviceAddress; //Store the I2C address from user

  return (isConnected());
}

//Initialize the Serial port
boolean SFE_UBLOX_GPS::begin(Stream &serialPort)
{
  commType = COMM_TYPE_SERIAL;
  _serialPort = &serialPort; //Grab which port the user wants us to use

  return (isConnected());
}

void SFE_UBLOX_GPS::factoryReset()
{
  // Copy default settings to permanent
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_CFG;
  packetCfg.len = 13;
  packetCfg.startingSpot = 0;
  for (uint8_t i = 0; i < 4; i++)
  {
    payloadCfg[0 + i] = 0xff; // clear mask: copy default config to permanent config
    payloadCfg[4 + i] = 0x00; // save mask: don't save current to permanent
    payloadCfg[8 + i] = 0x00; // load mask: don't copy permanent config to current
  }
  payloadCfg[12] = 0xff;     // all forms of permanent memory
  sendCommand(packetCfg, 0); // don't expect ACK
  hardReset();               // cause factory default config to actually be loaded and used cleanly
}

void SFE_UBLOX_GPS::hardReset()
{
  // Issue hard reset
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RST;
  packetCfg.len = 4;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = 0xff;      // cold start
  payloadCfg[1] = 0xff;      // cold start
  payloadCfg[2] = 0;         // 0=HW reset
  payloadCfg[3] = 0;         // reserved
  sendCommand(packetCfg, 0); // don't expect ACK
}

//Changes the serial baud rate of the Ublox module, can't return success/fail 'cause ACK from modem
//is lost due to baud rate change
void SFE_UBLOX_GPS::setSerialRate(uint32_t baudrate, uint8_t uartPort, uint16_t maxWait)
{
  //Get the current config values for the UART port
  getPortSettings(uartPort, maxWait); //This will load the payloadCfg array with current port settings
#ifdef DEBUG
  debug.print("Current baud rate: ");
  debug.println(((uint32_t)payloadCfg[10] << 16) | ((uint32_t)payloadCfg[9] << 8) | payloadCfg[8]);
#endif

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 20;
  packetCfg.startingSpot = 0;

  //payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[8] = baudrate;
  payloadCfg[9] = baudrate >> 8;
  payloadCfg[10] = baudrate >> 16;
  payloadCfg[11] = baudrate >> 24;
#ifdef DEBUG
  debug.print("New baud rate:");
  debug.println(((uint32_t)payloadCfg[10] << 16) | ((uint32_t)payloadCfg[9] << 8) | payloadCfg[8]);
#endif

  sendCommand(packetCfg);
}

//Changes the I2C address that the Ublox module responds to
//0x42 is the default but can be changed with this command
boolean SFE_UBLOX_GPS::setI2CAddress(uint8_t deviceAddress, uint16_t maxWait)
{
  //Get the current config values for the I2C port
  getPortSettings(COM_PORT_I2C); //This will load the payloadCfg array with current port settings

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 20;
  packetCfg.startingSpot = 0;

  //payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[4] = deviceAddress << 1; //DDC mode LSB

  if (sendCommand(packetCfg, maxWait) == true)
  {
    //Success! Now change our internal global.
    _gpsI2Caddress = deviceAddress; //Store the I2C address from user
    return (true);
  }
  return (false);
}

//Want to see the NMEA messages on the Serial port? Here's how
void SFE_UBLOX_GPS::setNMEAOutputPort(Stream &nmeaOutputPort)
{
  _nmeaOutputPort = &nmeaOutputPort; //Store the port from user
}

//Called regularly to check for available bytes on the user' specified port
boolean SFE_UBLOX_GPS::checkUblox()
{
  if (commType == COMM_TYPE_I2C)
    checkUbloxI2C();
  else if (commType == COMM_TYPE_SERIAL)
    checkUbloxSerial();
  return false;
}

//Polls I2C for data, passing any new bytes to process()
boolean SFE_UBLOX_GPS::checkUbloxI2C()
{
  if (millis() - lastCheck >= I2C_POLLING_WAIT_MS)
  {
    //Get the number of bytes available from the module
    uint16_t bytesAvailable = 0;
    _i2cPort->beginTransmission(_gpsI2Caddress);
    _i2cPort->write(0xFD);                     //0xFD (MSB) and 0xFE (LSB) are the registers that contain number of bytes available
    if (_i2cPort->endTransmission(false) != 0) //Send a restart command. Do not release bus.
      return (false);                          //Sensor did not ACK

    _i2cPort->requestFrom((uint8_t)_gpsI2Caddress, (uint8_t)2);
    if (_i2cPort->available())
    {
      uint8_t msb = _i2cPort->read();
      uint8_t lsb = _i2cPort->read();
      bytesAvailable = (uint16_t)msb << 8 | lsb;
    }

    if (bytesAvailable == 0)
    {
#ifdef DEBUG
      debug.println("No bytes available");
#endif
      lastCheck = millis(); //Put off checking to avoid I2C bus traffic
      return true;
    }

    while (bytesAvailable)
    {
      _i2cPort->beginTransmission(_gpsI2Caddress);
      _i2cPort->write(0xFF);                     //0xFF is the register to read general NMEA data from
      if (_i2cPort->endTransmission(false) != 0) //Send a restart command. Do not release bus.
        return (false);                          //Sensor did not ACK

      //Limit to 32 bytes or whatever the buffer limit is for given platform
      uint16_t bytesToRead = bytesAvailable;
      if (bytesToRead > I2C_BUFFER_LENGTH)
        bytesToRead = I2C_BUFFER_LENGTH;

      _i2cPort->requestFrom((uint8_t)_gpsI2Caddress, (uint8_t)bytesToRead);
      if (_i2cPort->available())
      {
        for (uint16_t x = 0; x < bytesToRead; x++)
        {
          process(_i2cPort->read()); //Grab the actual character and process it
        }
      }
      else
        return (false); //Sensor did not respond

      bytesAvailable -= bytesToRead;
    }
  }

  return (true);

} //end checkUbloxI2C()

//Checks Serial for data, passing any new bytes to process()
boolean SFE_UBLOX_GPS::checkUbloxSerial()
{
  while (_serialPort->available())
  {
    process(_serialPort->read());
  }
  return (true);

} //end checkUbloxSerial()

//Processes NMEA and UBX binary sentences one byte at a time
//Take a given byte and file it into the proper array
void SFE_UBLOX_GPS::process(uint8_t incoming)
{

#ifdef DEBUG
  //if (currentSentence == NONE && incoming == 0xB5) //UBX binary frames start with 0xB5, aka μ
  //	debug.println(); //Show new packet start

  //debug.print(" ");
  //debug.print(incoming, HEX);
#endif

  if (currentSentence == NONE || currentSentence == NMEA)
  {
    if (incoming == 0xB5) //UBX binary frames start with 0xB5, aka μ
    {
      //This is the start of a binary sentence. Reset flags.
      //We still don't know the response class
      ubxFrameCounter = 0;

      rollingChecksumA = 0; //Reset our rolling checksums
      rollingChecksumB = 0;

      currentSentence = UBX;
    }
    else if (incoming == '$')
    {
      currentSentence = NMEA;
    }
    else if (incoming == 0xD3) //RTCM frames start with 0xD3
    {
      rtcmFrameCounter = 0;
      currentSentence = RTCM;
    }
    else
    {
      //This character is unknown or we missed the previous start of a sentence
    }
  }

  //Depending on the sentence, pass the character to the individual processor
  if (currentSentence == UBX)
  {
    //Decide what type of response this is
    if (ubxFrameCounter == 0 && incoming != 0xB5)      //ISO 'μ'
      currentSentence = NONE;                          //Something went wrong. Reset.
    else if (ubxFrameCounter == 1 && incoming != 0x62) //ASCII 'b'
      currentSentence = NONE;                          //Something went wrong. Reset.
    else if (ubxFrameCounter == 2)                     //Class
    {
      packetAck.counter = 0;
      packetAck.valid = false;
      packetCfg.counter = 0;
      packetCfg.valid = false;

      //We can now identify the type of response
      if (incoming == UBX_CLASS_ACK)
        ubxFrameClass = CLASS_ACK;
      else
        ubxFrameClass = CLASS_NOT_AN_ACK;
    }

    ubxFrameCounter++;

    //Depending on this frame's class, pass different structs and payload arrays
    if (ubxFrameClass == CLASS_ACK)
      processUBX(incoming, &packetAck);
    else if (ubxFrameClass == CLASS_NOT_AN_ACK)
      processUBX(incoming, &packetCfg);
  }
  else if (currentSentence == NMEA)
  {
    processNMEA(incoming); //Process each NMEA character
  }
  else if (currentSentence == RTCM)
  {
    processRTCM(incoming); //Deal with RTCM bytes
  }
}

//This is the default or generic NMEA processor. We're only going to pipe the data to serial port so we can see it.
//User could overwrite this function to pipe characters to nmea.process(c) of tinyGPS or MicroNMEA
//Or user could pipe each character to a buffer, radio, etc.
void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  //If user has assigned an output port then pipe the characters there
  if (_nmeaOutputPort != NULL)
    _nmeaOutputPort->write(incoming); //Echo this byte to the serial port
}

//We need to be able to identify an RTCM packet and then the length
//so that we know when the RTCM message is completely received and we then start
//listening for other sentences (like NMEA or UBX)
//RTCM packet structure is very odd. I never found RTCM STANDARD 10403.2 but
//http://d1.amobbs.com/bbs_upload782111/files_39/ourdev_635123CK0HJT.pdf is good
//https://dspace.cvut.cz/bitstream/handle/10467/65205/F3-BP-2016-Shkalikava-Anastasiya-Prenos%20polohove%20informace%20prostrednictvim%20datove%20site.pdf?sequence=-1
//Lead me to: https://forum.u-blox.com/index.php/4348/how-to-read-rtcm-messages-from-neo-m8p
//RTCM 3.2 bytes look like this:
//Byte 0: Always 0xD3
//Byte 1: 6-bits of zero
//Byte 2: 10-bits of length of this packet including the first two-ish header bytes, + 6.
//byte 3 + 4 bits: Msg type 12 bits
//Example: D3 00 7C 43 F0 ... / 0x7C = 124+6 = 130 bytes in this packet, 0x43F = Msg type 1087
void SFE_UBLOX_GPS::processRTCMframe(uint8_t incoming)
{
  if (rtcmFrameCounter == 1)
  {
    rtcmLen = (incoming & 0x03) << 8; //Get the last two bits of this byte. Bits 8&9 of 10-bit length
  }
  else if (rtcmFrameCounter == 2)
  {
    rtcmLen |= incoming; //Bits 0-7 of packet length
    rtcmLen += 6;        //There are 6 additional bytes of what we presume is header, msgType, CRC, and stuff
  }
  /*else if (rtcmFrameCounter == 3)
  {
    rtcmMsgType = incoming << 4; //Message Type, MS 4 bits
  }
  else if (rtcmFrameCounter == 4)
  {
    rtcmMsgType |= (incoming >> 4); //Message Type, bits 0-7
  }*/

  rtcmFrameCounter++;

  processRTCM(incoming); //Here is where we expose this byte to the user

  if (rtcmFrameCounter == rtcmLen)
  {
    //We're done!
    currentSentence = NONE; //Reset and start looking for next sentence type
  }
}

//This function is called for each byte of an RTCM frame
//Ths user can overwrite this function and process the RTCM frame as they please
//Bytes can be piped to Serial or other interface. The consumer could be a radio or the internet (Ntrip broadcaster)
void SFE_UBLOX_GPS::processRTCM(uint8_t incoming)
{
  //Radio.sendReliable((String)incoming); //An example of passing this byte to a radio

  //Serial.write(incoming); //An example of passing this byte out the serial port

  //Debug printing
  //  Serial.print(" ");
  //  if(incoming < 0x10) Serial.print("0");
  //  if(incoming < 0x10) Serial.print("0");
  //  Serial.print(incoming, HEX);
  //  if(rtcmFrameCounter % 16 == 0) Serial.println();
}

//Given a character, file it away into the uxb packet structure
//Set valid = true once sentence is completely received and passes CRC
//The payload portion of the packet can be 100s of bytes but the max array
//size is roughly 64 bytes. startingSpot can be set so we only record
//a subset of bytes within a larger packet.
void SFE_UBLOX_GPS::processUBX(uint8_t incoming, ubxPacket *incomingUBX)
{
  //Add all incoming bytes to the rolling checksum
  //Stop at len+4 as this is the checksum bytes to that should not be added to the rolling checksum
  if (incomingUBX->counter < incomingUBX->len + 4)
    addToChecksum(incoming);

  if (incomingUBX->counter == 0)
  {
    incomingUBX->cls = incoming;
  }
  else if (incomingUBX->counter == 1)
  {
    incomingUBX->id = incoming;
  }
  else if (incomingUBX->counter == 2) //Len LSB
  {
    incomingUBX->len = incoming;
  }
  else if (incomingUBX->counter == 3) //Len MSB
  {
    incomingUBX->len |= incoming << 8;
  }
  else if (incomingUBX->counter == incomingUBX->len + 4) //ChecksumA
  {
    incomingUBX->checksumA = incoming;
  }
  else if (incomingUBX->counter == incomingUBX->len + 5) //ChecksumB
  {
    incomingUBX->checksumB = incoming;

    currentSentence = NONE; //We're done! Reset the sentence to being looking for a new start char

    //Validate this sentence
    if (incomingUBX->checksumA == rollingChecksumA && incomingUBX->checksumB == rollingChecksumB)
    {
#ifdef DEBUG
      debug.print("Received: ");
      printPacket(incomingUBX);
#endif
      incomingUBX->valid = true;
      processUBXpacket(incomingUBX); //We've got a valid packet, now do something with it
    }
#ifdef DEBUG
    else
      debug.println("Checksum failed. Response too big?");
#endif
  }
  else //Load this byte into the payload array
  {
    //If a UBX_NAV_PVT packet comes in asynchronously, we need to fudge the startingSpot
    uint16_t startingSpot = incomingUBX->startingSpot;
    if (incomingUBX->cls == UBX_CLASS_NAV && incomingUBX->id == UBX_NAV_PVT)
      startingSpot = 20;
    //Begin recording if counter goes past startingSpot
    if ((incomingUBX->counter - 4) >= startingSpot)
    {
      //Check to see if we have room for this byte
      if (((incomingUBX->counter - 4) - startingSpot) < MAX_PAYLOAD_SIZE)         //If counter = 208, starting spot = 200, we're good to record.
        incomingUBX->payload[incomingUBX->counter - 4 - startingSpot] = incoming; //Store this byte into payload array
    }
  }

  incomingUBX->counter++;
}

//Once a packet has been received and validated, identify this packet's class/id and update internal flags
void SFE_UBLOX_GPS::processUBXpacket(ubxPacket *msg)
{
  switch (msg->cls)
  {
  case UBX_CLASS_ACK:
    //We don't want to store ACK packets, just set commandAck flag
    if (msg->id == UBX_ACK_ACK && msg->payload[0] == packetCfg.cls && msg->payload[1] == packetCfg.id)
    {
      //The ack we just received matched the CLS/ID of last packetCfg sent
#ifdef DEBUG
      debug.println("Command sent/ack'd successfully");
#endif
      commandAck = true;
    }
    break;

  case UBX_CLASS_NAV:
    if (msg->id == UBX_NAV_PVT && msg->len == 92)
    {
      //Parse various byte fields into global vars
      constexpr int startingSpot = 20; //fixed value used in processUBX
      fixType = extractByte(20 - startingSpot);
      carrierSolution = extractByte(21 - startingSpot) >> 6; //Get 6th&7th bits of this byte
      SIV = extractByte(23 - startingSpot);
      longitude = extractLong(24 - startingSpot);
      latitude = extractLong(28 - startingSpot);
      altitude = extractLong(32 - startingSpot);
      altitudeMSL = extractLong(36 - startingSpot);
      groundSpeed = extractLong(60 - startingSpot);
      headingOfMotion = extractLong(64 - startingSpot);
      pDOP = extractLong(76 - startingSpot);

      //Mark all datums as fresh (not read before)
      moduleQueried.all = true;
      moduleQueried.longitude = true;
      moduleQueried.latitude = true;
      moduleQueried.altitude = true;
      moduleQueried.altitudeMSL = true;
      moduleQueried.SIV = true;
      moduleQueried.fixType = true;
      moduleQueried.carrierSolution = true;
      moduleQueried.groundSpeed = true;
      moduleQueried.headingOfMotion = true;
      moduleQueried.pDOP = true;
    }
    break;
  }
}

//Given a packet and payload, send everything including CRC bytes via I2C port
boolean SFE_UBLOX_GPS::sendCommand(ubxPacket outgoingUBX, uint16_t maxWait)
{
  commandAck = false;         //We're about to send a command. Begin waiting for ack.
  calcChecksum(&outgoingUBX); //Sets checksum A and B bytes of the packet

#ifdef DEBUG
  debug.print("Sending: ");
  printPacket(&outgoingUBX);
#endif

  if (commType == COMM_TYPE_I2C)
  {
    if (!sendI2cCommand(outgoingUBX, maxWait))
      return false;
  }
  else if (commType == COMM_TYPE_SERIAL)
  {
    sendSerialCommand(outgoingUBX);
  }

  if (maxWait > 0)
  {
    //Give waitForResponse the cls/id to check for
    return waitForResponse(outgoingUBX.cls, outgoingUBX.id, maxWait); //Wait for Ack response
  }
  return true;
}

boolean SFE_UBLOX_GPS::sendI2cCommand(ubxPacket outgoingUBX, uint16_t maxWait)
{
  //Point at 0xFF data register
  _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress); //There is no register to write to, we just begin writing data bytes
  _i2cPort->write(0xFF);
  if (_i2cPort->endTransmission() != 0) //Don't release bus
    return (false);                     //Sensor did not ACK

  //Write header bytes
  _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress); //There is no register to write to, we just begin writing data bytes
  _i2cPort->write(UBX_SYNCH_1);                         //μ - oh ublox, you're funny. I will call you micro-blox from now on.
  _i2cPort->write(UBX_SYNCH_2);                         //b
  _i2cPort->write(outgoingUBX.cls);
  _i2cPort->write(outgoingUBX.id);
  _i2cPort->write(outgoingUBX.len & 0xFF);   //LSB
  _i2cPort->write(outgoingUBX.len >> 8);     //MSB
  if (_i2cPort->endTransmission(false) != 0) //Do not release bus
    return (false);                          //Sensor did not ACK

  //Write payload. Limit the sends into 32 byte chunks
  //This code based on ublox: https://forum.u-blox.com/index.php/20528/how-to-use-i2c-to-get-the-nmea-frames
  uint16_t bytesToSend = outgoingUBX.len;

  //"The number of data bytes must be at least 2 to properly distinguish
  //from the write access to set the address counter in random read accesses."
  uint16_t startSpot = 0;
  while (bytesToSend > 1)
  {
    uint8_t len = bytesToSend;
    if (len > I2C_BUFFER_LENGTH)
      len = I2C_BUFFER_LENGTH;

    _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
    //_i2cPort->write(outgoingUBX.payload, len); //Write a portion of the payload to the bus

    for (uint16_t x = 0; x < len; x++)
      _i2cPort->write(outgoingUBX.payload[startSpot + x]); //Write a portion of the payload to the bus

    if (_i2cPort->endTransmission(false) != 0) //Don't release bus
      return (false);                          //Sensor did not ACK

    //*outgoingUBX.payload += len; //Move the pointer forward
    startSpot += len; //Move the pointer forward
    bytesToSend -= len;
  }

  //Write checksum
  _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
  if (bytesToSend == 1)
    _i2cPort->write(outgoingUBX.payload, 1);
  _i2cPort->write(outgoingUBX.checksumA);
  _i2cPort->write(outgoingUBX.checksumB);

  //All done transmitting bytes. Release bus.
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

//Given a packet and payload, send everything including CRC bytesA via Serial port
void SFE_UBLOX_GPS::sendSerialCommand(ubxPacket outgoingUBX)
{
  //Write header bytes
  _serialPort->write(UBX_SYNCH_1); //μ - oh ublox, you're funny. I will call you micro-blox from now on.
  _serialPort->write(UBX_SYNCH_2); //b
  _serialPort->write(outgoingUBX.cls);
  _serialPort->write(outgoingUBX.id);
  _serialPort->write(outgoingUBX.len & 0xFF); //LSB
  _serialPort->write(outgoingUBX.len >> 8);   //MSB

  //Write payload.
  for (int i = 0; i < outgoingUBX.len; i++)
  {
    _serialPort->write(outgoingUBX.payload[i]);
  }

  //Write checksum
  _serialPort->write(outgoingUBX.checksumA);
  _serialPort->write(outgoingUBX.checksumB);
}

//Returns true if I2C device ack's
boolean SFE_UBLOX_GPS::isConnected()
{
  if (commType == COMM_TYPE_I2C)
  {
    _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
    return _i2cPort->endTransmission() == 0;
  }
  else if (commType == COMM_TYPE_SERIAL)
  {
    // Query navigation rate to see whether we get a meaningful response
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_RATE;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    return sendCommand(packetCfg);
  }
  return false;
}

//Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
//This is called before we send a command message
void SFE_UBLOX_GPS::calcChecksum(ubxPacket *msg)
{
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (uint16_t i = 0; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}

//Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
//This is used when receiving messages from module
void SFE_UBLOX_GPS::addToChecksum(uint8_t incoming)
{
  rollingChecksumA += incoming;
  rollingChecksumB += rollingChecksumA;
}

//Pretty prints the current ubxPacket
void SFE_UBLOX_GPS::printPacket(ubxPacket *packet)
{
#ifdef DEBUG
  debug.print("CLS:");
  debug.print(packet->cls, HEX);

  debug.print(" ID:");
  debug.print(packet->id, HEX);

  //debug.print(" Len: 0x");
  //debug.print(packet->len, HEX);

  debug.print(" Payload:");

  for (int x = 0; x < packet->len; x++)
  {
    debug.print(" ");
    debug.print(packet->payload[x], HEX);
  }
  debug.println();
#endif
}

//=-=-=-=-=-=-=-= Specific commands =-=-=-=-=-=-=-==-=-=-=-=-=-=-=
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Poll the module until and ack is received
boolean SFE_UBLOX_GPS::waitForResponse(uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  commandAck = false;      //Reset flag
  packetCfg.valid = false; //This will go true when we receive a response to the packet we sent

  unsigned long startTime = millis();
  while (millis() - startTime < maxTime)
  {
    checkUblox(); //See if new data is available. Process bytes as they come in.

    if (commandAck == true)
      return (true); //If the packet we just sent was a CFG packet then we'll get an ACK
    if (packetCfg.valid == true)
    {
      //Did we receive a config packet that matches the cls/id we requested?
      if (packetCfg.cls == requestedClass && packetCfg.id == requestedID)
      {
#ifdef DEBUG
        debug.println(F("CLS/ID match!"));
#endif
        return (true); //If the packet we just sent was a NAV packet then we'll just get data back
      }
#ifdef DEBUG
      else
      {
        debug.print(F("Packet didn't match CLS/ID"));
        printPacket(&packetCfg);
      }
#endif
    }

    delay(1);
  }

#ifdef DEBUG
  debug.println(F("waitForResponse timeout"));
#endif

  return (false);
}

//Save current configuration to flash and BBR (battery backed RAM)
//This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
boolean SFE_UBLOX_GPS::saveConfiguration(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_CFG;
  packetCfg.len = 12;
  packetCfg.startingSpot = 0;

  //Clear packet payload
  for (uint8_t x = 0; x < packetCfg.len; x++)
    packetCfg.payload[x] = 0;

  packetCfg.payload[4] = 0xFF; //Set any bit in the saveMask field to save current config to Flash and BBR

  if (sendCommand(packetCfg, maxWait) == false)
    return (false); //If command send fails then bail

  return (true);
}

//Reset module to factory defaults
//This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
boolean SFE_UBLOX_GPS::factoryDefault(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_CFG;
  packetCfg.len = 12;
  packetCfg.startingSpot = 0;

  //Clear packet payload
  for (uint8_t x = 0; x < packetCfg.len; x++)
    packetCfg.payload[x] = 0;

  packetCfg.payload[0] = 0xFF; //Set any bit in the clearMask field to clear saved config
  packetCfg.payload[8] = 0xFF; //Set any bit in the loadMask field to discard current config and rebuild from lower non-volatile memory layers

  if (sendCommand(packetCfg, maxWait) == false)
    return (false); //If command send fails then bail

  return (true);
}

//Given a key, return its value
//This is how the new Ublox modules are communicating, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GPS::getVal(uint16_t group, uint16_t id, uint8_t size, uint8_t layer, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALGET;
  packetCfg.len = 4 + 4 * 1; //Only one key at a time
  packetCfg.startingSpot = 0;

  //Clear packet payload
  for (uint8_t x = 0; x < packetCfg.len; x++)
    packetCfg.payload[x] = 0;

  payloadCfg[0] = 0; //Message Version - set to 0
  payloadCfg[1] = layer;

  //Create key
  uint32_t key = 0;
  key |= (uint32_t)id;
  key |= (uint32_t)group << 16;
  key |= (uint32_t)size << 28;

#ifdef DEBUG
  debug.print("key: 0x");
  debug.print(key, HEX);
  debug.println();
#endif

  //Load key into outgoing payload
  payloadCfg[4] = key >> 8 * 0; //Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;

  //Send VALGET command with this key
  if (sendCommand(packetCfg, maxWait) == false)
    return (false); //If command send fails then bail

  //Pull the requested value from the response
  return (extractByte(8)); //Look for our response value at location 4+4*N
}

//Get the current TimeMode3 settings - these contain survey in statuses
boolean SFE_UBLOX_GPS::getSurveyMode(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_TMODE3;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  return (sendCommand(packetCfg, maxWait));
}

//Control Survey-In for NEO-M8P
boolean SFE_UBLOX_GPS::setSurveyMode(uint8_t mode, uint16_t observationTime, float requiredAccuracy, uint16_t maxWait)
{
  if (getSurveyMode() == false) //Ask module for the current TimeMode3 settings. Loads into payloadCfg.
    return (false);

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_TMODE3;
  packetCfg.len = 40;
  packetCfg.startingSpot = 0;

  //Clear packet payload
  for (uint8_t x = 0; x < packetCfg.len; x++)
    packetCfg.payload[x] = 0;

  //payloadCfg should be loaded with poll response. Now modify only the bits we care about
  payloadCfg[2] = mode; //Set mode. Survey-In and Disabled are most common.

  payloadCfg[24] = observationTime & 0xFF; //svinMinDur in seconds
  payloadCfg[25] = observationTime >> 8;   //svinMinDur in seconds

  uint32_t svinAccLimit = requiredAccuracy * 10000; //Convert m to 0.1mm
  payloadCfg[28] = svinAccLimit & 0xFF;             //svinAccLimit in 0.1mm increments
  payloadCfg[29] = svinAccLimit >> 8;
  payloadCfg[30] = svinAccLimit >> 16;

  return (sendCommand(packetCfg, maxWait)); //Wait for ack
}

//Begin Survey-In for NEO-M8P
boolean SFE_UBLOX_GPS::enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint16_t maxWait)
{
  return (setSurveyMode(SVIN_MODE_ENABLE, observationTime, requiredAccuracy, maxWait));
}

//Stop Survey-In for NEO-M8P
boolean SFE_UBLOX_GPS::disableSurveyMode(uint16_t maxWait)
{
  return (setSurveyMode(SVIN_MODE_DISABLE, 0, 0, maxWait));
}

//Reads survey in status and sets the global variables
//for status, position valid, observation time, and mean 3D StdDev
//Returns true if commands was successful
boolean SFE_UBLOX_GPS::getSurveyStatus(uint16_t maxWait)
{
  //Reset variables
  svin.active = false;
  svin.valid = false;
  svin.observationTime = 0;
  svin.meanAccuracy = 0;

  packetCfg.cls = UBX_CLASS_NAV;
  packetCfg.id = UBX_NAV_SVIN;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(packetCfg, maxWait) == false)
    return (false); //If command send fails then bail

  //We got a response, now parse the bits into the svin structure
  svin.observationTime = extractLong(8);

  uint32_t tempFloat = extractLong(28);
  svin.meanAccuracy = tempFloat / 10000.0; //Convert 0.1mm to m

  svin.valid = payloadCfg[36];
  svin.active = payloadCfg[37]; //1 if survey in progress, 0 otherwise

  return (true);
}

//Given a message number turns on a message ID for output over a given portID (UART, I2C, SPI, USB, etc)
//To disable a message, set secondsBetween messages to 0
//Note: This function will return false if the message is already enabled
//For base station RTK output we need to enable various sentences

//NEO-M8P has four:
//1005 = 0xF5 0x05 - Stationary RTK reference ARP
//1077 = 0xF5 0x4D - GPS MSM7
//1087 = 0xF5 0x57 - GLONASS MSM7
//1230 = 0xF5 0xE6 - GLONASS code-phase biases, set to once every 10 seconds

//ZED-F9P has six:
//1005, 1074, 1084, 1094, 1124, 1230

//Much of this configuration is not documented and instead discerned from u-center binary console
boolean SFE_UBLOX_GPS::enableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint8_t secondsBetweenMessages, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 8;
  packetCfg.startingSpot = 0;

  //Clear packet payload
  for (uint8_t x = 0; x < packetCfg.len; x++)
    packetCfg.payload[x] = 0;

  packetCfg.payload[0] = UBX_RTCM_MSB;                    //MSB, always 0xF5. Interesting, these are not little endian
  packetCfg.payload[1] = messageNumber;                   //LSB
  packetCfg.payload[2 + portID] = secondsBetweenMessages; //Byte 2 is I2C, byte 3 is UART1, etc.

  return (sendCommand(packetCfg, maxWait));
}

//Disable a given message on a given port by setting secondsBetweenMessages to zero
boolean SFE_UBLOX_GPS::disableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint16_t maxWait)
{
  return (enableRTCMmessage(messageNumber, portID, 0, maxWait));
}

//Loads the payloadCfg array with the current protocol bits located the UBX-CFG-PRT register for a given port
boolean SFE_UBLOX_GPS::getPortSettings(uint8_t portID, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 1;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = portID;

  return (sendCommand(packetCfg, maxWait));
}

//Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
//Port 0=I2c, 1=UART1, 2=UART2, 3=USB, 4=SPI
//Bit:0 = UBX, :1=NMEA, :5=RTCM3
boolean SFE_UBLOX_GPS::setPortOutput(uint8_t portID, uint8_t outStreamSettings, uint16_t maxWait)
{
  //Get the current config values for this port ID
  if (getPortSettings(portID) == false)
    return (false); //Something went wrong. Bail.

  //Yes, this is the depreciated way to do it but it's still supported on v27 so it
  //covers both ZED-F9P (v27) and SAM-M8Q (v18)

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 20;
  packetCfg.startingSpot = 0;

  //payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[14] = outStreamSettings; //OutProtocolMask LSB - Set outStream bits

  return (sendCommand(packetCfg, maxWait));
}

//Configure a given port to input UBX, NMEA, RTCM3 or a combination thereof
//Port 0=I2c, 1=UART1, 2=UART2, 3=USB, 4=SPI
//Bit:0 = UBX, :1=NMEA, :5=RTCM3
boolean SFE_UBLOX_GPS::setPortInput(uint8_t portID, uint8_t inStreamSettings, uint16_t maxWait)
{
  //Get the current config values for this port ID
  //This will load the payloadCfg array with current port settings
  if (getPortSettings(portID) == false)
    return (false); //Something went wrong. Bail.

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 20;
  packetCfg.startingSpot = 0;

  //payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[12] = inStreamSettings; //InProtocolMask LSB - Set inStream bits

  return (sendCommand(packetCfg, maxWait));
}

//Configure a port to output UBX, NMEA, RTCM3 or a combination thereof
boolean SFE_UBLOX_GPS::setI2COutput(uint8_t comSettings, uint16_t maxWait)
{
  return (setPortOutput(COM_PORT_I2C, comSettings, maxWait));
}
boolean SFE_UBLOX_GPS::setUART1Output(uint8_t comSettings, uint16_t maxWait)
{
  return (setPortOutput(COM_PORT_UART1, comSettings, maxWait));
}
boolean SFE_UBLOX_GPS::setUART2Output(uint8_t comSettings, uint16_t maxWait)
{
  return (setPortOutput(COM_PORT_UART2, comSettings, maxWait));
}
boolean SFE_UBLOX_GPS::setUSBOutput(uint8_t comSettings, uint16_t maxWait)
{
  return (setPortOutput(COM_PORT_USB, comSettings, maxWait));
}
boolean SFE_UBLOX_GPS::setSPIOutput(uint8_t comSettings, uint16_t maxWait)
{
  return (setPortOutput(COM_PORT_SPI, comSettings, maxWait));
}

//Set the rate at which the module will give us an updated navigation solution
//Expects a number that is the updates per second. For example 1 = 1Hz, 2 = 2Hz, etc.
//Max is 40Hz(?!)
boolean SFE_UBLOX_GPS::setNavigationFrequency(uint8_t navFreq, uint16_t maxWait)
{
  //if(updateRate > 40) updateRate = 40; //Not needed: module will correct out of bounds values

  //Query the module for the latest lat/long
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RATE;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(packetCfg, maxWait) == false) //This will load the payloadCfg array with current settings of the given register
    return (false);                             //If command send fails then bail

  uint16_t measurementRate = 1000 / navFreq;

  //payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[0] = measurementRate & 0xFF; //measRate LSB
  payloadCfg[1] = measurementRate >> 8;   //measRate MSB

  return (sendCommand(packetCfg, maxWait));
}

//Get the rate at which the module is outputting nav solutions
uint8_t SFE_UBLOX_GPS::getNavigationFrequency(uint16_t maxWait)
{
  //Query the module for the latest lat/long
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RATE;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(packetCfg, maxWait) == false) //This will load the payloadCfg array with current settings of the given register
    return (0);                                 //If command send fails then bail

  uint16_t measurementRate = 0;

  //payloadCfg is now loaded with current bytes. Get what we need
  measurementRate = extractInt(0); //Pull from payloadCfg at measRate LSB

  measurementRate = 1000 / measurementRate; //This may return an int when it's a float, but I'd rather not return 4 bytes
  return (measurementRate);
}

//Enable or disable automatic navigation message generation by the GPS. This changes the way getPVT
//works.
boolean SFE_UBLOX_GPS::setAutoPVT(boolean enable, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_PVT;
  payloadCfg[2] = enable ? 1 : 0; // rate relative to navigation freq.

  bool ok = sendCommand(packetCfg, maxWait);
  if (ok)
    autoPVT = enable;
  moduleQueried.all = false;
  return ok;
}

//Given a spot in the payload array, extract four bytes and build a long
uint32_t SFE_UBLOX_GPS::extractLong(uint8_t spotToStart)
{
  uint32_t val = 0;
  val |= (int32_t)payloadCfg[spotToStart + 0] << 8 * 0;
  val |= (int32_t)payloadCfg[spotToStart + 1] << 8 * 1;
  val |= (int32_t)payloadCfg[spotToStart + 2] << 8 * 2;
  val |= (int32_t)payloadCfg[spotToStart + 3] << 8 * 3;
  return (val);
}

//Given a spot in the payload array, extract two bytes and build an int
uint16_t SFE_UBLOX_GPS::extractInt(uint8_t spotToStart)
{
  uint16_t val = 0;
  val |= (int16_t)payloadCfg[spotToStart + 0] << 8 * 0;
  val |= (int16_t)payloadCfg[spotToStart + 1] << 8 * 1;
  return (val);
}

//Given a spot, extract byte the payload
uint8_t SFE_UBLOX_GPS::extractByte(uint8_t spotToStart)
{
  return (payloadCfg[spotToStart]);
}

//Get the latest Position/Velocity/Time solution and fill all global variables
boolean SFE_UBLOX_GPS::getPVT(uint16_t maxWait)
{
  if (autoPVT)
  {
    //The GPS is automatically reporting, we just check whether we got unread data
    checkUblox();
    return moduleQueried.all;
  }
  else
  {
    //The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_PVT;
    packetCfg.len = 0;
    //packetCfg.startingSpot = 20; //Begin listening at spot 20 so we can record up to 20+MAX_PAYLOAD_SIZE = 84 bytes Note:now hard-coded in processUBX

    //The data is parsed as part of processing the response
    return sendCommand(packetCfg, maxWait);
    return (false); //If command send fails then bail
  }
}

//Get the current 3D high precision positional accuracy - a fun thing to watch
//Returns a long representing the 3D accuracy in millimeters
uint32_t SFE_UBLOX_GPS::getPositionAccuracy(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_NAV;
  packetCfg.id = UBX_NAV_HPPOSECEF;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(packetCfg, maxWait) == false)
    return (0); //If command send fails then bail

  uint32_t tempAccuracy = extractLong(24); //We got a response, now extract a long beginning at a given position

  if ((tempAccuracy % 10) >= 5)
    tempAccuracy += 5; //Round fraction of mm up to next mm if .5 or above
  tempAccuracy /= 10;  //Convert 0.1mm units to mm

  return (tempAccuracy);
}
//Get the current latitude in degrees
//Returns a long representing the number of degrees *10^-7
int32_t SFE_UBLOX_GPS::getLatitude(uint16_t maxWait)
{
  if (moduleQueried.latitude == false)
    getPVT();
  moduleQueried.latitude = false; //Since we are about to give this to user, mark this data as stale

  return (latitude);
}

//Get the current longitude in degrees
//Returns a long representing the number of degrees *10^-7
int32_t SFE_UBLOX_GPS::getLongitude(uint16_t maxWait)
{
  if (moduleQueried.longitude == false)
    getPVT();
  moduleQueried.longitude = false; //Since we are about to give this to user, mark this data as stale
  moduleQueried.all = false;

  return (longitude);
}

//Get the current altitude in mm according to ellipsoid model
int32_t SFE_UBLOX_GPS::getAltitude(uint16_t maxWait)
{
  if (moduleQueried.altitude == false)
    getPVT();
  moduleQueried.altitude = false; //Since we are about to give this to user, mark this data as stale
  moduleQueried.all = false;

  return (altitude);
}

//Get the current altitude in mm according to mean sea level
//Ellipsoid model: https://www.esri.com/news/arcuser/0703/geoid1of3.html
//Difference between Ellipsoid Model and Mean Sea Level: https://eos-gnss.com/elevation-for-beginners/
int32_t SFE_UBLOX_GPS::getAltitudeMSL(uint16_t maxWait)
{
  if (moduleQueried.altitudeMSL == false)
    getPVT();
  moduleQueried.altitudeMSL = false; //Since we are about to give this to user, mark this data as stale
  moduleQueried.all = false;

  return (altitudeMSL);
}

//Get the number of satellites used in fix
uint8_t SFE_UBLOX_GPS::getSIV(uint16_t maxWait)
{
  if (moduleQueried.SIV == false)
    getPVT();
  moduleQueried.SIV = false; //Since we are about to give this to user, mark this data as stale
  moduleQueried.all = false;

  return (SIV);
}

//Get the current fix type
//0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
uint8_t SFE_UBLOX_GPS::getFixType(uint16_t maxWait)
{
  if (moduleQueried.fixType == false)
    getPVT();
  moduleQueried.fixType = false; //Since we are about to give this to user, mark this data as stale
  moduleQueried.all = false;

  return (fixType);
}

//Get the carrier phase range solution status
//Useful when querying module to see if it has high-precision RTK fix
//0=No solution, 1=Float solution, 2=Fixed solution
uint8_t SFE_UBLOX_GPS::getCarrierSolutionType(uint16_t maxWait)
{
  if (moduleQueried.carrierSolution == false)
    getPVT();
  moduleQueried.carrierSolution = false; //Since we are about to give this to user, mark this data as stale
  moduleQueried.all = false;

  return (carrierSolution);
}

//Get the ground speed in mm/s
int32_t SFE_UBLOX_GPS::getGroundSpeed(uint16_t maxWait)
{
  if (moduleQueried.groundSpeed == false)
    getPVT();
  moduleQueried.groundSpeed = false; //Since we are about to give this to user, mark this data as stale
  moduleQueried.all = false;

  return (groundSpeed);
}

//Get the heading of motion (as opposed to heading of car) in degrees * 10^-5
int32_t SFE_UBLOX_GPS::getHeading(uint16_t maxWait)
{
  if (moduleQueried.headingOfMotion == false)
    getPVT();
  moduleQueried.headingOfMotion = false; //Since we are about to give this to user, mark this data as stale
  moduleQueried.all = false;

  return (headingOfMotion);
}

//Get the positional dillution of precision * 10^-2
uint16_t SFE_UBLOX_GPS::getPDOP(uint16_t maxWait)
{
  if (moduleQueried.pDOP == false)
    getPVT();
  moduleQueried.pDOP = false; //Since we are about to give this to user, mark this data as stale
  moduleQueried.all = false;

  return (pDOP);
}

//Get the current protocol version of the Ublox module we're communicating with
//This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
uint8_t SFE_UBLOX_GPS::getProtocolVersionHigh(uint16_t maxWait)
{
  if (moduleQueried.versionNumber == false)
    getProtocolVersion();
  moduleQueried.versionNumber = false;
  return (versionHigh);
}

//Get the current protocol version of the Ublox module we're communicating with
//This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
uint8_t SFE_UBLOX_GPS::getProtocolVersionLow(uint16_t maxWait)
{
  if (moduleQueried.versionNumber == false)
    getProtocolVersion();
  moduleQueried.versionNumber = false;
  return (versionLow);
}

//Get the current protocol version of the Ublox module we're communicating with
//This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
boolean SFE_UBLOX_GPS::getProtocolVersion(uint16_t maxWait)
{
  //Send packet with only CLS and ID, length of zero. This will cause the module to respond with the contents of that CLS/ID.
  packetCfg.cls = UBX_CLASS_MON;
  packetCfg.id = UBX_MON_VER;

  //We will send the command repeatedly, increasing the startingSpot as we go
  //Then we look at each extension field of 30 bytes
  for (uint8_t extensionNumber = 0; extensionNumber < 10; extensionNumber++)
  {
    packetCfg.len = 0;
    packetCfg.startingSpot = 40 + (30 * extensionNumber);

    if (sendCommand(packetCfg, maxWait) == false)
      return (false); //If command send fails then bail

#ifdef DEBUG
    debug.print("Extension ");
    debug.print(extensionNumber);
    debug.print(": ");
    for (int location = 0; location < MAX_PAYLOAD_SIZE; location++)
    {
      if (payloadCfg[location] == '\0')
        break;
      debug.write(payloadCfg[location]);
    }
    debug.println();
#endif

    //Now we need to find "PROTVER=18.00" in the incoming byte stream
    if (payloadCfg[0] == 'P' && payloadCfg[6] == 'R')
    {
      versionHigh = (payloadCfg[8] - '0') * 10 + (payloadCfg[9] - '0');  //Convert '18' to 18
      versionLow = (payloadCfg[11] - '0') * 10 + (payloadCfg[12] - '0'); //Convert '00' to 00
      return (versionLow);
    }
  }

  moduleQueried.versionNumber = true; //Mark this data as new

  return (true);
}

//Relative Positioning Information in NED frame
//Returns true if commands was successful
boolean SFE_UBLOX_GPS::getRELPOSNED(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_NAV;
  packetCfg.id = UBX_NAV_RELPOSNED;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(packetCfg, maxWait) == false)
    return (false); //If command send fails then bail

  //We got a response, now parse the bits

  uint16_t refStationID = extractInt(2);
  Serial.print("refStationID: ");
  Serial.println(refStationID);

  int32_t tempRelPos;

  tempRelPos = extractLong(8);
  relPosInfo.relPosN = tempRelPos / 100.0; //Convert cm to m

  tempRelPos = extractLong(12);
  relPosInfo.relPosE = tempRelPos / 100.0; //Convert cm to m

  tempRelPos = extractLong(16);
  relPosInfo.relPosD = tempRelPos / 100.0; //Convert cm to m

  uint32_t tempAcc;

  tempAcc = extractLong(24);
  relPosInfo.accN = tempAcc / 10000.0; //Convert 0.1 mm to m

  tempAcc = extractLong(28);
  relPosInfo.accE = tempAcc / 10000.0; //Convert 0.1 mm to m

  tempAcc = extractLong(32);
  relPosInfo.accD = tempAcc / 10000.0; //Convert 0.1 mm to m

  uint8_t flags = payloadCfg[36];

  relPosInfo.gnssFixOk = flags & (1 << 0);
  relPosInfo.diffSoln = flags & (1 << 1);
  relPosInfo.relPosValid = flags & (1 << 2);
  relPosInfo.carrSoln = (flags & (0b11 << 3)) >> 3;
  relPosInfo.isMoving = flags & (1 << 5);
  relPosInfo.refPosMiss = flags & (1 << 6);
  relPosInfo.refObsMiss = flags & (1 << 7);

  return (true);
}