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
void SFE_UBLOX_GPS::begin(TwoWire &wirePort)
{
	commType = COMM_TYPE_I2C;
	_i2cPort = &wirePort; //Grab which port the user wants us to use

	//We expect caller to begin their I2C port, with the speed of their choice external to the library
	//But if they forget, we start the hardware here.
	_i2cPort->begin();

	setI2CReadAddress(0x42); //By default, use 0x42
}

//Sets the internal global variable that is the I2C address we read from
//This does not change the I2C address of the module
//0x42 is the default but can be changed via software command
void SFE_UBLOX_GPS::setI2CReadAddress(uint8_t deviceAddress)
{
	_gpsI2Caddress = deviceAddress; //Store the I2C address from user
}

//Want to see the NMEA messages on the Serial port? Here's how
void SFE_UBLOX_GPS::setNMEAOutputPort(Stream &nmeaOutputPort)
{
	_nmeaOutputPort = &nmeaOutputPort; //Store the port from user
}

//Called regularly to check for available bytes on the user' specified port
boolean SFE_UBLOX_GPS::checkUblox()
{
	if(commType == COMM_TYPE_I2C)
		checkUbloxI2C();
	else if(commType == COMM_TYPE_SERIAL)
		checkUbloxSerial();
}

//Polls I2C for data, passing any new bytes to process()
//Times out after given amount of time
boolean SFE_UBLOX_GPS::checkUbloxI2C()
{
  if (millis() - lastCheck >= I2C_POLLING_WAIT_MS)
  {
    //Get the number of bytes available from the module
    uint16_t bytesAvailable = 0;
    _i2cPort->beginTransmission(_gpsI2Caddress);
    _i2cPort->write(0xFD); //0xFD (MSB) and 0xFE (LSB) are the registers that contain number of bytes available
    if (_i2cPort->endTransmission(false) != 0) //Send a restart command. Do not release bus.
      return (false); //Sensor did not ACK

    _i2cPort->requestFrom((uint8_t)_gpsI2Caddress, (uint8_t)2);
    if (_i2cPort->available())
    {
      uint8_t msb = _i2cPort->read();
      uint8_t lsb = _i2cPort->read();
      bytesAvailable = (uint16_t)msb << 8 | lsb;
    }

    if (bytesAvailable == 0)
      lastCheck = millis(); //Put off checking to avoid I2C bus traffic

    while (bytesAvailable)
    {
      _i2cPort->beginTransmission(_gpsI2Caddress);
      _i2cPort->write(0xFF); //0xFF is the register to read general NMEA data from
      if (_i2cPort->endTransmission(false) != 0) //Send a restart command. Do not release bus.
        return (false); //Sensor did not ACK

      //Limit to 32 bytes or whatever the buffer limit is for given platform
      uint16_t bytesToRead = bytesAvailable;
      if (bytesToRead > I2C_BUFFER_LENGTH) bytesToRead = I2C_BUFFER_LENGTH;

      _i2cPort->requestFrom((uint8_t)_gpsI2Caddress, (uint8_t)bytesToRead);
      if (_i2cPort->available())
      {
        for (uint16_t x = 0 ; x < bytesToRead ; x++)
        {
          process(_i2cPort->read()); //Grab the actual character and process it
        }
      }
      else
        return (false); //Sensor did not respond

      bytesAvailable -= bytesToRead;
    }
  } //end timed read

  return (true);

} //end checkUbloxI2C()

//Polls Serial for data, passing any new bytes to process()
//Times out after given amount of time
boolean SFE_UBLOX_GPS::checkUbloxSerial()
{
	//Todo
	return (true);

} //end checkUbloxSerial()

//Processes NMEA and UBX binary sentences one byte at a time
//Take a given byte and file it into the proper array
void SFE_UBLOX_GPS::process(uint8_t incoming)
{
  if (currentSentence == NONE || currentSentence == NMEA)
  {
    if (incoming == 0xB5) //UBX binary frames start with 0xB5
    {
      //This is the start of a binary sentence. Reset flags.
      //We still don't know the response class
      ubxFrameCounter = 0;
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
    if (ubxFrameCounter == 0 && incoming != 0xB5)
      currentSentence = NONE; //Something went wrong. Reset.
    else if (ubxFrameCounter == 1 && incoming != 0x62)
      currentSentence = NONE; //Something went wrong. Reset.
    else if (ubxFrameCounter == 2) //Class
    {
      //We can now identify the type of response
      if (incoming == UBX_CLASS_ACK)
      {
        ubxFrameClass = CLASS_ACK;
        packetAck.counter = 0;
        packetAck.valid = false;
      }
      else
      {
        ubxFrameClass = CLASS_NOT_AN_ACK;
        packetCfg.counter = 0;
        packetCfg.valid = false;
      }
    }

    ubxFrameCounter++;

    //Depending on this frame's class, pass different structs and payload arrays
    if (ubxFrameClass == CLASS_ACK)
      processUBX(incoming, &packetAck);
    else if (ubxFrameClass == CLASS_NOT_AN_ACK)
      processUBX(incoming, &packetCfg);
    else
      ; //Serial.println("No frame class set");
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
	if(_nmeaOutputPort != NULL)
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
    rtcmLen += 6; //There are 6 additional bytes of what we presume is header, msgType, CRC, and stuff
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
void SFE_UBLOX_GPS::processUBX(uint8_t incoming, ubxPacket *incomingUBX)
{
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
    //Validate this sentence

    uint8_t tempA = incomingUBX->checksumA;
    uint8_t tempB = incoming;

    calcChecksum(incomingUBX); //Calc checksum across this message. Results stored in message.

    currentSentence = NONE; //We're done! Reset the sentence to being looking for a new start char

    if (incomingUBX->checksumA == tempA && incomingUBX->checksumB == tempB)
    {
      //Serial.print("Frame cleared: ");
      //printFrame(incomingUBX);

      incomingUBX->valid = true;
      processUBXpacket(incomingUBX); //We've got a valid packet, now do something with it
    }
  }
  else //Load this byte into the appropriate array
  {
    incomingUBX->payload[incomingUBX->counter - 4] = incoming; //Store this byte into payload array
  }

  incomingUBX->counter++;
}

//Once a packet has been received and validated, identify this packet's class/id and update internal flags
void SFE_UBLOX_GPS::processUBXpacket(ubxPacket *msg)
{
  if (msg->cls == UBX_CLASS_ACK)
  {
    //We don't want to store ACK packets, just set commandAck flag
    if (msg->id == UBX_ACK_ACK)
    {
      if (msg->payload[0] == packetCfg.cls && msg->payload[1] == packetCfg.id)
      {
        //The ack we just received matched the CLS/ID of last packetCfg sent
        //Serial.println("Command sent/ack'd successfully");
        commandAck = true;
      }
    }
    //else
    //{
      //Serial.println("Command send fail");
      //module.ackReceived = false;
    //}
  }
}

//Given a packet and payload, send everything including CRC bytes
boolean SFE_UBLOX_GPS::sendCommand(ubxPacket outgoingUBX, uint16_t maxWait)
{
  commandAck = false; //We're about to send a command. Begin waiting for ack.

  calcChecksum(&outgoingUBX); //Sets checksum A and B bytes of the packet

  //Point at 0xFF data register
  _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress); //There is no register to write to, we just begin writing data bytes
  _i2cPort->write(0xFF);
  if (_i2cPort->endTransmission() != 0) //Don't release bus
    return (false); //Sensor did not ACK

  //Write header bytes
  _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress); //There is no register to write to, we just begin writing data bytes
  _i2cPort->write(UBX_SYNCH_1);
  _i2cPort->write(UBX_SYNCH_2);
  _i2cPort->write(outgoingUBX.cls);
  _i2cPort->write(outgoingUBX.id);
  _i2cPort->write(outgoingUBX.len & 0xFF); //LSB
  _i2cPort->write(outgoingUBX.len >> 8); //MSB
  if (_i2cPort->endTransmission(false) != 0) //Don't release bus
    return (false); //Sensor did not ACK

  //Write payload. Limit the sends into 32 byte chunks
  //This code based on ublox: https://forum.u-blox.com/index.php/20528/how-to-use-i2c-to-get-the-nmea-frames
  uint16_t bytesToSend = outgoingUBX.len;

  //"The number of data bytes must be at least 2 to properly distinguish
  //from the write access to set the address counter in random read accesses."
  uint16_t startSpot = 0;
  while (bytesToSend > 1)
  {
    uint8_t len = bytesToSend;
    if (len > I2C_BUFFER_LENGTH) len = I2C_BUFFER_LENGTH;

    _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
    //_i2cPort->write(outgoingUBX.payload, len); //Write a portion of the payload to the bus

    for (uint16_t x = 0 ; x < len ; x++)
      _i2cPort->write(outgoingUBX.payload[startSpot + x]); //Write a portion of the payload to the bus

    if (_i2cPort->endTransmission(false) != 0) //Don't release bus
      return (false); //Sensor did not ACK

    //*outgoingUBX.payload += len; //Move the pointer forward
    startSpot += len; //Move the pointer forward
    bytesToSend -= len;
  }

  //Write checksum
  _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
  if (bytesToSend == 1) _i2cPort->write(outgoingUBX.payload, 1);
  _i2cPort->write(outgoingUBX.checksumA);
  _i2cPort->write(outgoingUBX.checksumB);
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK

  if (maxWait > 0)
  {
    if (waitForResponse(maxWait) == false) //Wait for Ack response
      return (false);
  }
  return (true);
}

//Returns true if I2C device ack's
boolean SFE_UBLOX_GPS::isConnected()
{
  _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

//Given a message, calc and store the two byte "8-Bit Fletcher" checksum
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

  for (uint16_t i = 0 ; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}

//=-=-=-=-=-=-=-= Specific commands =-=-=-=-=-=-=-=

//Poll the module until and ack is received
boolean SFE_UBLOX_GPS::waitForResponse(uint16_t maxTime)
{
  commandAck = false; //Reset flag
  packetCfg.valid = false; //This will go true when we receive a response to the packet we sent

  long startTime = millis();
  while (millis() - startTime < maxTime)
  {
    checkUblox(); //See if new data is available. Process bytes as they come in.

    //If the packet we just sent was a CFG packet then we'll get an ACK
    //If the packet we just sent was a NAV packet then we'll just get data back
    if (commandAck == true) return (true);
    if (packetCfg.valid == true) return (true);
  }
  //Serial.println("waitForResponse timeout");
  return (false);
}


//Get the current TimeMode3 settings - these contain survey in statuses
boolean SFE_UBLOX_GPS::getSurveyMode(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_TMODE3;
  packetCfg.len = 0;

  return ( sendCommand(packetCfg, maxWait) );
}

//Control Survey-In for NEO-M8P
boolean SFE_UBLOX_GPS::setSurveyMode(uint8_t mode, uint16_t observationTime, float requiredAccuracy, uint16_t maxWait)
{
  if (getSurveyMode() == false) //Ask module for the current TimeMode3 settings. Loads into payloadCfg.
    return (false);
  //Serial.println("Current settings obtained");

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_TMODE3;
  packetCfg.len = 40;

  //payloadCfg should be loaded with poll response. Now modify only the bits we care about
  payloadCfg[2] = mode; //Set mode. Survey-In and Disabled are most common.

  payloadCfg[24] = observationTime & 0xFF; //svinMinDur in seconds
  payloadCfg[25] = observationTime >> 8; //svinMinDur in seconds

  uint32_t svinAccLimit = requiredAccuracy * 10000; //Convert m to 0.1mm
  payloadCfg[28] = svinAccLimit & 0xFF; //svinAccLimit in 0.1mm increments
  payloadCfg[29] = svinAccLimit >> 8;
  payloadCfg[30] = svinAccLimit >> 16;

  //Serial.println("Sending new settings");
  return ( sendCommand(packetCfg, maxWait) ); //Wait for ack
}

//Begin Survey-In for NEO-M8P
boolean SFE_UBLOX_GPS::enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint16_t maxWait)
{
	return(setSurveyMode(SVIN_MODE_ENABLE, observationTime, requiredAccuracy, maxWait));	
}

//Stop Survey-In for NEO-M8P
boolean SFE_UBLOX_GPS::disableSurveyMode(uint16_t maxWait)
{
	return(setSurveyMode(SVIN_MODE_DISABLE, 0, 0, maxWait));	
}


//Reads survey in status and sets the global variables 
//for status, position valid, observation time, and mean 3D StdDev
//Returns true if commands was successful
boolean SFE_UBLOX_GPS::getSurveyStatus(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_NAV;
  packetCfg.id = UBX_NAV_SVIN;
  packetCfg.len = 0;

  //Reset variables
  svin.active = false;
  svin.valid = false;
  svin.observationTime = 0;
  svin.meanAccuracy = 0;

  if(sendCommand(packetCfg, maxWait) == false)
    return(false); //If command send fails then bail

  //We got a response, now parse the bits into the svin structure
  svin.observationTime |= payloadCfg[8] << 8*0;
  svin.observationTime |= payloadCfg[9] << 8*1;
  svin.observationTime |= payloadCfg[10] << 8*2;
  svin.observationTime |= payloadCfg[11] << 8*3;

  uint32_t tempFloat = 0;
  tempFloat |= payloadCfg[28] << 8*0;
  tempFloat |= payloadCfg[29] << 8*1;
  //tempFloat |= payloadCfg[30] << 8*2;
  //tempFloat |= payloadCfg[31] << 8*3;
  svin.meanAccuracy = tempFloat / 10000.0; //Convert 0.1mm to m

  svin.valid = payloadCfg[36];
  svin.active = payloadCfg[37]; //1 if survey in progress, 0 otherwise

  return(true);
}

//Given a message number turns on a message ID for output over a given portID (UART, I2C, SPI, USB, etc)
//To disable a message, set secondsBetween messages to 0
//For base station RTK output we need:
//1005 = 0xF5 0x05 - Stationary RTK reference ARP
//1077 = 0xF5 0x4D - GPS MSM7
//1087 = 0xF5 0x57 - GLONASS MSM7
//1230 = 0xF5 0xE6 - GLONASS code-phase biases, set to once every 10 seconds
//Much of this configuration is not documented and instead discerned from u-center binary console
boolean SFE_UBLOX_GPS::enableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint8_t secondsBetweenMessages, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 8;

  //Clear packet payload
  for(uint8_t x = 0 ; x < packetCfg.len ; x++)
    packetCfg.payload[x] = 0;

  packetCfg.payload[0] = UBX_RTCM_MSB; //MSB, always 0xF5. Interesting, these are not little endian
  packetCfg.payload[1] = messageNumber; //LSB
  packetCfg.payload[2 + portID] = secondsBetweenMessages; //Byte 2 is I2C, byte 3 is UART1, etc.
  
  return ( sendCommand(packetCfg, maxWait) );
}

//Disable a given message on a given port by setting secondsBetweenMessages to zero
boolean SFE_UBLOX_GPS::disableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint16_t maxWait)
{
	return(enableRTCMmessage(messageNumber, portID, 0, maxWait));
}

//Enable/Disable RTCM3 (both input and output) for a given port
//Use to enable RTCM3 on I2C port (ID 0)
boolean SFE_UBLOX_GPS::setRTCMport(uint8_t portID, boolean enableRTCM3, uint16_t maxWait)
{
  //Get the current config values for this port ID
  getPortSettings(portID);

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 20;

  //msg_payload is now loaded with current bytes. Change only the ones we need to
  payloadCfg[13] |= (1 << 5); //InProtocolMask LSB - Set inRtcm3
  payloadCfg[15] |= (1 << 5); //OutProtocolMask LSB - Set outRtcm3

  return ( sendCommand(packetCfg, maxWait) );
}

//Returns the current protocol bits in the UBX-CFG-PRT command for a given port
boolean SFE_UBLOX_GPS::getPortSettings(uint8_t portID, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 1;
  payloadCfg[0] = portID;

  return ( sendCommand(packetCfg, maxWait) );
}
