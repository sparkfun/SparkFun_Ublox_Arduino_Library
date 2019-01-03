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

#ifndef SPARKFUN_UBLOX_ARDUINO_LIBRARY_H
#define SPARKFUN_UBLOX_ARDUINO_LIBRARY_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>


//Platform specific configurations

//Define the size of the I2C buffer based on the platform the user has
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

//I2C_BUFFER_LENGTH is defined in Wire.H
#define I2C_BUFFER_LENGTH BUFFER_LENGTH

#elif defined(__SAMD21G18A__)

//SAMD21 uses RingBuffer.h
#define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE

#elif __MK20DX256__
//Teensy

#elif ARDUINO_ARCH_ESP32
//ESP32 based platforms

#else

//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32

#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Registers
const uint8_t UBX_SYNCH_1 = 0xB5;
const uint8_t UBX_SYNCH_2 = 0x62;

const uint8_t UBX_CLASS_NAV = 0x01;
const uint8_t UBX_CLASS_RXM = 0x02;
const uint8_t UBX_CLASS_INF = 0x04;
const uint8_t UBX_CLASS_ACK = 0x05;
const uint8_t UBX_CLASS_CFG = 0x06;
const uint8_t UBX_CLASS_UPD = 0x09;
const uint8_t UBX_CLASS_MON = 0x0A;
const uint8_t UBX_CLASS_AID = 0x0B;
const uint8_t UBX_CLASS_TIM = 0x0D;
const uint8_t UBX_CLASS_ESF = 0x10;
const uint8_t UBX_CLASS_MGA = 0x13;
const uint8_t UBX_CLASS_LOG = 0x21;
const uint8_t UBX_CLASS_SEC = 0x27;
const uint8_t UBX_CLASS_HNR = 0x28;

const uint8_t UBX_CFG_PRT = 0x00; //Used to configure port specifics
const uint8_t UBX_CFG_RATE = 0x08; //Used to set port baud rates

const uint8_t UBX_CFG_TMODE3 = 0x71; //Used to enable Survey In Mode
const uint8_t SVIN_MODE_DISABLE = 0x00;
const uint8_t SVIN_MODE_ENABLE = 0x01;

const uint8_t UBX_NAV_POSLLH = 0x02; //Used for obtaining lat/long/alt in low precision
const uint8_t UBX_NAV_HPPOSLLH = 0x14; //Used for obtaining lat/long/alt in high precision

const uint8_t UBX_NAV_SVIN = 0x3B; //Used for checking Survey In status
const uint8_t UBX_NAV_HPPOSECEF = 0x13; //Find our positional accuracy (high precision)

//The following are used to enable RTCM messages
const uint8_t UBX_CFG_MSG = 0x01;
const uint8_t UBX_RTCM_MSB = 0xF5; //All RTCM enable commands have 0xF5 as MSB
const uint8_t UBX_RTCM_1005 = 0x05; //Stationary RTK reference ARP
const uint8_t UBX_RTCM_1077 = 0x4D; //GPS MSM7
const uint8_t UBX_RTCM_1087 = 0x57; //GLONASS MSM7
const uint8_t UBX_RTCM_1230 = 0xE6; //GLONASS code-phase biases, set to once every 10 seconds
const uint8_t UBX_RTCM_I2C_PORT = 0;
const uint8_t UBX_RTCM_UART1_PORT = 1;
const uint8_t UBX_RTCM_UART2_PORT = 2;
const uint8_t UBX_RTCM_USB_PORT = 3;
const uint8_t UBX_RTCM_SPI_PORT = 4;

const uint8_t UBX_ACK_NACK = 0x00;
const uint8_t UBX_ACK_ACK = 0x01;

#define MAX_PAYLOAD_SIZE 64 //Some commands are larger than 64 bytes but this covers most

//-=-=-=-=- UBX binary specific variables
typedef struct
{
	uint8_t cls;
	uint8_t id;
	uint16_t len;
	uint8_t counter; //Should really be 16 bit but no commands are larger than 255 currently
	uint8_t *payload;
	uint8_t checksumA;
	uint8_t checksumB;
	boolean valid; //Goes true when both checksums pass
} ubxPacket;

	
class SFE_UBLOX_GPS
{
  public:
    SFE_UBLOX_GPS(void);

    //By default use the default I2C address, and use Wire port
    void begin(TwoWire &wirePort = Wire);

	boolean isConnected(); //Returns turn if device answers on _gpsI2Caddress address

	boolean checkUblox(); //Checks module with user selected commType
	boolean checkUbloxI2C(); //Method for I2C polling of data, passing any new bytes to process()
	boolean checkUbloxSerial(); //Method for serial polling of data, passing any new bytes to process()

	void process(uint8_t incoming); //Processes NMEA and UBX binary sentences one byte at a time
	void processUBX(uint8_t incoming, ubxPacket *incomingUBX); //Given a character, file it away into the uxb packet structure
	void processRTCMframe(uint8_t incoming); //Monitor the incoming bytes for start and length bytes 
	void processRTCM(uint8_t incoming) __attribute__((weak)); //Given rtcm byte, do something with it. User can overwrite if desired to pipe bytes to radio, internet, etc.
	
	void processUBXpacket(ubxPacket *msg); //Once a packet has been received and validated, identify this packet's class/id and update internal flags
	void processNMEA(char incoming) __attribute__((weak)); //Given a nmea character, do something with it. User can overwrite if desired to use something like tinyGPS or MicroNMEA libraries
	
	void calcChecksum(ubxPacket *msg); //Sets the checksumA and checksumB of a given messages
	boolean sendCommand(ubxPacket outgoingUBX, uint16_t maxWait = 250); //Given a packet and payload, send everything including CRC bytes

	void setI2CReadAddress(uint8_t deviceAddress); //Sets the internal variable for the address of ublox module we want to read from
	void setNMEAOutputPort(Stream &nmeaOutputPort); //Sets the internal variable for the port to direct NMEA characters to
	
	boolean waitForResponse(uint16_t maxTime = 250); //Poll the module until and ack is received

	boolean getSurveyMode(uint16_t maxWait = 250); //Get the current TimeMode3 settings
	boolean setSurveyMode(uint8_t mode, uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = 250); //Control survey in mode
	boolean enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = 250); //Begin Survey-In for NEO-M8P
	boolean disableSurveyMode(uint16_t maxWait = 250); //Stop Survey-In mode
	
	boolean getSurveyStatus(uint16_t maxWait); //Reads survey in status and sets the global variables 
	boolean enableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint8_t secondsBetweenMessages, uint16_t maxWait = 250); //Given a message number turns on a message ID for output over given PortID
	boolean disableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint16_t maxWait = 250); //Turn off given RTCM message from a given port
	boolean setRTCMport(uint8_t portID, boolean enableRTCM3, uint16_t maxWait = 250); //Enable/Disable RTCM3 (both input and output) for a given port

	boolean getPortSettings(uint8_t portID, uint16_t maxWait = 250); //Returns the current protocol bits in the UBX-CFG-PRT command for a given port
	
	uint32_t getPositionAccuracy(uint16_t maxWait = 500); //Returns the 3D accuracy of the current high-precision fix, in mm. Supported on NEO-M8P, ZED-F9P,
	
	uint32_t getLatitude(uint16_t maxWait = 250); //Returns the current latitude in degrees * 10^-7. Auto selects between HighPrecision and Regular depending on ability of module.
	uint32_t getLongitude(uint16_t maxWait = 250); //Returns the current longitude in degrees * 10-7. Auto selects between HighPrecision and Regular depending on ability of module.

	struct svinStructure {
		boolean active;
		boolean valid;
		uint16_t observationTime;
		float meanAccuracy;
	} svin;

	uint16_t rtcmFrameCounter = 0;
	
  private:

	//Depending on the sentence type the processor will load characters into different arrays
	enum SentenceTypes
	{
		NONE = 0,
		NMEA,
		UBX,
		RTCM
	} currentSentence = NONE;

	//Depending on the ubx binary response class, store binary responses into different places
	enum classTypes
	{
		CLASS_NONE = 0,
		CLASS_ACK,
		CLASS_NOT_AN_ACK
	} ubxFrameClass = CLASS_NONE;
	
	enum commTypes
	{
		COMM_TYPE_I2C = 0,
		COMM_TYPE_SERIAL,
		COMM_TYPE_SPI
	} commType = COMM_TYPE_I2C; //Controls which port we look to for incoming bytes

	//Variables
    TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
	Stream *_nmeaOutputPort = NULL; //The user can assign an output port to print NMEA sentences if they wish
	
	uint8_t _gpsI2Caddress = 0x42; //Default 7-bit unshifted address of the ublox 6/7/8/M8 series
	//This can be changed using the ublox configuration software
	
	//These are pointed at from within the ubxPacket
	uint8_t payloadAck[2];
	uint8_t payloadCfg[MAX_PAYLOAD_SIZE];

	//Init the packet structures and init them with pointers to the payloadAck and payloadCfg arrays
	ubxPacket packetAck = {0, 0, 0, 0, payloadAck, 0, 0, false};
	ubxPacket packetCfg = {0, 0, 0, 0, payloadCfg, 0, 0, false};

	const uint8_t I2C_POLLING_WAIT_MS = 25; //Limit checking of new characters to every X ms
	unsigned long lastCheck = 0;
	boolean commandAck = false; //This goes true after we send a command and it's ack'd
	uint8_t ubxFrameCounter;

	uint16_t rtcmLen = 0;
};

#endif
