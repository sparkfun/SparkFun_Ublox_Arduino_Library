/*
	This is a library written for the Ublox ZED-F9P and NEO-M8P-2
	SparkFun sells these at its website: www.sparkfun.com
	Do you like this library? Help support SparkFun. Buy a board!
	https://www.sparkfun.com/products/15136
	https://www.sparkfun.com/products/15005
	https://www.sparkfun.com/products/15733
	https://www.sparkfun.com/products/15193
	https://www.sparkfun.com/products/15210

	Written by Nathan Seidle @ SparkFun Electronics, September 6th, 2018

	This library handles configuring and handling the responses 
	from a Ublox GPS module. Works with most modules from Ublox including
	the Zed-F9P, NEO-M8P-2, NEO-M9N, ZOE-M8Q, SAM-M8Q, and many others.

	https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

	Development environment specifics:
	Arduino IDE 1.8.5

	SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
	The MIT License (MIT)
	Copyright (c) 2016 SparkFun Electronics
	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
	associated documentation files (the "Software"), to deal in the Software without restriction, 
	including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
	and/or sell copies of the Software, and to permit persons to whom the Software is furnished to 
	do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial 
	portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
	NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
	IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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

//#elif __MK20DX256__
//Teensy

#endif

#ifndef I2C_BUFFER_LENGTH

//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32
//#define I2C_BUFFER_LENGTH 16 //For testing on Artemis

#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Define a digital pin to aid checksum failure capture and analysis
//Leave set to -1 if not needed
const int checksumFailurePin = -1;

// Global Status Returns
typedef enum
{
	SFE_UBLOX_STATUS_FAIL = 0, // FAIL == 0
	SFE_UBLOX_STATUS_SUCCESS, // SUCCESS == 1
	SFE_UBLOX_STATUS_CRC_FAIL,
	SFE_UBLOX_STATUS_TIMEOUT,
	SFE_UBLOX_STATUS_COMMAND_UNKNOWN,
	SFE_UBLOX_STATUS_OUT_OF_RANGE,
	SFE_UBLOX_STATUS_INVALID_ARG,
	SFE_UBLOX_STATUS_INVALID_OPERATION,
	SFE_UBLOX_STATUS_MEM_ERR,
	SFE_UBLOX_STATUS_HW_ERR,
	SFE_UBLOX_STATUS_DATA_SENT,
	SFE_UBLOX_STATUS_DATA_RECEIVED,
	SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
} sfe_ublox_status_e;

typedef enum
{
  SFE_UBLOX_PACKET_VALIDITY_NOT_VALID = 0, // NOT_VALID == 0
  SFE_UBLOX_PACKET_VALIDITY_VALID, // VALID == 1
  SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED
} sfe_ublox_packet_validity_e;

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
const uint8_t UBX_CLASS_NMEA = 0xF0;

const uint8_t UBX_CFG_PRT = 0x00;  //Used to configure port specifics
const uint8_t UBX_CFG_RST = 0x04;  //Used to reset device
const uint8_t UBX_CFG_RATE = 0x08; //Used to set port baud rates
const uint8_t UBX_CFG_CFG = 0x09;  //Used to save current configuration
const uint8_t UBX_CFG_RXM = 0x11;  //Used to set receiver power management (power save mode)
const uint8_t UBX_CFG_NAV5 = 0x24; //Used to configure the navigation engine including the dynamic model

const uint8_t UBX_CFG_VALSET = 0x8A; //Used for config of higher version Ublox modules (ie protocol v27 and above)
const uint8_t UBX_CFG_VALGET = 0x8B; //Used for config of higher version Ublox modules (ie protocol v27 and above)
const uint8_t UBX_CFG_VALDEL = 0x8C; //Used for config of higher version Ublox modules (ie protocol v27 and above)

const uint8_t UBX_CFG_GEOFENCE = 0x69; //Used to configure a geofence
const uint8_t UBX_CFG_ANT = 0x13;	  //Used to configure the antenna control settings
const uint8_t UBX_NAV_GEOFENCE = 0x39; //Used to poll the geofence status

const uint8_t UBX_CFG_TMODE3 = 0x71; //Used to enable Survey In Mode
const uint8_t SVIN_MODE_DISABLE = 0x00;
const uint8_t SVIN_MODE_ENABLE = 0x01;

const uint8_t UBX_NAV_PVT = 0x07;		//All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites
const uint8_t UBX_NAV_HPPOSECEF = 0x13; //Find our positional accuracy (high precision)
const uint8_t UBX_NAV_HPPOSLLH = 0x14;  //Used for obtaining lat/long/alt in high precision
const uint8_t UBX_NAV_SVIN = 0x3B;		//Used for checking Survey In status
const uint8_t UBX_NAV_RELPOSNED = 0x3C; //Relative Positioning Information in NED frame

const uint8_t UBX_NMEA_GGA = 0x00;
const uint8_t UBX_NMEA_GLL = 0x01;
const uint8_t UBX_NMEA_GNS = 0x0D;
const uint8_t UBX_NMEA_GRS = 0x06;
const uint8_t UBX_NMEA_GSA = 0x02;
const uint8_t UBX_NMEA_GST = 0x07;
const uint8_t UBX_NMEA_GSV = 0x03;
const uint8_t UBX_NMEA_RMC = 0x04;
const uint8_t UBX_NMEA_VTG = 0x05;
const uint8_t UBX_NMEA_ZDA = 0x08;

const uint8_t UBX_MON_VER = 0x04;   //Used for obtaining Protocol Version
const uint8_t UBX_MON_TXBUF = 0x08; //Used for query tx buffer size/state

//The following are used to enable RTCM messages
const uint8_t UBX_CFG_MSG = 0x01;
const uint8_t UBX_RTCM_MSB = 0xF5;  //All RTCM enable commands have 0xF5 as MSB
const uint8_t UBX_RTCM_1005 = 0x05; //Stationary RTK reference ARP
const uint8_t UBX_RTCM_1074 = 0x4A; //GPS MSM4
const uint8_t UBX_RTCM_1077 = 0x4D; //GPS MSM7
const uint8_t UBX_RTCM_1084 = 0x54; //GLONASS MSM4
const uint8_t UBX_RTCM_1087 = 0x57; //GLONASS MSM7
const uint8_t UBX_RTCM_1094 = 0x5E; //Galileo MSM4
const uint8_t UBX_RTCM_1124 = 0x7C; //BeiDou MSM4
const uint8_t UBX_RTCM_1230 = 0xE6; //GLONASS code-phase biases, set to once every 10 seconds

const uint8_t UBX_ACK_NACK = 0x00;
const uint8_t UBX_ACK_ACK = 0x01;
const uint8_t UBX_ACK_NONE = 0x02; //Not a real value

//The following consts are used to configure the various ports and streams for those ports. See -CFG-PRT.
const uint8_t COM_PORT_I2C = 0;
const uint8_t COM_PORT_UART1 = 1;
const uint8_t COM_PORT_UART2 = 2;
const uint8_t COM_PORT_USB = 3;
const uint8_t COM_PORT_SPI = 4;

const uint8_t COM_TYPE_UBX = (1 << 0);
const uint8_t COM_TYPE_NMEA = (1 << 1);
const uint8_t COM_TYPE_RTCM3 = (1 << 5);

//The following consts are used to generate KEY values for the advanced protocol functions of VELGET/SET/DEL
const uint8_t VAL_SIZE_1 = 0x01;  //One bit
const uint8_t VAL_SIZE_8 = 0x02;  //One byte
const uint8_t VAL_SIZE_16 = 0x03; //Two bytes
const uint8_t VAL_SIZE_32 = 0x04; //Four bytes
const uint8_t VAL_SIZE_64 = 0x05; //Eight bytes

//These are the Bitfield layers definitions for the UBX-CFG-VALSET message (not to be confused with Bitfield deviceMask in UBX-CFG-CFG)
const uint8_t VAL_LAYER_RAM = (1 << 0);
const uint8_t VAL_LAYER_BBR = (1 << 1);
const uint8_t VAL_LAYER_FLASH = (1 << 2);

//Below are various Groups, IDs, and sizes for various settings
//These can be used to call getVal/setVal/delVal
const uint8_t VAL_GROUP_I2COUTPROT = 0x72;
const uint8_t VAL_GROUP_I2COUTPROT_SIZE = VAL_SIZE_1; //All fields in I2C group are currently 1 bit

const uint8_t VAL_ID_I2COUTPROT_UBX = 0x01;
const uint8_t VAL_ID_I2COUTPROT_NMEA = 0x02;
const uint8_t VAL_ID_I2COUTPROT_RTCM3 = 0x03;

const uint8_t VAL_GROUP_I2C = 0x51;
const uint8_t VAL_GROUP_I2C_SIZE = VAL_SIZE_8; //All fields in I2C group are currently 1 byte

const uint8_t VAL_ID_I2C_ADDRESS = 0x01;

enum dynModel // Possible values for the dynamic platform model
{
	DYN_MODEL_PORTABLE = 0,
	// 1 is not defined
	DYN_MODEL_STATIONARY = 2,
	DYN_MODEL_PEDESTRIAN,
	DYN_MODEL_AUTOMOTIVE,
	DYN_MODEL_SEA,
	DYN_MODEL_AIRBORNE1g,
	DYN_MODEL_AIRBORNE2g,
	DYN_MODEL_AIRBORNE4g,
	DYN_MODEL_WRIST, // Not supported in protocol versions less than 18
	DYN_MODEL_BIKE,  // Supported in protocol versions 19.2
};

#ifndef MAX_PAYLOAD_SIZE

#define MAX_PAYLOAD_SIZE 256 //We need ~220 bytes for getProtocolVersion on most ublox modules
//#define MAX_PAYLOAD_SIZE 768 //Worst case: UBX_CFG_VALSET packet with 64 keyIDs each with 64 bit values

#endif

//-=-=-=-=- UBX binary specific variables
typedef struct
{
	uint8_t cls;
	uint8_t id;
	uint16_t len;		   //Length of the payload. Does not include cls, id, or checksum bytes
	uint16_t counter;	  //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
	uint16_t startingSpot; //The counter value needed to go past before we begin recording into payload array
	uint8_t *payload;
	uint8_t checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
	uint8_t checksumB;
	sfe_ublox_packet_validity_e valid; //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
} ubxPacket;

// Struct to hold the results returned by getGeofenceState (returned by UBX-NAV-GEOFENCE)
typedef struct
{
	uint8_t status;	// Geofencing status: 0 - Geofencing not available or not reliable; 1 - Geofencing active
	uint8_t numFences; // Number of geofences
	uint8_t combState; // Combined (logical OR) state of all geofences: 0 - Unknown; 1 - Inside; 2 - Outside
	uint8_t states[4]; // Geofence states: 0 - Unknown; 1 - Inside; 2 - Outside
} geofenceState;

// Struct to hold the current geofence parameters
typedef struct
{
	uint8_t numFences; // Number of active geofences
	int32_t lats[4];   // Latitudes of geofences (in degrees * 10^-7)
	int32_t longs[4];  // Longitudes of geofences (in degrees * 10^-7)
	uint32_t rads[4];  // Radii of geofences (in m * 10^-2)
} geofenceParams;

class SFE_UBLOX_GPS
{
public:
	SFE_UBLOX_GPS(void);

	//By default use the default I2C address, and use Wire port
	boolean begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = 0x42); //Returns true if module is detected
	//serialPort needs to be perviously initialized to correct baud rate
	boolean begin(Stream &serialPort); //Returns true if module is detected

	boolean isConnected(); //Returns turn if device answers on _gpsI2Caddress address

	boolean checkUblox();		//Checks module with user selected commType
	boolean checkUbloxI2C();	//Method for I2C polling of data, passing any new bytes to process()
	boolean checkUbloxSerial(); //Method for serial polling of data, passing any new bytes to process()

	void process(uint8_t incoming);							   //Processes NMEA and UBX binary sentences one byte at a time
	void processUBX(uint8_t incoming, ubxPacket *incomingUBX); //Given a character, file it away into the uxb packet structure
	void processRTCMframe(uint8_t incoming);				   //Monitor the incoming bytes for start and length bytes
	void processRTCM(uint8_t incoming) __attribute__((weak));  //Given rtcm byte, do something with it. User can overwrite if desired to pipe bytes to radio, internet, etc.

	void processUBXpacket(ubxPacket *msg);				   //Once a packet has been received and validated, identify this packet's class/id and update internal flags
	void processNMEA(char incoming) __attribute__((weak)); //Given a NMEA character, do something with it. User can overwrite if desired to use something like tinyGPS or MicroNMEA libraries

	void calcChecksum(ubxPacket *msg);											   //Sets the checksumA and checksumB of a given messages
	sfe_ublox_status_e sendCommand(ubxPacket outgoingUBX, uint16_t maxWait = 250); //Given a packet and payload, send everything including CRC bytes, return true if we got a response
	sfe_ublox_status_e sendI2cCommand(ubxPacket outgoingUBX, uint16_t maxWait = 250);
	void sendSerialCommand(ubxPacket outgoingUBX);

	void printPacket(ubxPacket *packet); //Useful for debugging

	void factoryReset(); //Send factory reset sequence (i.e. load "default" configuration and perform hardReset)
	void hardReset();	//Perform a reset leading to a cold start (zero info start-up)

	boolean setI2CAddress(uint8_t deviceAddress, uint16_t maxTime = 250);							  //Changes the I2C address of the Ublox module
	void setSerialRate(uint32_t baudrate, uint8_t uartPort = COM_PORT_UART1, uint16_t maxTime = 250); //Changes the serial baud rate of the Ublox module, uartPort should be COM_PORT_UART1/2
	void setNMEAOutputPort(Stream &nmeaOutputPort);													  //Sets the internal variable for the port to direct NMEA characters to

	boolean setNavigationFrequency(uint8_t navFreq, uint16_t maxWait = 250); //Set the number of nav solutions sent per second
	uint8_t getNavigationFrequency(uint16_t maxWait = 250);					 //Get the number of nav solutions sent per second currently being output by module
	boolean saveConfiguration(uint16_t maxWait = 250);						 //Save current configuration to flash and BBR (battery backed RAM)
	boolean factoryDefault(uint16_t maxWait = 250);							 //Reset module to factory defaults

	sfe_ublox_status_e waitForACKResponse(uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = 250);   //Poll the module until a config packet and an ACK is received
	sfe_ublox_status_e waitForNoACKResponse(uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = 250); //Poll the module until a config packet is received

// getPVT will only return data once in each navigation cycle. By default, that is once per second.
// Therefore we should set getPVTmaxWait to slightly longer than that.
// If you change the navigation frequency to (e.g.) 4Hz using setNavigationFrequency(4)
// then you should use a shorter maxWait for getPVT. 300msec would be about right: getPVT(300)
// The same is true for getHPPOSLLH.
#define getPVTmaxWait 1100		// Default maxWait for getPVT and all functions which call it
#define getHPPOSLLHmaxWait 1100 // Default maxWait for getHPPOSLLH and all functions which call it

	boolean assumeAutoPVT(boolean enabled, boolean implicitUpdate = true);				 //In case no config access to the GPS is possible and PVT is send cyclically already
	boolean setAutoPVT(boolean enabled, uint16_t maxWait = 250);						 //Enable/disable automatic PVT reports at the navigation frequency
	boolean getPVT(uint16_t maxWait = getPVTmaxWait);									 //Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Retruns true if new PVT is available.
	boolean setAutoPVT(boolean enabled, boolean implicitUpdate, uint16_t maxWait = 250); //Enable/disable automatic PVT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	boolean getHPPOSLLH(uint16_t maxWait = getHPPOSLLHmaxWait);							 //Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Retruns true if new PVT is available.
	void flushPVT();																	 //Mark all the PVT data as read/stale. This is handy to get data alignment after CRC failure

	int32_t getLatitude(uint16_t maxWait = getPVTmaxWait);			  //Returns the current latitude in degrees * 10^-7. Auto selects between HighPrecision and Regular depending on ability of module.
	int32_t getLongitude(uint16_t maxWait = getPVTmaxWait);			  //Returns the current longitude in degrees * 10-7. Auto selects between HighPrecision and Regular depending on ability of module.
	int32_t getAltitude(uint16_t maxWait = getPVTmaxWait);			  //Returns the current altitude in mm above ellipsoid
	int32_t getAltitudeMSL(uint16_t maxWait = getPVTmaxWait);		  //Returns the current altitude in mm above mean sea level
	uint8_t getSIV(uint16_t maxWait = getPVTmaxWait);				  //Returns number of sats used in fix
	uint8_t getFixType(uint16_t maxWait = getPVTmaxWait);			  //Returns the type of fix: 0=no, 3=3D, 4=GNSS+Deadreckoning
	uint8_t getCarrierSolutionType(uint16_t maxWait = getPVTmaxWait); //Returns RTK solution: 0=no, 1=float solution, 2=fixed solution
	int32_t getGroundSpeed(uint16_t maxWait = getPVTmaxWait);		  //Returns speed in mm/s
	int32_t getHeading(uint16_t maxWait = getPVTmaxWait);			  //Returns heading in degrees * 10^-7
	uint16_t getPDOP(uint16_t maxWait = getPVTmaxWait);				  //Returns positional dillution of precision * 10^-2
	uint16_t getYear(uint16_t maxWait = getPVTmaxWait);
	uint8_t getMonth(uint16_t maxWait = getPVTmaxWait);
	uint8_t getDay(uint16_t maxWait = getPVTmaxWait);
	uint8_t getHour(uint16_t maxWait = getPVTmaxWait);
	uint8_t getMinute(uint16_t maxWait = getPVTmaxWait);
	uint8_t getSecond(uint16_t maxWait = getPVTmaxWait);
	uint16_t getMillisecond(uint16_t maxWait = getPVTmaxWait);
	int32_t getNanosecond(uint16_t maxWait = getPVTmaxWait);
	uint32_t getTimeOfWeek(uint16_t maxWait = getPVTmaxWait);

	int32_t getHighResLatitude(uint16_t maxWait = getHPPOSLLHmaxWait);
	int32_t getHighResLongitude(uint16_t maxWait = getHPPOSLLHmaxWait);
	int32_t getElipsoid(uint16_t maxWait = getHPPOSLLHmaxWait);
	int32_t getMeanSeaLevel(uint16_t maxWait = getHPPOSLLHmaxWait);
	int32_t getGeoidSeparation(uint16_t maxWait = getHPPOSLLHmaxWait);
	uint32_t getHorizontalAccuracy(uint16_t maxWait = getHPPOSLLHmaxWait);
	uint32_t getVerticalAccuracy(uint16_t maxWait = getHPPOSLLHmaxWait);

	//Port configurations
	boolean setPortOutput(uint8_t portID, uint8_t comSettings, uint16_t maxWait = 250); //Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
	boolean setPortInput(uint8_t portID, uint8_t comSettings, uint16_t maxWait = 250);  //Configure a given port to input UBX, NMEA, RTCM3 or a combination thereof
	boolean getPortSettings(uint8_t portID, uint16_t maxWait = 250);					//Returns the current protocol bits in the UBX-CFG-PRT command for a given port

	boolean setI2COutput(uint8_t comSettings, uint16_t maxWait = 250);   //Configure I2C port to output UBX, NMEA, RTCM3 or a combination thereof
	boolean setUART1Output(uint8_t comSettings, uint16_t maxWait = 250); //Configure UART1 port to output UBX, NMEA, RTCM3 or a combination thereof
	boolean setUART2Output(uint8_t comSettings, uint16_t maxWait = 250); //Configure UART2 port to output UBX, NMEA, RTCM3 or a combination thereof
	boolean setUSBOutput(uint8_t comSettings, uint16_t maxWait = 250);   //Configure USB port to output UBX, NMEA, RTCM3 or a combination thereof
	boolean setSPIOutput(uint8_t comSettings, uint16_t maxWait = 250);   //Configure SPI port to output UBX, NMEA, RTCM3 or a combination thereof

	//Functions to turn on/off message types for a given port ID (see COM_PORT_I2C, etc above)
	boolean configureMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint8_t sendRate, uint16_t maxWait = 250);
	boolean enableMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint8_t sendRate = 1, uint16_t maxWait = 250);
	boolean disableMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint16_t maxWait = 250);
	boolean enableNMEAMessage(uint8_t msgID, uint8_t portID, uint8_t sendRate = 1, uint16_t maxWait = 250);
	boolean disableNMEAMessage(uint8_t msgID, uint8_t portID, uint16_t maxWait = 250);
	boolean enableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint8_t sendRate, uint16_t maxWait = 250); //Given a message number turns on a message ID for output over given PortID
	boolean disableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint16_t maxWait = 250);					//Turn off given RTCM message from a given port

	//General configuration (used only on protocol v27 and higher - ie, ZED-F9P)
	uint8_t getVal8(uint16_t group, uint16_t id, uint8_t size, uint8_t layer = VAL_LAYER_BBR, uint16_t maxWait = 250); //Returns the value at a given group/id/size location
	uint8_t getVal8(uint32_t keyID, uint8_t layer = VAL_LAYER_BBR, uint16_t maxWait = 250);							   //Returns the value at a given group/id/size location
	uint8_t setVal(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_BBR, uint16_t maxWait = 250);			   //Sets the 16-bit value at a given group/id/size location
	uint8_t setVal8(uint32_t keyID, uint8_t value, uint8_t layer = VAL_LAYER_BBR, uint16_t maxWait = 250);			   //Sets the 8-bit value at a given group/id/size location
	uint8_t setVal16(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_BBR, uint16_t maxWait = 250);		   //Sets the 16-bit value at a given group/id/size location
	uint8_t setVal32(uint32_t keyID, uint32_t value, uint8_t layer = VAL_LAYER_BBR, uint16_t maxWait = 250);		   //Sets the 32-bit value at a given group/id/size location
	uint8_t newCfgValset8(uint32_t keyID, uint8_t value, uint8_t layer = VAL_LAYER_BBR);							   //Define a new UBX-CFG-VALSET with the given KeyID and 8-bit value
	uint8_t newCfgValset16(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_BBR);							   //Define a new UBX-CFG-VALSET with the given KeyID and 16-bit value
	uint8_t newCfgValset32(uint32_t keyID, uint32_t value, uint8_t layer = VAL_LAYER_BBR);							   //Define a new UBX-CFG-VALSET with the given KeyID and 32-bit value
	uint8_t addCfgValset8(uint32_t keyID, uint8_t value);															   //Add a new KeyID and 8-bit value to an existing UBX-CFG-VALSET ubxPacket
	uint8_t addCfgValset16(uint32_t keyID, uint16_t value);															   //Add a new KeyID and 16-bit value to an existing UBX-CFG-VALSET ubxPacket
	uint8_t addCfgValset32(uint32_t keyID, uint32_t value);															   //Add a new KeyID and 32-bit value to an existing UBX-CFG-VALSET ubxPacket
	uint8_t sendCfgValset8(uint32_t keyID, uint8_t value, uint16_t maxWait = 250);									   //Add the final KeyID and 8-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
	uint8_t sendCfgValset16(uint32_t keyID, uint16_t value, uint16_t maxWait = 250);								   //Add the final KeyID and 16-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
	uint8_t sendCfgValset32(uint32_t keyID, uint32_t value, uint16_t maxWait = 250);								   //Add the final KeyID and 32-bit value to an existing UBX-CFG-VALSET ubxPacket and send it

	//Functions used for RTK and base station setup
	boolean getSurveyMode(uint16_t maxWait = 250);																   //Get the current TimeMode3 settings
	boolean setSurveyMode(uint8_t mode, uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = 250); //Control survey in mode
	boolean enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = 250);			   //Begin Survey-In for NEO-M8P
	boolean disableSurveyMode(uint16_t maxWait = 250);															   //Stop Survey-In mode

	boolean getSurveyStatus(uint16_t maxWait); //Reads survey in status and sets the global variables

	uint32_t getPositionAccuracy(uint16_t maxWait = 1100); //Returns the 3D accuracy of the current high-precision fix, in mm. Supported on NEO-M8P, ZED-F9P,

	uint8_t getProtocolVersionHigh(uint16_t maxWait = 500); //Returns the PROTVER XX.00 from UBX-MON-VER register
	uint8_t getProtocolVersionLow(uint16_t maxWait = 500);  //Returns the PROTVER 00.XX from UBX-MON-VER register
	boolean getProtocolVersion(uint16_t maxWait = 500);		//Queries module, loads low/high bytes

	boolean getRELPOSNED(uint16_t maxWait = 1100); //Get Relative Positioning Information of the NED frame

	void enableDebugging(Stream &debugPort = Serial);  //Given a port to print to, enable debug messages
	void disableDebugging(void);					   //Turn off debug statements
	void debugPrint(char *message);					   //Safely print debug statements
	void debugPrintln(char *message);				   //Safely print debug statements
	const char *statusString(sfe_ublox_status_e stat); //Pretty print the return value

	//Support for geofences
	boolean addGeofence(int32_t latitude, int32_t longitude, uint32_t radius, byte confidence = 0, byte pinPolarity = 0, byte pin = 0, uint16_t maxWait = 1100); // Add a new geofence
	boolean clearGeofences(uint16_t maxWait = 1100);																											 //Clears all geofences
	boolean getGeofenceState(geofenceState &currentGeofenceState, uint16_t maxWait = 1100);																		 //Returns the combined geofence state
	boolean clearAntPIO(uint16_t maxWait = 1100);																												 //Clears the antenna control pin settings to release the PIOs
	geofenceParams currentGeofenceParams;																														 // Global to store the geofence parameters

	boolean powerSaveMode(bool power_save = true, uint16_t maxWait = 1100);

	//Change the dynamic platform model using UBX-CFG-NAV5
	boolean setDynamicModel(dynModel newDynamicModel = DYN_MODEL_PORTABLE, uint16_t maxWait = 1100);

	//Survey-in specific controls
	struct svinStructure
	{
		boolean active;
		boolean valid;
		uint16_t observationTime;
		float meanAccuracy;
	} svin;

	//Relative Positioning Info in NED frame specific controls
	struct frelPosInfoStructure
	{
		uint16_t refStationID;

		float relPosN;
		float relPosE;
		float relPosD;

		long relPosLength;
		long relPosHeading;

		int8_t relPosHPN;
		int8_t relPosHPE;
		int8_t relPosHPD;
		int8_t relPosHPLength;

		float accN;
		float accE;
		float accD;

		bool gnssFixOk;
		bool diffSoln;
		bool relPosValid;
		uint8_t carrSoln;
		bool isMoving;
		bool refPosMiss;
		bool refObsMiss;
	} relPosInfo;

	//The major datums we want to globally store
	uint16_t gpsYear;
	uint8_t gpsMonth;
	uint8_t gpsDay;
	uint8_t gpsHour;
	uint8_t gpsMinute;
	uint8_t gpsSecond;
	uint16_t gpsMillisecond;
	int32_t gpsNanosecond;

	int32_t latitude;		 //Degrees * 10^-7 (more accurate than floats)
	int32_t longitude;		 //Degrees * 10^-7 (more accurate than floats)
	int32_t altitude;		 //Number of mm above ellipsoid
	int32_t altitudeMSL;	 //Number of mm above Mean Sea Level
	uint8_t SIV;			 //Number of satellites used in position solution
	uint8_t fixType;		 //Tells us when we have a solution aka lock
	uint8_t carrierSolution; //Tells us when we have an RTK float/fixed solution
	int32_t groundSpeed;	 //mm/s
	int32_t headingOfMotion; //degrees * 10^-5
	uint16_t pDOP;			 //Positional dilution of precision
	uint8_t versionLow;		 //Loaded from getProtocolVersion().
	uint8_t versionHigh;

	uint32_t timeOfWeek;
	int32_t highResLatitude;
	int32_t highResLongitude;
	int32_t elipsoid;
	int32_t meanSeaLevel;
	int32_t geoidSeparation;
	uint32_t horizontalAccuracy;
	uint32_t verticalAccuracy;

	uint16_t rtcmFrameCounter = 0; //Tracks the type of incoming byte inside RTCM frame

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

	//Functions
	uint32_t extractLong(uint8_t spotToStart); //Combine four bytes from payload into long
	uint16_t extractInt(uint8_t spotToStart);  //Combine two bytes from payload into int
	uint8_t extractByte(uint8_t spotToStart);  //Get byte from payload
	void addToChecksum(uint8_t incoming);	  //Given an incoming byte, adjust rollingChecksumA/B

	//Variables
	TwoWire *_i2cPort;				//The generic connection to user's chosen I2C hardware
	Stream *_serialPort;			//The generic connection to user's chosen Serial hardware
	Stream *_nmeaOutputPort = NULL; //The user can assign an output port to print NMEA sentences if they wish
	Stream *_debugSerial;			//The stream to send debug messages to if enabled

	uint8_t _gpsI2Caddress = 0x42; //Default 7-bit unshifted address of the ublox 6/7/8/M8/F9 series
	//This can be changed using the ublox configuration software

	boolean _printDebug = false; //Flag to print the serial commands we are sending to the Serial port for debug

	//These are pointed at from within the ubxPacket
	uint8_t payloadAck[2];
	uint8_t payloadCfg[MAX_PAYLOAD_SIZE];

	//Init the packet structures and init them with pointers to the payloadAck and payloadCfg arrays
	ubxPacket packetAck = {0, 0, 0, 0, 0, payloadAck, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
	ubxPacket packetCfg = {0, 0, 0, 0, 0, payloadCfg, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

	//Limit checking of new data to every X ms
	//If we are expecting an update every X Hz then we should check every half that amount of time
	//Otherwise we may block ourselves from seeing new data
	uint8_t i2cPollingWait = 100; //Default to 100ms. Adjusted when user calls setNavigationFrequency()

	unsigned long lastCheck = 0;
	boolean autoPVT = false;			  //Whether autoPVT is enabled or not
	boolean autoPVTImplicitUpdate = true; // Whether autoPVT is triggered by accessing stale data (=true) or by a call to checkUblox (=false)
	uint8_t commandAck = UBX_ACK_NONE;	//This goes to UBX_ACK_ACK after we send a command and it's ack'd
	uint16_t ubxFrameCounter;			  //It counts all UBX frame. [Fixed header(2bytes), CLS(1byte), ID(1byte), length(2bytes), payload(x bytes), checksums(2bytes)]

	uint8_t rollingChecksumA; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
	uint8_t rollingChecksumB; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes

	//Create bit field for staleness of each datum in PVT we want to monitor
	//moduleQueried.latitude goes true each time we call getPVT()
	//This reduces the number of times we have to call getPVT as this can take up to ~1s per read
	//depending on update rate
	struct
	{
		uint32_t gpsiTOW : 1;
		uint32_t gpsYear : 1;
		uint32_t gpsMonth : 1;
		uint32_t gpsDay : 1;
		uint32_t gpsHour : 1;
		uint32_t gpsMinute : 1;
		uint32_t gpsSecond : 1;
		uint32_t gpsNanosecond : 1;

		uint32_t all : 1;
		uint32_t longitude : 1;
		uint32_t latitude : 1;
		uint32_t altitude : 1;
		uint32_t altitudeMSL : 1;
		uint32_t SIV : 1;
		uint32_t fixType : 1;
		uint32_t carrierSolution : 1;
		uint32_t groundSpeed : 1;
		uint32_t headingOfMotion : 1;
		uint32_t pDOP : 1;
		uint32_t versionNumber : 1;
	} moduleQueried;

	struct
	{
		uint16_t all : 1;
		uint16_t timeOfWeek : 1;
		uint16_t highResLatitude : 1;
		uint16_t highResLongitude : 1;
		uint16_t elipsoid : 1;
		uint16_t meanSeaLevel : 1;
		uint16_t geoidSeparation : 1;
		uint16_t horizontalAccuracy : 1;
		uint16_t verticalAccuracy : 1;
	} highResModuleQueried;

	uint16_t rtcmLen = 0;
};

#endif
