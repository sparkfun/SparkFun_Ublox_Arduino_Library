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

  Modified by David Mann @ Loggerhead Instruments, 16 April 2019
  - Added support for parsing date and time
  - Added functions getYear(), getMonth(), getDay(), getHour(), getMinute(), getSecond()

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

//#elif __MK20DX256__
//Teensy

#endif

#ifndef I2C_BUFFER_LENGTH

//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32

#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Registers
const uint8_t UBX_SYNCH_1 = 0xB5;
const uint8_t UBX_SYNCH_2 = 0x62;


//The following are UBX Class IDs. Descriptions taken from ZED-F9P Interface Description Document page 32
const uint8_t UBX_CLASS_NAV = 0x01; //Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
const uint8_t UBX_CLASS_RXM = 0x02; //Receiver Manager Messages: Satellite Status, RTC Status
const uint8_t UBX_CLASS_INF = 0x04; //Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
const uint8_t UBX_CLASS_ACK = 0x05; //Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
const uint8_t UBX_CLASS_CFG = 0x06; //Configuration Input Messages: Configure the receiver.
const uint8_t UBX_CLASS_UPD = 0x09; //Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
const uint8_t UBX_CLASS_MON = 0x0A; //Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
const uint8_t UBX_CLASS_AID = 0x0B; //(NEO-M8P ONLY!!!) AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
const uint8_t UBX_CLASS_TIM = 0x0D; //Timing Messages: Time Pulse Output, Time Mark Results
const uint8_t UBX_CLASS_ESF = 0x10; //(NEO-M8P ONLY!!!) External Sensor Fusion Messages: External Sensor Measurements and Status Information
const uint8_t UBX_CLASS_MGA = 0x13; //Multiple GNSS Assistance Messages: Assistance data for various GNSS
const uint8_t UBX_CLASS_LOG = 0x21; //Logging Messages: Log creation, deletion, info and retrieval
const uint8_t UBX_CLASS_SEC = 0x27; //Security Feature Messages
const uint8_t UBX_CLASS_HNR = 0x28; //(NEO-M8P ONLY!!!) High Rate Navigation Results Messages: High rate time, position speed, heading

//The following are used for configuration. Descriptions are from the ZED-F9P Interface Description pg 33-34
const uint8_t UBX_CFG_ANT = 0x13; //Antenna Control Settings
const uint8_t UBX_CFG_CFG = 0x09; //Clear, Save, and Load Configurations
const uint8_t UBX_CFG_DAT = 0x06; //Depending on packet length, either: Set User-defined Datum or The currently defined Datum
const uint8_t UBX_CFG_DGNSS = 0x70; //DGNSS configuration
const uint8_t UBX_CFG_GEOFENCE = 0x69; //Geofencing configuration
const uint8_t UBX_CFG_GNSS = 0x3E; //GNSS system configuration
const uint8_t UBX_CFG_INF = 0x02; //Depending on packet length, either: poll configuration for one protocol, or information message configuration
const uint8_t UBX_CFG_ITFM = 0x39; //Jamming/Interference Monitor...
const uint8_t UBX_CFG_LOGFILTER = 0x47; //Data Logger Configuration
const uint8_t UBX_CFG_MSG = 0x01; //Depending on packet length, either: poll a message configuration, or Set Message Rate(s), or Set Message Rate
const uint8_t UBX_CFG_NAV5 = 0x24; //Navigation Engine Settings
const uint8_t UBX_CFG_NAVX5 = 0x23; //Navigation Engine Expert Settings
const uint8_t UBX_CFG_NMEA = 0x17; //Extended NMEA protocol configuration V1
const uint8_t UBX_CFG_ODO = 0x1E; //Odometer, Low-speed COG Engine...
const uint8_t UBX_CFG_PRT = 0x00; //Depending on packet length, either: polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
const uint8_t UBX_CFG_PWR = 0x57; //Put receiver in a defined power state
const uint8_t UBX_CFG_RATE = 0x08; //Navigation/Measurement Rate Settings
const uint8_t UBX_CFG_RINV = 0x34; //Contents of Remote Inventory
const uint8_t UBX_CFG_RST = 0x04; //Reset Receiver / Clear Backup Data...
const uint8_t UBX_CFG_TMODE3 = 0x71; //Time Mode Settings 3
const uint8_t UBX_CFG_TP5 = 0x31; //Time Pulse Parameters
const uint8_t UBX_CFG_USB = 0x1B; //USB Configuration
const uint8_t UBX_CFG_VALDEL = 0x8C; //Deletes values corresponding to...
const uint8_t UBX_CFG_VALGET = 0x8B; //Configuration Items
const uint8_t UBX_CFG_VALSET = 0x8A; //Sets values corresponding to provided...


const uint8_t SVIN_MODE_DISABLE = 0x00;
const uint8_t SVIN_MODE_ENABLE = 0x01;

//The following are used to enable RTCM messages
const uint8_t UBX_RTCM_MSB = 0xF5;  //All RTCM enable commands have 0xF5 as MSB
const uint8_t UBX_RTCM_1005 = 0x05; //Stationary RTK reference ARP
const uint8_t UBX_RTCM_1074 = 0x4A; //GPS MSM4
const uint8_t UBX_RTCM_1077 = 0x4D; //GPS MSM7
const uint8_t UBX_RTCM_1084 = 0x54; //GLONASS MSM4
const uint8_t UBX_RTCM_1087 = 0x57; //GLONASS MSM7
const uint8_t UBX_RTCM_1094 = 0x5E; //Galileo MSM4
const uint8_t UBX_RTCM_1124 = 0x7C; //BeiDou MSM4
const uint8_t UBX_RTCM_1230 = 0xE6; //GLONASS code-phase biases, set to once every 10 seconds

//The following are used to enable NMEA messages. Descriptions come from the NMEA messages overview in the ZED-F9P Interface Description
const uint8_t UBX_NMEA_MSB = 0xF0; //All NMEA enable commands have 0xF0 as MSB
const uint8_t UBX_NMEA_DTM = 0x0A; //xxDTM (datum reference)
const uint8_t UBX_NMEA_GAQ = 0x45; //xxGAQ (poll a standard message (if the current talker ID is GA))
const uint8_t UBX_NMEA_GBQ = 0x44; //xxGBQ (poll a standard message (if the current Talker ID is GB))
const uint8_t UBX_NMEA_GBS = 0x09; //xxGBS (GNSS satellite fault detection)
const uint8_t UBX_NMEA_GGA = 0x00; //xxGGA (Global positioning system fix data)
const uint8_t UBX_NMEA_GLL = 0x01; //xxGLL (latitude and long, whith time of position fix and status)
const uint8_t UBX_NMEA_GLQ = 0x43; //xxGLQ (poll a standard message (if the current Talker ID is GL))
const uint8_t UBX_NMEA_GNQ = 0x42; //xxGNQ (poll a standard message (if the current Talker ID is GN))
const uint8_t UBX_NMEA_GNS = 0x0D; //xxGNS (GNSS fix data)
const uint8_t UBX_NMEA_GPQ = 0x040; //xxGPQ (poll a standard message (if the current Talker ID is GP))
const uint8_t UBX_NMEA_GRS = 0x06; //xxGRS (GNSS range residuals)
const uint8_t UBX_NMEA_GSA = 0x02; //xxGSA (GNSS DOP and Active satellites)
const uint8_t UBX_NMEA_GST = 0x07; //xxGST (GNSS Pseudo Range Error Statistics)
const uint8_t UBX_NMEA_GSV = 0x03; //xxGSV (GNSS satellites in view)
const uint8_t UBX_NMEA_RMC = 0x04; //xxRMC (Recommended minimum data)
const uint8_t UBX_NMEA_TXT = 0x41; //xxTXT (text transmission)
const uint8_t UBX_NMEA_VLW = 0x0F; //xxVLW (dual ground/water distance)
const uint8_t UBX_NMEA_VTG = 0x05; //xxVTG (course over ground and Ground speed)
const uint8_t UBX_NMEA_ZDA = 0x08; //xxZDA (Time and Date)

//The following are used to configure the NMEA protocol main talker ID and GSV talker ID
const uint8_t UBX_NMEA_MAINTALKERID_NOTOVERRIDDEN = 0x00; //main talker ID is system dependent
const uint8_t UBX_NMEA_MAINTALKERID_GP = 0x01; //main talker ID is GPS
const uint8_t UBX_NMEA_MAINTALKERID_GL = 0x02; //main talker ID is GLONASS
const uint8_t UBX_NMEA_MAINTALKERID_GN = 0x03; //main talker ID is combined receiver
const uint8_t UBX_NMEA_MAINTALKERID_GA = 0x04; //main talker ID is Galileo
const uint8_t UBX_NMEA_MAINTALKERID_GB = 0x05; //main talker ID is BeiDou
const uint8_t UBX_NMEA_GSVTALKERID_GNSS = 0x00; //GNSS specific Talker ID (as defined by NMEA)
const uint8_t UBX_NMEA_GSVTALKERID_MAIN = 0x01; //use the main Talker ID

//The following are used to configure INF UBX messages (information messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const uint8_t UBX_INF_CLASS = 0x04; //All INF messages have 0x04 as the class
const uint8_t UBX_INF_DEBUG = 0x04; //ASCII output with debug contents
const uint8_t UBX_INF_ERROR = 0x00; //ASCII output with error contents
const uint8_t UBX_INF_NOTICE = 0x02; //ASCII output with informational contents
const uint8_t UBX_INF_TEST = 0x03; //ASCII output with test contents
const uint8_t UBX_INF_WARNING = 0x01; //ASCII output with warning contents

//The following are used to configure LOG UBX messages (loggings messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const uint8_t UBX_LOG_CREATE = 0x07; //Create Log File
const uint8_t UBX_LOG_ERASE = 0x03; //Erase Logged Data
const uint8_t UBX_LOG_FINDTIME = 0x0E; //Depending on packet length, either: Find index of a log entry based on a..., or response to FINDTIME requested
const uint8_t UBX_LOG_INFO = 0x08; //Depending on packet length, either: Poll for log information, or Log information
const uint8_t UBX_LOG_RETRIEVEPOSEXTRA = 0x0F; //Odometer log entry
const uint8_t UBX_LOG_RETRIEVEPOS = 0x0B; //Position fix log entry
const uint8_t UBX_LOG_RETRIEVESTRING = 0x0D; //Byte string log entry
const uint8_t UBX_LOG_RETRIEVE = 0x09; //Request log data
const uint8_t UBX_LOG_STRING = 0x04; //Store arbitrary string on on-board flash

//The following are used to configure MGA UBX messages (Multiple GNSS Assistance Messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const uint8_t UBX_MGA_ACK_DATA0 = 0x60;  //Multiple GNSS Acknowledge message
const uint8_t UBX_MGA_BDS_EPH = 0x03; //BDS Ephemeris Assistance
const uint8_t UBX_MGA_BDS_ALM = 0x03; //BDS Almanac Assistance
const uint8_t UBX_MGA_BDS_HEALTH = 0x03; //BDS Health Assistance
const uint8_t UBX_MGA_BDS_UTC = 0x03; //BDS UTC Assistance
const uint8_t UBX_MGA_BDS_IONO = 0x03; //BDS Ionospheric Assistance
const uint8_t UBX_MGA_DBD = 0x80; //Depending on packet length, either: Poll the Navigation Database, or Navigation Database Dump Entry
const uint8_t UBX_MGA_GAL_EPH = 0x02; //Galileo Ephemeris Assistance
const uint8_t UBX_MGA_GAL_ALM = 0x02; //Galileo Almanac Assitance
const uint8_t UBX_MGA_GAL_TIMOFFSET = 0x02; //Galileo GPS time offset assistance
const uint8_t UBX_MGA_GAL_UTC = 0x02; //Galileo UTC Assistance
const uint8_t UBX_MGA_GLO_EPH = 0x06; //GLONASS Ephemeris Assistance
const uint8_t UBX_MGA_GLO_ALM = 0x06; //GLONASS Almanac Assistance
const uint8_t UBX_MGA_GLO_TIMEOFFSET = 0x06; //GLONASS Auxiliary Time Offset Assistance
const uint8_t UBX_MGA_GPS_EPH = 0x00; //GPS Ephemeris Assistance
const uint8_t UBX_MGA_GPS_ALM = 0x00; //GPS Almanac Assistance
const uint8_t UBX_MGA_GPS_HEALTH = 0x00; //GPS Health Assistance
const uint8_t UBX_MGA_GPS_UTC = 0x00; //GPS UTC Assistance
const uint8_t UBX_MGA_GPS_IONO = 0x00; //GPS Ionosphere Assistance
const uint8_t UBX_MGA_INI_POS_XYZ = 0x40; //Initial Position Assistance
const uint8_t UBX_MGA_INI_POS_LLH = 0x40; //Initial Position Assitance
const uint8_t UBX_MGA_INI_TIME_UTC = 0x40; //Initial Time Assistance
const uint8_t UBX_MGA_INI_TIME_GNSS = 0x40; //Initial Time Assistance
const uint8_t UBX_MGA_INI_CLKD = 0x40; //Initial Clock Drift Assitance
const uint8_t UBX_MGA_INI_FREQ = 0x40; //Initial Frequency Assistance
const uint8_t UBX_MGA_INI_EOP = 0x40; //Earth Orientation Parameters Assistance
const uint8_t UBX_MGA_QZSS_EPH = 0x05; //QZSS Ephemeris Assistance
const uint8_t UBX_MGA_QZSS_ALM = 0x05; //QZSS Almanac Assistance
const uint8_t UBX_MGA_QZAA_HEALTH = 0x05; //QZSS Health Assistance

//The following are used to configure the MON UBX messages (monitoring messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35)
const uint8_t UBX_MON_COMMS = 0x36; //Comm port information
const uint8_t UBX_MON_GNSS = 0x28; //Information message major GNSS...
const uint8_t UBX_MON_HW2 = 0x0B; //Extended Hardware Status
const uint8_t UBX_MON_HW3 = 0x37; //HW I/O pin information
const uint8_t UBX_MON_HW = 0x09; //Hardware Status
const uint8_t UBX_MON_IO = 0x02; //I/O Subsystem Status
const uint8_t UBX_MON_MSGPP = 0x06; //Message Parse and Process Status
const uint8_t UBX_MON_PATCH = 0x27; //Output information about installed...
const uint8_t UBX_MON_RF = 0x38; //RF information
const uint8_t UBX_MON_RXBUF = 0x07; //Receiver Buffer Status
const uint8_t UBX_MON_RXR = 0x21; //Receiver Status Information
const uint8_t UBX_MON_TXBUF = 0x08; //Transmitter Buffer Status
const uint8_t UBX_MON_VER = 0x04; //Receiver/Software Version

//The following are used to configure the NAV UBX messages (navigation results messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35-36)
const uint8_t UBX_NAV_CLOCK = 0x22; //Clock Solution
const uint8_t UBX_NAV_DOP = 0x04; //Dilution of precision
const uint8_t UBX_NAV_EOE = 0x61; //End of Epoch
const uint8_t UBX_NAV_GEOFENCE = 0x39; //Geofencing status
const uint8_t UBX_NAV_HPPOSECEF = 0x13; //High Precision Position Solution in ECEF
const uint8_t UBX_NAV_HPPOSLLH = 0x14; //High Precision Geodetic Position Solution
const uint8_t UBX_NAV_ODO = 0x09; //Odometer Solution
const uint8_t UBX_NAV_ORB = 0x34; //GNSS Orbit Database Info
const uint8_t UBX_NAV_POSECEF = 0x01; //Position Solution in ECEF
const uint8_t UBX_NAV_POSLLH = 0x02; //Geodetic Position Solution
const uint8_t UBX_NAV_PVT = 0x07; //Navigation Position Velocity Time...
const uint8_t UBX_NAV_RELPOSNED = 0x3C; //Relative Positioning Information in..
const uint8_t UBX_NAV_RESETODO = 0x10; //Reset odometer
const uint8_t UBX_NAV_SAT = 0x35; //Satellite Information
const uint8_t UBX_NAV_SIG = 0x43; //Signal Information
const uint8_t UBX_NAV_STATUS = 0x03; //Receiver Navigation Status
const uint8_t UBX_NAV_SVIN = 0x3B; //Survey-in data
const uint8_t UBX_NAV_TIMEBDS = 0x24; //BDS Time Solution
const uint8_t UBX_NAV_TIMEGAL = 0x25; //Galileo Time Solution
const uint8_t UBX_NAV_TIMEGLO = 0x23; //GLO Time Solution
const uint8_t UBX_NAV_TIMEGPS = 0x20; //GPS Time Solution
const uint8_t UBX_NAV_TIMELS = 0x26; //Leap second event information
const uint8_t UBX_NAV_TIMEUTC = 0x21; //UTC Time Solution
const uint8_t UBX_NAV_VELECEF = 0x11; //Velocity Solution in ECEF
const uint8_t UBX_NAV_VELNED = 0x12; //Velocity Solution in NED

//The following are used to configure the RXM UBX messages (receiver manager messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const uint8_t UBX_RXM_MEASX = 0x14; //Satellite Measurements for RRLP
const uint8_t UBX_RXM_PMREQ = 0x41; //Requests a Power Management task (two differenent packet sizes)
const uint8_t UBX_RXM_RAWX = 0x15; //Multi-GNSS Raw Measurement Data
const uint8_t UBX_RXM_RLM = 0x59; //Galileo SAR Short-RLM report (two different packet sizes)
const uint8_t UBX_RXM_RTCM = 0x32; //RTCM input status
const uint8_t UBX_RXM_SFRBX = 0x13; //Boradcast Navigation Data Subframe

//The following are used to configure the SEC UBX messages (security feature messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const uint8_t UBX_SEC_UNIQID = 0x03; //Unique chip ID

//The following are used to configure the TIM UBX messages (timing messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const uint8_t UBX_TIM_TM2 = 0x03; //Time mark data
const uint8_t UBX_TIM_TP = 0x01; //Time Pulse Timedata
const uint8_t UBX_TIM_VRFY = 0x06; //Sourced Time Verification

//The following are used to configure the UPD UBX messages (firmware update messages). Descriptions from UBX messages overview (ZED-F9P Interface Description Document page 36)
const uint8_t UBX_UPD_SOS = 0x14; //Different based on message size (Poll Backup Fil Restore Status, Create Backup File in Flash, Clear Backup File in Flash, Backup File Creation Acknowledge, System Restored from Backup


const uint8_t UBX_ACK_NACK = 0x00;
const uint8_t UBX_ACK_ACK = 0x01;

//The following are used to set the Nav5 dynamic platform model, which provide more accuract position output for the situation. Description extracted from ZED-F9P Integration Manual
const uint8_t UBX_NAV5_DYNMODEL_PORTABLE = 0x00; //Applications with low acceleration, e.g. portable devices. Suitable for most situations.
const uint8_t UBX_NAV5_DYNMODEL_STATIONARY = 0x02; //Used in timing applications (antenna must be stationary) or other stationary applications. Velocity restricted to 0 m/s. Zero dynamics assumed.
const uint8_t UBX_NAV5_DYNMODEL_PEDESTRIAN = 0x03; //Applications with low acceleration and speed, e.g. how a pedestrian would move. Low acceleration assumed.
const uint8_t UBX_NAV5_DYNMODEL_AUTOMOTIVE = 0x04; //Used for applications with equivalent dynamics to those of a passenger car. Low vertical acceleration assumed
const uint8_t UBX_NAV5_DYNMODEL_SEA = 0x05; //Recommended for applications at sea, with zero vertical velocity. Zero vertical velocity assumed. Sea level assumed.
const uint8_t UBX_NAV5_DYNMODEL_AIRBORNE1 = 0x06; //Airborne <1g acceleration. Used for applications with a higher dynamic range and greater vertical acceleration than a passenger car. No 2D position fixes supported.
const uint8_t UBX_NAV5_DYNMODEL_AIRBORNE2 = 0x07; //Airborne <2g acceleration. Recommended for typical airborne environments. No 2D position fixes supported.
const uint8_t UBX_NAV5_DYNMODEL_AIRBORNE3 = 0x08; //Airborne <4g acceleration. Only recommended for extremely dynamic environments. No 2D position fixes supported.
const uint8_t UBX_NAV5_DYNMODEL_WRIST = 0x09; //Only recommended for wrist worn applications. Receiver will filter out arm motion.
const uint8_t UBX_NAV5_DYNMODEL_BIKE = 0x0A; //No description given in the integration manual

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

const uint8_t VAL_LAYER_RAM = 0;
const uint8_t VAL_LAYER_BBR = 1;
const uint8_t VAL_LAYER_FLASH = 2;
const uint8_t VAL_LAYER_DEFAULT = 7;

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




#ifndef MAX_PAYLOAD_SIZE

#define MAX_PAYLOAD_SIZE 64 //Some commands are larger than 64 bytes but this covers most

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
	boolean valid; //Goes true when both checksums pass
} ubxPacket;

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

	void calcChecksum(ubxPacket *msg);									//Sets the checksumA and checksumB of a given messages
	boolean sendCommand(ubxPacket outgoingUBX, uint16_t maxWait = 250); //Given a packet and payload, send everything including CRC bytes, return true if we got a response
	boolean sendI2cCommand(ubxPacket outgoingUBX, uint16_t maxWait = 250);
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

	boolean waitForResponse(uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = 250); //Poll the module until and ack is received

	boolean setAutoPVT(boolean enabled, uint16_t maxWait = 250); //Enable/disable automatic PVT reports at the navigation frequency
	boolean getPVT(uint16_t maxWait = 1000);					 //Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Retruns true if new PVT is available.

	int32_t getLatitude(uint16_t maxWait = 250);			//Returns the current latitude in degrees * 10^-7. Auto selects between HighPrecision and Regular depending on ability of module.
	int32_t getLongitude(uint16_t maxWait = 250);			//Returns the current longitude in degrees * 10-7. Auto selects between HighPrecision and Regular depending on ability of module.
	int32_t getAltitude(uint16_t maxWait = 250);			//Returns the current altitude in mm above ellipsoid
	int32_t getAltitudeMSL(uint16_t maxWait = 250);			//Returns the current altitude in mm above mean sea level
	uint8_t getSIV(uint16_t maxWait = 250);					//Returns number of sats used in fix
	uint8_t getFixType(uint16_t maxWait = 250);				//Returns the type of fix: 0=no, 3=3D, 4=GNSS+Deadreckoning
	uint8_t getCarrierSolutionType(uint16_t maxWait = 250); //Returns RTK solution: 0=no, 1=float solution, 2=fixed solution
	int32_t getGroundSpeed(uint16_t maxWait = 250);			//Returns speed in mm/s
	int32_t getHeading(uint16_t maxWait = 250);				//Returns heading in degrees * 10^-7
	uint16_t getPDOP(uint16_t maxWait = 250);				//Returns positional dillution of precision * 10^-2
	uint16_t getYear(uint16_t maxWait = 250);
	uint8_t getMonth(uint16_t maxWait = 250);
	uint8_t getDay(uint16_t maxWait = 250);
	uint8_t getHour(uint16_t maxWait = 250);
	uint8_t getMinute(uint16_t maxWait = 250);
	uint8_t getSecond(uint16_t maxWait = 250);

	//Port configurations
	boolean setPortOutput(uint8_t portID, uint8_t comSettings, uint16_t maxWait = 250); //Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
	boolean setPortInput(uint8_t portID, uint8_t comSettings, uint16_t maxWait = 250);  //Configure a given port to input UBX, NMEA, RTCM3 or a combination thereof
	boolean getPortSettings(uint8_t portID, uint16_t maxWait = 250);					//Returns the current protocol bits in the UBX-CFG-PRT command for a given port

	boolean setI2COutput(uint8_t comSettings, uint16_t maxWait = 250);   //Configure I2C port to output UBX, NMEA, RTCM3 or a combination thereof
	boolean setUART1Output(uint8_t comSettings, uint16_t maxWait = 250); //Configure UART1 port to output UBX, NMEA, RTCM3 or a combination thereof
	boolean setUART2Output(uint8_t comSettings, uint16_t maxWait = 250); //Configure UART2 port to output UBX, NMEA, RTCM3 or a combination thereof
	boolean setUSBOutput(uint8_t comSettings, uint16_t maxWait = 250);   //Configure USB port to output UBX, NMEA, RTCM3 or a combination thereof
	boolean setSPIOutput(uint8_t comSettings, uint16_t maxWait = 250);   //Configure SPI port to output UBX, NMEA, RTCM3 or a combination thereof

	//General configuration (used only on protocol v27 and higher - ie, ZED-F9P)
	uint8_t getVal8(uint16_t group, uint16_t id, uint8_t size, uint8_t layer = VAL_LAYER_BBR, uint16_t maxWait = 250); //Returns the value at a given group/id/size location
	uint8_t getVal8(uint32_t keyID, uint8_t layer = VAL_LAYER_BBR, uint16_t maxWait = 250);							   //Returns the value at a given group/id/size location
	uint8_t setVal(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_BBR, uint16_t maxWait = 250);			   //Returns the value at a given group/id/size location

	//Functions used for RTK and base station setup
	boolean getSurveyMode(uint16_t maxWait = 250);																   //Get the current TimeMode3 settings
	boolean setSurveyMode(uint8_t mode, uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = 250); //Control survey in mode
	boolean enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = 250);			   //Begin Survey-In for NEO-M8P
	boolean disableSurveyMode(uint16_t maxWait = 250);															   //Stop Survey-In mode


	boolean getSurveyStatus(uint16_t maxWait);																				  //Reads survey in status and sets the global variables
	boolean enableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint8_t secondsBetweenMessages, uint16_t maxWait = 250); //Given a message number turns on a message ID for output over given PortID
	boolean disableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint16_t maxWait = 250);								  //Turn off given RTCM message from a given port

	boolean enableNMEAmessage(uint8_t messageNumber, uint8_t portID, uint16_t maxWait = 250);  				//Used to enable a specific NMEA message out a specific port
	boolean disableNMEAmessage(uint8_t messageNumber, uint8_t portID, uint16_t maxWait = 250);				//Used to disable a specific NMEA message out a specific port
	
	boolean enableUBXmessage(uint8_t messageType, uint8_t messageNumber, uint8_t portID, uint16_t maxWait = 250);		//Used to enable specific UBX messages out a specific port
	boolean disableUBXmessage(uint8_t messageType, uint8_t messageNumber, uint8_t portID, uint16_t maxWait = 250);		//Used to disable specific UBX message out a specific port
	
	boolean setNav5DynamicPlatformModel(uint8_t dynamicModel, uint16_t maxWait = 250); //Used to set the NAV5 dynamic platform model

	boolean setNMEATalkerID(uint8_t mainTalkerId, uint8_t gsvTalkerId, uint16_t maxWait = 250); //Used to set the main Talker ID and GSV Talker ID for the GPS

	boolean setFixedMode(uint8_t mode, uint8_t pos_type, double fixed_latitude, double fixed_longitude, double fixed_altitude, double fixedPosAcc, uint16_t maxWait = 250); //Sets the GPS to Fixed mode, using the supplied values
	
	uint32_t getPositionAccuracy(uint16_t maxWait = 500); //Returns the 3D accuracy of the current high-precision fix, in mm. Supported on NEO-M8P, ZED-F9P,

	uint8_t getProtocolVersionHigh(uint16_t maxWait = 1000); //Returns the PROTVER XX.00 from UBX-MON-VER register
	uint8_t getProtocolVersionLow(uint16_t maxWait = 1000);  //Returns the PROTVER 00.XX from UBX-MON-VER register
	boolean getProtocolVersion(uint16_t maxWait = 1000);	 //Queries module, loads low/high bytes

	boolean getRELPOSNED(uint16_t maxWait = 1000); //Get Relative Positioning Information of the NED frame

	void enableDebugging(Stream &debugPort = Serial); //Given a port to print to, enable debug messages
	void disableDebugging(void);

	//Survey-in specific controls
	struct svinStructure
	{
		boolean active;
		boolean valid;
		long meanX;
		long meanY;
		long meanZ;
		int8_t meanXHP;
		int8_t meanYHP;
		int8_t meanZHP;
		uint16_t observationTime;
		float meanAccuracy;
	} svin;

	//NMEA messages port information
	struct nmeaMessagePortStructure{
		uint8_t i2c;
		uint8_t uart1;
		uint8_t uart2;
		uint8_t usb;
		uint8_t spi;
	} nmea;
	
	//UBX messages port information
	struct ubxMessagePortStructure{
		uint8_t i2c;
		uint8_t uart1;
		uint8_t uart2;
		uint8_t usb;
		uint8_t spi;
	} ubx;
	
	//NAV5 information (not currently used because of bitmask)
	struct nav5Structure{
		uint8_t dynMode1;
		uint8_t fixMode;
		int32_t fixedAlt;
		uint8_t fixedAltVar;
		int8_t minElev;
		uint8_t drLimit;
		uint16_t pDop;
		uint16_t tDop;
		uint16_t pAcc;
		uint16_t tAcc;
		uint8_t staticHoldThresh;
		uint8_t dgnssTimeout;
		uint8_t cnoThresh;
		uint16_t staticHoldMaxDist;
		uint8_t utcStandar;
	} nav5;
	
	//NMEA protocol configuration information
	struct nmeaProtocolStructure{
		int8_t filter;
		uint8_t nmeaVersion;
		uint8_t numSV;
		int8_t flags;
		int32_t gnssToFilter;
		uint8_t svNumbering;
		uint8_t mainTalkerId;
		uint8_t gsvTalkerId;
		uint8_t version;
		uint16_t bdsTalkerId;
		uint8_t reserved1;
	} nmeaProtocol;
	
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
	ubxPacket packetAck = {0, 0, 0, 0, 0, payloadAck, 0, 0, false};
	ubxPacket packetCfg = {0, 0, 0, 0, 0, payloadCfg, 0, 0, false};

	const uint8_t I2C_POLLING_WAIT_MS = 25; //Limit checking of new characters to every X ms
	unsigned long lastCheck = 0;
	boolean autoPVT = false;	//Whether autoPVT is enabled or not
	boolean commandAck = false; //This goes true after we send a command and it's ack'd
	uint8_t ubxFrameCounter;

	uint8_t rollingChecksumA; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
	uint8_t rollingChecksumB; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes

	//Create bit field for staleness of each datum in PVT we want to monitor
	//moduleQueried.latitude goes true each time we call getPVT()
	//This reduces the number of times we have to call getPVT as this can take up to ~1s per read
	//depending on update rate
	struct
	{
		uint16_t gpsYear : 1;
		uint16_t gpsMonth : 1;
		uint16_t gpsDay : 1;
		uint16_t gpsHour : 1;
		uint16_t gpsMinute : 1;
		uint16_t gpsSecond : 1;

		uint16_t all : 1;
		uint16_t longitude : 1;
		uint16_t latitude : 1;
		uint16_t altitude : 1;
		uint16_t altitudeMSL : 1;
		uint16_t SIV : 1;
		uint16_t fixType : 1;
		uint16_t carrierSolution : 1;
		uint16_t groundSpeed : 1;
		uint16_t headingOfMotion : 1;
		uint16_t pDOP : 1;
		uint16_t versionNumber : 1;
	} moduleQueried;

	uint16_t rtcmLen = 0;
};

#endif
