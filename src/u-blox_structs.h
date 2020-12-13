/*
	This is a library written for the u-blox ZED-F9P and NEO-M8P-2
	SparkFun sells these at its website: www.sparkfun.com
	Do you like this library? Help support SparkFun. Buy a board!
	https://www.sparkfun.com/products/16481
	https://www.sparkfun.com/products/15136
	https://www.sparkfun.com/products/15005
	https://www.sparkfun.com/products/15733
	https://www.sparkfun.com/products/15193
	https://www.sparkfun.com/products/15210

	Written by Nathan Seidle @ SparkFun Electronics, September 6th, 2018

	This library handles configuring and handling the responses
	from a u-blox GPS module. Works with most modules from u-blox including
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

#ifndef __u_blox_structs_h__
#define __u_blox_structs_h__

//-=-=-=-=- UBX binary specific variables
typedef struct
{
	uint8_t cls;
	uint8_t id;
	uint16_t len;		   //Length of the payload. Does not include cls, id, or checksum bytes
	uint16_t counter;	   //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
	uint16_t startingSpot; //The counter value needed to go past before we begin recording into payload array
	uint8_t *payload;
	uint8_t checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
	uint8_t checksumB;
	sfe_ublox_packet_validity_e valid;			 //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
	sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
} ubxPacket;

//Additional flags and pointers that need to be stored with each message type
typedef struct
{
  boolean auto; // Will this message be delivered and parsed "automatically" (without polling)
  boolean implicitUpdate; // Is the update triggered by accessing stale data (=true) or by a call to checkUblox (=false)
  boolean addToFileBuffer; // Should the raw UBX data be added to the file buffer?
} ubxPacketExtras;

// UBX-NAV-DOP (0x01 0x04): Dilution of precision
const uint16_t UBX_NAV_DOP_LEN = 18;
typedef struct
{
  ubxPacket header;
  ubxPacketExtras extras;
  struct
  {
    uint32_t iTOW; // GPS time of week of the navigation epoch: ms
    uint16_t gDOP; // Geometric DOP: * 0.01
    uint16_t pDOP; // Position DOP: * 0.01
    uint16_t tDOP; // Time DOP: * 0.01
    uint16_t vDOP; // Vertical DOP: * 0.01
    uint16_t hDOP; // Horizontal DOP: * 0.01
    uint16_t nDOP; // Northing DOP: * 0.01
    uint16_t eDOP; // Easting DOP: * 0.01
  } data;
	struct
  {
		uint32_t all : 1;

    uint32_t iTOW : 1;
    uint32_t gDOP : 1;
    uint32_t pDOP : 1;
    uint32_t tDOP : 1;
    uint32_t vDOP : 1;
    uint32_t hDOP : 1;
    uint32_t nDOP : 1;
    uint32_t eDOP : 1;
  } moduleQueried;
} UBX_NAV_DOP_t;

// UBX-NAV-PVT (0x01 0x07): Navigation position velocity time solution
const uint16_t UBX_NAV_PVT_LEN = 92;
typedef struct
{
  ubxPacket header;
  ubxPacketExtras extras;
  struct
  {
    uint32_t iTOW; // GPS time of week of the navigation epoch: ms
    uint16_t year; // Year (UTC)
    uint8_t month; // Month, range 1..12 (UTC)
    uint8_t day; // Day of month, range 1..31 (UTC)
    uint8_t hour; // Hour of day, range 0..23 (UTC)
    uint8_t min; // Minute of hour, range 0..59 (UTC)
    uint8_t sec; // Seconds of minute, range 0..60 (UTC)
    struct
    {
      uint8_t validDate : 1; // 1 = valid UTC Date
      uint8_t validTime : 1; // 1 = valid UTC time of day
      uint8_t fullyResolved : 1; // 1 = UTC time of day has been fully resolved (no seconds uncertainty).
      uint8_t validMag : 1; // 1 = valid magnetic declination
    } valid;
    uint32_t tAcc; // Time accuracy estimate (UTC): ns
    int32_t nano; // Fraction of second, range -1e9 .. 1e9 (UTC): ns
    uint8_t fixType; // GNSSfix Type:
                        // 0: no fix
                        // 1: dead reckoning only
                        // 2: 2D-fix
                        // 3: 3D-fix
                        // 4: GNSS + dead reckoning combined
                        // 5: time only fix
    struct
    {
      uint8_t gnssFixOK : 1; // 1 = valid fix (i.e within DOP & accuracy masks)
      uint8_t diffSoln : 1; // 1 = differential corrections were applied
      uint8_t psmState : 3;
      uint8_t headVehValid : 1; // 1 = heading of vehicle is valid, only set if the receiver is in sensor fusion mode
      uint8_t carrSoln : 2; // Carrier phase range solution status:
                              // 0: no carrier phase range solution
                              // 1: carrier phase range solution with floating ambiguities
                              // 2: carrier phase range solution with fixed ambiguities
    } flags;
    struct
    {
      uint8_t reserved : 5;
      uint8_t confirmedAvai : 1; // 1 = information about UTC Date and Time of Day validity confirmation is available
      uint8_t confirmedDate : 1; // 1 = UTC Date validity could be confirmed
      uint8_t confirmedTime : 1; // 1 = UTC Time of Day could be confirmed
    } flags2;
    uint8_t numSV; // Number of satellites used in Nav Solution
    int32_t lon; // Longitude: deg * 1e-7
    int32_t lat; // Latitude: deg * 1e-7
    int32_t height; // Height above ellipsoid: mm
    int32_t hMSL; // Height above mean sea level: mm
    uint32_t hAcc; // Horizontal accuracy estimate: mm
    uint32_t vAcc; // Vertical accuracy estimate: mm
    int32_t velN; // NED north velocity: mm/s
    int32_t velE; // NED east velocity: mm/s
    int32_t velD; // NED down velocity: mm/s
    int32_t gSpeed; // Ground Speed (2-D): mm/s
    int32_t headMot; // Heading of motion (2-D): deg * 1e-5
    uint32_t sAcc; // Speed accuracy estimate: mm/s
    uint32_t headAcc; // Heading accuracy estimate (both motion and vehicle): deg * 1e-5
    uint16_t pDOP; // Position DOP * 0.01
    struct
    {
      uint8_t invalidLlh : 1; // 1 = Invalid lon, lat, height and hMSL
    } flags3;
    uint8_t reserved1[5];
    int32_t headVeh; // Heading of vehicle (2-D): deg * 1e-5
    int16_t magDec; // Magnetic declination: deg * 1e-2
    uint16_t magAcc; // Magnetic declination accuracy: deg * 1e-2
  } data;
  struct
	{
		uint32_t all : 1;

		uint32_t iTOW : 1;
		uint32_t year : 1;
		uint32_t month : 1;
		uint32_t day : 1;
		uint32_t hour : 1;
		uint32_t min : 1;
		uint32_t sec : 1;

		uint32_t validDate : 1;
		uint32_t validTime : 1;
		uint32_t fullyResolved : 1;
		uint32_t validMag : 1;

		uint32_t tAcc : 1;
		uint32_t nano : 1;
		uint32_t fixType : 1;
		uint32_t gnssFixOk : 1;
		uint32_t diffSoln : 1;
		uint32_t psmState : 1;
		uint32_t headVehValid : 1;
		uint32_t carrSoln : 1;

		uint32_t confirmedAvai : 1;
		uint32_t confirmedDate : 1;
		uint32_t confirmedTime : 1;

		uint32_t numSV : 1;
    uint32_t lon : 1;
    uint32_t lat : 1;
    uint32_t height : 1;
    uint32_t hMSL : 1;
    uint32_t hAcc : 1;
    uint32_t vAcc : 1;
    uint32_t velN : 1;
    uint32_t velE : 1;
	}  moduleQueried1;
	struct
	{
    uint32_t velD : 1;
    uint32_t gSpeed : 1;
    uint32_t headMot : 1;
    uint32_t sAcc : 1;
    uint32_t headAcc : 1;
    uint32_t pDOP : 1;
    uint32_t invalidLlh : 1;
    uint32_t headVeh : 1;
    uint32_t magDec : 1;
    uint32_t magAcc : 1;
	} moduleQueried2;
} UBX_NAV_PVT_t;

// UBX-NAV-HPPOSLLH (0x01 0x14): High precision geodetic position solution
const uint16_t UBX_NAV_HPPOSLLH_LEN = 36;
typedef struct
{
  ubxPacket header;
  ubxPacketExtras extras;
  struct
  {
    uint8_t version; // Message version (0x00 for this version)
    uint8_t reserved1[2];
    struct
    {
      uint8_t invalidLlh : 1; // 1 = Invalid lon, lat, height, hMSL, lonHp, latHp, heightHp and hMSLHp
    } flags;
    uint32_t iTOW; // GPS time of week of the navigation epoch: ms
    int32_t lon; // Longitude: deg * 1e-7
    int32_t lat; // Latitude: deg * 1e-7
    int32_t height; // Height above ellipsoid: mm
    int32_t hMSL; // Height above mean sea level: mm
    int8_t lonHp; // High precision component of longitude: deg * 1e-9
    int8_t latHp; // High precision component of latitude: deg * 1e-9
    int8_t heightHp; // High precision component of height above ellipsoid: mm * 0.1
    int8_t hMSLHp; // High precision component of height above mean sea level: mm * 0.1
    uint32_t hAcc; // Horizontal accuracy estimate: mm * 0.1
    uint32_t vAcc; // Vertical accuracy estimate: mm * 0.1
  } data;
	struct
  {
		uint32_t all : 1;

    uint32_t version : 1;

    uint32_t invalidLlh : 1;

    uint32_t iTOW : 1;
    uint32_t lon : 1;
    uint32_t lat : 1;
    uint32_t height : 1;
    uint32_t hMSL : 1;
    uint32_t lonHp : 1;
    uint32_t latHp : 1;
    uint32_t heightHp : 1;
    uint32_t hMSLHp : 1;
    uint32_t hAcc : 1;
    uint32_t vAcc : 1;
  } moduleQueried;
} UBX_NAV_HPPOSLLH_t;

// UBX-NAV-SVIN (0x01 0x3B): Survey-in data
const uint16_t UBX_NAV_SVIN_LEN = 40;
typedef struct
{
  ubxPacket header;
  ubxPacketExtras extras;
  struct
  {
    uint8_t version; // Message version (0x00 for this version)
    uint8_t reserved1[3];
    uint32_t iTOW; // GPS time of week of the navigation epoch: ms
    uint32_t dur; // Passed survey-in observation time: s
    int32_t meanX; // Current survey-in mean position ECEF X coordinate: cm
    int32_t meanY; // Current survey-in mean position ECEF Y coordinate: cm
    int32_t meanZ; // Current survey-in mean position ECEF Z coordinate: cm
    int8_t meanXHP; // Current high-precision survey-in mean position ECEF X coordinate: mm * 0.1
    int8_t meanYHP; // Current high-precision survey-in mean position ECEF Y coordinate: mm * 0.1
    int8_t meanZHP; // Current high-precision survey-in mean position ECEF Z coordinate: mm * 0.1
    uint8_t reserved2;
    uint32_t meanAcc; // Current survey-in mean position accuracy: mm * 0.1
    uint32_t obs; // Number of position observations used during survey-in
    int8_t valid; // Survey-in position validity flag, 1 = valid, otherwise 0
    int8_t active; // Survey-in in progress flag, 1 = in-progress, otherwise 0
    uint8_t reserved3[2];
  } data;
	struct
  {
		uint32_t all : 1;

    uint32_t version : 1;
    uint32_t iTOW : 1;
    uint32_t dur : 1;
    uint32_t meanX : 1;
    uint32_t meanY : 1;
    uint32_t meanZ : 1;
    uint32_t meanXHP : 1;
    uint32_t meanYHP : 1;
    uint32_t meanZHP : 1;
    uint32_t meanAcc : 1;
    uint32_t obs : 1;
    uint32_t valid : 1;
    uint32_t active : 1;
  } moduleQueried;
} UBX_NAV_SVIN_t;

// UBX-NAV-RELPOSNED (0x01 0x3C): Relative positioning information in NED frame
const uint16_t UBX_NAV_RELPOSNED_LEN = 40;
typedef struct
{
  ubxPacket header;
  ubxPacketExtras extras;
  struct
  {
    uint8_t version; // Message version (0x00 for this version)
    uint8_t reserved1;
    uint16_t refStationId; // Reference Station ID
    uint32_t iTOW; // GPS time of week of the navigation epoch: ms
    int32_t relPosN; // North component of relative position vector: cm
    int32_t relPosE; // East component of relative position vector: cm
    int32_t relPosD; // Down component of relative position vector: cm
    int8_t relPosHPN; // High-precision North component of relative position vector: mm * 0.1
    int8_t relPosHPE; // High-precision East component of relative position vector: mm * 0.1
    int8_t relPosHPD; // High-precision Down component of relative position vector: mm * 0.1
    uint8_t reserved2;
    uint32_t accN; // Accuracy of relative position North component: mm * 0.1
    uint32_t accE; // Accuracy of relative position East component: mm * 0.1
    uint32_t accD; // Accuracy of relative position Down component: mm * 0.1
    struct
    {
      uint32_t gnssFixOK : 1; // A valid fix (i.e within DOP & accuracy masks)
      uint32_t diffSoln : 1; // 1 if differential corrections were applied
      uint32_t relPosValid : 1; // 1 if relative position components and accuracies are valid
      uint32_t carrSoln : 2; // Carrier phase range solution status:
                              // 0 = no carrier phase range solution
                              // 1 = carrier phase range solution with floating ambiguities
                              // 2 = carrier phase range solution with fixed ambiguities
      uint32_t isMoving : 1; // 1 if the receiver is operating in moving baseline mode
      uint32_t refPosMiss : 1; // 1 if extrapolated reference position was used to compute moving baseline solution this epoch
      uint32_t refObsMiss : 1; // 1 if extrapolated reference observations were used to compute moving baseline solution this epoch
    } flags;
  } data;
	struct
  {
		uint32_t all : 1;

    uint32_t version : 1;
    uint32_t refStationId : 1;
    uint32_t iTOW : 1;
    uint32_t relPosN : 1;
    uint32_t relPosE : 1;
    uint32_t relPosD : 1;
    uint32_t relPosHPN : 1;
    uint32_t relPosHPE : 1;
    uint32_t relPosHPD : 1;
    uint32_t accN : 1;
    uint32_t accE : 1;
    uint32_t accD

    uint32_t gnssFixOK : 1;
    uint32_t diffSoln : 1;
    uint32_t relPosValid : 1;
    uint32_t carrSoln : 1;
    uint32_t isMoving : 1;
    uint32_t refPosMiss : 1;
    uint32_t refObsMiss : 1;
  } moduleQueried;
} UBX_NAV_RELPOSNED_t;

// HNR-specific structs

// UBX-HNR-ATT (0x28 0x01): Attitude solution
typedef struct
{
  ubxPacket header;
  ubxPacketExtras extras;
  struct
  {
		uint32_t iTOW; // GPS time of week of the navigation epoch: ms
    uint8_t version;
    uint8_t reserved1[3];
    int32_t roll; // Vehicle roll: Degrees * 1e-5
    int32_t pitch; // Vehicle pitch: Degrees * 1e-5
    int32_t heading; // Vehicle heading: Degrees * 1e-5
    uint32_t accRoll; // Vehicle roll accuracy: Degrees * 1e-5
    uint32_t accPitch; // Vehicle pitch accuracy: Degrees * 1e-5
    uint32_t accHeading; // Vehicle heading accuracy: Degrees * 1e-5
  } data;
	struct
  {
		uint32_t all : 1;

    uint32_t iTOW : 1;
    uint32_t version : 1;
    uint32_t roll : 1;
    uint32_t pitch : 1;
    uint32_t heading : 1;
    uint32_t accRoll : 1;
    uint32_t accPitch : 1;
    uint32_t accHeading : 1;
  } moduleQueried;
} UBX_HNR_ATT_t;

// UBX-HNR-INS (0x28 0x02): Vehicle dynamics information
typedef struct
{
  ubxPacket header;
  ubxPacketExtras extras;
  struct
  {
    struct
    {
      uint32_t version : 8; // Message version (0x00 for this version)
      uint32_t xAngRateValid : 1; // Compensated x-axis angular rate data validity flag (0: not valid, 1: valid)
      uint32_t yAngRateValid : 1; // Compensated y-axis angular rate data validity flag (0: not valid, 1: valid)
      uint32_t zAngRateValid : 1; // Compensated z-axis angular rate data validity flag (0: not valid, 1: valid)
      uint32_t xAccelValid : 1; // Compensated x-axis acceleration data validity flag (0: not valid, 1: valid)
      uint32_t yAccelValid : 1; // Compensated y-axis acceleration data validity flag (0: not valid, 1: valid)
      uint32_t zAccelValid : 1; // Compensated z-axis acceleration data validity flag (0: not valid, 1: valid)
    } bitfield0;
    uint8_t reserved1[4]
    uint32_t iTOW; // GPS time of week of the HNR epoch: ms
    int32_t xAngRate; // Compensated x-axis angular rate: Degrees/s * 1e-3
    int32_t yAngRate; // Compensated y-axis angular rate: Degrees/s * 1e-3
    int32_t zAngRate; // Compensated z-axis angular rate: Degrees/s * 1e-3
    int32_t xAccel; // Compensated x-axis acceleration (with gravity): m/s^2 * 1e-2
    int32_t yAccel; // Compensated y-axis acceleration (with gravity): m/s^2 * 1e-2
    int32_t zAccel; // Compensated z-axis acceleration (with gravity): m/s^2 * 1e-2
  } data;
	struct
  {
		uint32_t all : 1;

    uint32_t version : 1;
    uint32_t xAngRateValid : 1;
    uint32_t yAngRateValid : 1;
    uint32_t zAngRateValid : 1;
    uint32_t xAccelValid : 1;
    uint32_t yAccelValid : 1;
    uint32_t zAccelValid : 1;

    uint32_t iTOW : 1;
    uint32_t xAngRate : 1;
    uint32_t yAngRate : 1;
    uint32_t zAngRate : 1;
    uint32_t xAccel : 1;
    uint32_t yAccel : 1;
    uint32_t zAccel : 1;
  } moduleQueried;
} UBX_HNR_INS_t;

typedef struct
{
  ubxPacket header;
  ubxPacketExtras extras;
  struct
  {
    uint32_t iTOW; // GPS time of week of the HNR epoch: ms
    uint16_t year; // Year (UTC)
    uint8_t month; // Month, range 1..12 (UTC)
    uint8_t day; // Day of month, range 1..31 (UTC)
    uint8_t hour; // Hour of day, range 0..23 (UTC)
    uint8_t min; // Minute of hour, range 0..59 (UTC)
    uint8_t sec; // Seconds of minute, range 0..60 (UTC)
    struct
    {
      uint8_t validDate : 1; // 1 = Valid UTC Date
      uint8_t validTime : 1; // 1 = Valid UTC Time of Day
      uint8_t fullyResolved : 1; // 1 = UTC Time of Day has been fully resolved
    } valid;
    int32_t nano; // Fraction of second (UTC): ns
    uint8_t gpsFix; // GPSfix Type, range 0..5
                      // 0x00 = No Fix
                      // 0x01 = Dead Reckoning only
                      // 0x02 = 2D-Fix
                      // 0x03 = 3D-Fix
                      // 0x04 = GPS + dead reckoning combined
                      // 0x05 = Time only fix
                      // 0x06..0xff: reserved
    struct
    {
      uint8_t gpsFixOK : 1; // >1 = Fix within limits (e.g. DOP & accuracy)
      uint8_t diffSoln : 1; // 1 = DGPS used
      uint8_t WKNSET : 1; // 1 = Valid GPS week number
      uint8_t TOWSET : 1; // 1 = Valid GPS time of week (iTOW & fTOW)
      uint8_t headVehValid : 1; // 1= Heading of vehicle is valid
    } flags;
    uint8_t reserved1[2];
    int32_t lon; // Longitude: Degrees * 1e-7
    int32_t lat; // Latitude: Degrees * 1e-7
    int32_t height; // Height above ellipsoid: mm
    int32_t hMSL; // Height above MSL: mm
    int32_t gSpeed; // Ground Speed (2-D): mm/s
    int32_t speed; // Speed (3-D): mm/s
    int32_t headMot; // Heading of motion (2-D): Degrees * 1e-5
    int32_t headVeh; // Heading of vehicle (2-D): Degrees * 1e-5
    uint32_t hAcc; // Horizontal accuracy: mm
    uint32_t vAcc; // Vertical accuracy: mm
    uint32_t sAcc; // Speed accuracy: mm/s
    uint32_t headAcc; // Heading accuracy: Degrees * 1e-5
  } data;
	struct
  {
		uint32_t all : 1;

    uint32_t iTOW : 1;
    uint32_t year : 1;
    uint32_t month : 1;
    uint32_t day : 1;
    uint32_t hour : 1;
    uint32_t min : 1;
    uint32_t sec : 1;

    uint32_t validDate : 1;
    uint32_t validTime : 1;
    uint32_t fullyResolved : 1;

		uint32_t nano : 1;
    uint32_t gpsFix : 1;

    uint32_t gpsFixOK : 1;
    uint32_t diffSoln : 1;
    uint32_t WKNSET : 1;
    uint32_t TOWSET : 1;
    uint32_t headVehValid : 1;

    uint32_t lon : 1;
    uint32_t lat : 1;
    uint32_t height : 1;
    uint32_t hMSL : 1;
    uint32_t gSpeed : 1;
    uint32_t speed : 1;
    uint32_t headMot : 1;
    uint32_t headVeh : 1;
    uint32_t hAcc : 1;
    uint32_t vAcc : 1;
    uint32_t sAcc : 1;
    uint32_t headAcc : 1;
  } moduleQueried;
} UBX_HNR_PVT_t;


#endif
