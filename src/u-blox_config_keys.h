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
const uint8_t VAL_LAYER_ALL = VAL_LAYER_RAM | VAL_LAYER_BBR | VAL_LAYER_FLASH; //Not valid with getVal()

//Below are various Groups, IDs, and sizes for various settings
//These can be used to call getVal/setVal/delVal
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint8_t VAL_ID_PROT_UBX = 0x01;
const uint8_t VAL_ID_PROT_NMEA = 0x02;
const uint8_t VAL_ID_PROT_RTCM3 = 0x04;

const uint8_t VAL_GROUP_I2C = 0x51;
const uint8_t VAL_GROUP_I2COUTPROT = 0x72;
const uint8_t VAL_GROUP_UART1INPROT = 0x73;
const uint8_t VAL_GROUP_UART1OUTPROT = 0x74;
const uint8_t VAL_GROUP_UART2INPROT = 0x75;
const uint8_t VAL_GROUP_UART2OUTPROT = 0x76;
const uint8_t VAL_GROUP_USBINPROT = 0x77;
const uint8_t VAL_GROUP_USBOUTPROT = 0x78;

const uint8_t VAL_GROUP_UART_SIZE = VAL_SIZE_1; //All fields in UART group are currently 1 bit
const uint8_t VAL_GROUP_I2C_SIZE = VAL_SIZE_8;  //All fields in I2C group are currently 1 byte

const uint8_t VAL_ID_I2C_ADDRESS = 0x01;

//Below are the key values for a given configuration setting

//CFG-NMEA
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_NMEA_PROTVER = 0x10930001;
const uint32_t UBLOX_CFG_NMEA_MAXSVS = 0x10930002;
const uint32_t UBLOX_CFG_NMEA_COMPAT = 0x10930003;
const uint32_t UBLOX_CFG_NMEA_CONSIDER = 0x10930004;
const uint32_t UBLOX_CFG_NMEA_LIMIT82 = 0x10930005;
const uint32_t UBLOX_CFG_NMEA_HIGHPREC = 0x10930006;
const uint32_t UBLOX_CFG_NMEA_SVNUMBERING = 0x20930007;
const uint32_t UBLOX_CFG_NMEA_FILT_GPS = 0x10930011;
const uint32_t UBLOX_CFG_NMEA_FILT_SBAS = 0x10930012;
const uint32_t UBLOX_CFG_NMEA_FILT_GAL = 0x10930013;
const uint32_t UBLOX_CFG_NMEA_FILT_QZSS = 0x10930015;
const uint32_t UBLOX_CFG_NMEA_FILT_GLO = 0x10930016;
const uint32_t UBLOX_CFG_NMEA_FILT_BDS = 0x10930017;
const uint32_t UBLOX_CFG_NMEA_OUT_INVFIX = 0x10930021;
const uint32_t UBLOX_CFG_NMEA_OUT_MSKFIX = 0x10930022;
const uint32_t UBLOX_CFG_NMEA_OUT_INVTIME = 0x10930023;
const uint32_t UBLOX_CFG_NMEA_OUT_INVDATE = 0x10930024;
const uint32_t UBLOX_CFG_NMEA_OUT_ONLYGPS = 0x10930025;
const uint32_t UBLOX_CFG_NMEA_OUT_FROZENCOG = 0x10930026;
const uint32_t UBLOX_CFG_NMEA_MAINTALKERID = 0x20930031;
const uint32_t UBLOX_CFG_NMEA_GSVTALKERID = 0x20930032;
const uint32_t UBLOX_CFG_NMEA_BDSTALKERID = 0x30930033;

//CFG-RATE
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_RATE_MEAS = 0x30210001;
const uint32_t UBLOX_CFG_RATE_NAV = 0x30210002;
const uint32_t UBLOX_CFG_RATE_TIMEREF = 0x20210003;

//CFG-I2C
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_I2C_ADDRESS = 0x20510001;
const uint32_t UBLOX_CFG_I2C_ENABLED = 0x10510003;

const uint32_t UBLOX_CFG_I2CINPROT_UBX = 0x10710001;
const uint32_t UBLOX_CFG_I2CINPROT_NMEA = 0x10710002;
const uint32_t UBLOX_CFG_I2CINPROT_RTCM3X = 0x10710004;

const uint32_t UBLOX_CFG_I2COUTPROT_UBX = 0x10720001;
const uint32_t UBLOX_CFG_I2COUTPROT_NMEA = 0x10720002;
const uint32_t UBLOX_CFG_I2COUTPROT_RTCM3X = 0x10720004;

//CFG-UART1
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_UART1_BAUDRATE = 0x40520001;
const uint32_t UBLOX_CFG_UART1_ENABLED = 0x10520005;

const uint32_t UBLOX_CFG_UART1INPROT_UBX = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART1INPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_UBX << (8 * 0)); //0x10730001
const uint32_t UBLOX_CFG_UART1INPROT_NMEA = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART1INPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_NMEA << (8 * 0));
const uint32_t UBLOX_CFG_UART1INPROT_RTCM3X = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART1INPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_RTCM3 << (8 * 0));

const uint32_t UBLOX_CFG_UART1OUTPROT_UBX = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART1OUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_UBX << (8 * 0));
const uint32_t UBLOX_CFG_UART1OUTPROT_NMEA = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART1OUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_NMEA << (8 * 0));
const uint32_t UBLOX_CFG_UART1OUTPROT_RTCM3X = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART1OUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_RTCM3 << (8 * 0));

//CFG-UART2
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_UART2_BAUDRATE = 0x40530001;
const uint32_t UBLOX_CFG_UART2_ENABLED = 0x10530005;

const uint32_t UBLOX_CFG_UART2INPROT_UBX = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART2INPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_UBX << (8 * 0));
const uint32_t UBLOX_CFG_UART2INPROT_NMEA = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART2INPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_NMEA << (8 * 0));
const uint32_t UBLOX_CFG_UART2INPROT_RTCM3X = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART2INPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_RTCM3 << (8 * 0));

const uint32_t UBLOX_CFG_UART2OUTPROT_UBX = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART2OUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_UBX << (8 * 0));
const uint32_t UBLOX_CFG_UART2OUTPROT_NMEA = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART2OUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_NMEA << (8 * 0));
const uint32_t UBLOX_CFG_UART2OUTPROT_RTCM3X = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART2OUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_RTCM3 << (8 * 0));

//CFG-USB
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_USBINPROT_UBX = 0x10770001;
const uint32_t UBLOX_CFG_USBINPROT_NMEA = 0x10770002;
const uint32_t UBLOX_CFG_USBINPROT_RTCM3X = 0x10770004;

const uint32_t UBLOX_CFG_USBOUTPROT_UBX = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_USBOUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_UBX << (8 * 0));
const uint32_t UBLOX_CFG_USBOUTPROT_NMEA = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_USBOUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_NMEA << (8 * 0));
const uint32_t UBLOX_CFG_USBOUTPROT_RTCM3X = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_USBOUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_RTCM3 << (8 * 0));

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
