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

const uint32_t UBLOX_CFG_I2C_ADDRESS = 0x20510001;
const uint32_t UBLOX_CFG_I2C_ENABLED = 0x10510003;

const uint32_t UBLOX_CFG_I2CINPROT_UBX = 0x10710001;
const uint32_t UBLOX_CFG_I2CINPROT_NMEA = 0x10710002;
const uint32_t UBLOX_CFG_I2CINPROT_RTCM3X = 0x10710004;

const uint32_t UBLOX_CFG_I2COUTPROT_UBX = 0x10720001;
const uint32_t UBLOX_CFG_I2COUTPROT_NMEA = 0x10720002;
const uint32_t UBLOX_CFG_I2COUTPROT_RTCM3X = 0x10720004;

const uint32_t UBLOX_CFG_UART1INPROT_UBX = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART1INPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_UBX << (8 * 0)); //0x10730001
const uint32_t UBLOX_CFG_UART1INPROT_NMEA = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART1INPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_NMEA << (8 * 0));
const uint32_t UBLOX_CFG_UART1INPROT_RTCM3X = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART1INPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_RTCM3 << (8 * 0));

const uint32_t UBLOX_CFG_UART1OUTPROT_UBX = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART1OUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_UBX << (8 * 0));
const uint32_t UBLOX_CFG_UART1OUTPROT_NMEA = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART1OUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_NMEA << (8 * 0));
const uint32_t UBLOX_CFG_UART1OUTPROT_RTCM3X = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART1OUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_RTCM3 << (8 * 0));

const uint32_t UBLOX_CFG_UART2INPROT_UBX = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART2INPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_UBX << (8 * 0));
const uint32_t UBLOX_CFG_UART2INPROT_NMEA = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART2INPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_NMEA << (8 * 0));
const uint32_t UBLOX_CFG_UART2INPROT_RTCM3X = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART2INPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_RTCM3 << (8 * 0));

const uint32_t UBLOX_CFG_UART2OUTPROT_UBX = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART2OUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_UBX << (8 * 0));
const uint32_t UBLOX_CFG_UART2OUTPROT_NMEA = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART2OUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_NMEA << (8 * 0));
const uint32_t UBLOX_CFG_UART2OUTPROT_RTCM3X = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_UART2OUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_RTCM3 << (8 * 0));

const uint32_t UBLOX_CFG_USBINPROT_UBX = 0x10770001;
const uint32_t UBLOX_CFG_USBINPROT_NMEA = 0x10770002;
const uint32_t UBLOX_CFG_USBINPROT_RTCM3X = 0x10770004;

const uint32_t UBLOX_CFG_USBOUTPROT_UBX = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_USBOUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_UBX << (8 * 0));
const uint32_t UBLOX_CFG_USBOUTPROT_NMEA = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_USBOUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_NMEA << (8 * 0));
const uint32_t UBLOX_CFG_USBOUTPROT_RTCM3X = ((VAL_GROUP_UART_SIZE << 4) << (8 * 3)) | (VAL_GROUP_USBOUTPROT << (8 * 2)) | (0x00 << (8 * 1)) | (VAL_ID_PROT_RTCM3 << (8 * 0));
