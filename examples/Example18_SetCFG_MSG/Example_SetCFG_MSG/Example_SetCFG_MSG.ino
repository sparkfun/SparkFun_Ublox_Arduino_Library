
#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

unsigned long lastGPSSend = 0;

void setup()
{  
  Serial.begin(9600);     // Serial debug output over USB visible from Arduino IDE
  Serial1.begin(9600);    // NMEA serial UART output port to SPP bluetooth device such as HC-06

  Wire.begin();  
  
  if (myGPS.begin() == false)
  {    
    Serial.println(F("Error initializing GPS"));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_NMEA | COM_TYPE_UBX);
  myGPS.setCFG_MSG(UBX_CLASS_NAV, UBX_NAV_PVT, 0);    //Some of the other examples enable this message
  myGPS.setCFG_MSG(UBX_CLASS_NMEA, UBX_NMEA_GLL, 0);  //Several of these are on by default on virgin ublox board
  myGPS.setCFG_MSG(UBX_CLASS_NMEA, UBX_NMEA_GSA, 0);
  myGPS.setCFG_MSG(UBX_CLASS_NMEA, UBX_NMEA_GSV, 0);
  myGPS.setCFG_MSG(UBX_CLASS_NMEA, UBX_NMEA_RMC, 0);
  myGPS.setCFG_MSG(UBX_CLASS_NMEA, UBX_NMEA_GGA, 1);  //Only leaving GGA/VTG enabled at current navigation rate
  myGPS.setCFG_MSG(UBX_CLASS_NMEA, UBX_NMEA_VTG, 1);
  
  myGPS.setNMEAOutputPort(Serial1);  
}

void loop()
{  
  if (millis() - lastGPSSend > 200){
    myGPS.checkUblox(); //See if new data is available, NMEA message will be auto-forwarded out Serial1
    lastGPSSend = millis();
  }  
}
