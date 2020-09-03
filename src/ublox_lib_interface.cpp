#include "ublox_lib_interface.h"

#include "SparkFun_Ublox_Zephyr_Library.h"


SFE_UBLOX_GPS myGPS;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

uint8_t set_gpio_dev(struct device *gpio_dev) {
    if (myGPS.init_gpio_pins(*gpio_dev) == false) {
        return -1;
    }

	// This will pipe all NMEA sentences to the serial port so we can see them
	//myGPS.setNMEAOutputPort();    // uncomment this to see raw prints
    // turn on debugging
    myGPS.enableDebugging();
    return 0;
}

uint8_t gps_begin(struct device *i2c_dev) {
    if (myGPS.begin(*i2c_dev) == false) {
        return -1;
    }

    myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGPS.saveConfiguration(); //Save the current settings to flash and BBR

    return 0;
}

void check_ublox(void) {
    myGPS.checkUblox();
}

void get_position(void)
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (k_uptime_get_32() - lastTime > 1000)
  {
    lastTime = k_uptime_get_32(); //Update the timer

    long latitude = myGPS.getLatitude();
    long longitude = myGPS.getLongitude();
    long altitude = myGPS.getAltitude();

    uint8_t SIV = myGPS.getSIV();

    printk("Position: Lat: %ld, Lon: %ld, Alt: %ld, SIV: %d\n", latitude, longitude, altitude, SIV);
  }
}

void get_datetime(void) {
    int year = myGPS.getYear();
    int month = myGPS.getMonth();
    int day = myGPS.getDay();
    int hour = myGPS.getHour();
    int minute = myGPS.getMinute();
    int second = myGPS.getSecond();

    printk("DateTime: %d-%d-%d %d:%d:%d\n", year, month, day, hour, minute, second);

    printk("Time is ");
    if (myGPS.getTimeValid() == false)
    {
      printk("not ");
    }
    printk("valid. Date is ");
    if (myGPS.getDateValid() == false)
    {
      printk("not ");
    }
    printk("valid.\n");
}