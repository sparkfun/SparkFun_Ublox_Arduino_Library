How I2C library works
===========================================================

When the user calls one of the methods the library will poll the Ublox module for new data. 

* Wait for a minimum of 25 ms between polls (configured dynamically when update rate is set)
* Write 0xFD to module
* Read two bytes (0xFD and 0xFE) for bytes available
* If 0x7F or 0xFF then no bytes are available
* Otherwise, read number of bytes and process into NMEA, UBX, or RTCM frame.
* If checksum is valid, flag frame as complete.


