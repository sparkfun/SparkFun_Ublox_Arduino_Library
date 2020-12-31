How I2C (aka DDC) communication works with a u-blox module
===========================================================

When the user calls one of the methods the library will poll the u-blox module for new data.

* Wait for a minimum of 25 ms between polls (configured dynamically when update rate is set)
* Write 0xFD to module
* Read two bytes (0xFD and 0xFE) for bytes available
* If 0x7F or 0xFF then no bytes are available
* Otherwise, read number of bytes and process into NMEA, UBX, or RTCM frame.
* If checksum is valid, flag frame as complete.

This library was originally written to use the I2C interface but Serial has been implemented as well.

How data is processed by this library
===========================================================

A method will call **sendCommand()**. This will begin waiting for a response with either **waitForACKResponse()** or **waitForNoACKResponse()** depending on the command we have sent (CFG commands generate an ACK where others like PVT do not).

Once **waitForACKResponse()** or **waitForNoACKResponse()** is called the library will start checking the u-blox module for new bytes. These bytes may be part of a NMEA sentence, an RTCM sentence, or a UBX packet. The library will file each byte into the appropriate container. Once a given sentence or packet is complete, the appropriate processUBX(), processNMEA() will be called. These functions deal with specific processing for each type.

Note: When interfacing to a u-blox module over I2C **checkUbloxI2C()** will read all bytes currently sitting in the I2C buffer. This may pick up multiple UBX packets. For example, an ACK for a VALSET may be mixed in with an **AutoPVT** response. We cannot tell **checkUbloxI2C()** to stop once a given ACK is found because we run the risk of leaving unprocessed bytes in the I2C buffer and losing them. We don't have this issue with **checkUbloxSerial()**.

**processUBX()** will check the CRC of the UBX packet. If validated, the packet will be marked as valid. Once a packet is marked as valid then **processUBXpacket()** is called to extract the contents. This is most commonly used to get the position, velocity, and time (PVT) out of the packet but is also used to check the nature of an ACK packet.

Once a packet has been processed, **waitForACKResponse()/waitForNoACKResponse()** makes the appropriate decision what to do with it. If a packet satisfies the CLS/ID and characteristics of what **waitForACKResponse()/waitForNoACKResponse()** is waiting for, then it returns back to **sendCommand()**. If the packet didn't match or was invalid then **waitForACKResponse()/waitForNoACKResponse()** will continue to wait until the correct packet is received or we time out. **sendCommand()** then returns with a value from the **sfe_ublox_status_e** enum depending on the success of **waitForACKResponse()/waitForNoACKResponse()**.

If we are getting / polling data from the module, **sendCommand()** will return **SFE_UBLOX_STATUS_DATA_RECEIVED** if the get was successful.

If we are setting / writing data to the module, **sendCommand()** will return **SFE_UBLOX_STATUS_DATA_SENT** if the set was successful.

We are proud that this library still compiles and runs on the original RedBoard (ATmega328P). We achieve that by being very careful about how much RAM we allocate to packet storage. We use only three buffers or containers to store the incoming data:
- **packetBuf** (packetBuffer) - is small and is used to store only the head (and tail) of incoming UBX packets until we know they are. If the packet is _expected_ (i.e. it matches the Class and ID in the packet passed in **sendCommand()**) then the incoming bytes are diverted into **packetCfg** or **packetAck**. Unexpected packets are ignored.
- **packetCfg** (packetConfiguration) - is used to store an _expected_ incoming UBX packet of up to 256 bytes. E.g. **getProtocolVersion()** returns about 220 bytes. Message data requested by a higher function is returned in packetCfg.
- **packetAck** (packetAcknowledge) - is small and is used to store the ACK or NACK accompanying any _expected_ packetCfg.

**AutoPVT**, **AutoHPPOSLLH** and **AutoDOP** packets can arrive at any time. They too _have_ to be stored and processed in **packetCfg**. This means there are circumstances where the library can get the data it is expecting from the module, but it is overwritten (e.g. by an **AutoPVT** packet) before **sendCommand()** is able to return. In this case, **sendCommand()** will return the error **SFE_UBLOX_STATUS_DATA_OVERWRITTEN**. We should simply call the library function again, but we will need to reset the packet contents first as they will indeed have been overwritten as the error implies.

Need a command that is not currently "built-in" to the library? You can do that using a Custom Command. Check out [Example20_SendCustomCommand](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/blob/master/examples/Example20_SendCustomCommand/Example20_SendCustomCommand.ino) for further details. Note: this will of course increase your RAM use.
