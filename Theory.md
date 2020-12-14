How I<sup>2</sup>C (aka DDC) communication works with a uBlox module
===========================================================

When the user calls one of the methods the library will poll the Ublox module for new data.

* Wait for a minimum of 25 ms between polls (configured dynamically when update rate is set)
* Write 0xFD to module
* Read two bytes (0xFD and 0xFE) for bytes available
* If 0x7F or 0xFF then no bytes are available
* Otherwise, read number of bytes and process into NMEA, UBX, or RTCM frame.
* If checksum is valid, flag frame as complete.

This library was originally written to use the I<sup>2</sup>C interface but Serial has been implemented as well.

How data is processed by this library
===========================================================

In Version 1 of this library, we tried to minimize memory usage by being very careful about how much RAM we allocated to UBX packet storage and processing. We used only three buffers or containers to store the incoming data: **packetBuf** (packetBuffer); **packetCfg** (packetConfiguration); and **packetAck** (packetAcknowledge). Once data was received and validated, it would be copied out of **packetCfg** and into 'global' variables with names like ```gpsSecond``` or ```latitude```. We also introduced the concept of _Polling vs. Auto-Reporting_ where messages like PVT (Position, Velocity, Time) could be generated and parsed "automatically". This meant that functions like ```getLatitude``` could be non-blocking, returning the most recent data and requesting fresh data when necessary. But it also meant that _polled_ messages could be _overwritten_ (in **packetCfg**) by any _auto-reported_ messages. The library dealt with this successfully, but it was a headache.

Version 1 had two main drawbacks. As time went on:
- the RAM use increased as we had to add new 'global' storage for each new data type
- the number of messages which needed "auto" processing through **packetCfg** became complex, requiring significant code changes each time a new "auto" message was added. (We started with NAV-PVT. Then came NAV-HPPOSLLH and NAV-DOP. Things got complicated when HNR-ATT, HNR-INS and HNR-PVT were added to the mix.)

Version 2 of the library does things differently. Whilst of course trying to keep the library backward-compatible as much as possible, we have taken a fresh approach:
- The library no longer uses 'global' (permanently-allocated) storage for the GNSS data. Instead:
  - Each message type has a **typedef struct** defined which matches the format of the UBX message. (_typedef structs_ are just definitions, they don't occupy memory.)
  - The struct allows each data field (latitude, longitude, etc.) to be read simply and easily using dot notation. Flags etc. are supported by bit definitions in the struct.
  - A variable to store that message is only _allocated_ in RAM if/when required. The allocation is done using the "linked list" technique used by OpenLog Artemis.
- _Any_ message can be "auto" if required, but can be polled too.
- An optional _callback_ can be associated with the arrival of each message type. A simple scheduler triggers the callbacks once I<sup>2</sup>C/Serial data reception is complete.
  - This means that your code no longer needs to wait for the arrival of a message, you are able to request (e.g.) PVT and your callback is called once the data arrives.
  - The callbacks are not re-entrant.
  - The callback receives a _copy_ of the data, so data reception and processing can continue while the callback is executing. Data integrity is preserved.
- Incoming data can be copied to a separate buffer to allow automatic writing to a file on SD card, which will be useful for (e.g.) RAWX logging.
  - Data is stored in a RingBuffer, the size of which can be set by calling ```setFileBufferSize``` _before_ ```.begin```.
  - The default buffer size is zero - to save memory.
  - To simplify SD card writing, data can be copied from the RingBuffer to a user-defined linear buffer first using ```extractFileBufferData```.
  - Data reception and processing can continue during the SD write.
  - User-defined code does the actual writing of data from the linear buffer to the SD card. The u-blox GNSS library does not perform the writing and so is not tied to any particular SD library.
  - The logged files can be played back and analyzed with (e.g.) u-center or RTKLIB.

In terms of RAM, you may find that your total RAM use is lower using v2 compared to v1, but it does of course depend on how many message types are being processed. The downside to this is that it is difficult to know in advance how much RAM is required, since it is only allocated if/when required. If the processor runs out of RAM (i.e. the _alloc_ fails) then an error is generated.
