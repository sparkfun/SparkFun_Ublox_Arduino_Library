SparkFun Ublox Arduino Library
===========================================================

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><a href="https://www.sparkfun.com/products/15136"><img src="https://cdn.sparkfun.com//assets/parts/1/3/5/1/4/15136-SparkFun_GPS-RTK2_Board_-_ZED-F9P__Qwiic_-03.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/15005"><img src="https://cdn.sparkfun.com//assets/parts/1/3/3/2/0/15005-SparkFun_GPS-RTK__Qwiic__-_NEO-M8P-2-00.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/15106"><img src="https://cdn.sparkfun.com//assets/parts/1/3/4/6/9/15106-GPS_Breakout_Ublox_SAM-M8Q__Qwiic_-01.jpg"></a></td>
  </tr>
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/15136">SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)</a></td>
    <td><a href="https://www.sparkfun.com/products/15005">SparkFun GPS-RTK - NEO-M8P-2 (GPS-15005)</a></td>
    <td><a href="https://www.sparkfun.com/products/15106">SparkFun SAM-M8Q Breakout (Qwiic) (SPX-15106)</a></td>
  </tr>
</table>

Ublox makes some incredible GPS receivers covering everything from low-cost, highly configurable modules such as the SAM-M8Q all the way up to the surveyor grade ZED-F9P with precision of the diameter of a dime. This library focuses on configuration and control of Ublox devices over I2C (called DDC by Ublox) and Serial. The UBX protocol is supported over both I2C and serial, and is a much easier and lighterweight interface to a GPS module. Stop parsing NMEA data! And simply ask for the datums you need.

This library can be installed via the Arduino Library manager. Search for **SparkFun Ublox**.

Want to help? Please do! We are always looking for ways to improve and build out features of this library.

* Special thanks to [tve](https://github.com/tve) for building out serial additions and examples.
* We are always interested in adding SPI support with a checkUbloxSPI() function

Need a library for the Ublox and Particle? Checkout the [Particle library](https://github.com/aseelye/SparkFun_Ublox_Particle_Library) fork.

Repository Contents
-------------------

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE. 
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE. 
* **library.properties** - General library properties for the Arduino package manager. 

Documentation
--------------

* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.

Products That Use This Library 
---------------------------------

* [GPS-15136](https://www.sparkfun.com/products/15136) - SparkFun GPS-RTK2 ZED-F9P
* [GPS-15005](https://www.sparkfun.com/products/15005) - SparkFun GPS-RTK NEO-M8P-2
* [SPX-14980](https://www.sparkfun.com/products/14980) - SparkX GPS-RTK Black
* [SPX-15106](https://www.sparkfun.com/products/15106) - SparkX SAM-M8Q

License Information
-------------------

This product is _**open source**_! 

Various bits of the code have different licenses applied. Anything SparkFun wrote is beerware; if you see me (or any other SparkFun employee) at the local, and you've found our code helpful, please buy us a round!

Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and release anything derivative under the same license.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
