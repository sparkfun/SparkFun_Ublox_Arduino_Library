/*
Copyright (c) 2020 Balamurugan Kandan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "SparkFun_Ublox_Arduino_Library.h"
SFE_UBLOX_GPS myGPS;

int main(int argc, char** argv)
{
    if(argc == 1) {
        printf("\nublox_f9p_test <ublox_com> <pseudo_com> (ublox_f9p_test '/dev/ttyACM0' '/dev/pts/1')"); 
        return 0;
    } else if (argc == 2) { 
        for(int counter=0;counter<argc;counter++) 
            printf("\nargv[%d]: %s",counter,argv[counter]);        
    } else if(argc >= 3) {
        printf ("\nMore number of arguments...");
        return 0;
    } 

    Stream seriComm(argv[1]);
    if (!seriComm.isConnected()) {
        printf ("Ublox is not connected. Please connect ublox GNSS module and try again...\n");
        return 0;
    }
    myGPS.begin(seriComm);
    myGPS.setNavigationFrequency(10); //Set output to 20 times a second
    myGPS.saveConfiguration(); //Save the current settings to flash and BBR
    
    printf ("\n----------------------------------------\n");
    while(true) {
        if (myGPS.getPVT()) {
          printf ("%02d/%02d/%02d %02d:%02d:%02d %d:%d\n", myGPS.getDay(), myGPS.getMonth(), myGPS.getYear(), 
                                                           myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond(), 
                                                           myGPS.getMillisecond(), myGPS.getNanosecond());
          printf("Latitude                : %2.8f (deg)\n", myGPS.getLatitude() * 1e-7);
          printf("Longitude               : %2.8f (deg)\n", myGPS.getLongitude() * 1e-7);
          printf("Altitude                : %d (mm)\n", myGPS.getAltitude());
          printf("Altitude MSL            : %d (mm)\n", myGPS.getAltitudeMSL());
          printf("SIV                     : %d\n", myGPS.getSIV());
          printf("PDOP                    : %f\n", myGPS.getPDOP() * 1e-2); 
          printf("Fix type                : %d\n", myGPS.getFixType());
          printf ("Ground Speed           : %d\n", myGPS.getGroundSpeed());
          printf ("VelN                   : %08d (mm/s)\n", myGPS.getNedNorthVel());
          printf ("VelE                   : %08d (mm/s)\n", myGPS.getNedEastVel());
          printf ("VelD                   : %08d (mm/s)\n", myGPS.getNedDownVel());
          printf ("VAcc                   : %08d (mm)\n", myGPS.getVerticalAccEst());
          printf ("HAcc                   : %08d (mm)\n", myGPS.getHorizontalAccEst());
          int solnType = myGPS.getCarrierSolutionType();
          if (solnType == 0) printf ("### No RTK Fix yet ###\n");
          else if (solnType == 1) printf ("&&& DGNSS/Float &&&\n");
          else if (solnType == 2) printf ("*** DGNSS/Fix ***\n");
          printf ("\n----------------------------------------\n");
          usleep(50);
        }

        usleep(25);
    }

    return 1;
}
