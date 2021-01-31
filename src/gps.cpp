#include "gps.h"


#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include "PID_v1.h"
#include "kalman.h"
#include <pthread.h>
#include <sys/types.h>
#include <sys/wait.h>
#include "gps_hal.h"

double latitude = 0;
unsigned long latitude_t;
double longitude = 0;
unsigned int gpsheading =0;
unsigned int gpsvelocity =0;

unsigned long longitude_t;
double latitude_error;
double longitude_error;
int gps_fix = -1;



/*******************************************************************************
* function name	: IMUThread
* description	: heartbeat function ,if receive new data ,clear counter,or,
*				  call heatbeat main func
* param[in] 	: task_table[4]
* param[out] 	: none
* return 		: none
*******************************************************************************/

//gps_data_t gpsData;

void *GpsThread(void *)
{
	
     GpsHandle();
	
}
#if 0
{
    // Init libgps so we can get gps data
    // connect to GPSd

    memset(&gpsData, 0, sizeof(gpsData));
    printf("Connecting To GPSD\n");
    while (gps_open("localhost", "2947", &gpsData) < 0)
    {
        fprintf(stderr,"Could not connect to GPSd\n");
    sleep(10);
    }

    printf("Successfully connected to GPSD\n");
    //register for updates
    gps_stream(&gpsData, WATCH_ENABLE | WATCH_JSON, NULL);

    int nofixCount = 0;
    while (1) //gpsData.status==0)
    {
        //block for up to 0.5 seconds
        if (gps_waiting(&gpsData, 500))
        {
            //dunno when this would happen but its an error
            if(gps_read(&gpsData) == -1)
            {
//                fprintf(stderr,"GPSd Error\n");
        latitude = 0.0;
        longitude = 0.0;
        latitude_error = 0;
        longitude_error = 0;
        gps_fix = -1;
        nofixCount++;
            }
            else
            {
                //status>0 means you have data
                if(gpsData.status > 0)
                {
                    //sometimes if your GPS doesnt have a fix, it sends you data anyway
                    //the values for the fix are NaN. this is a clever way to check for NaN.
            //Also assume we have no fix if the epx or epy fix values (accuracy) are 0
                    if(gpsData.fix.longitude != gpsData.fix.longitude || gpsData.fix.altitude != gpsData.fix.altitude ||
                     gpsData.fix.epx != gpsData.fix.epx || gpsData.fix.epy != gpsData.fix.epy)
                    {
//                        fprintf(stderr,"Could not get a GPS fix.\n");
            latitude = 0.0;
            longitude = 0.0;
            latitude_error = 0;
            longitude_error = 0;
            gps_fix = MODE_NO_FIX;
            gpsData.status = STATUS_NO_FIX;
            nofixCount++;
                    }
                    //otherwise you have a legitimate fix!
                    else
                    {
            latitude = gpsData.fix.latitude;
            longitude = gpsData.fix.longitude;

            // Recent gpsd versions don't set fix.epx and fix.epy correctly (at least not with my gps unit).
            // Use dop.hdop instead, and do the calculation here to find the horizontal position error in meters
            // (Code lifted from gpsd_error_model() in libgpsd_core.c)
//          double h_uere =
//                  (session->gpsdata.status == STATUS_DGPS_FIX ? H_UERE_WITH_DGPS : H_UERE_NO_DGPS);
//          latitude_error = gpsData.dop.hdop * h_uere; // gpsData.fix.epy;
//          longitude_error = gpsData.dop.hdop * h_ueru; // gpsData.fix.epx;
            latitude_error = gpsData.fix.epy;
            longitude_error = gpsData.fix.epx;
            gps_fix = gpsData.fix.mode;
            nofixCount = 0;

            FILE *telemetry = fopen("./telemetry.dat","w");
            if (telemetry)
            {
                // Lat | Long | Lat_accuracy | Long_accuracy | waypoint_lat | waypoint_long
                fprintf(telemetry, "%f|%f|%d|%d|%f|%f\n", latitude, longitude, (int)latitude_error, (int)longitude_error, waypoints[currentWaypoint].latitude, waypoints[currentWaypoint].longitude);
                fclose(telemetry);
            }
                    }
                }
            }
        }
        //apparently gps_stream disables itself after a few seconds.. in this case, gps_waiting returns false.
        //we want to re-register for updates and keep checking we dont have a fix yet.
        else
    {
            gps_stream(&gpsData, WATCH_ENABLE | WATCH_JSON, NULL);
        nofixCount++;
    }

    // If we've been without a fix for 10 iterations (10 seconds), set the fix mode to NO_FIX
    if (nofixCount > 10)
        gps_fix = MODE_NO_FIX;


    // Sleep so we don't kill the CPU.  Updates every second shuold be enough
    usleep(100000);
    }

    // Should never get here
    return NULL;
   
    while(1)
    {
       usleep(100000);
    }
}

 #endif 
