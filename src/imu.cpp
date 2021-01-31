#include "imu.h"


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
//#include "include.h"

#include "imu.h"

#include <stdexcept>
#include <errno.h>
#include "navi_manage.h"

#include "RTIMULib.h"


int magX, magY, magZ;
unsigned long magX_t, magY_t, magZ_t;
double accelX, accelY, accelZ;
unsigned long accelX_t, accelY_t, accelZ_t;
double gyroX, gyroY, gyroZ;
unsigned long gyroX_t, gyroY_t, gyroZ_t;
int gyroDeltaT;
unsigned long gyroDeltaT_t;


Kalman xFilter(0.125, 4, 1, 0);
Kalman yFilter(0.125, 4, 1, 0);
Kalman zFilter(0.125, 4, 1, 0);

Kalman xAccelFilter(0.125, 4, 1, 0);
Kalman yAccelFilter(0.125, 4, 1, 0);
Kalman zAccelFilter(0.125, 4, 1, 0);

Kalman xGyroFilter(0.125, 4, 1, 0);
Kalman yGyroFilter(0.125, 4, 1, 0);
Kalman zGyroFilter(0.125, 4, 1, 0);


float rollAngle = 0;
float pitchAngle = 0;
float heading = 0;
unsigned long heading_t;

/*******************************************************************************
* function name	: IMUThread
* description	: heartbeat function ,if receive new data ,clear counter,or,
*				  call heatbeat main func
* param[in] 	: task_table[4]
* param[out] 	: none
* return 		: none
*******************************************************************************/


void *IMUThread(void *)
{
    int sampleCount = 0;
//    int sampleRate = 0;
    uint64_t rateTimer;
    uint64_t displayTimer;
    uint64_t now;

    //  Using RTIMULib here allows it to use the .ini file generated by RTIMULibDemo.
    //  Or, you can create the .ini in some other directory by using:
    //      RTIMUSettings *settings = new RTIMUSettings("<directory path>", "RTIMULib");
    //  where <directory path> is the path to where the .ini file is to be loaded/saved

    RTIMUSettings *settings = new RTIMUSettings("/home/pi/21navigpsbot","RTIMULib");
    sleep(2);
    RTIMU *imu = RTIMU::createIMU(settings);
    sleep(3);
    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
    {
	printf("No IMU found\n");
	exit(1);
    }

    //  This is an opportunity to manually override any settings before the call IMUInit

    //  set up IMU

    imu->IMUInit();

    //  this is a convenient place to change fusion parameters

    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    //  set up for rate timer

    rateTimer = displayTimer = RTMath::currentUSecsSinceEpoch();

    //  now just process data

    long impactTimer = getmillis();
    double prevGx = 0;
    double prevGy = 0;
    double prevGz = 0;
    long lastMillis = getmillis();
    while (1)
    {
	//  poll at the rate recommended by the IMU

//	usleep(imu->IMUGetPollInterval() * 1000);
	usleep(96500);
	while (imu->IMURead())
	{
	    RTIMU_DATA imuData = imu->getIMUData();
	    sampleCount++;

	    now = RTMath::currentUSecsSinceEpoch();

	    //  display 10 times per second

	    if ((now - displayTimer) > 96500)
	    {
//		printf("Sample rate %d: %s\r", sampleRate, RTMath::displayDegrees("", imuData.fusionPose));
		fflush(stdout);
		displayTimer = now;
		rollAngle = imuData.fusionPose.x() * (180 / M_PI);
		pitchAngle = imuData.fusionPose.y() * (180 / M_PI);
		heading = imuData.fusionPose.z() * (180 / M_PI);

		if (heading < 0)
		    heading += 360;

		magX = imuData.compass.x();
		magY = imuData.compass.y();
		magZ = imuData.compass.z();
		accelX = imuData.accel.x() * 10;   // m/s^2
		accelY = imuData.accel.y() * 10;   // m/s^2
		accelZ = imuData.accel.z() * 10;   // m/s^2

		xAccelFilter.update(accelX);
		yAccelFilter.update(accelY);
		zAccelFilter.update(accelZ);
		accelX = xAccelFilter.GetValue();
	    accelY = yAccelFilter.GetValue();
		accelZ = zAccelFilter.GetValue();

		gyroX = imuData.gyro.x() * 100;  // degrees / sec
		gyroY = imuData.gyro.y() * 100;  // degrees / sec
		gyroZ = imuData.gyro.z() * 100;  // degrees / sec
		xGyroFilter.update(gyroX);
		yGyroFilter.update(gyroY);
		zGyroFilter.update(gyroZ);
//		gyroX = xGyroFilter.GetValue();
//		gyroY = yGyroFilter.GetValue();
//		gyroZ = zGyroFilter.GetValue();

// *jdl* 10/25/2016 get rid of the roll and pitch filters - too slow to respond
////		rollFilter.update(rollAngle);
////		pitchFilter.update(pitchAngle);
////		rollAngle = rollFilter.GetValue();
////		pitchAngle = pitchFilter.GetValue();

		gyroDeltaT = getmillis() - lastMillis;
		lastMillis = getmillis();
	    }

	    //  update rate every second

	    if ((now - rateTimer) > 1000000)
	    {
//		sampleRate = sampleCount;
		sampleCount = 0;
		rateTimer = now;
	    }

	    // Detect impacts
	    const float ouchThreshold = 15.0;
//	    printf("%.1f %.1f %.1f \n",fabs(gyroX),fabs(gyroY),fabs(gyroZ));
#if 0 // qu xiao jiance  yinwei meijiaozhun 
	    if ( (fabs(gyroX) > ouchThreshold || fabs(gyroY) > ouchThreshold || fabs(gyroZ) > ouchThreshold)
			&& (fabs(prevGx) > ouchThreshold || fabs(prevGy) > ouchThreshold || fabs(prevGz) > ouchThreshold))
	    {
		// Only if it's been more than 1 second since the last impact, and we're not currently moving
		//if (millis() - impactTimer > 1000 && leftMotorPower == 0 && rightMotorPower == 0)
          if (millis() - impactTimer > 1000 )
		{
		    pid_t child_PID;
		    child_PID = fork();
		    if(child_PID >= 0)
		    {
			if(child_PID == 0)
			{
			    // Child process
			    printf("Child process! PID=%d, Parent PID=%d\n", getpid(), getppid());
			    execl("/usr/bin/espeak", "/usr/bin/espeak", "Ouch.", (char *)0);
			    _exit(0);
			}
			else
			{
			    int status;
			    waitpid(child_PID, &status, 0);
			}
		    }
		    impactTimer = millis();
		}
	    }
	    prevGx = gyroX;
	    prevGy = gyroY;
	    prevGz = gyroZ;
#endif 
	}
    }
}

