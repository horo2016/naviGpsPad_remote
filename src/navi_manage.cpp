#include "navi_manage.h"


#include "osp_syslog.h"

#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
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
#include "geocoords.h"
#include "gps.h"
#include "imu.h"
#include <stdexcept>
#include <errno.h>
#include <RTIMULib.h>
#include"stm32_control.h"
#include "check_dis_module.h"



#include<sched.h>
#include "dwa_demo.h"
#include "raspi_sonar.h"





char  GLOBAL_STATUS =0;
char  GLOBAL_SWITCH =0;
float rollOffset = 0.0; //-0.4;
float pitchOffset = 0.0; // 3.3;


Kalman headingFilter(0.125, 4, 1, 0);

// PID controller variables for heading
double targetHeading, headingPIDInput, headingPIDOutput;
PID headingPID(&headingPIDInput, &headingPIDOutput, &targetHeading,1,0,0, DIRECT);




double waypointlongitude;
double waypointlatitude;
GeoCoordinate waypoints[256];
int waypointCount = 0;
int currentWaypoint = 0;
double waypointRange = 0.0;


float voltage1;
unsigned long voltage1_t;
float voltage2;
unsigned long voltage2_t;

bool voltageHysteresis = 0;

int movethreadid =0;
int rotatethreadid =0;
int movethreaddwaid =0;


/*******************************************************************************
 * function name        : is_file_exist
 * description  : set video and picture path of storage
 * param[in]    : path-file path
 * param[out]   : none
 * return              : 0-exist, -1-not exist
 *******************************************************************************/
int is_file_exist(const char *path)
{
        if(path == NULL)
                return -1;

        if(0 == access(path, F_OK))
                return 0;

        return -1;
}

/*******************************************************************************
* function name	: ReadWaypointsFile
* description	: from file waypoint.dat read datas from internet ,format :lat|long  
*				  call heatbeat main func
* param[in] 	: 将经纬度从文件中读到waypoints 中
* param[out] 	: none
* return 		: none
 *******************************************************************************/

void ReadWaypointsFile()
{
    FILE *waypointFile = fopen("waypoints.data", "r");
    if (waypointFile == NULL)
     {
	 DEBUG(LOG_ERR,"waypoints read err \n");	
	 return;
	}
    waypointCount = 0;
    while (!feof(waypointFile))
    {
	char line[256];
	fgets(line, 256, waypointFile);
	const char *wpLong = strtok(line, "|");
	const char *wpLat = strtok(0, "|");
	if (wpLat && wpLong)
	{
	    GeoCoordinate waypoint(wpLat, wpLong);
	    waypoints[waypointCount] = waypoint;
	    waypointCount++;
	}
    }
    printf("the first latitude:%f longititude %f \n",waypoints[0].latitude,waypoints[0].longitude);
    printf("waypoints all count is %d read from waypoint file \n",waypointCount);
    fclose(waypointFile);
}
// Quick and dirty function to get elapsed time in milliseconds.  This will wrap at 32 bits (unsigned long), so
// it's not an absolute time-since-boot indication.  It is useful for measuring short time intervals in constructs such
// as 'if (lastMillis - millis() > 1000)'.  Just watch out for the wrapping issue, which will happen every 4,294,967,295
// milliseconds - unless you account for this, I don't recommend using this function for anything that will cause death or
// disembowelment when it suddenly wraps around to zero (e.g. avionics control on an aircraft)...
unsigned long getmillis()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    unsigned long count = tv.tv_sec * 1000000 + tv.tv_usec;
    return count / 1000;
}

/*****
 *    判断是否在范围内
 * @param raduis    圆的半径
 * @param lat       点的纬度
 * @param lng       点的经度
 * @param lat1      圆的纬度
 * @param lng1      圆的经度
 * @return  
 */
 char isInRange(int raduis,double present_lat,double present_lng,double lat_circle,double lng_circle){
    
    double R = 6378137.0;
    float const PI_F = 3.14159265F;
    double dLat = (lat_circle- present_lat ) * M_PI / 180;
    double dLng = (lng_circle - present_lng )* M_PI / 180;
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(present_lat * PI / 180) * cos(lat_circle* PI / 180) * sin(dLng / 2) * sin(dLng / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double d = R * c;
    double dis = round(d);//近似值
    DEBUG(LOG_DEBUG,"currenpoint range circle  distance :%.2f  %d\n",dis,raduis);
    if (dis <= raduis){  //点在圆内
        return 1;
    }else {
        return 0;
    }
}
 
 // Initialization stuff - open and configure the serial device, etc.
 void Setup()
 {

 
     // Set up the PID controllers for heading and wall following
     headingPIDInput = 0;
   //  headingPID.SetOutputLimits(-NORMAL_SPEED, NORMAL_SPEED);
   //  headingPID.SetMode(AUTOMATIC);
 
  
 }
 //heading ：车的航向角相对于真北方向
// bearing： 目的地相对于真北方向的方位角
//返回：最优的转弯方案：转弯方向，转弯角度，行驶距离
int HeadingAnalysis(int Heading,int Bearing)
{
 int headingtmp;
 char t_dirction =0;
 if((Heading - Bearing) < 0)
 {
	 headingtmp = Heading + 360;
	 if(headingtmp - Bearing <= 180)
	 {
	 	//左转
	 	printf("turn left %d\n",headingtmp - Bearing);
		t_dirction = 1 ;
	 }else if(headingtmp - Bearing > 180){
	 	printf("turn right %d\n",360 - (headingtmp-Bearing));
		t_dirction = 2 ;
	 }
 }else if((Heading - Bearing) > 0)
 {
 	if(Heading - Bearing < 180)
	 {
		printf("turn left %d\n",Heading - Bearing);
		t_dirction = 1 ;
		
	 }
 	else if(Heading - Bearing > 180)
	 {
	 	printf("turn right %d \n",360 - (Heading-Bearing));
		t_dirction = 2 ;
	 }
 }
 return t_dirction;
 
}
 float prevHeading = 0;
 
/*******************************************************************************
* function name	: SteerToHeading
* description	: heartbeat function ,if receive new data ,clear counter,or,
*				  call heatbeat main func
* param[in] 	: task_table[4]
* param[out] 	: none
* return 		: none
*
* Steer to heading subsumption task.  If active and not subsumed by a higher priority task, 
* this will set the motor speeds
* to steer to the given heading (targetHeading)
 *******************************************************************************/
 void SteerToHeading()
 {
     // Filter the mag data to eliminate noise
 //    xFilter.update(magX);
 //    magX = xFilter.GetValue();
 //    yFilter.update(magY);
 //    magY = yFilter.GetValue();
 //    zFilter.update(magZ);
 //    magZ = zFilter.GetValue();
 
     // Do the same with the accelerometer data
 //    xAccFilter.update(accelX);
 //    accelX = xAccFilter.GetValue();
 //    yAccFilter.update(accelY);
 //    accelY = yAccFilter.GetValue();
 //    zAccFilter.update(accelZ);
 //    accelZ = zAccFilter.GetValue();
 
     float filteredHeading = heading;
     float adjustedHeading = filteredHeading;
 
     // Deal with the 0 == 360 problem
     float diff = targetHeading - filteredHeading;
     if (diff > 180)
     adjustedHeading += 360;
     else if (diff < -180)
     adjustedHeading -= 360;
 
 
     // If we've just crossed the 0/360 boundary, reset the filter so the compass updates immediately
     // instead of waiting for the filter to wrap around and catch up
     if (filteredHeading < 90 && prevHeading > 270)
     headingFilter.reset(0.125, 4, 1, 0);
     else if (filteredHeading > 270 && prevHeading < 90)
     headingFilter.reset(0.125, 4, 1, 360);
     prevHeading = filteredHeading;
 

 
     headingFilter.update(filteredHeading);
     filteredHeading = headingFilter.GetValue();
	 
     headingPIDInput = adjustedHeading;//filteredHeading;  // adjustedHeading ?
     headingPID.Compute();
 #if 0 
     printf("\033[2J");
     printf("\033[H");
     printf("Roll Angle:  %c%3.1f degrees\n", rollAngle + rollOffset < 0 ? '\0' : ' ', rollAngle + rollOffset);
     printf("Pitch Angle: %c%3.1f degrees\n", pitchAngle + pitchOffset < 0 ? '\0' : ' ', pitchAngle + pitchOffset);
     printf("Raw Heading:         %f\n", heading);

     printf("\033[1mFiltered Heading:    %d\033[0m \n", (int)filteredHeading);
     printf("\033[1mTarget Heading:      %d\033[0m \n", (int)targetHeading);
     printf("PID error:           %d\n", (int)headingPIDOutput);
#endif
   //  fprintf(outFile, "Roll Angle:  %c%3.1f degrees\n",  rollAngle + rollOffset < 0 ? ' ' : ' ', rollAngle + rollOffset);
   //  fprintf(outFile, "Pitch Angle: %c%3.1f degrees\n",  pitchAngle + pitchOffset < 0 ? ' ' : ' ', pitchAngle + pitchOffset);
    // fprintf(outFile, "Heading: %d\n", (int)filteredHeading);
    // fprintf(outFile, "Target Heading: %d\n", (int)targetHeading);
   //  fprintf(outFile, "PID error:           %d\n", (int)headingPIDOutput);
 
 
  //   steerToHeadingControl->leftMotorPower = NORMAL_SPEED - headingPIDOutput;
   //  steerToHeadingControl->rightMotorPower = NORMAL_SPEED + headingPIDOutput;
 
  //   steerToHeadingControl->active = steerToHeadingMode;
 }
 
/*******************************************************************************
* function name	: SteerToHeadingOfGPS
* description	: heartbeat function ,if receive new data ,clear counter,or,
*				  call heatbeat main func
* param[in] 	: task_table[4]
* param[out] 	: none
* return 		: none
*
* Steer to heading subsumption task.  If active and not subsumed by a higher priority task, 
* this will set the motor speeds
* to steer to the given heading (targetHeading)
 *******************************************************************************/
 void SteerToHeadingOfGPS()
 {
     // Filter the mag data to eliminate noise
      int  filteredHeading = 0;//heading;
      int tmpgpsheading = (int)gpsheading;
	  int tmpgpsvelocity = (int)gpsvelocity;
	  int tmp_currentheading =0;
	  int direction =0;
	  if((gpsvelocity > 1.0)&&(tmpgpsheading != 0))//m/s
	  
 			tmp_currentheading  = tmpgpsheading;
      else {
			for(int i =0;i<3;i++)
				filteredHeading += (int)heading;
			tmp_currentheading = filteredHeading /3;
	  }
     direction =  (tmp_currentheading- (int)targetHeading);
	  if(abs(direction) >=20)
	  	{
	  	  if(HeadingAnalysis(tmp_currentheading, (int)targetHeading) == 1)
	  	  cmd_send(5,0.20);//left 
		 else  
	  	  cmd_send(5,-0.20);
	  	}
	  else if(abs(direction) >=10)
	  	{
	  	  if(HeadingAnalysis(tmp_currentheading, (int)targetHeading) == 1)
	  	  cmd_send(5,0.10);
		 else  
	  	  cmd_send(5,-0.10);
	  	}   
	else if(abs(direction) < 10)
	  	{
	  	  
	  	  cmd_send(5,0);
	  	} 
 
  //   steerToHeadingControl->leftMotorPower = NORMAL_SPEED - headingPIDOutput;
   //  steerToHeadingControl->rightMotorPower = NORMAL_SPEED + headingPIDOutput;
 
  //   steerToHeadingControl->active = steerToHeadingMode;
 }
 /*******************************************************************************
 * function name : DetectObstacles
 * description   : heartbeat function ,if receive new data ,clear counter,or,
 *                 call heatbeat main func
 * param[in]     : task_table[4]
 * param[out]    : none
 * return        : none
 *
 * Detect obstacles subsumption task.  If an obstacle is detected, set active flag to subsume all other tasks.  This
 * will generally be the highest priority task (except for manual control), since we always want to avoid obstacles
 * regardless of what other tasks are active.
 * return degree 
 *******************************************************************************/

 int  DetectObstacles()
 {
     char ret = -1;
     // Need to set the servo to LEFT, CENTER, or RIGHT, then wait a few hundres ms for it to get there, then grab the
     // distance reading.  Reading the distance while the servo is moving will generate too much noise.
     //
     // ...
     //
     // This doesn't do anything currently
     // TODO:  Do something useful here
     //
      int distanceAhead = global_dis;
     if (distanceAhead > 4 && distanceAhead < 40 ) // cm
     {
       ret = true ;   //  detectObstaclesControl->active = true;
     }
      else
          ret = false;  //detectObstaclesControl->active = false;

      //如果存在障碍物，返回避障动作,及角度值
 }
  /*******************************************************************************
 * function name : CalculateHeadingToWaypoint
 * description   : 计算当前距离到航点的角度
 *                 call heatbeat main func
 * param[in]     : task_table[4]
 * param[out]    : none
 * return        : none
 *
 *******************************************************************************/
 void CalculateHeadingToWaypoint()
 {
     GeoCoordinate current(latitude, longitude);
     GeoCoordinate waypoint = waypoints[currentWaypoint];
 
     // getBearing() expects its waypoint coordinates in radians
     waypoint.latitude = waypoint.latitude * PI / 180.0;
     waypoint.longitude = waypoint.longitude * PI / 180.0;
 
     // targetHeading is the value used by the heading PID controller.  By changing this, we change the heading
     // to which the SteerToHeading subsumption task will try to steer us.
     targetHeading = getBearing(current, waypoint);
     DEBUG(LOG_DEBUG,"---->calculat two points degress is %d \n",(unsigned int)targetHeading);
   
     return;
 }
   /*******************************************************************************
 * function name : CalculateDistanceToWaypoint
 * description   : caclulate  到航向点的距离distance
 *                 call heatbeat main func
 * param[in]     : task_table[4]
 * param[out]    : none
 * return        : none
 *
 *******************************************************************************/
 void CalculateDistanceToWaypoint()
 {
     GeoCoordinate current(latitude, longitude);
     GeoCoordinate waypoint = waypoints[currentWaypoint];
 
     // getDistance() expects its waypoint coordinates in radians
     waypoint.latitude = waypoint.latitude * PI / 180.0;
     waypoint.longitude = waypoint.longitude * PI / 180.0;

 
     // targetHeading is the value used by the heading PID controller.  By changing this, we change the heading
     // to which the SteerToHeading subsumption task will try to steer us.
     double waypointRangetmp = getDistance(current, waypoint);
     waypointRange = waypointRangetmp*1000;
     if (waypointRange < 3.0) // 3.0 meters
     currentWaypoint++;
     if (currentWaypoint >= waypointCount)
     currentWaypoint = 0;
     DEBUG(LOG_DEBUG,"--->caculate two points distance is %.1f m\n",waypointRange); 
     return;
 }
 
 /*******************************************************************************
 * function name : rotateDegreesThread
 * description   : caclulate  rotate Degrees Thread
 *                 call heatbeat main func
 * param[in]     : task_table[4]
 * param[out]    : none
 * return        : none
 *
 *******************************************************************************/
 static void *rotateDegreesThread(void *threadParam)
 {
     // Make sure there's only one rotate thread running at a time.
     // TODO: proper thread synchronization would be better here
  
     rotatethreadid = pthread_self();
     int finnalheading = *(int*)threadParam;
     free(threadParam);  // Must have been malloc()'d by the caller of this thread routine!!

	 static bool threadActive = false;
     if (threadActive)
     return 0;
     threadActive = true;
     DEBUG(LOG_DEBUG,"rotateDegreesThread  start ****************\n");
     
     int startHeading =  (int)heading ;//headingFilter.GetValue();//这里获得是真北方向角，所以要转动imu找到真北方向
     
     DEBUG(LOG_DEBUG,"startHeading %d  \n",startHeading);
     int degrees  =  startHeading - finnalheading;//得到最终的真北方向
 //    if (targetHeadingtmp < 0)
   //  targetHeadingtmp += 360;
     //if (targetHeadingtmp > 359)
    // targetHeadingtmp -=360;
     char  done = 0;
     int heading_sum =0;
	 int waitquit =0;
	 DEBUG(LOG_DEBUG,"targetHeading %d ,turn degrees:%d \n",finnalheading,degrees);
  do{
	     if (degrees < 0)
	     {
	  
	        cmd_send2(0.0,-0.2);
	     }
	     else
	     {

	         cmd_send2(0.0,0.2);
	     }
	     usleep(5000);
             if(waitquit ++> 3000)
	       {
		waitquit =0;
		cmd_send2(0.0,0.0);
                 usleep(500000);
		}
   
     // Backup method - use the magnetometer to see what direction we're facing.  Stop turning when we reach the target heading.
	     int currentHeading  = int(heading);//headingFilter.GetValue();
	     heading_sum =0;
		 	    currentHeading  = int(heading);
		 		heading_sum += currentHeading;
		 int currentHeading_sec = heading_sum ;
	     DEBUG(LOG_DEBUG,"Rotating: currentHeading =%d,targetHeading = %d\n", currentHeading_sec, finnalheading);
	     if (abs(currentHeading_sec - finnalheading) <= 10)
	     {
	         done = 1;
	     }
	  //   if (currentHeading < startHeading && degrees > 0)
	    //     startHeading = currentHeading;
	    // if (currentHeading > startHeading && degrees < 0)
	      //   startHeading = currentHeading;
 
     //sleep(1);//不要太长否则容易转过头 
     }
     while (!done);
    cmd_send2(0,0);
     threadActive = 0;
      GLOBAL_STATUS = MOVE_STATUS ;
	   DEBUG(LOG_DEBUG,"rotate thread exit \n");
      pthread_exit(NULL);
     return 0;
 }
 static void *rotateDegreesThread1(void *threadParam)
 {
     // Make sure there's only one rotate thread running at a time.
     // TODO: proper thread synchronization would be better here
   
 
     rotatethreadid = pthread_self();
     int degrees = *(int*)threadParam;
     free(threadParam);  // Must have been malloc()'d by the caller of this thread routine!!

     static bool threadActive = false;
     if (threadActive)
       return 0;
     threadActive = true;
     int startHeading =  (int)heading;//这里获得是真北方向角，所以要转动imu找到真北方向
      if (startHeading < -180)
          startHeading += 360;
     if (startHeading > 180)
        startHeading -=360;
  
     int targetHeadingtmp =  startHeading + degrees;
     if (targetHeadingtmp < -180)
     targetHeadingtmp += 360;
     if (targetHeadingtmp > 180)
     targetHeadingtmp -=360;
     char  done = 0;
     DEBUG(LOG_DEBUG,"curreent heading :%d targetHeading %d ,degrees:%d \n", (int)heading,targetHeadingtmp,degrees);
  do{
	  

            // Backup method - use the magnetometer to see what direction we're facing.  Stop turning when we reach the target heading.
	     int currentHeading = (int)heading;//headingFilter.GetValue();
	     if (currentHeading < -180)
                  currentHeading += 360;
            if (currentHeading > 180)
                  currentHeading -=360;
	     int reamainDegrees = targetHeadingtmp - currentHeading ;
	     if (reamainDegrees < -180)
     	          reamainDegrees += 360;
            if (reamainDegrees > 180)
                  reamainDegrees -=360;
	     DEBUG(LOG_DEBUG,"Rotating:startYaw=%d,currentHeading=%d,targetHeading=%d,remainYaw:%d\n",startHeading, currentHeading,targetHeadingtmp,reamainDegrees);
	     
	      if ( reamainDegrees < 0)
             {

                cmd_send2(0.0,0.3);
             }
             else
             {

                 cmd_send2(0.0,-0.3);
             }
	    if (abs(reamainDegrees) <= 10)
	     {
	         done = 1;
	     }
 
     usleep(1000);//不要太长否则容易转过头 
     }
     while (!done);
    cmd_send2(0,0);
     threadActive = 0;
     return 0;
 }
 
 /*******************************************************************************
 * function name : rotateDegreesThread
 * description   : caclulate  rotate Degrees Thread
 *                 call heatbeat main func
 * param[in]     : task_table[4]
 * param[out]    : none
 * return        : none
 *
 *******************************************************************************/
 static void *moveDistanceThread(void *threadParam)
 {
     // Make sure there's only one rotate thread running at a time.
     // TODO: proper thread synchronization would be better here
   movethreadid = pthread_self();
     printf("into movedistance thread top %lu \n",movethreadid); 
     int meters = *(int *)threadParam;
     free(threadParam);  // Must have been malloc()'d by the caller of this thread routine!!


     DEBUG(LOG_DEBUG,"moveDistanceThread  start ****************\n");
     int startPosition = positionx;
     DEBUG(LOG_DEBUG,"startPosition %d  \n",startPosition);
     int targetPosition = startPosition + meters;
    
     char  done = 0;
	 DEBUG(LOG_DEBUG,"targetPosition: %d ,meters:%d \n",targetPosition,meters);
  do{
  	 
	   
     obstacleAvoidance();

     // Backup method - use the magnetometer to see what direction we're facing.  Stop turning when we reach the target heading.
     int currentPosition = (int)positionx;
    // DEBUG(LOG_DEBUG,"MOve: currentPosition = %d   targetPosition = %d\n", currentPosition, targetPosition);
     if ((currentPosition <= targetPosition) && (meters < 0) && (startPosition > targetPosition))
     {
         done = 1;
     }
     if ((currentPosition >= targetPosition) && (meters > 0) && (startPosition < targetPosition))
     {
         done = 1;
     }

     usleep(10000);
     }
     while ((!done)&&(GLOBAL_STATUS == MOVE_STATUS));
    cmd_send2(0.0,0.0);
	
      
	    DEBUG(LOG_DEBUG,"move distance  thread exit \n");
      pthread_exit(NULL);
     return 0;
 }
 static void *moveDistanceDwaThread(void *threadParam)
 {
     // Make sure there's only one rotate thread running at a time.
     // TODO: proper thread synchronization would be better here
     movethreaddwaid = pthread_self();
     printf("into movedistance thread top %lu \n",movethreaddwaid); 
     int meters = *(int *)threadParam;
     free(threadParam);  // Must have been malloc()'d by the caller of this thread routine!!


     DEBUG(LOG_DEBUG,"moveDistanceThread  start ****************\n");
     int startPosition = positionx;
     DEBUG(LOG_DEBUG,"startPosition %d  \n",startPosition);
     int targetPosition = startPosition + meters;
    
     char  done = 0;
	 DEBUG(LOG_DEBUG,"targetPosition: %d ,meters:%d \n",targetPosition,meters);
  do{
  	 
	 done = dwa_loop((float)meters);

     usleep(10000);
     }
     while ((!done)&&(GLOBAL_STATUS == MOVE_STATUS));
    cmd_send(0,0);
     
	    DEBUG(LOG_DEBUG,"move distance  thread exit \n");
      pthread_exit(NULL);
     return 0;
 }
bool is_thread_alive(pthread_t tid)
{
        bool bAlive = false;
        if(tid)
        {		
            int ret = pthread_tryjoin_np(tid, NULL);
            if (ret != 0) {
                /* Handle error */
                if(EBUSY == ret)
                {
                    bAlive = true;
                }
            }
        }	
 
        return bAlive;
}
 /*******************************************************************************
 * function name : RotateDegrees
 * description   : caclulate  rotate Degrees  N DEGREE 
 *                 call heatbeat main func
 * param[in]     : task_table[4]
 * param[out]    : none
 * return        : none
 *
 *******************************************************************************/
  void RotateDegrees(int degrees)
 {
     pthread_t rotThreadId;
     static bool threadActive = false;
     if (threadActive){
       //printf("the specified thread (%lu) is into \n",rotatethreadid);
       int kill_rc = is_thread_alive(rotatethreadid);
	if(kill_rc == true){
	//	printf("the specified thread (%lu) is alive\n",rotatethreadid);
	       return ;
		}
		else{
			printf("the specified thread (%lu) did not exists or already quit\n",rotatethreadid);
		     
		}	 
	 }
     
	   threadActive = true;
   
	 
      
     int *rotationDegrees = (int *)malloc(sizeof(int));
     *rotationDegrees = degrees;
   
     pthread_create(&rotThreadId, NULL, rotateDegreesThread, rotationDegrees);
 }
  void RotateDegreesByManual(int degrees)
 {
     pthread_t rotThreadId;
     static bool threadActive = false;
     if (threadActive){
       printf("the specified thread (%lu) is into \n",rotatethreadid);
       int kill_rc = is_thread_alive(rotatethreadid);
        if(kill_rc == true){
                printf("the specified thread (%lu) is alive\n",rotatethreadid);
               return ;
                }
                else{
                        printf("the specified thread (%lu) did not exists or already quit\n",rotatethreadid);

                }
         }

           threadActive = true;



     int *rotationDegrees = (int *)malloc(sizeof(int));
     *rotationDegrees = degrees;

     pthread_create(&rotThreadId, NULL, rotateDegreesThread1, rotationDegrees);
 }


 /*******************************************************************************
 * function name : MoveDistance
 * description   : caclulate    distance  N meters 
 *                 call heatbeat main func
 * param[in]     : task_table[4]
 * param[out]    : none
 * return        : none
 *
 *******************************************************************************/
  void MoveDistance(int meters)
 {
    
     pthread_t rotThreadId;
     static bool threadActive = false;
	 
     if (threadActive){
       printf("the specified MoveDistance thread (%lu) is into \n",rotThreadId);
 

	    int kill_rc = pthread_kill(movethreadid,0);
		if(kill_rc == ESRCH)
		printf("the specified thread (%lu) did not exists or already quit\n",movethreadid);
		else if(kill_rc == EINVAL)
		printf("signal is invalid\n");
		else{
		printf("the specified thread (%lu) is alive\n",movethreadid);
		return;
		}
	 }
     
	   threadActive = true;
	   
	 int *MoviMeters = (int *)malloc(sizeof(int));
     *MoviMeters = meters;
     if(pthread_create(&rotThreadId, NULL, moveDistanceThread,MoviMeters))
    {
    	    perror( "pthread_create error "); 
            DEBUG(LOG_ERR,"MOVE THREAD CREATE ERROR \n");
    }
	
 }
  /*******************************************************************************
   * function name : MoveDistanceDwa
   * description   : caclulate	  distance	N meters 
   *				 call heatbeat main func
   * param[in]	   : task_table[4]
   * param[out]    : none
   * return 	   : none
   *
   *******************************************************************************/
	void MoveDistanceDwa(int meters)
   {
	  
	   pthread_t rotThreadId;
	   static bool threadActive = false;
	   
	   if (threadActive){
		
		  int kill_rc = pthread_kill(movethreaddwaid,0);
		  if(kill_rc == ESRCH)
		  printf("the specified thread (%lu) did not exists or already quit\n",movethreaddwaid);
		  else if(kill_rc == EINVAL)
		  printf("signal is invalid\n");
		  else{
		  printf("the specified thread (%lu) is alive\n",movethreaddwaid);
		  return;
		  }
	   }
	   
		 threadActive = true;
		 
	   int *MoviMeters = (int *)malloc(sizeof(int));
	   *MoviMeters = meters;
	   if(pthread_create(&rotThreadId, NULL, moveDistanceDwaThread,MoviMeters))
	  {
			  perror( "pthread_create error "); 
			  DEBUG(LOG_ERR,"MOVE THREAD CREATE ERROR \n");
	  }
	  
   }

/*******************************************************************************
* function name	: loop_handle
* description	: heartbeat function ,if receive new data ,clear counter,or,
*				  call heatbeat main func
* param[in] 	: task_table[4]
* param[out] 	: none
* return 		: none
*******************************************************************************/
void *navimanage_handle (void *arg)
{

	char ret = -1;

   
    unsigned long lastDISMillis = 0;
    unsigned long lastSubMillis = 0;
    unsigned long lastGPSMillis = 0;
    unsigned long motorsOffMillis = 0;
    bool motorsRunning = false;
 //   ReadWaypointsFile();
    DEBUG(LOG_DEBUG,"navimanage_handle   \n");
    
    char onceread =0;
    while (1)
    {
    	unsigned long loopTime = getmillis();
		while(GLOBAL_SWITCH)
		{   
	  	    if(onceread ==0)
	       	    {
		     onceread =1;
	             if(is_file_exist("waypoints.data")!=0)
			{
				GLOBAL_STATUS=STOP_STATUS;
				GLOBAL_SWITCH =0 ;
				onceread =0;
				DEBUG(LOG_ERR,"waypoint file is not exist \n");
				break;	
			}
		     }
		    if((latitude ==0.0)&&(longitude == 0.0))
		    {
			DEBUG(LOG_ERR,"GPS CANNOT LOCATION PLEASE CHECK \n");
	                GLOBAL_STATUS = STOP_STATUS;
	                GLOBAL_SWITCH = 0;
	                break;

		    }
	            if ((getmillis() - lastSubMillis > SUBSUMPTION_INTERVAL))
	           {
	              switch(GLOBAL_STATUS)
	              {
	                case STANDBY_STATUS://启动后的初始状态
					DEBUG(LOG_DEBUG,"STANDBY STATUS \n");
	                lastGPSMillis =0 ;
		        	ReadWaypointsFile();
				    CalculateHeadingToWaypoint();
		        	CalculateDistanceToWaypoint();
	                GLOBAL_STATUS = ROTATE_STATUS ;
	                break;
	                case ROTATE_STATUS :
					if (abs((int)heading - (int)targetHeading) > 10)
					{
						
	                              RotateDegrees(targetHeading);//这里需要根据求出的角度进行转动
	             
					//  DEBUG(LOG_ERR,"TARGET HEADING IS invalid , continue\n");
					//  break;//不合法 返回
					}else {
						 GLOBAL_STATUS = MOVE_STATUS ;
					}
					
					 SteerToHeading();	
	                break;
	                case MOVE_STATUS :
					 if((waypointRange > 200))//大于100m 认为不合法 所以规划路径时需要注意
					{	
						DEBUG(LOG_ERR,"distance > 200m \n");
						 break;
					}	
					 MoveDistance(waypointRange);
					 
	                 SteerToHeading();//行驶中依然根据航向脚纠偏算法I

	                 break;    
		         case AVOIDOBJ_STATUS:
	                // 根据超声波获得反馈值进行避障
	                cmd_send(4, 0.3);
	              //  int tmp_degree = DetectObstacles();
	              //    if (tmp_degree)
	              //       GLOBAL_STATUS = ROTATE_STATUS ;
	        	 break;
	                 case WAYPOINTARRIVE_STATUS:
	                 if(currentWaypoint < waypointCount )
	                 {
	                    currentWaypoint ++;
						DEBUG(LOG_DEBUG,"currentWaypoint ++\n");
	                    GLOBAL_STATUS = ROTATE_STATUS ;
	                 }
	                 else  if(currentWaypoint >= waypointCount ){
	                    GLOBAL_STATUS = STOP_STATUS ;
						DEBUG(LOG_ERR,"currentWaypoint >= waypointCount stop status\n");
	                 }
	                 break;
	                case STOP_STATUS :
	                
	                break;
	                case MANUAL_STATUS :
	                break;
	                default :
	                break;

	              }//end switch
		         // 必须先运行一次 standby 状态 只有当在目的地附近3米内才会转换状态
		         ret =  isInRange(3, latitude , longitude, waypoints[currentWaypoint].latitude, waypoints[currentWaypoint].longitude);
	                 if (ret == 1) //点在圆圈内
	                 {   
	 						DEBUG(LOG_DEBUG,"arrive into circle scal \n");
	                         GLOBAL_STATUS = WAYPOINTARRIVE_STATUS ;
	                 }

	     
	          	lastSubMillis = getmillis();
	            }//end  sub loop
	      
	          	if ( getmillis() - lastGPSMillis > CALCULATE_GPS_HEADING_INTERVAL)
	        	{
	        	 		if(GLOBAL_STATUS == MOVE_STATUS)
						SteerToHeadingOfGPS();
	            		CalculateHeadingToWaypoint();
		        	    CalculateDistanceToWaypoint();
	            		lastGPSMillis = getmillis(); //
	            }
				if ( getmillis() - lastDISMillis > DIS_BLINK_INTERVAL)
	        	{
	        	 		/*if(GLOBAL_STATUS == MOVE_STATUS)
	        	 		{
							if (DetectObstacles() == true)
								GLOBAL_STATUS == AVOIDOBJ_STATUS;
	        	 			}else if(GLOBAL_STATUS == AVOIDOBJ_STATUS)
								{
								if (DetectObstacles() == false)
								GLOBAL_STATUS == MOVE_STATUS;

							}*/
							
	            		lastDISMillis = getmillis(); //lastDISMillis
	            }
	     	}//end while switch on
     	if(GLOBAL_STATUS == MANUAL_STATUS )
		{
			//SteerToHeading();
		}
     
          	// Shut down if the battery level drops below 10.8V
    	if(voltage1 > 11.2)
    	    voltageHysteresis = 1;
    	if(voltageHysteresis  && voltage1 < 10.8)
    	{
    	    signal(SIGCHLD, SIG_IGN);
    	    long shutdownPID = fork();
    	    if (shutdownPID >= 0)
    	    {
        		if (shutdownPID == 0)
        		{
        		    // Child process
        		    execl(getenv("SHELL"),"sh","-c","sudo shutdown -h now",NULL);
        		    _exit(0);
        		}
    	    }
    	}
        unsigned long now = getmillis();
    	if (now - loopTime < 1)
            usleep((1 - (now - loopTime)) * 1000);
    }//end while1

}

