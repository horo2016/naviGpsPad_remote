#include "gps_hal.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <signal.h>
#include <unistd.h>

#include <time.h> 
#include <sys/time.h> 

#include <sys/shm.h>

#include<pthread.h>
#include"osp_syslog.h"
#include "gps.h"
/*******************************************************************************
* function name	: IMUThread
* description	: heartbeat function ,if receive new data ,clear counter,or,
*				  call heatbeat main func
* param[in] 	: task_table[4]
* param[out] 	: none
* return 		: none
*******************************************************************************/

//gps_data_t gpsData;
#define MYKEY 1234
#define BUF_SIZE 1024




typedef struct
{    
    char isvalid;
    Location gpsInf;
    float gpsheading;
    float gpsvelocity;
}use_shared;

void GpsHandle()
{

   int shmid; 
   void *shmptr;
	use_shared *shared;


	system("sudo /home/pi/app/gpsKalmanServer >/dev/null  2&>1 & ");
	sleep(2);
    if((shmid = shmget(MYKEY,BUF_SIZE,0666|IPC_CREAT)) ==-1) 
    { 
        printf("shmget error \n"); 
        exit(1); 
     }
     if((shmptr =shmat(shmid,0,0))==(void *)-1) 
     { 
        printf("shmat error!\n"); 
        exit(1); 
     }
	 
	shared = (use_shared*)shmptr;
	printf("gps client task create ... \n");
	while (1)
	{
	 if(shared->isvalid !=0 ){
	         printf("lat:%f \n",shared->gpsInf.lat);
		 printf("lng:%f \n",shared->gpsInf.lng);
		printf("gpsheading %f %f",shared->gpsheading,shared->gpsvelocity);
		 latitude = shared->gpsInf.lat;
		 longitude = shared->gpsInf.lng; 
		shared->isvalid  =0;
	 	}else{
			 DEBUG(LOG_DEBUG,"gps data read invalid no receive  \n");
		}
	  // longitude = 116.293532;
	  // latitude =40.151789;
	   usleep(980000);
			 

	}
	if(shmdt(shmptr) == -1)
	{
		fprintf(stderr, "shmdt failed\n");
		exit(EXIT_FAILURE);
	}
	//É¾³ý¹²ÏíÄÚ´æ
	if(shmctl(shmid, IPC_RMID, 0) == -1)
	{
		fprintf(stderr, "shmctl(IPC_RMID) failed\n");
		exit(EXIT_FAILURE);
	}
	exit(EXIT_SUCCESS);
	return 0;
}

