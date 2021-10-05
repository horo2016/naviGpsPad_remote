#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <signal.h>
#include <unistd.h>

#include <time.h> 
#include <sys/time.h> 

#include "cJSON.h"
#include <pthread.h>
#include "imu.h"
#include "cpu_sys.h"
#include "stm32_control.h"
#include "navi_manage.h"
#include "config_conf.h"

#include "client_pub_sub.h"
#include "cpu_sys.h"

// int mysock = 0;

#define TURNROUND "turnround"
#define RUNFORWARD "runforward"
#define AVOIDANCE "avoidance"



int toStop = 0;




static int GetNextPackID(void);


 char  send_buf[0xff]={0};
int Creatstatejson(float head,float roll,float pitch)
{
	

    cJSON * root =  cJSON_CreateObject();
   if(!root) {
         printf("get root faild !\n");
     }

  //  cJSON_AddItemToObject(root, "\"devid\"", cJSON_CreateString(chargename));
    cJSON_AddItemToObject(root, "heading", cJSON_CreateNumber((short)head));
    cJSON_AddItemToObject(root, "roll", cJSON_CreateNumber((short)roll));
    cJSON_AddItemToObject(root, "pitch",cJSON_CreateNumber((short)pitch));
    cJSON_AddItemToObject(root, "cpuload", cJSON_CreateNumber((char)cpuPercentage));
    cJSON_AddItemToObject(root, "cputemp", cJSON_CreateNumber((char)cpuTemperature));
    cJSON_AddItemToObject(root, "wifisignal", cJSON_CreateNumber(wifiSignalStrength));
    cJSON_AddItemToObject(root, "velspeed", cJSON_CreateNumber((int)velspeed));
    cJSON_AddItemToObject(root, "angspeed", cJSON_CreateNumber(angspeed));
    cJSON_AddItemToObject(root, "targetheading", cJSON_CreateNumber((int)targetHeading));
    cJSON_AddItemToObject(root, "distance", cJSON_CreateNumber((int)waypointRange));
    cJSON_AddItemToObject(root, "nextwaypoint_lon", cJSON_CreateNumber(waypointlongitude));
    cJSON_AddItemToObject(root, "nextwaypoint_lat", cJSON_CreateNumber(waypointlatitude));
    
    memcpy(send_buf,cJSON_Print(root),strlen(cJSON_Print(root)));
      
    
    
    cJSON_Delete(root);

    return 0;
}

/*******************************************************************************
 * function name	: get_value_from_config_file
 * description	: get value from config
 * param[in] 	: file-config file, key-dest string, value-key value
 * param[out] 	: none
 * return 		: 0-exist, -1-not exist
 *******************************************************************************/
int get_value_from_cmdline( char* buf, char *key, char *value)
{
	int value_len = 0 ;

	char *sub_str = NULL ;
	size_t line_len = 0 ;
	size_t len = 0;


		//printf("zdf config file : %s\n", buf);
		sub_str = strstr(buf, key);

		if(sub_str != NULL){//video="/home/linaro/video"
			sub_str += (strlen(key) + 1); //=" length is 2 bytes
		//	printf("zdf sub_str : %s\n", sub_str);
			value_len = get_strchr_len(sub_str, '\n');
		//	printf("zdf value len = %d\n", value_len);
			memcpy(value, sub_str, value_len);
			
			return 0;
		}

	


	return -1;
}
void cfinish(int sig)
{
    signal(SIGINT, NULL);
	toStop = 1;
}

void stop_init(void)
{
	signal(SIGINT, cfinish);
	signal(SIGTERM, cfinish);
}

void User_MsgContl(char  *msg)
{
 
	char tmp_value[0xff]={0};
      if( get_value_from_cmdline((char *)msg,RUNFORWARD,tmp_value )==0){
            printf("runforward %d \n",atoi(tmp_value));
			GLOBAL_STATUS = MOVE_STATUS;
			MoveDistance(atoi(tmp_value));
	}
      else  if( get_value_from_cmdline((char *)msg,TURNROUND,tmp_value )==0)
	{
	   GLOBAL_STATUS = MANUAL_STATUS;
           RotateDegreesByManual(atoi(tmp_value));
	} 
	    else  if( get_value_from_cmdline((char *)msg,AVOIDANCE,tmp_value )==0)
	{
	        GLOBAL_STATUS = MANUAL_STATUS;
			MoveDistanceDwa(atoi(tmp_value));
			
               // system("sudo ./dwa_control ");
	}else  if( get_value_from_cmdline((char *)msg,"start",tmp_value )==0)
	{
           GLOBAL_STATUS = STANDBY_STATUS;
	   GLOBAL_SWITCH = 1;
	}else  if( get_value_from_cmdline((char *)msg,"stop",tmp_value )==0)
        {
           GLOBAL_STATUS = STOP_STATUS;
           GLOBAL_SWITCH = 0;
        }

	// 处理后销毁数据
	

}

static int GetNextPackID(void)
{
	 static unsigned int pubpacketid = 0;
	 return pubpacketid++;
}

void *Mqtt_ClentTask(void *argv)
{
   sleep(2);
   mainSub(chargename);
	while (1)
	{
		
		printf("mainSub ReConnect\n");
		sleep(3);
		mainSub(chargename);
	}




}
void *Mqtt_PublishTask(void *argv)
{
	
	stop_init();

	
	sleep(2);
    mainPub(chargename);
	while (1)
	{
		
		printf("ReConnect\n");
		sleep(3);
	}

	#if 0  
	{
		

		if (user_Client.Status == Connect){
			Creatstatejson(heading,rollAngle,pitchAngle);
			//printf("mqtt sensor: %s \n",send_buf);
			if (MQTTMsgPublish(user_Client, "/update/state", QOS0, 0,
				(unsigned char *)send_buf,strlen(send_buf)) != 0)
			{
				printf("<0\n");
			}	
			memset(send_buf,0,sizeof(send_buf));
		}
		sleep(1);
	}
	#endif

	return 0;
}
