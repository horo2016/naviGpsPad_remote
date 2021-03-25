#include "online_client.h"

#include "cpu_sys.h"
#include "cJSON.h"
#include <stdio.h>
#include <sys/time.h>
#include "stdlib.h"
#include "time.h"
#include "stdarg.h"
#include  <string.h>
#include "md5.h"

#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>

#include <pthread.h>
#include <sys/types.h>
#include <sys/wait.h>



#define READ_DATA_SIZE	1024
#define MD5_SIZE		16
#define MD5_STR_LEN		(MD5_SIZE * 2)

//需要的时间
#define MINS 2 //3分钟

#define DELAY_TIME (60*MINS)
//腾讯云的api　key
#define TENCENT_API_KEY "6cda35f9b9e7ccb1e300e13cd3f470fa"
#define TENCENT_API_KEY2  "f7d01bc04f0e8597bd7ebaa2e87f1ac1"

#define MQTT_HOSt  "www.woyilian.com"
char json_file[1024];
char txsecreat_buf[0xff]={0};
char tx_time[0xff];


static void process_qrcodeText(const void *p, size_t length);

void get_str_md5(const char *str, char *md5_str)
{
	MD5_CTX md5;
	unsigned char md5_value[MD5_SIZE];
	int i=0;
 
	MD5Init(&md5);
	MD5Update(&md5,(unsigned char *) str, strlen(str));
	
	MD5Final(&md5, md5_value);
 
	for(i = 0; i < MD5_SIZE; i++)
	{
		snprintf(md5_str + i*2, 2+1, "%02x", md5_value[i]);
	}
	md5_str[MD5_STR_LEN] = '\0'; // add end
}

int CreatOnlinejson()
{
	char tmp_buf[0xff]={0};
	char topic_buf[0xff]={0};
	unsigned char  value_buf[0xff]={0};
	 char  send_buf[0xff]={0};

	memcpy(topic_buf,"online",sizeof("online"));
	
	memset(tmp_buf,0,sizeof(tmp_buf));
	sprintf(tmp_buf,"\"%s\"",chargename);

	

    cJSON * root =  cJSON_CreateObject();
    if(!root) {
         printf("get root faild !\n");
     }else printf("get root success!\n");
 
    cJSON_AddItemToObject(root, "\"type\"", cJSON_CreateNumber(1));
    cJSON_AddItemToObject(root, "\"sn\"", cJSON_CreateString(tmp_buf));//
   
  
    memcpy(value_buf,cJSON_Print(root),strlen(cJSON_Print(root)));
      
	printf("value_buf %s \n",value_buf);

	sprintf(send_buf,"mosquitto_pub -h  %s -t %s  -m \"%s\"",MQTT_HOSt,topic_buf,value_buf);

	printf("publish:%s\n",send_buf);
	system(send_buf);


    cJSON_Delete(root);

    return 0;
}



int CreatQrcodejson()
{
	char tmp_buf[0xff]={0};
	char topic_buf[0xff]={0};
	unsigned char  value_buf[0xff]={0};
	 char  send_buf[0xff]={0};
	 char md5_str[MD5_STR_LEN + 1];
	 char md5_str_id[MD5_STR_LEN + 1];

	 char md5_str_license[12];

//	memcpy(topic_buf,"online",sizeof("online"));
	 time_t cur_time;
	struct tm *tm_buf = NULL;

	time (&cur_time);
	tm_buf = localtime (&cur_time);
	memset (tmp_buf, 0, sizeof (tmp_buf));
	sprintf ((char *)tmp_buf, "%d-%d-%d %02d:%02d:%02d", tm_buf->tm_year + 1900, tm_buf->tm_mon + 1, tm_buf->tm_mday, tm_buf->tm_hour, tm_buf->tm_min, tm_buf->tm_sec);

	memset(md5_str_license,0,12);
	get_str_md5((const char *)tmp_buf, md5_str);
	memcpy(md5_str_license,md5_str,5);
	//printf("time %s \n",md5_str);
	
	get_str_md5((const char *)chargename, md5_str_id);
	//printf("id:%s-%s\n",chargename,md5_str_id);
	memcpy(md5_str_license+5,md5_str_id,5);

	

	


	sprintf(topic_buf,"{\"TIME\":\"%s\",\"SN\":\"%s\",\"LICENSE\":\"%s\"}",tmp_buf,chargename,md5_str_license);
	//printf("value_buf %s \n",topic_buf);

	//sprintf(send_buf,"echo -e \"%s\"  > qrcode.txt",topic_buf);
	process_qrcodeText(topic_buf,strlen(topic_buf));
	system("sudo cp qrcode.txt /usr/local/boa/qrcodejs/");


   

    return 0;
}
static void process_qrcodeText(const void *p, size_t length)
{
    FILE *save_fd  = fopen("qrcode.txt", "w");
    fputc ('.', stdout);
    fwrite(p, sizeof(char), length, save_fd);
    fflush (stdout);
    fclose(save_fd);
}

void read_json()
{
	FILE *fp=NULL;
	if((fp=fopen("res.json","a+"))==NULL)
	{
	perror("fopen error\n");
	}
	
	fread(json_file,1024,1,fp);
	fclose(fp);
}
//十进制数字转换成十六进制字符串
char * num2hex(unsigned int num)
{
    int i;
    unsigned int tmp;
unsigned int mask = 0xf;            //掩码0x 0000 000f
    static char hex[64];  //存储十六进制字符串

    hex[0] = '0';               //前两位固定不变，为0x
    hex[1] = 'x'; 
    
    //为其余元素赋值
    for(i = 9; i >= 2; i--)
    {
        tmp = num & mask;
	    hex[i] = (tmp >= 10) ? ((tmp - 10)  + 'a') : (tmp + '0'); 
	    num  = num >> 4;
    }

    return (hex+2);
}

void start_rtmpPush()
{
	char tmpBuf[0xff]={0};
	  
	   
	  sprintf(tmpBuf,"ffmpeg	-f	alsa  -ac 1   -i hw:1,0    \
	  	-f video4linux2	-s	320x240   -r  20 -i /dev/video0 \
	  	-vcodec libx264  -preset:v ultrafast -tune:v zerolatency	\
	  	-x264opts b-adapt=2:bframes=0:aq-strength=1:psy-rd=0.8,0 -acodec aac	\
	  	-max_delay 100  -f   flv	 -g 5 -b 400000  \  
	  	\"rtmp://129197.livepush.myqcloud.com/live/%s?txSecret=%s&txTime=%s\" > /dev/null",chargename,txsecreat_buf,tx_time);
	  printf("%s\n",tmpBuf);
	  system(tmpBuf);


}
void send_dataGetUrl()
{
   char tmpBuf[0xff]={0};
   
	
   sprintf(tmpBuf,"curl -o res.json -s -d \"key=%s&streamName=%s\"  -X POST \"http://www.woyilian.com/gps_car/Login/getPushUrl\"",TENCENT_API_KEY2,chargename);
   printf("%s\n",tmpBuf);
   system(tmpBuf);
}
int  parse_retcjson(char *a)
{

	

	cJSON *root=cJSON_Parse(a); 

	cJSON* msg = cJSON_GetObjectItem(root,"msg");
	if(!strcmp(msg->valuestring,"failed"))
			return -1;
	cJSON* test_arr = cJSON_GetObjectItem(root,"data");

	int arr_size = cJSON_GetArraySize(test_arr);//return arr_size 2
	
	cJSON* arr_item = test_arr->child;//子对象

	for(int i = 0;i <=(arr_size-1)/*0*/;++i){
	cJSON *secreat = cJSON_GetObjectItem(arr_item,"txSecret");
	cJSON *txtime = cJSON_GetObjectItem(arr_item,"txTime");
	memset(txsecreat_buf,0,0xff);
	memset(tx_time,0,0xff);
	strcpy(txsecreat_buf,secreat->valuestring);
	strcpy(tx_time,txtime->valuestring);
	printf("secreat value =%s\n",secreat->valuestring);
	printf("txtime value =%s\n",txtime->valuestring);
	arr_item = arr_item->next;//下一个子对象
	}
	
	cJSON_Delete(root);

}

void *Online_ClientTask(void *arg)
{
	

	/*memset(txsecreat_buf,0,0xff);
	strcpy(txsecreat_buf,TENCENT_API_KEY);
	strcat(txsecreat_buf,chargename);
	strcat(txsecreat_buf,txtime);
	printf("split:%s\n",txsecreat_buf);
	get_str_md5((const char *)strdup(txsecreat_buf), tencent_license);
	printf("tencent_license:%s \n",tencent_license);
	*/

	CreatQrcodejson();
	while(1)
		{
			CreatOnlinejson();
			sleep(DELAY_TIME);
	}
}


void *Rtmp_pushTask(void *arg)
{
	int ret =0;

	sleep(2);
	send_dataGetUrl();
	usleep(800000);
	read_json();
	ret = parse_retcjson(json_file);
	if(ret == -1){
	printf("json file parse failed \n");
	}else 
	{
	start_rtmpPush();

	}

}

