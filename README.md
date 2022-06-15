# navigpspad remote raspbery
raspberry robot 
该工程放到机器人终端上。
IMU :可以使用rtimulib 库通过IIC读取gy85或者  mpu9250的陀螺仪、加速度、地磁计数据，使用mqtt传输到网页上显示。
GPS：使用串口读取 M8N的数据通过卡尔曼滤波后传输到网页上显示。


Makefile 通过指定   
#是否用远程mqtt server 
#CXX_OPTS += -DMQTT_REMOTE_SERVER 
#是否将终端作为mqtt server 
CXX_OPTS +=-DMQTT_TERMINAL_SERVER

机器人终端安装mosquitto 的库，并使用BOA服务器和终端进行交互。
交互页面参考 QGC和mission planer 设计。
![image](https://github.com/horo2016/gps_web_server/blob/master/%E5%BE%AE%E4%BF%A1%E5%9B%BE%E7%89%87_20220614202619.png)
