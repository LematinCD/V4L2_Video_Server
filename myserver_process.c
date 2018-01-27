/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename:myserver_process.c
*   Author  :Lematin
*   Date    :2017-2-17
*   Describe:
*
********************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <semaphore.h>

#include "myserver_process.h"
#include "myconfig.h"

static unsigned int cmd = 4;
static unsigned int Length = 0;
static unsigned char dev_name[12] ="/dev/video0\n";
static unsigned int dev_name_length = 12;
static Open_Camera_Flag = 0;
typedef struct my_image_tag{
        unsigned int image_size;
        unsigned char image_buf[MY_WIDE*MY_HIGH];
}test_my_image;
test_my_image my_image;
pthread_mutex_t camera_mutex;

enum my_net_cmd{
      	Login = 0x1,
        LoginSuccess,
        LoginFailed,

        Register,
        RegisterSuccess,
        RegisterFailed,

        
        Change_Password,
        Change_Password_Success,
        Change_Password_Failed,

        Dev_list,

        Open_Camera_Yes_Or_No = 0x20,
		Open_Camera_Yes,
		Open_Camera_No,
        Open_Camera_Success,
        Open_Camera_Failed,
        
        Close_Camera_Yes_Or_No,
		Close_Camera_Yes,
		Close_Camera_No,
        Close_Camera_Success,
        Close_Camera_Failed,
        
        /*灯 控制命令组 开 关 亮度*/
        Open_Led = 0x30,
        Open_Led_Success,
        Open_Led_Failed,
        Close_Led,
        Close_Led_Success,
        Close_Led_Failed,


        Open_Fan = 0x40,
        Open_Fan_Success,
        Open_Fan_Failed,
        Close_Fan,
        Close_Fan_Success,
        Close_Fan_Failed,
        
       	Open_Beep,
        Open_Beep_Success,
        Open_Beep_Failed,
        Close_Beep,
        Close_Beep_Success,
        Close_Beep_Failed,
        

        Open_Sensor = 0x60,
        Open_Sensor_Success,
        Open_Seneor_Failed,
        Close_Sensor,
        Close_Sensor_Success,
        Close_Sensor_Faile,

		Quit
}my_cmd;


static unsigned int uchar_to_uint(const unsigned char *bufdata,int offset)
{
	int i = 0;
	unsigned int intdata = 0;
	intdata =((bufdata[offset] & 0xFF)
			|((bufdata[offset+1] & 0xFF)<<8) 
			|((bufdata[offset+2] & 0xFF)<<16) 
			|((bufdata[offset+3]&0xFF)<<24));	
	return  intdata;        
}

static void toString(char *dstbuf,char *srcbuf,unsigned int size)
{
	int i = 0;
	for(i=0;i<size/2;i++){
		dstbuf[i] = srcbuf[2*i];
	}
	dstbuf[i]='\0';
}

void send_video_data_handle(void *args)
{
	int ret=0,count = 0;
	if(0 != pthread_mutex_lock(&camera_mutex)){
		printf("write cam_mutex locl fail!\n");
		return;
	}
	
//	Length = my_image.image_size+5;
//	memcpy(my_image.image_buf,(char *)&Length,4);
//	my_image.image_buf[4] = (char)Open_Camera_Success;
//	printf("send_video_data_handle my_image.image_buf[4]:%x\n",my_image.image_buf[4]);
//	printf("conn_fd:%d,image_size:%d\n",*(int*)args,my_image.image_size);
	ret = write(*(int *)args,my_image.image_buf,my_image.image_size);
	count+=ret;
	while(count!=my_image.image_size){
		ret = write(*(int *)args,my_image.image_buf,my_image.image_size-count);
		count+=ret;
	}
	if(0 != pthread_mutex_unlock(&camera_mutex)){
		printf("write cam_mutex locl fail!\n");
		return;
	}
		
}
void recv_video_data_handle(void *args,int length)
{
	int ret=0,count =0,i=0;

	if(0 != pthread_mutex_lock(&camera_mutex)){
		printf("write cam_mutex locl fail!\n");
		return;
	}
	memset(&my_image,0,sizeof(my_image));
	my_image.image_size = length+4;
	//ret=read(*(int *)args,my_image.image_buf,my_image.image_size);
	//count+=ret;
	while(count!=my_image.image_size){
		ret=read(*(int *)args,&my_image.image_buf[count],my_image.image_size-count);
		count+=ret;
	}
	//printf("my_image.image_size:%d\n",my_image.image_size);
	for(i=0;i<10;i++){
//	printf("recv_video_data_handle image_buf[%d]:%x\n",i,my_image.image_buf[i]);
		
	}
	if(0 != pthread_mutex_unlock(&camera_mutex)){
		printf("write cam_mutex locl fail!\n");
		return;
	}
}


void uartdata_send_handle(void *args)
{
	  
}

void uartdata_recv_handle(void *args,char flag)
{
	  
}


void login_regster_changepwd_handle(int *conn_fd,char *recv_buf,char flag,char *send_buf)
{
}




/*重写处理函数*/
void *my_net_process(void *args)
{
	if(0 != pthread_mutex_init(&camera_mutex,NULL)){
		printf("uart_read_mutex error!\n");
		return;
	}

	int *conn_fd = (int *)args;
#ifdef DEBUG	
	printf("conn_fd:%d\n",*conn_fd);
#endif	
	int ret = 0;
	int i = 0;
	unsigned int length = 0;
        char send_buf[255];
        char recv_buf[255];
	int net_data_size =0;        
	while(1){
		memset(send_buf,0,255);
        memset(recv_buf,0,255);
		if(0>read(*conn_fd,recv_buf,5))	
			return NULL;
		printf("conn_fd:%d,recv_buf[4]:%x\n",*conn_fd,recv_buf[4]);
		switch(recv_buf[4]){
			case Login:  
			case Register:
			case Change_Password:
				login_regster_changepwd_handle(conn_fd,recv_buf,recv_buf[4],send_buf);  
				break;

			case Dev_list:
                break;
			case Open_Camera_Yes_Or_No:
				printf("open camera yes or no,conn_fd:%d\n",*conn_fd);
				if(Open_Camera_Flag == 0){
					send_buf[4] = (unsigned char)Open_Camera_No;
					write(*conn_fd,send_buf,5);
				}else if(Open_Camera_Flag == 1){
					send_buf[4] = (unsigned char )Open_Camera_Yes;
					write(*conn_fd,send_buf,5);
					read(*conn_fd,recv_buf,5);
					length = uchar_to_uint(recv_buf,0);
					recv_video_data_handle(conn_fd,length);
				}
				break;
			case Open_Camera_Success:
				//recv_video_data_handle(conn_fd,length);
				break;
			case Open_Camera_Yes:
				Open_Camera_Flag = 1;
				printf("open camera from android!conn_fd:%d\n",*conn_fd);
				send_video_data_handle(conn_fd);
				break; 
			case Close_Camera_No:
				printf("close camera from android!!!!!!!!!!!!!!!!!!!!!!!!1\n");
				Open_Camera_Flag = 0;
				send_buf[4] = (unsigned char)Close_Camera_Success;
				write(*conn_fd,send_buf,5);
				//close(*conn_fd);
				//free(conn_fd);
				return;
				break;


			case Open_Beep:
			case Close_Beep:
			case Open_Led:
			case Close_Led:
			case Open_Fan:
			case Close_Fan:
				uartdata_recv_handle(conn_fd,recv_buf[4]);
				break;

			case Open_Sensor:
                                uartdata_send_handle(conn_fd);
				break;
			case Close_Sensor:
				close(*conn_fd);
				free(conn_fd);
				return;
				break;
			case Quit:
				close(*conn_fd);
				free(conn_fd);
				return;
				break;	

			default:
				break;        

		}
                usleep(50000);
	}
	
}
