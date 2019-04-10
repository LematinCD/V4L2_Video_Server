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
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>



#include <jpeglib.h>
#include <opencv2/objdetect/objdetect.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp> 
#include "myserver_process.h"
#include "myserver_thread_pool.h"
#include "myconfig.h"
using namespace cv;

#define	RUN 	1
#define STOP	0

static unsigned int cmd = 4;
static unsigned int Length = 0;
static unsigned int dev_name_length = 12;
static int Open_Camera_Flag = 0;
typedef struct my_image_tag{
        unsigned int image_size;
        unsigned char image_buf[MY_WIDE*MY_HIGH];
}test_my_image;
test_my_image my_image;
pthread_mutex_t camera_mutex;

pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
int status = STOP;

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
	
	ret = write(*(int *)args,my_image.image_buf,my_image.image_size);
	count+=ret;
	while(count!=my_image.image_size){
		ret = write(*(int *)args,my_image.image_buf,my_image.image_size-count);
		count+=ret;
	}
	//printf("Lex---send over!\n");
	if(0 != pthread_mutex_unlock(&camera_mutex)){
		printf("write cam_mutex locl fail!\n");
		return;
	}
		
}

Mat Jpeg2Mat(unsigned char *jpegData, int jpegSize)
{
	struct jpeg_error_mgr jerr;
	struct jpeg_decompress_struct cinfo;
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_decompress(&cinfo);
	jpeg_mem_src(&cinfo, jpegData, jpegSize);  
	
	jpeg_read_header(&cinfo, TRUE);
	jpeg_start_decompress(&cinfo);

	int nRowSize = cinfo.output_width * cinfo.output_components;
	int w =cinfo.output_width;
	int h =cinfo.output_height;

	char *bmpBuffer = (char*)malloc(h*w*3);
	JSAMPARRAY pBuffer = (*cinfo.mem->alloc_sarray)((j_common_ptr) &cinfo, JPOOL_IMAGE, nRowSize, 1);
	while(cinfo.output_scanline < cinfo.output_height)
	{
		jpeg_read_scanlines(&cinfo, pBuffer, 1);
		int start=nRowSize*(cinfo.output_scanline-1);
		memcpy(bmpBuffer+start, pBuffer[0],nRowSize);
	}
	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);

	Mat mMat(h, w, CV_8UC3,bmpBuffer);
	Mat m(h, w, CV_8UC3, cv::Scalar(0,0,255));
	cvtColor(mMat, m, COLOR_RGB2BGR);
	free(bmpBuffer);
	bmpBuffer=NULL;
	return m;
}


long unsigned int MoveDetect(Mat background, Mat frame,Mat *ret_frame)
{
	Mat result = frame.clone();
	//1.将background和frame转为灰度图
	Mat gray1, gray2;
	cvtColor(background, gray1, CV_BGR2GRAY);
	cvtColor(frame, gray2, CV_BGR2GRAY);
	//2.将background和frame做差
	Mat diff;
	absdiff(gray1, gray2, diff);
	//imshow("diff", diff);
	//3.对差值图diff_thresh进行阈值化处理
	Mat diff_thresh;
	threshold(diff, diff_thresh, 50, 255, CV_THRESH_BINARY);
	//imshow("diff_thresh", diff_thresh);
	//4.腐蚀
	Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(15, 15));
	erode(diff_thresh, diff_thresh, kernel_erode);
	//imshow("erode", diff_thresh);
	//5.膨胀
	dilate(diff_thresh, diff_thresh, kernel_dilate);
	//imshow("dilate", diff_thresh);
	//6.查找轮廓并绘制轮廓
	vector<vector<Point> > contours;
	findContours(diff_thresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	drawContours(result, contours, -1, Scalar(0, 0, 255), 2);//在result上绘制轮廓
	//7.查找正外接矩形
	vector<Rect> boundRect(contours.size());
#ifdef DEBUG
	for (int i = 0; i < contours.size(); i++)
	{
		boundRect[i] = boundingRect(contours[i]);
		rectangle(result, boundRect[i], Scalar(0, 255, 0), 2);//在result上绘制正外接矩形
	}
	*ret_frame = result.clone(); 
#endif
	return  contours.size();

}
void thread_send_alarm_resume()
{
	printf("Lex---thread_send_alarm_resume\n");
	if (status == STOP)
	{
		pthread_mutex_lock(&mut);
		status = RUN;
		pthread_cond_signal(&cond);
		printf("pthread run!\n");
		pthread_mutex_unlock(&mut);
	}
	else
	{
		printf("pthread send_alarm run already\n");
	}
}

void thread_send_alarm_pause()
{
	printf("Lex---thread_send_alarm_pause\n");
	if (status == RUN)
	{
		pthread_mutex_lock(&mut);
		status = STOP;
		printf("thread stop!\n");
		pthread_mutex_unlock(&mut);
	}
	else
	{
		printf("pthread send alarm pause already\n");
	}
}

#define IPSTR "39.104.164.214"
#define PORT 8091
#define BUFSIZE 1024
int send_alarm_handle()
{
	printf("Lex---send_alarm_handle()\n");
    int sockfd, ret;
    struct sockaddr_in servaddr;
    char str1[4096], str2[4096], recv_str3[4096],buf[BUFSIZE], *str;
    socklen_t len;
    fd_set   t_set;

    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
		printf("创建网络连接失败,本线程即将终止---socket error!\n");
		goto err;
    }

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    if (inet_pton(AF_INET, IPSTR, &servaddr.sin_addr) <= 0 ){
		printf("创建网络连接失败,本线程即将终止--inet_pton error!\n");
		goto err;
    }

    if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0){
		printf("连接到服务器失败,connect error!\n");
		goto err;
    }
    printf("与远端建立了连接\n");


    //发送数据
    memset(str2, 0, 4096);

    //json格式 字符串的拼接
    strcat(str2, "{ \r\n");
    strcat(str2,  " \"equipmentId\": \"123\",\r\n");
    strcat(str2,  " \"equipmentName\": \"摄像头\",\r\n");
    strcat(str2,  " \"equipmentAlaMsg\": \"物体移动\"\r\n");
    strcat(str2,  "}\r\n"); 
    //printf("length=%d\n",strlen(str2)); 
    //printf("%s\n",str2);


    str=(char *)malloc(4096);
    len = strlen(str2);
    sprintf(str, "%d", len);

    memset(str1, 0, 4096);
    strcat(str1, "POST http://39.104.164.214:8091/alarm  HTTP/1.1\r\n");
    strcat(str1, "Host:39.104.164.214:8091\r\n");
    //strcat(str1, "Content-Type: application/x-www-form-urlencoded\n");
    strcat(str1, "Content-Type:application/json\r\n");
    //Content-Type:application/json
    strcat(str1, "Content-Length: ");
    strcat(str1, str);
    strcat(str1, "\r\n\r\n");

    strcat(str1, str2);
    strcat(str1, "\r\n\r\n");
    //printf("%s\n",str1);

    ret = write(sockfd,str1,strlen(str1));
    if (ret < 0) {
		printf("发送失败！错误代码是%d，错误信息是'%s'\n",errno, strerror(errno));
		goto err;
    }else{
		printf("消息发送成功，共发送了%d个字节！\n\n", ret);
		printf("%s\n",str1);
    }
	
    read(sockfd,recv_str3,1024);
    printf("Lex---recv_str3:%s\n",recv_str3);
	
    FD_ZERO(&t_set);
    FD_SET(sockfd, &t_set);
    close(sockfd);
    free(str);
    return 0;

err:
	FD_ZERO(&t_set);
    FD_SET(sockfd, &t_set);
    close(sockfd);
	thread_send_alarm_pause();
	return -1;
}



void *pthread_send_alarm_fun(void *arg)
{
	while(1){
		pthread_mutex_lock(&mut);
		while(!status)
		{
			pthread_cond_wait(&cond, &mut);
		}
		pthread_mutex_unlock(&mut);
		send_alarm_handle();
		//if(send_alarm_handle() == 0){
			//thread_send_alarm_pause();
		//}
		printf("lex---pthread_send_alarm_fun\n");
		thread_send_alarm_pause();
		usleep(10000);
	}
}

//添加发送警报线程
int thread_send_alarm_init()
{
	pool_add_task(pthread_send_alarm_fun,NULL);
}
static Mat pre_frame;//存储上一帧图像
static int alarm_frame_count = 0;
   
void recv_video_data_handle(void *args,int length)
{
	int ret=0,count =0,i=0;
	int fd;
	long unsigned int result;
	Mat ret_frame;
	if(0 != pthread_mutex_lock(&camera_mutex)){
		printf("write cam_mutex locl fail!\n");
		return;
	}
	memset(&my_image,0,sizeof(my_image));
	my_image.image_size = length+4;
	while(count!=my_image.image_size){
		ret=read(*(int *)args,&my_image.image_buf[count],my_image.image_size-count);
		count+=ret;
	}
	Mat frame = Jpeg2Mat(&my_image.image_buf[9],count-9);
	if(pre_frame.empty()){
		result = MoveDetect(frame, frame,&ret_frame);//调用MoveDetect()进行运动物体检测，返回值存入res
	}else{
		result = MoveDetect(pre_frame, frame,&ret_frame);//调用MoveDetect()进行运动物体检测，返回值存入res
	}
	pre_frame = frame.clone();
#ifdef DEBUG
	imshow("result",ret_frame);
	waitKey(50);
#endif
	
	if(result>3){
		
		alarm_frame_count++;
		printf("lex---alarm_frame_count:%d\n",alarm_frame_count);
		
	}
	if(alarm_frame_count >= 3){
		printf("Lex---检测到物体移动\n");
		thread_send_alarm_resume();
		alarm_frame_count = 0;
	}
	if(0 != pthread_mutex_unlock(&camera_mutex)){
		printf("write cam_mutex locl fail!\n");
		return;
	}
}





/*重写处理函数*/
void *my_net_process(void *args)
{
	if(0 != pthread_mutex_init(&camera_mutex,NULL)){
		printf("uart_read_mutex error!\n");
		return NULL;
	}

	int *conn_fd = (int *)args;
#ifdef DEBUG	
	printf("conn_fd:%d\n",*conn_fd);
#endif	
	int ret = 0;
	int i = 0;
	unsigned int length = 0;
    unsigned char send_buf[255];
    unsigned char recv_buf[255];
	int net_data_size =0;        
	while(1){
		memset(send_buf,0,255);
        memset(recv_buf,0,255);
		if(0>read(*conn_fd,recv_buf,5))	
			return NULL;
		//printf("conn_fd:%d,recv_buf[4]:%x\n",*conn_fd,recv_buf[4]);
		switch(recv_buf[4]){
			case Login:  
			case Register:
			case Change_Password:
				break;

			case Dev_list:
                break;
			case Open_Camera_Yes_Or_No:
				//printf("open camera yes or no,conn_fd:%d\n",*conn_fd);
				if(Open_Camera_Flag == 0){
					send_buf[4] = (unsigned char)Open_Camera_No;
					write(*conn_fd,send_buf,5);
				}else if(Open_Camera_Flag == 1){
					send_buf[4] = (unsigned char )Open_Camera_Yes;
					write(*conn_fd,send_buf,5);
					read(*conn_fd,recv_buf,5);
					length = uchar_to_uint(recv_buf,0);
					recv_video_data_handle(conn_fd,length);	
					//printf("Lex---recv\n");
				}
				break;
			case Open_Camera_Success:
				//recv_video_data_handle(conn_fd,length);
				break;
			case Open_Camera_Yes:
				Open_Camera_Flag = 1;
				//printf("open camera from android!conn_fd:%d\n",*conn_fd);
				send_video_data_handle(conn_fd);
				break; 
			case Close_Camera_No:
				//printf("close camera from android!!!!!!!!!!!!!!!!!!!!!!!!1\n");
				Open_Camera_Flag = 0;
				send_buf[4] = (unsigned char)Close_Camera_Success;
				write(*conn_fd,send_buf,5);
				//close(*conn_fd);
				//free(conn_fd);
				return NULL;
				break;


			case Open_Beep:
			case Close_Beep:
			case Open_Led:
			case Close_Led:
			case Open_Fan:
			case Close_Fan:
				//uartdata_recv_handle(conn_fd,recv_buf[4]);
				break;

			case Open_Sensor:
                                //uartdata_send_handle(conn_fd);
				break;
			case Close_Sensor:
				close(*conn_fd);
				free(conn_fd);
				return NULL;
				break;
			case Quit:
				close(*conn_fd);
				free(conn_fd);
				return NULL;
				break;	

			default:
				break;        

		}
                usleep(50000);
	}
	
}
