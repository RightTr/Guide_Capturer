#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include "guideusbcamera.h"
#include <unistd.h>
#include <malloc.h>
#include <string.h>
#include "sys/time.h"
#include "time.h"
#include <pthread.h>
#include <stdbool.h>
#include <fcntl.h>


int frameCallBack(guide_usb_frame_data_t *pVideoData);
int connectStatusCallBack(guide_usb_device_status_e deviceStatus);

int main(int arc, char* argv[])
{
    if(arc < 4)
    {
    	printf("please input width height cmos_model device_version after ./Demo \n");
    }
    int nWidth = atoi(argv[1]);
    int nHeight = atoi(argv[2]);
	int nMode = atoi(argv[3]);  
	int nVersion = atoi(argv[4]);    
    
    guide_usb_setLogLevel(LOG_TEST);
    int ret = guide_usb_initialize("/dev/video0");
    if(ret < 0)
    {
        printf("Initial fail:%d \n",ret);
        return -1;
    }

    guide_usb_device_info_t* deviceInfo = (guide_usb_device_info_t*)malloc(sizeof (guide_usb_device_info_t));
    deviceInfo->width = nWidth;
    deviceInfo->height = nHeight;
    deviceInfo->video_mode = nMode;
    deviceInfo->device_version = nVersion;

    ret = guide_usb_openStream(deviceInfo,(OnFrameDataReceivedCB)frameCallBack,(OnDeviceConnectStatusCB)connectStatusCallBack);
    if(ret < 0)
    {
        printf("Open fail! %d \n",ret);
        return ret;
    }

    int count = 60000;
    while(count--)
    {
      usleep(10);
    }

    ret = guide_usb_closeStream();
    printf("close usb return %d\n",ret);

    ret = guide_usb_exit();
    printf("exit return %d\n",ret);

    return ret;
}

int connectStatusCallBack(guide_usb_device_status_e deviceStatus)
{
    if(deviceStatus == DEVICE_CONNECT_OK)
    {
        printf("VideoStream is Starting...\n");
    }
    else
    {
        printf("VideoStream is closing...\n");
    }
    return 0;
}

int frameCallBack(guide_usb_frame_data_t *pVideoData)
{
  printf("frameCallBack, received data frame_width = %d, frame_height = %d \n", pVideoData->frame_width, pVideoData->frame_height);
  return 0;
}

