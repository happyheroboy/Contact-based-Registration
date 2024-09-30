#ifndef HIKCamera_H_INCLUDED
#define HIKCamera_H_INCLUDED
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include<io.h>
#else
#include<unistd.h>
#endif

#include <stdlib.h>
#include "MvCameraControl.h"
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/video/video.hpp>
using namespace std;
using namespace cv;


//海康威视相机SDK改写

class Camera {
private:
    void* handle;
    bool g_bExit;
    int nRet;
    unsigned int g_nPayloadSize;
    unsigned char* pDataForRGB;
    MV_CC_DEVICE_INFO* pDeviceInfo;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    MVCC_INTVALUE stParam;
    MV_FRAME_OUT stOutFrame;
    MV_CC_PIXEL_CONVERT_PARAM CvtParam;
public:
    Camera();
    void PrintDeviceInfo();
    void close_cam();
    void start_cam(unsigned int nIndex);
    void get_pic(Mat* srcimg);
    void re_iso();
};
#endif // HIKCamera_H_INCLUDED
#pragma once
#pragma once
