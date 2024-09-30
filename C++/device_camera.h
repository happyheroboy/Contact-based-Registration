#pragma once
#include "config.h"

using namespace std;
using namespace cv;

class device_camera
{
	public:
	// º£¿µÏà»ú
	static bool get_HIKcamera(void*& handle,int i);
	static bool HIKframe2mat(MV_FRAME_OUT& pstImage,Mat& srcImage);
};

