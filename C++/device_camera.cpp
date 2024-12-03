#include "device_camera.h"

bool device_camera::get_HIKcamera(void*& handle,int i)
{
	MV_CC_DEVICE_INFO_LIST stDeviceList;
	memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
	int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_CAMERALINK_DEVICE, &stDeviceList);

	if (nRet != MV_OK)
	{
		printf("Enum Devices fail! nRet [0x%x]\n", nRet);
		return false;
	}

	if (stDeviceList.nDeviceNum <= 0)
	{
		printf("Find No Devices!\n");
		return false;
	}

	// 选择第一个海康相机
	nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[i]);
	//nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
	if (nRet != MV_OK)
	{
		printf("Create Handle fail! nRet [0x%x]\n", nRet);
		return false;
	}

	// 打开
	nRet = MV_CC_OpenDevice(handle);
	if (nRet != MV_OK)
	{
		printf("Open Device fail! nRet [0x%x]\n", nRet);
		return false;
	}
	nRet = MV_CC_StartGrabbing(handle);
	if (MV_OK != nRet)
	{
		printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
		return false;
	}

	return true;
}

bool device_camera::HIKframe2mat(MV_FRAME_OUT& pstImage, Mat& srcImage)
{
	if (pstImage.stFrameInfo.enPixelType == PixelType_Gvsp_Mono8)
	{
		srcImage = Mat(pstImage.stFrameInfo.nHeight, pstImage.stFrameInfo.nWidth, CV_8UC1, pstImage.pBufAddr);
		return true;
	}
	else
	{
		printf("only support Mono8 pixel format\n");
		return false;
	}

	return false;
}