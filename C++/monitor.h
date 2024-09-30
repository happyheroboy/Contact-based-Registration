#pragma once
#include "config.h"
#include "Data.h"
/*
#include "constructor_hexPair.h"
#include "marker_deltille.h"
#include "solver_pose.h"*/

class monitor
{
public:
	static void debug_show(const Data::detector& data);
	static void debug_show(const Data::constructor& data);
	static void debug_show(const vector<Mat1b>& img, const Data::poseSolver& data);
	static void Xray_show(const Mat& img, const Mat& RT, const Mat1d& T, const vector<Mat1d>& sys_key_points);
	static void XrayImg_show( Mat& img, const Mat1d& T,const Mat1d& PinLocCT, const Mat1d& EndLocCT, String name, const Mat1d& samplePtsCT, double radius_drill);//正位片显示
	static void registration_show(const vector<Mat1d>& sys_key_points, const vector<Mat1b>& img, const vector<Vec3d>& normalxyz, const Data::poseSolver& data);
	static void CT_img_show(vector<Mat>& CT_img, const Mat& RT, const vector<Mat1d> CT_img_scal, const vector<Mat1d>& sys_key_points);//CT图片上显示
	static void ceWeiXrayImg_show(Mat& img, const Mat1d& T, const Mat1d& PinLocCT, const Mat1d& EndLocCT, String name, const Mat1d& samplePtsCT, double radius_drill);//侧位片显示
	static void putTextInChinese(Mat& dst, const char* str, Point org, Scalar color, int fontsize,
		const char* fn = "Arial", bool italic = false, bool underline = false);//显示中文汉字
	static void GetStringSize(HDC hdc, const char* str, int* w, int* h);

	static void recordChange(Mat& img, const Mat1d& T, vector<Mat1d>& drill_center_r_vector,string name, string mainFilename);


	//static void recordChange(Mat& img, const Mat1d& T, vector<Mat1d>& drill_center_r_vector, Mat& Xray_img_ZhengWei_new);//记录上一次挫头的位置

	/*static void show(const Mat& img, const constructor_hexPair::record& record);
	static void show(const Mat& img, const maker_deltille::record& record);
	static void show(const Mat& img, visualization::PCLVisualizer viewer, int v1, int v2, const solver_pose::record& record);*/

private:
	
	// 抗锯齿绘制中的级数
	static const int shift = 5;

	static Mat3b get_imgShow(const Mat& img);
	static void put_timecost(Mat3b& img, const vector<Data::time_cost>& info);
	static void put_number(Mat3b& img, const vector<String>& name, const vector<double>& times, const String suffix);
	static void put_number(Mat3b& img, const vector<String>& name, const vector<double>& times, const vector<String>& suffix);
	static void DrawArc(Mat& src, Point2f ArcCenter, Point2f StartPoint, Point2f EndPoint, int Fill, Scalar color);
	static string ToString(double val);

};

