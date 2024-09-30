#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/gapi/core.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include<opencv2/opencv.hpp>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <MvCameraControl.h>


//#include <pcl/visualization/cloud_viewer.h>
//#include "vtkoutputwindow.h" 

#include <map>
#include <fstream>
#include<iomanip>

using namespace cv;
using namespace std;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

#define TIME_START(X)	double X = (double)getTickCount();
#define TIME_END(X)		X = 1000 * ((double)getTickCount() - X) / getTickFrequency();
#define TRY_THROW(X,Y)	if (!(X))	throw __FUNCTION__ + string(", ") + (Y);

typedef Vec<double, 5> Vec5d;
typedef Vec<double, 6> Vec6d;
typedef Vec<double, 7> Vec7d;
typedef Vec<double, 12> Vec12d;
typedef Vec<int, 10> Vec10i;
typedef Vec<int, 11> Vec11i;

typedef Matx<double, 3, 4> MatRT;		// RT
typedef Matx<double, 3, 4> camMatrix;	// 相机参数

// 检查path指向的文件后缀名是否为suffix
bool check_suffix(const string path, const string suffix);

// vector -> mat
template<typename T>
Mat vec2mat(const vector<T>&vec)
{
	if (vec.empty())	return Mat();
	return Mat(vec).reshape(1).t();
}

template<typename T, int num>
Mat vec2mat(const Vec<T, num>&vec)
{
	return Mat(vec).reshape(1);
}

// 齐次->非齐次
vector<Vec3d> remove_homo(const vector<Vec4d>& homo);
vector<Vec2d> remove_homo(const vector<Vec3d>& homo);
Mat remove_homo(const Mat& homo);

// 非齐次->齐次
vector<Vec4d> add_homo(const vector<Vec3d>& inhomo);
vector<Vec3d> add_homo(const vector<Vec2d>& inhomo);
Mat add_homo(const Mat& inhomo);

// 比较函数
struct ComparePoint2f{
	bool operator () (const Point2f& a, const Point2f& b) const{
		return (a.x < b.x) || (a.x == b.x && a.y < b.y);
	}
};

struct CompareVec2i {
	bool operator () (const Vec2i& a, const Vec2i& b) const {
		return (a[0] < b[0]) || (a[0] == b[0] && a[1] < b[1]);
	}
};

struct CompareVec3i{
	bool operator () (const Vec3i& a, const Vec3i& b) const{
		return (a[0] < b[0]) || (a[0] == b[0] && a[1] < b[1]) || (a[0] == b[0] && a[1] == b[1] && a[2] < b[2]);
	}
};