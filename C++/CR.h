#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <wingdi.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>

#include<opencv2/opencv.hpp>
#include <opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/gapi/core.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

using namespace cv;
using namespace std;
using namespace pcl;

class CR
{
	//接触式配准 类
private:
	float Alpha_angle = 15; //前倾角
	float Gamma_angle = 45; //外展角
	bool isLeft = 1; // 是不是左髋
	double radius = 21.0; // 球头半径
	Mat1d vLeft_ASIS = (Mat1d(3, 1) << 123.9426, -154.9814, -270.9255);//虚拟  左髂前上棘位置
	Mat1d vRight_ASIS = (Mat1d(3, 1) << -118.5298, -153.2016, -262.4054);//虚拟  右髂前上棘位置

	Mat1d v_CP;// 虚拟  接触点位置

	Mat1d N_Zaxis; //球头入射向量
	Eigen::Matrix4f transformMatrix;//旋转矩阵 —》使髋臼开口朝上
	PointCloud<PointXYZ>::Ptr acetabulum_cloud; // 髋臼点云
	PointCloud<PointXYZ>::Ptr rotated_acetabulum_cloud; // 旋转后髋臼点云

	PointXYZ L_bPt; // 最低落点位置
	PointXYZ L_cPt; // 最低接触点位置

public:

	CR(const string filename); //构造函数
	~CR(); //析构函数

	// 导入STL模型
	void importModel(const string filename); 

	//将前倾角和外展角转换为入射向量
	void angle2Vector();

	//根据入射向量构建新坐标系，得到旧坐标系到新坐标系的旋转矩阵
	void buildNewCoordinate();
	
	//迭代求RT
	void apply(const Mat1d& rLeft_ASIS, const Mat1d& rRight_ASIS, const Mat1d& real_CP, const Mat1d& end_pinpoint, Mat1d& R_rotation, Mat1d& t_trans);

	//采样点优化
	void optimizedSamples(const PointXYZ origina_centerP, const PointCloud<PointXYZ>::Ptr& sample_Cloud, const PointXYZ cPt, PointCloud<PointXYZ>::Ptr& newSamplePt, PointXYZ& new_centerP);

	void getVitualContactPt();

	void getRTMatrix(const Mat1d& rLeft_ASIS, const Mat1d& rRight_ASIS, const Mat1d& real_CP, Mat1d& R_rotation);
	
	//构建采样点网格
	void buildSampleGrid(const PointXYZ initial_P, int n, double edgeLength, const PointCloud<PointXYZ>::Ptr& grid);

	void FindLowestBPt(const PointCloud<PointXYZ>::Ptr& sample_Cloud);
	
	// 显示
	void displayPointCloud(const PointCloud<PointXYZ>::Ptr& cloud); 

	//显示配准三角形情况
	void tri_display(const Mat1d& rLeft_ASIS, const Mat1d& rRight_ASIS, const Mat1d& real_CP, const Mat1d& R_Matrix, const Mat1d& t_Matrix);
	
	// 落点下降模拟
	PointCloud<PointXYZ>::Ptr downSimulationToFindBC(const PointXYZ initial_P, const PointCloud<PointXYZ>::Ptr& cloud, int radius); // 
	
	// 筛选柱状电云
	PointCloud<PointXYZ>::Ptr extractPointsInCylinder(const PointCloud<PointXYZ>::Ptr or_cloud, const PointXYZ center, double radius);
	
	// 计算最低落点
	void computeBottomPoint(const PointCloud<PointXYZ>::Ptr ac_cloud, const PointCloud<PointXYZ>::Ptr& grid, double radius, PointXYZ& Lowest_bPt, PointXYZ& Lowest_cPt);
};

