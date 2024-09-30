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
	//�Ӵ�ʽ��׼ ��
private:
	float Alpha_angle = 15; //ǰ���
	float Gamma_angle = 45; //��չ��
	bool isLeft = 1; // �ǲ�������
	double radius = 21.0; // ��ͷ�뾶
	Mat1d vLeft_ASIS = (Mat1d(3, 1) << 123.9426, -154.9814, -270.9255);//����  ����ǰ�ϼ�λ��
	Mat1d vRight_ASIS = (Mat1d(3, 1) << -118.5298, -153.2016, -262.4054);//����  ����ǰ�ϼ�λ��

	Mat1d v_CP;// ����  �Ӵ���λ��

	Mat1d N_Zaxis; //��ͷ��������
	Eigen::Matrix4f transformMatrix;//��ת���� ����ʹ�žʿ��ڳ���
	PointCloud<PointXYZ>::Ptr acetabulum_cloud; // �žʵ���
	PointCloud<PointXYZ>::Ptr rotated_acetabulum_cloud; // ��ת���žʵ���

	PointXYZ L_bPt; // ������λ��
	PointXYZ L_cPt; // ��ͽӴ���λ��

public:

	CR(const string filename); //���캯��
	~CR(); //��������

	// ����STLģ��
	void importModel(const string filename); 

	//��ǰ��Ǻ���չ��ת��Ϊ��������
	void angle2Vector();

	//����������������������ϵ���õ�������ϵ��������ϵ����ת����
	void buildNewCoordinate();
	
	//������RT
	void apply(const Mat1d& rLeft_ASIS, const Mat1d& rRight_ASIS, const Mat1d& real_CP, const Mat1d& end_pinpoint, Mat1d& R_rotation, Mat1d& t_trans);

	//�������Ż�
	void optimizedSamples(const PointXYZ origina_centerP, const PointCloud<PointXYZ>::Ptr& sample_Cloud, const PointXYZ cPt, PointCloud<PointXYZ>::Ptr& newSamplePt, PointXYZ& new_centerP);

	void getVitualContactPt();

	void getRTMatrix(const Mat1d& rLeft_ASIS, const Mat1d& rRight_ASIS, const Mat1d& real_CP, Mat1d& R_rotation);
	
	//��������������
	void buildSampleGrid(const PointXYZ initial_P, int n, double edgeLength, const PointCloud<PointXYZ>::Ptr& grid);

	void FindLowestBPt(const PointCloud<PointXYZ>::Ptr& sample_Cloud);
	
	// ��ʾ
	void displayPointCloud(const PointCloud<PointXYZ>::Ptr& cloud); 

	//��ʾ��׼���������
	void tri_display(const Mat1d& rLeft_ASIS, const Mat1d& rRight_ASIS, const Mat1d& real_CP, const Mat1d& R_Matrix, const Mat1d& t_Matrix);
	
	// ����½�ģ��
	PointCloud<PointXYZ>::Ptr downSimulationToFindBC(const PointXYZ initial_P, const PointCloud<PointXYZ>::Ptr& cloud, int radius); // 
	
	// ɸѡ��״����
	PointCloud<PointXYZ>::Ptr extractPointsInCylinder(const PointCloud<PointXYZ>::Ptr or_cloud, const PointXYZ center, double radius);
	
	// ����������
	void computeBottomPoint(const PointCloud<PointXYZ>::Ptr ac_cloud, const PointCloud<PointXYZ>::Ptr& grid, double radius, PointXYZ& Lowest_bPt, PointXYZ& Lowest_cPt);
};

