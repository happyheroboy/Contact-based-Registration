#pragma once
#include "config.h"
#include "Data.h"
#include "kdtree.h"
#include <algorithm>
#include <iomanip>
#include <iostream>

// %% using namespace cv;
// %% using namespace std;

struct ptList_score {
	Point2f p;
	double score = 0;
	bool isRejected = false;
};

class PointForTree : public std::array<float, 2>
{
public:

	// dimension of kd-tree space
	// KDTree class accesses this member
	static const int DIM = 2;

	// the constructors
	PointForTree() {}
	PointForTree(float x, float y)
	{
		(*this)[0] = x;
		(*this)[1] = y;
	}

	operator Point2f() const { return Point2f((*this)[0], (*this)[1]); }
};

class detector_cross
{
public:

	void apply(Data::detector& data);

private:

	// 全局参数	
	bool parallel;
	int mask_r;
	int mask_R;

	// 模板拟合时需要使用的系数矩阵及其伪逆矩阵
	void createA();
	Mat1f A_tri, pA_tri;

	// 与matlab的同名方法相同
	void meshgrid(const Range xgv, const Range ygv, Mat1f& X, Mat1f& Y);

	// 提取图像中的疑似交叉点，输出其亚像素坐标，包含方法: FAST -> cornerSubPix -> kdTree NMS
	void preFilter(const Mat1b& img, const int FAST_T, const TermCriteria subPix, vector<Point2f>& ptList);

	// 使用kdTree寻找聚集的点，并给出删除建议
	void merge_kdTree(const vector<Point2f>& ptList, const Size pt_scope, const float r, vector<bool>& remove);

	// 基于三次多项式拟合猴鞍面，产生亚像素检测结果并判别
	void refine_tri(const Mat1b& img, vector<Point2f>& ptList);

	// 关于一个点的refine_tri，会多次修改p（Point2f(-1,-1)表示该点被判别为非特征）
	void refine_tri_single(const Mat1b& img, Point2f& p);

private:
	//***************************************************************************************************
	struct p_cornerSubPix :ParallelLoopBody
	{
		detector_cross* p_class;
		Mat1b img;
		TermCriteria subPix;
		Mat_<Point2f> ptList;

		p_cornerSubPix(detector_cross* p_class ,const Mat1b& img, const TermCriteria subPix, vector<Point2f>& ptList)
		{
			this->p_class = p_class;
			this->img = img;
			this->subPix = subPix;
			this->ptList = Mat_<Point2f>(ptList);
		}
		void operator()(const Range& idx) const
		{
			for (int t = idx.start; t < idx.end; t++)
			{
				vector<Point2f> p_refine = { ptList(t) };
				cornerSubPix(img, p_refine, Size(p_class->mask_R, p_class->mask_R), Size(-1, -1), subPix);
				((Point2f*)ptList.data)[t] = p_refine[0];
			}
		}
	};
	//***************************************************************************************************
	struct p_refine_tri :ParallelLoopBody
	{
		detector_cross* p_class;
		Mat1b img;
		float J;
		Mat_<Point2f> ptList;
		
		p_refine_tri(detector_cross* p_class, const Mat1b& img, vector<Point2f>& ptList)
		{
			this->p_class = p_class;
			this->img = img;
			this->J = J;
			this->ptList = Mat_<Point2f>(ptList);
		}
		void operator()(const Range& idx) const
		{
			for (int t = idx.start; t < idx.end; t++)
				p_class->refine_tri_single(img, ((Point2f*)ptList.data)[t]);
		}
	};
};
