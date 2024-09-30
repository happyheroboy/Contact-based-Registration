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

	// ȫ�ֲ���	
	bool parallel;
	int mask_r;
	int mask_R;

	// ģ�����ʱ��Ҫʹ�õ�ϵ��������α�����
	void createA();
	Mat1f A_tri, pA_tri;

	// ��matlab��ͬ��������ͬ
	void meshgrid(const Range xgv, const Range ygv, Mat1f& X, Mat1f& Y);

	// ��ȡͼ���е����ƽ���㣬��������������꣬��������: FAST -> cornerSubPix -> kdTree NMS
	void preFilter(const Mat1b& img, const int FAST_T, const TermCriteria subPix, vector<Point2f>& ptList);

	// ʹ��kdTreeѰ�Ҿۼ��ĵ㣬������ɾ������
	void merge_kdTree(const vector<Point2f>& ptList, const Size pt_scope, const float r, vector<bool>& remove);

	// �������ζ���ʽ��Ϻﰰ�棬���������ؼ�������б�
	void refine_tri(const Mat1b& img, vector<Point2f>& ptList);

	// ����һ�����refine_tri�������޸�p��Point2f(-1,-1)��ʾ�õ㱻�б�Ϊ��������
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
