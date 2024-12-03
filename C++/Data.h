#pragma once
#include "config.h"

class Data
{
public:
	struct time_cost
	{
		string name;
		double time;
		time_cost(string name, double time)
		{
			this->name = name;
			this->time = time;
		};
	};

	// 特征检测器的输入/输出
	struct detector
	{
		// 输入数据
		Mat1b in_img;					// 检测对象

		// 输入参数
		bool p_parallel = false;		// 是否并行
		int p_FAST_T = 25;				// FAST的阈值
		const int p_mask_r = 3;			// 检测模板的半径(因与FAST半径绑定，不建议修改)
		const int p_mask_R = 2 * p_mask_r + 1;			// 检测模板的直径		
		const TermCriteria p_subPix = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 5, 0.03);	// opencv交叉点亚像素定位算法的迭代参数(没必要修改)
		
		// 标准输出
		vector<Point2f>	o_ptList;		// 特征的亚像素坐标

		// 调试用输出
		vector<Point2f> d_ptPre;		// 预检特征的亚像素坐标
		vector<time_cost> d_time_cost;	// 自定义的耗时记录

		// 函数
		detector() {};
		~detector(){};
		void reset_output()
		{
			o_ptList.clear();

			d_ptPre.clear();
			d_time_cost.clear();
		}
	};

	// 结构组织器的输入/输出
	struct constructor
	{
		// 输入数据
		Mat1b in_img;				// 需要图像以判断元素状态
		vector<Point2f> in_ptList;	// 待组织的点

		// 输入参数
		bool p_parallel = false;	// 是否并行
		float p_distort_T = 1.6;	// 若三角形的长短边超过了这个比例，则认为其变形过度，不构成hexpair(1.6来源于45°倾斜视角)
		uchar p_gray_T = 50;		// 三角形内部的灰度差异若超过此阈值，则认为其内部有标记

		// 标准输出
		vector<Vec3f> o_fList;

		// 调试用输出
		vector<int> d_IDList;
		vector<int> d_coreIDList;
		vector<time_cost> d_time_cost;	// 自定义的耗时记录
		vector<Vec3i>	d_triList;		// 三角形顶点表
		vector<Vec11i> d_hexInfo;		// hexInfo由指向ptList的10个索引和一个key值组成，10个索引依次为：中心边缘的起点、终点，相邻白色三角形的顶点(白色三角形永远在起点到终点的向量的右侧)，之后顺时针排序的顶点，在ptList中的索引

		map<Vec3i, int, CompareVec3i> d_tri_IdxtoState;	// 三角形状态表
		map<int, Vec10i> map_key2idx;					// key->idx 查询表

		// 函数
		constructor() {};
		~constructor() {};
		void reset_output()
		{
			o_fList.clear();
			d_hexInfo.clear();
			d_tri_IdxtoState.clear();
			d_time_cost.clear();
		}
	};

	// 位姿求解器的输入/输出
	struct poseSolver
	{
		// 输入数据
		vector<vector<Vec3f>> in_cam_fList;

		// 输入参数
		int p_iter = 5;	// PnP中beta的迭代计算次数

		// 标准输出
		MatRT o_RT;

		// 调试用输出
		vector<vector<Vec3f>> d_repro_fList;
		vector<double> d_repro_e;
		vector<time_cost> d_time_cost;	// 自定义的耗时记录
		vector<Mat1d>sys_key_points;
		// 函数
		poseSolver() {};
		~poseSolver() {};
		void reset_output()
		{
			d_repro_fList.clear();
			d_repro_e.clear();
			d_time_cost.clear();
		}
	};

};

