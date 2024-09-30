#pragma once
#include "config.h"
#include "Data.h"
#include "track_points_KF.h"

class solver_pose
{
public:
	struct modelPts
	{
		vector<Vec4d> fPts;		// 模型的特征点
		vector<Vec4d> keyPts;   // 模型的关键点

		
	};

	// 特征观测情况【模型角度】
	struct model_info 
	{
		int ob_num;							// 有效观测的数目，即cam_fList中有效特征点的总数		
		vector<vector<Vec3f>> cam_IDuv;		// cam_fList的筛选，移除了不在model.fList中的特征点
		vector<vector<Vec3f>> cam_xyz;		// 与cam_IDuv相同，但储存对应特征点在模型坐标系中的坐标 [camID] = [x, y, z]
		vector<vector<Vec4d>> model_IDuv;	// 模型被相机观测的状况，[model.fList中的idx][] - [camID, markerID, u, v]
		Mat1d model_IDxyz;					// 被观测的点在模型坐标系中的xyz坐标，每列：[markerID, x, y, z]

		vector<int> model_idx;              // 被观测到至少一次的特征点的索引, [] - [model.fList中的idx]
	};

	// 特征观测情况【模型控制点角度】
	struct cPts_info
	{
		Mat1d model_controlPts;				// 四个控制点在模型坐标系中的xyz坐标，每列：[x,y,z]
		Mat1d model_observed_IDBC;			// 被观测的点在模型控制点坐标系中的质心坐标，每列：[markerID, a1, a2, a3, a4]
		Mat1d map_ID2BC;					// 根据markerID查在模型控制点坐标系中的质心坐标，每列：[a1, a2, a3, a4]
	};

	// 特征观测情况【系统角度】
	struct sys_info
	{
		Mat1d system_controlPts;	// 四个控制点在系统坐标系中的xyz坐标，每列：[x,y,z]'
		Mat1d sys_IDxyz;			// 被观测的点在模型坐标系中的xyz坐标，每列：[markerID, x, y, z]'
	};


	// 构造函数，读取“.camParam”文件
	solver_pose(const string& camParam_path, const string& model_path);

	void apply(Data::poseSolver& data,const int isfeed, MatRT& RT);

private:
	//运动滤波
	track_points_KF tracker;
	//PoseInformation Pose6D;


	// 算法参数
	int beta_iter_num = 5;

	// 外部读取的参数与读取方法
	vector<camMatrix> camSys_param;
	modelPts model;
	int maxID;
	map<int, int> map_fID2mIdx;					// 根据feature的ID查询在model的特征列表中的索引

//BA参数
	Mat1d euler_trans;
	
	double  trans[3], rot[3];

	void load_camParam(const string path);
	void load_model(const string path);
	void check_param();
	void initial();

	void conclude(const model_info& m_info, const MatRT& RT, vector<vector<Vec3f>>& repro_uv, vector<double>& repro_e);

	// 特征观测情况【模型角度】
	void to_model_sys(const vector<vector<Vec3f>>& cam_fList, map<int, int>& map_fID2mIdx, model_info& info);

	// 特征观测情况【模型控制点坐标系角度】
	void to_cPts_system(const model_info& m_info, cPts_info& info);

	void get_model_controlPts(const model_info& m_info, Mat1d& model_controlPts);
	void get_BaryCentricCoordinates(const model_info& m_info, const cPts_info& c_info, Mat1d& model_IDBC, Mat1d& map_ID2BC);

	// 特征观测情况【观测控制点坐标系角度】
	void to_sys_system(const model_info& m_info, const cPts_info& c_info, sys_info& info);
	void get_Mb(const model_info& m_info, const Mat1d& map_ID2BC, Mat1d& Mb);
	void get_sys_controlPts(const Mat1d& Mb, const Mat1d& model_controlPts, Mat1d& sys_controlPts);
	
	// PnP
	void mPnP(const model_info& m_info, MatRT& RT);
	MatRT get_RT(const model_info& m_info, const cPts_info& c_info, const sys_info& s_info);
	
	void repro(const Mat1d& base_uv, const Mat1d& est_xyz, const MatRT& RT, const camMatrix& P, Mat1d& repro_uv, Mat1d& repro_e);
	Mat1d RT2Rodrigues_t(const MatRT& RT);
	MatRT bundleAdjustment(const model_info& m_info, const MatRT& RT);
	void buildProblem(Problem* problem, const model_info& m_info);
	MatRT Rodrigues2RT(const Mat1d& euler_trans);

	void get_sys_Key_point(const Mat1d fliter_sys_keypoints, const MatRT RT, vector<Mat1d>& sys_key_points);




};
