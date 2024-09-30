#pragma once
#include "config.h"
#include "Data.h"
#include "track_points_KF.h"

class solver_pose
{
public:
	struct modelPts
	{
		vector<Vec4d> fPts;		// ģ�͵�������
		vector<Vec4d> keyPts;   // ģ�͵Ĺؼ���

		
	};

	// �����۲������ģ�ͽǶȡ�
	struct model_info 
	{
		int ob_num;							// ��Ч�۲����Ŀ����cam_fList����Ч�����������		
		vector<vector<Vec3f>> cam_IDuv;		// cam_fList��ɸѡ���Ƴ��˲���model.fList�е�������
		vector<vector<Vec3f>> cam_xyz;		// ��cam_IDuv��ͬ���������Ӧ��������ģ������ϵ�е����� [camID] = [x, y, z]
		vector<vector<Vec4d>> model_IDuv;	// ģ�ͱ�����۲��״����[model.fList�е�idx][] - [camID, markerID, u, v]
		Mat1d model_IDxyz;					// ���۲�ĵ���ģ������ϵ�е�xyz���꣬ÿ�У�[markerID, x, y, z]

		vector<int> model_idx;              // ���۲⵽����һ�ε������������, [] - [model.fList�е�idx]
	};

	// �����۲������ģ�Ϳ��Ƶ�Ƕȡ�
	struct cPts_info
	{
		Mat1d model_controlPts;				// �ĸ����Ƶ���ģ������ϵ�е�xyz���꣬ÿ�У�[x,y,z]
		Mat1d model_observed_IDBC;			// ���۲�ĵ���ģ�Ϳ��Ƶ�����ϵ�е��������꣬ÿ�У�[markerID, a1, a2, a3, a4]
		Mat1d map_ID2BC;					// ����markerID����ģ�Ϳ��Ƶ�����ϵ�е��������꣬ÿ�У�[a1, a2, a3, a4]
	};

	// �����۲������ϵͳ�Ƕȡ�
	struct sys_info
	{
		Mat1d system_controlPts;	// �ĸ����Ƶ���ϵͳ����ϵ�е�xyz���꣬ÿ�У�[x,y,z]'
		Mat1d sys_IDxyz;			// ���۲�ĵ���ģ������ϵ�е�xyz���꣬ÿ�У�[markerID, x, y, z]'
	};


	// ���캯������ȡ��.camParam���ļ�
	solver_pose(const string& camParam_path, const string& model_path);

	void apply(Data::poseSolver& data,const int isfeed, MatRT& RT);

private:
	//�˶��˲�
	track_points_KF tracker;
	//PoseInformation Pose6D;


	// �㷨����
	int beta_iter_num = 5;

	// �ⲿ��ȡ�Ĳ������ȡ����
	vector<camMatrix> camSys_param;
	modelPts model;
	int maxID;
	map<int, int> map_fID2mIdx;					// ����feature��ID��ѯ��model�������б��е�����

//BA����
	Mat1d euler_trans;
	
	double  trans[3], rot[3];

	void load_camParam(const string path);
	void load_model(const string path);
	void check_param();
	void initial();

	void conclude(const model_info& m_info, const MatRT& RT, vector<vector<Vec3f>>& repro_uv, vector<double>& repro_e);

	// �����۲������ģ�ͽǶȡ�
	void to_model_sys(const vector<vector<Vec3f>>& cam_fList, map<int, int>& map_fID2mIdx, model_info& info);

	// �����۲������ģ�Ϳ��Ƶ�����ϵ�Ƕȡ�
	void to_cPts_system(const model_info& m_info, cPts_info& info);

	void get_model_controlPts(const model_info& m_info, Mat1d& model_controlPts);
	void get_BaryCentricCoordinates(const model_info& m_info, const cPts_info& c_info, Mat1d& model_IDBC, Mat1d& map_ID2BC);

	// �����۲�������۲���Ƶ�����ϵ�Ƕȡ�
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
