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

	// ���������������/���
	struct detector
	{
		// ��������
		Mat1b in_img;					// ������

		// �������
		bool p_parallel = false;		// �Ƿ���
		int p_FAST_T = 25;				// FAST����ֵ
		const int p_mask_r = 3;			// ���ģ��İ뾶(����FAST�뾶�󶨣��������޸�)
		const int p_mask_R = 2 * p_mask_r + 1;			// ���ģ���ֱ��		
		const TermCriteria p_subPix = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 5, 0.03);	// opencv����������ض�λ�㷨�ĵ�������(û��Ҫ�޸�)
		
		// ��׼���
		vector<Point2f>	o_ptList;		// ����������������

		// ���������
		vector<Point2f> d_ptPre;		// Ԥ������������������
		vector<time_cost> d_time_cost;	// �Զ���ĺ�ʱ��¼

		// ����
		detector() {};
		~detector(){};
		void reset_output()
		{
			o_ptList.clear();

			d_ptPre.clear();
			d_time_cost.clear();
		}
	};

	// �ṹ��֯��������/���
	struct constructor
	{
		// ��������
		Mat1b in_img;				// ��Ҫͼ�����ж�Ԫ��״̬
		vector<Point2f> in_ptList;	// ����֯�ĵ�

		// �������
		bool p_parallel = false;	// �Ƿ���
		float p_distort_T = 1.6;	// �������εĳ��̱߳������������������Ϊ����ι��ȣ�������hexpair(1.6��Դ��45����б�ӽ�)
		uchar p_gray_T = 50;		// �������ڲ��ĻҶȲ�������������ֵ������Ϊ���ڲ��б��

		// ��׼���
		vector<Vec3f> o_fList;

		// ���������
		vector<int> d_IDList;
		vector<int> d_coreIDList;
		vector<time_cost> d_time_cost;	// �Զ���ĺ�ʱ��¼
		vector<Vec3i>	d_triList;		// �����ζ����
		vector<Vec11i> d_hexInfo;		// hexInfo��ָ��ptList��10��������һ��keyֵ��ɣ�10����������Ϊ�����ı�Ե����㡢�յ㣬���ڰ�ɫ�����εĶ���(��ɫ��������Զ����㵽�յ���������Ҳ�)��֮��˳ʱ������Ķ��㣬��ptList�е�����

		map<Vec3i, int, CompareVec3i> d_tri_IdxtoState;	// ������״̬��
		map<int, Vec10i> map_key2idx;					// key->idx ��ѯ��

		// ����
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

	// λ�������������/���
	struct poseSolver
	{
		// ��������
		vector<vector<Vec3f>> in_cam_fList;

		// �������
		int p_iter = 5;	// PnP��beta�ĵ����������

		// ��׼���
		MatRT o_RT;

		// ���������
		vector<vector<Vec3f>> d_repro_fList;
		vector<double> d_repro_e;
		vector<time_cost> d_time_cost;	// �Զ���ĺ�ʱ��¼
		vector<Mat1d>sys_key_points;
		// ����
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

