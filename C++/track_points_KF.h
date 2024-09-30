#pragma once
#include "config.h"

using namespace cv;
using namespace std;

class track_points_KF
{
public:
	track_points_KF();
	Mat1d feed(const MatRT& RT, const vector<cv::Vec4d> model_key_points, const int& isfeed);
	Mat1d read();
private:
	const int pose_heat = 3;					// ÿ��pose tracker������֡��
	const double xy_T = 10;					// ��֡��xy�����ƶ�����xy_T mm���л�����̬
	//Mat1d measure_points;
	KalmanFilter KF;

	void initial_KF_static(KalmanFilter& KF, const Mat1d& initial_state);
	Mat1d mesure_point(const MatRT& RT, const vector<cv::Vec4d> model_key_points);
	Mat1d update_KF_static(KalmanFilter& KF, const Mat1d& measure, int pow);
	Mat1d state2measure(const Mat1d& state);
	MatRT state2RT(const Mat1d& state, const vector<cv::Vec4d> model_key_points);
	/*Mat1d update_KF_static(KalmanFilter& KF, const Mat1d& measure, int pow);*/

};

