///
///ʵ����򣺲ɼ���
/// 

#include <iostream>
#include <iomanip>
#include <sstream>
#include <conio.h>

//#include "marker_deltille.h"
//#include "solver_pose.h"
#include "device_camera.h"
#include "detector_cross.h"
#include "constructor_hexPair.h"
#include "solver_pose.h"
#include "monitor.h"
#include "Camera.h"
#include "CR.h"

void navigationWithCR();
void LoadSamplePts(const string path, vector<Vec4d>& tipPts, vector<Vec3d>& samplePts, Vec3d& vect_keytpoint);
MatRT state2RT(const Mat1d& state, const vector<cv::Vec4d> model_key_points);
MatRT GetRigidTrans(const Mat1d& realPts, const Mat1d& virtualPts);
void importModel(string filename, PointCloud<PointXYZ>::Ptr& grid);
void pelvis3D_Display(const PointCloud<PointXYZ>::Ptr& cloud, Mat1d p1, Mat1d p2);

void main(int argc, char** argv)
{
	cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
	navigationWithCR();
}

//����׼�Ĳɼ���
void navigationWithCR()
{
	Mat img0;
	Mat img1;

	int key;
	
	//���
	Camera cam0;
	Camera cam1;

	//�����
	cam0.start_cam(0);
	cam1.start_cam(1);

	// �����
	detector_cross detector;
	constructor_hexPair constructor("del_14x10.marker");

	//X ��Ƭλ��
	string X_rayImg_filename = ".//x_ray_image//";

	Mat1d takePinPoint;//�ɼ��������㣬��¼�ʼ��
	Mat1d takeEndPoint;//��¼��β�˵�

	vector<Mat> PinLocCTList;
	Mat1d PinLoc;
	Mat1d endLoc;

	//��׼�õ���ת����ϵ
	Mat1d R_rotation;
	Mat1d t_trans;
	MatRT RT_r2v;

	bool startRegistration = true;
	int process = 1;

	//��������ϵ
	Vec3d normal_X = Vec3d(1, 0, 0);
	Vec3d normal_Y = Vec3d(0, 1, 0);
	Vec3d normal_Z = Vec3d(0, 0, 1);
	double rake, abduction;


	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	importModel(".//ACMmodel//pelvis.stl", cloud);
	/**/

	Mat1d C;
	vector<Vec3d> normalxyz;
	
	double radius_drill = 0;
	vector<Mat1d> drill_center_r_vector;// ��¼��һ�ε�λ��

	int num = 1;
	bool isPush = false;
	vector<Vec3d> dataOutput;
	solver_pose solver("C://Users//user//Desktop//HIK5.camParam", "C://Users//user//Desktop//ע��ģ��//531dtt.model");

	while (true)
	{
		cam0.get_pic(&img0);
		cam1.get_pic(&img1);

		if (process == 1)
		{
			
			Data::detector detector_data1;
			Data::detector detector_data2;

			//cvtColor(frame, detector_data.in_img, COLOR_RGB2GRAY);
			img0.copyTo(detector_data1.in_img);
			img1.copyTo(detector_data2.in_img);
			detector_data1.p_parallel = true;
			detector_data2.p_parallel = true;

			assert(!detector_data1.in_img.empty());
			assert(detector_data1.in_img.type() == CV_8UC1 || detector_data1.in_img.type() == CV_8UC3);

			Mat3b imgShow;
			if (detector_data1.in_img.type() == CV_8UC1)		cvtColor(detector_data1.in_img, imgShow, COLOR_GRAY2RGB);
			else							detector_data1.in_img.copyTo(imgShow);

			detector.apply(detector_data1);
			detector.apply(detector_data2);
			TIME_START(time_ALL);
			Data::constructor constructor_data1;
			constructor_data1.in_img = detector_data1.in_img;
			constructor_data1.in_ptList = detector_data1.o_ptList;
			constructor.apply(constructor_data1);

			Data::constructor constructor_data2;
			constructor_data2.in_img = detector_data2.in_img;
			constructor_data2.in_ptList = detector_data2.o_ptList;
			constructor.apply(constructor_data2);

			Data::poseSolver pose_data;
			pose_data.in_cam_fList = vector<vector<Vec3f>>{ constructor_data1.o_fList };
			pose_data.in_cam_fList.push_back(constructor_data2.o_fList);
			int isfeed = 0;
			MatRT rt;
			solver.apply(pose_data, isfeed, rt);
			monitor::debug_show(detector_data1);
			monitor::debug_show(constructor_data1);
			monitor::debug_show(detector_data2);
			monitor::debug_show(constructor_data2);
			monitor::debug_show(vector<Mat1b>{detector_data1.in_img, detector_data2.in_img}, pose_data);

			//�ɼ������������Ϣ
			//��A���вɼ�
			if ((pose_data.sys_key_points.size() > 0) && ((waitKey(15) == 65)))
			{
				int a;
				PinLoc = pose_data.sys_key_points[1];
				endLoc = pose_data.sys_key_points[0];
				takePinPoint.push_back(PinLoc.t());
				takeEndPoint.push_back(endLoc.t());
				cout << "�ɼ��ɹ�������" << endl;
				cout << PinLoc << endl;
				cin >> a;
			}
			else
			{
				cout << "�� 'A'�� ���²ɼ�" << endl;
			}

			if (takePinPoint.rows >= 3)
			{
				cout << "�ɼ��Ǵ��������" << endl;
				if (startRegistration)
				{
					takePinPoint = takePinPoint.t();
					takeEndPoint = takeEndPoint.t();

					CR cr("D:/C++Projects/ContactRegistration V1.0/ContactRegistration V1.0/ACMmodel/left_ACM.stl");
					//����ģ��ģ��
					cr.apply(takePinPoint.col(0), takePinPoint.col(1), takePinPoint.col(2), takeEndPoint.col(2), R_rotation, t_trans);
					hconcat(R_rotation, t_trans, RT_r2v);
					startRegistration = false;
				}	
			}

			//��Q��һ��
			int key_wait = waitKey(15);
			if (key_wait == 81)
			{
				process = 2;
			}
			//Esc �˳�
			if (key_wait == 27)
			{
				break;
			}
		}
		else if (process == 2)
		{
			radius_drill = 21;
			if (radius_drill == 0)
			{
				cout << "�������ͷ�İ뾶��С��Ԥ��21����" << endl;
				cin >> radius_drill;
			}
			
			Data::detector detector_data1;
			Data::detector detector_data2;

			img0.copyTo(detector_data1.in_img);
			img1.copyTo(detector_data2.in_img);
			detector_data1.p_parallel = true;
			detector_data2.p_parallel = true;

			detector.apply(detector_data1);
			detector.apply(detector_data2);
			Data::constructor constructor_data1;
			constructor_data1.in_img = detector_data1.in_img;
			constructor_data1.in_ptList = detector_data1.o_ptList;
			constructor.apply(constructor_data1);

			Data::constructor constructor_data2;
			constructor_data2.in_img = detector_data2.in_img;
			constructor_data2.in_ptList = detector_data2.o_ptList;
			constructor.apply(constructor_data2);

			Data::poseSolver pose_data;
			pose_data.in_cam_fList = vector<vector<Vec3f>>{ constructor_data1.o_fList };
			pose_data.in_cam_fList.push_back(constructor_data2.o_fList);
			int isfeed = 0;
			MatRT rt;
			solver.apply(pose_data, isfeed, rt);
			monitor::debug_show(vector<Mat1b>{detector_data1.in_img, detector_data2.in_img}, pose_data);
			vector<Mat> matrices = { Mat(normal_X), Mat(normal_Y), Mat(normal_Z) };
			Mat R;
			hconcat(matrices, R);

			if (pose_data.sys_key_points.size() > 0)
			{

				Mat1d v = pose_data.sys_key_points[1] - pose_data.sys_key_points[0];
				//��ȡmodel��
				vector<Vec3d> samplePts;  //ģ�͵Ĳ�����
				vector<Vec4d> tipPts;  //ģ�͵�Բ��ͱʼ��
				Vec3d vec_keypoint;
				LoadSamplePts("S_P.model", tipPts, samplePts, vec_keypoint);
				Mat samPts_xyz = vec2mat(samplePts);
				//��RT
				Mat1d kf_sys_key_points;
				//Mat1d v_key_point= vec2mat(vec_keypoint);
				//Mat1d sys_v_key_point= rt*add_homo( v_key_point);
				cout << pose_data.sys_key_points[0] << endl;
				cout << pose_data.sys_key_points[1] << endl;
				double distance = norm(pose_data.sys_key_points[1], pose_data.sys_key_points[0], NORM_L2);
				double distance_model = 261.698428527953;
				double dis_scale = distance_model / distance;
				Mat1d v_key_point = v * dis_scale;
				kf_sys_key_points.push_back(pose_data.sys_key_points[1] - v_key_point);
				kf_sys_key_points.push_back(pose_data.sys_key_points[1]);

				//�����������RT��ͶӰ�ʺ���Բ
				MatRT RT1 = state2RT(kf_sys_key_points, tipPts);
				Mat1d modelIDxyz = vec2mat(tipPts);
				Mat1d modelxyz;
				modelIDxyz.rowRange(1, 4).copyTo(modelxyz);
				Mat1d sys_key_points = RT1 * add_homo(modelxyz);

				//������ת���������ϵ��
				Mat1d transf_samplePts = RT1 * add_homo(samPts_xyz);
				cout << sys_key_points << endl;
				cout << "KF_RT\n" << RT1 << endl;

				Mat1d sys_key_point_0, sys_key_point_1;
				sys_key_points.colRange(0, 1).copyTo(sys_key_point_0);
				sys_key_points.colRange(1, 2).copyTo(sys_key_point_1);

				Mat1d PinLocCT = RT_r2v * add_homo(sys_key_point_1);
				Mat1d EndLocCT = RT_r2v * add_homo(sys_key_point_0);
				Mat1d samplePtsCT = RT_r2v * add_homo(transf_samplePts);

				//pelvis3D_Display(cloud, PinLocCT, PinLocCT);

				if ((waitKey(15) == 81) || isPush)//Q
				{
					isPush = true;
					if (num <= 20)  //һ��Q�ɼ�����Ϊ20
					{
						normalxyz.push_back(PinLocCT);
						num++;
					}
					else
					{
						isPush = false;
						num = 1;
						cout << "���βɼ�����" << num << endl;
					}
				}
				else
				{
					cout << "�����²ɼ���֤��" << endl;
				}

				/// <summary>
				/// ��Ƕ�
				/// </summary>
			
				Mat1d v_direc = PinLocCT - EndLocCT;
				double dotP = v_direc.dot(normal_Y);
				double norm1 = sqrt(pow(v_direc.at<double>(0,0), 2) + pow(v_direc.at<double>(1,0), 2) + pow(v_direc.at<double>(2,0), 2));
				double norm2 = sqrt(pow(normal_Y[0], 2) + pow(normal_Y[1], 2) + pow(normal_Y[2], 2));
				rake = acos(dotP/(norm1* norm2)); 
				abduction = atan2(v_direc.at<double>(0, 0), v_direc.at<double>(0, 2));
				cout << v_direc.at<double>(0, 0) << endl;

				//��Ƭ�Ƕ���ʾ
				stringstream ss1, ss2;
				rake = round((90 - rake * 180 / CV_PI) * pow(10, 1)) * pow(10, -1);
				abduction = -round((abduction * 180 / CV_PI) * pow(10, 1)) * pow(10, -1);

				//�Ƕ�ֵ����һλС����
				ss1 << setiosflags(ios::fixed) << setprecision(1) << rake;
				ss2 << setiosflags(ios::fixed) << setprecision(1) << abduction;

				//��Ƭ��ʾ��еͶӰת����ϵ
				Mat1d T1 = (Mat1d(2, 4) << 2, 0, 0, 250.305404663086,
					0, 0, -2, 3066.39526367188);//(��λƬ*��*�ɹǵ�ת����ϵ)

				Mat1d T3 = (Mat1d(2, 4) << 2, 0, 0, 277.042999267578,
					0, 0, -2, -445.814575195313); //(��λƬ*��*�ɹǵ�ת����ϵ)

				Mat1d T5 = (Mat1d(2, 4) << 0, 2, 0, -25.4366149902344,
					0, 0, -2, 3066.39526367188);//(��λ*��*�ɹǵ�ת����ϵ)

				Mat1d T6 = (Mat1d(2, 4) << 0, -2, 0, -15.1028137207031,
					0, 0, -2, -445.814575195313);//(��λ*��*�ɹǵ�ת����ϵ)

				if (drill_center_r_vector.size() > 0)
				{
					/////��λƬ-�йɹ�/////
					Mat Xray_img_ZhengWei_new = imread(X_rayImg_filename + "Xray_img_ZhengWei_1.jpg");
					resize(Xray_img_ZhengWei_new, Xray_img_ZhengWei_new, Size(541, 408), INTER_LINEAR);
					monitor::XrayImg_show(Xray_img_ZhengWei_new, T1, PinLocCT, EndLocCT, "lin1", samplePtsCT, radius_drill);
					//��λ3D
					Mat Xray_img_ZhengWei_new3D = imread(X_rayImg_filename + "Xray_img_ZhengWei3D_1.png");
					resize(Xray_img_ZhengWei_new3D, Xray_img_ZhengWei_new3D, Size(541, 408), INTER_LINEAR);
					monitor::XrayImg_show(Xray_img_ZhengWei_new3D, T1, PinLocCT, EndLocCT, "lin1", samplePtsCT, radius_drill);
					//��λ �йɹ�
					Mat Xray_img_ceWei = imread(X_rayImg_filename + "HCT_YouGuGuCeiW3D.jpg");
					monitor::ceWeiXrayImg_show(Xray_img_ceWei, T5, PinLocCT, EndLocCT, "lin5", samplePtsCT, radius_drill);

					resize(Xray_img_ZhengWei_new, Xray_img_ZhengWei_new, Size(541 * 2, 408 * 2), INTER_LINEAR);
					resize(Xray_img_ZhengWei_new3D, Xray_img_ZhengWei_new3D, Size(541 * 2, 408 * 2), INTER_LINEAR);
					resize(Xray_img_ceWei, Xray_img_ceWei, Size(305 * 2, 408 * 2), INTER_LINEAR);

					//�йɹ���ʾ
					Mat image_X;
					hconcat(Xray_img_ZhengWei_new, Xray_img_ZhengWei_new3D, image_X);
					hconcat(image_X, Xray_img_ceWei, image_X);
					string angle1 = "ǰ���:";// �Ƕ���ʾ
					monitor::putTextInChinese(image_X, angle1.c_str(), Point2f(20, 500), Scalar(255, 255, 255), 100, "΢���ź�");
					putText(image_X, ss1.str(), Point2f(360, 550), cv::FONT_HERSHEY_PLAIN, 8, Scalar(255, 255, 255), 3);

					string angle2 = "��չ��:";
					monitor::putTextInChinese(image_X, angle2.c_str(), Point2f(20, 650), Scalar(255, 255, 255), 100, "΢���ź�");
					putText(image_X, ss2.str(), Point2f(360, 700), cv::FONT_HERSHEY_PLAIN, 8, Scalar(255, 255, 255), 3);
					namedWindow("Line3_Xray", WINDOW_NORMAL);
					imshow("Line3_Xray", image_X);

					////��λƬ-�޹ɹ�////

					Mat Xray_img_ZhengWeiW_new = imread(X_rayImg_filename + "Xray_img_ZhengWeiW_1.jpg");
					monitor::XrayImg_show(Xray_img_ZhengWeiW_new, T3, PinLocCT, EndLocCT, "lin3", samplePtsCT, radius_drill);

					Mat Xray_img_ZhengWeiW3D_new = imread(X_rayImg_filename + "Xray_img_ZhengWeiW3D_1.png");
					monitor::XrayImg_show(Xray_img_ZhengWeiW3D_new, T3, PinLocCT, EndLocCT, "lin3", samplePtsCT, radius_drill);

					Mat Xray_img_ceWeiW = imread(X_rayImg_filename + "HCT_WuGuGuCeiW3D.jpg");
					//monitor::ceWeiXrayImg_show(Xray_img_ceWeiW, T6, PinLocCT, EndLocCT, "lin6");
					monitor::ceWeiXrayImg_show(Xray_img_ceWeiW, T6, PinLocCT, EndLocCT, "lin6", samplePtsCT, radius_drill);

					//�޹ɹ���ʾ
					Mat image_Xw;
					hconcat(Xray_img_ZhengWeiW_new, Xray_img_ZhengWeiW3D_new, image_Xw);
					hconcat(image_Xw, Xray_img_ceWeiW, image_Xw);
					Mat resize_image_Xw;
					resize(image_Xw, resize_image_Xw, Size(2392, 248 * 2), INTER_LINEAR);

					monitor::putTextInChinese(resize_image_Xw, angle1.c_str(), Point2f(50, 300), Scalar(255, 255, 255), 50, "΢���ź�");
					putText(resize_image_Xw, ss1.str(), Point2f(230, 360), cv::FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 2);
					monitor::putTextInChinese(resize_image_Xw, angle2.c_str(), Point2f(50, 400), Scalar(255, 255, 255), 50, "΢���ź�");
					putText(resize_image_Xw, ss2.str(), Point2f(230, 460), cv::FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 2);
					namedWindow("Line4_Xray", WINDOW_NORMAL);
					imshow("Line4_Xray", resize_image_Xw);

				}
				else
				{
					//��-��λƬ 
					Mat Xray_img_ZhengWei = imread(X_rayImg_filename + "zhengWei_1.bmp");
					monitor::XrayImg_show(Xray_img_ZhengWei, T1, PinLocCT, EndLocCT, "lin1", samplePtsCT, radius_drill);
					//��-��λƬ 3D
					Mat Xray_img_ZhengWei3D = imread(X_rayImg_filename + "zhengwei3D_1.png");
					//Mat1d T2 = (Mat1d(2, 4) << 2, 0, 0, 275.167388916016,
					//	0, 0, -2, -1575.20507812500);
					monitor::XrayImg_show(Xray_img_ZhengWei3D, T1, PinLocCT, EndLocCT, "lin2", samplePtsCT, radius_drill);

					////��-�޹ɹ���λƬ 
					Mat Xray_img_ZhengWeiW = imread(X_rayImg_filename + "ZhengW_123.jpg");
					//Mat1d T3 = (Mat1d(2, 4) << 2, 0, 0, 221.907806396484,
					//	0, 0, -2, -1575.01318359375);
					monitor::XrayImg_show(Xray_img_ZhengWeiW, T3, PinLocCT, EndLocCT, "lin3", samplePtsCT, radius_drill);
					////��-�޹ɹ���λƬ3D 
					Mat Xray_img_ZhengWeiW3D = imread(X_rayImg_filename + "ZhengW_3D.jpg");
					//Mat1d T4 = (Mat1d(2, 4) << 2, 0, 0, 221.907806396484,
					//	0, 0, -2, -1575.01318359375);
					monitor::XrayImg_show(Xray_img_ZhengWeiW3D, T3, PinLocCT, EndLocCT, "lin4", samplePtsCT, radius_drill);

					//��-��λ3D
					Mat Xray_img_ceWei = imread(X_rayImg_filename + "cewei_1.png");
					monitor::ceWeiXrayImg_show(Xray_img_ceWei, T5, PinLocCT, EndLocCT, "lin5", samplePtsCT, radius_drill);
					//��-��λ�޹ɹ�3D
					Mat Xray_img_ceWeiW = imread(X_rayImg_filename + "CeiW_123.jpg");
					//monitor::ceWeiXrayImg_show(Xray_img_ceWeiW, T6, PinLocCT, EndLocCT, "lin6");
					monitor::ceWeiXrayImg_show(Xray_img_ceWeiW, T6, PinLocCT, EndLocCT, "lin6", samplePtsCT, radius_drill);

					resize(Xray_img_ZhengWei, Xray_img_ZhengWei, Size(556 * 2, 441 * 2), INTER_LINEAR);
					resize(Xray_img_ZhengWei3D, Xray_img_ZhengWei3D, Size(556 * 2, 441 * 2), INTER_LINEAR);
					resize(Xray_img_ceWei, Xray_img_ceWei, Size(319 * 2, 441 * 2), INTER_LINEAR);

					//�йɹ���ʾ
					Mat image_X;
					hconcat(Xray_img_ZhengWei, Xray_img_ZhengWei3D, image_X);
					hconcat(image_X, Xray_img_ceWei, image_X);
					string angle1 = "ǰ���:";// �Ƕ���ʾ
					monitor::putTextInChinese(image_X, angle1.c_str(), Point2f(20, 500), Scalar(255, 255, 255), 70, "΢���ź�");
					putText(image_X, ss1.str(), Point2f(320, 600), cv::FONT_HERSHEY_PLAIN, 8, Scalar(255, 255, 255), 3);

					string angle2 = "��չ��:";
					monitor::putTextInChinese(image_X, angle2.c_str(), Point2f(20, 650), Scalar(255, 255, 255), 70, "΢���ź�");
					putText(image_X, ss2.str(), Point2f(320, 750), cv::FONT_HERSHEY_PLAIN, 8, Scalar(255, 255, 255), 3);
					namedWindow("Line1_Xray", WINDOW_NORMAL);
					imshow("Line1_Xray", image_X);

					//�޹ɹ���ʾ
					Mat image_Xw;
					hconcat(Xray_img_ZhengWeiW, Xray_img_ZhengWeiW3D, image_Xw);
					hconcat(image_Xw, Xray_img_ceWeiW, image_Xw);
					Mat resize_image_Xw;
					resize(image_Xw, resize_image_Xw, Size(2882, 340 * 2), INTER_LINEAR);

					monitor::putTextInChinese(resize_image_Xw, angle1.c_str(), Point2f(50, 300), Scalar(255, 255, 255), 50, "΢���ź�");
					putText(resize_image_Xw, ss1.str(), Point2f(230, 360), cv::FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 2);
					monitor::putTextInChinese(resize_image_Xw, angle2.c_str(), Point2f(50, 400), Scalar(255, 255, 255), 50, "΢���ź�");
					putText(resize_image_Xw, ss2.str(), Point2f(230, 460), cv::FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 2);
					namedWindow("Line2_Xray", WINDOW_NORMAL);
					imshow("Line2_Xray", resize_image_Xw);
				}

				int key_wait = waitKey(15);
				if (key_wait == 82)//R
				{
					//��λ�йɹ�
					Mat Xray_img_ZhengWei_1 = imread(X_rayImg_filename + "HCT_ZhengWYouGuGu.jpg");
					Mat Xray_img_ZhengWei3D_1 = imread(X_rayImg_filename + "HCT_ZhengWYouGuGu3D.jpg");

					//��λ�޹ɹ�
					Mat Xray_img_ZhengWeiW_1 = imread(X_rayImg_filename + "HCT_WuGuGuZhengW.jpg");
					Mat Xray_img_ZhengWeiW3D_1 = imread(X_rayImg_filename + "HCT_WuGuGuZhengW3D.jpg");

					Mat1d drill_center_r;
					//�ʼ�� ��ĩ�˵� ��ͷ�뾶
					PinLocCT.copyTo(drill_center_r);
					drill_center_r.push_back(EndLocCT);
					drill_center_r.push_back(radius_drill);
					drill_center_r_vector.push_back(drill_center_r); //��¼��ͷ�Ĵ�С
					cout << "���������ͷ�İ뾶��С��" << endl;
					cin >> radius_drill;

					//��¼��һ�ε�λ�ã������ͼƬ
					monitor::recordChange(Xray_img_ZhengWei_1, T1, drill_center_r_vector, "Xray_img_ZhengWei_1.jpg", X_rayImg_filename);
					monitor::recordChange(Xray_img_ZhengWei3D_1, T1, drill_center_r_vector, "Xray_img_ZhengWei3D_1.png", X_rayImg_filename);

					monitor::recordChange(Xray_img_ZhengWeiW_1, T3, drill_center_r_vector, "Xray_img_ZhengWeiW_1.jpg", X_rayImg_filename);
					monitor::recordChange(Xray_img_ZhengWeiW3D_1, T3, drill_center_r_vector, "Xray_img_ZhengWeiW3D_1.png", X_rayImg_filename);
				}
			}
			int key_wait = waitKey(15);
			if (key_wait == 87)	//W
			{
				process = 1;
				takePinPoint.release();
				takeEndPoint.release();
				startRegistration = true;
			}
			key = waitKey(15);
			if (key == 27)//Esc
			{
				fstream PLeft48("C://Users//user//Desktop//8-18.txt", ios::out);
				if (!PLeft48.fail())
				{
					for (int i = 0; i < normalxyz.size(); i++)
					{
						Vec3d sys_key_point = normalxyz[i];
						for (int j = 0; j < 3; j++)
						{
							PLeft48 << double(sys_key_point[j]) << "\t";
						}
						PLeft48 << std::endl;
					}
				}
				else
					cout << "can not open" << endl;
				PLeft48.close();
				cam0.close_cam();
				cam1.close_cam();
				break;
			}
		}

	}

}

MatRT state2RT(const Mat1d& state, const vector<cv::Vec4d> model_key_points)
{
	MatRT RT;

	// centering
	Mat1d sysF_c, modF_c;
	Mat1d mean_center_model;
	Mat1d mean_center_sys;
	Mat1d model_points(3, 2, CV_64F);
	Mat1d sys_state_points(3, 2, CV_64F);
	for (int i = 0; i < 2; i++)
	{
		model_points(0, i) = model_key_points[i][1];
		model_points(1, i) = model_key_points[i][2];
		model_points(2, i) = model_key_points[i][3];
		sys_state_points(0, i) = state(3 * i);
		sys_state_points(1, i) = state(3 * i + 1);
		sys_state_points(2, i) = state(3 * i + 2);
	}
	//cout << state << endl;
	reduce(model_points, mean_center_model, 1, REDUCE_AVG);
	reduce(sys_state_points, mean_center_sys, 1, REDUCE_AVG);

	repeat(mean_center_sys, 1, sys_state_points.cols, sysF_c);
	repeat(mean_center_model, 1, model_points.cols, modF_c);

	Mat1d sysF_ = sys_state_points - sysF_c;
	Mat1d modF_ = model_points - modF_c;

	// get RT
	Mat1d w, u, vt;
	SVD::compute((sysF_ * modF_.t()), w, u, vt, SVD::FULL_UV);
	Mat1d R = u * vt;
	double detR = determinant(R);
	if (detR < 0)
	{
		vt.row(2) = -vt.row(2);
		R = u * vt;
	}
	Mat1d T = mean_center_sys - R * mean_center_model;

	hconcat(R, T, RT);
	return RT;
}

MatRT GetRigidTrans(const Mat1d& realPts, const Mat1d& virtualPts)
{
	MatRT RT;

	// centering
	Mat1d realPts_c, virtualPts_c;
	Mat1d mean_realPts, mean_virualPts;

	reduce(realPts.t(), mean_realPts, 1, REDUCE_AVG);
	reduce(virtualPts.t(), mean_virualPts, 1, REDUCE_AVG);

	repeat(mean_realPts, 1, realPts.rows, realPts_c);
	repeat(mean_virualPts, 1, virtualPts.rows, virtualPts_c);

	Mat1d realF_ = realPts.t() - realPts_c;
	Mat1d virtualF_ = virtualPts.t() - virtualPts_c;

	// get RT
	Mat1d w, u, vt;
	SVD::compute(virtualF_ * realF_.t(), w, u, vt, SVD::FULL_UV);
	Mat1d R = u * vt;
	double detR = determinant(R);
	if (detR < 0)   R.row(2) = -R.row(2);
	Mat1d T = mean_virualPts - R * mean_realPts;

	hconcat(R, T, RT);

	return RT;
}

void LoadSamplePts(const string path, vector<Vec4d>& tipPts, vector<Vec3d>& samplePts, Vec3d& vect_keytpoint)
{
	TRY_THROW(check_suffix(path, ".model"), "the file type must be .model!");
	ifstream input_file(path);
	TRY_THROW(input_file.is_open(), "fail to open model file!");

	// �� model
	int fNum, keyNum;
	input_file >> fNum;
	input_file >> keyNum;
	for (int i = 0; i < fNum; i++)
	{
		Vec4d info;
		input_file >> info[0] >> info[1] >> info[2] >> info[3];
		tipPts.push_back(info);
	}
	for (int i = 0; i < keyNum; i++)
	{
		Vec3d info;
		input_file >> info[0] >> info[1] >> info[2];
		samplePts.push_back(info);
	}
	for (int i = 0; i < 1; i++)
	{
		input_file >> vect_keytpoint[0] >> vect_keytpoint[1] >> vect_keytpoint[2];
	}
}

void importModel(string filename, PointCloud<PointXYZ>::Ptr& grid)
{
	// ����ģ�Ͷ���
	pcl::PolygonMesh mesh;

	// ����STLģ���ļ�
	pcl::io::loadPolygonFileSTL(filename, mesh);

	// ��ģ��ת��Ϊ���ƶ���
	pcl::fromPCLPointCloud2(mesh.cloud, *grid);
}

//��ʾ3D����ģ��
void pelvis3D_Display(const PointCloud<PointXYZ>::Ptr& cloud, Mat1d p1, Mat1d p2)
{
	PointXYZ point1;
	PointXYZ point2;

	point1.x = p1.at<double>(0, 0);
	point1.y = p1.at<double>(1, 0);
	point1.z = p1.at<double>(2, 0);

	point2.x = p2.at<double>(0, 0);
	point2.y = p2.at<double>(1, 0);
	point2.z = p2.at<double>(2, 0);

	// �������ӻ�����
	visualization::PCLVisualizer viewer("Point Cloud Viewer");

	// ���õ�����Ⱦ��ɫΪ��ɫ
	visualization::PointCloudColorHandlerCustom<PointXYZ> color_handler(cloud, 0, 255, 0);
	viewer.addPointCloud<PointXYZ>(cloud, color_handler, "cloud");
	// ���ÿ��ӻ�����
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	//���������
	viewer.addCoordinateSystem(50.0, "cloud", 0);
	viewer.setCameraPosition(0, -100, -300, 1, 0, 0, 0, 0, 1);
	viewer.addLine<PointXYZ>(point1, point2, "line");

	viewer.spinOnce();

}