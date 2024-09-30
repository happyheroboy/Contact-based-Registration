///
///实验程序：采集点
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

//有配准的采集点
void navigationWithCR()
{
	Mat img0;
	Mat img1;

	int key;
	
	//相机
	Camera cam0;
	Camera cam1;

	//开相机
	cam0.start_cam(0);
	cam1.start_cam(1);

	// 检测器
	detector_cross detector;
	constructor_hexPair constructor("del_14x10.marker");

	//X 线片位置
	string X_rayImg_filename = ".//x_ray_image//";

	Mat1d takePinPoint;//采集骨特征点，记录笔尖点
	Mat1d takeEndPoint;//记录笔尾端点

	vector<Mat> PinLocCTList;
	Mat1d PinLoc;
	Mat1d endLoc;

	//配准得到的转换关系
	Mat1d R_rotation;
	Mat1d t_trans;
	MatRT RT_r2v;

	bool startRegistration = true;
	int process = 1;

	//虚拟坐标系
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
	vector<Mat1d> drill_center_r_vector;// 记录上一次的位置

	int num = 1;
	bool isPush = false;
	vector<Vec3d> dataOutput;
	solver_pose solver("C://Users//user//Desktop//HIK5.camParam", "C://Users//user//Desktop//注册模型//531dtt.model");

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

			//采集输入骨特征信息
			//按A进行采集
			if ((pose_data.sys_key_points.size() > 0) && ((waitKey(15) == 65)))
			{
				int a;
				PinLoc = pose_data.sys_key_points[1];
				endLoc = pose_data.sys_key_points[0];
				takePinPoint.push_back(PinLoc.t());
				takeEndPoint.push_back(endLoc.t());
				cout << "采集成功！！！" << endl;
				cout << PinLoc << endl;
				cin >> a;
			}
			else
			{
				cout << "请 'A'键 重新采集" << endl;
			}

			if (takePinPoint.rows >= 3)
			{
				cout << "采集骨刺任务完成" << endl;
				if (startRegistration)
				{
					takePinPoint = takePinPoint.t();
					takeEndPoint = takeEndPoint.t();

					CR cr("D:/C++Projects/ContactRegistration V1.0/ContactRegistration V1.0/ACMmodel/left_ACM.stl");
					//导入模拟模型
					cr.apply(takePinPoint.col(0), takePinPoint.col(1), takePinPoint.col(2), takeEndPoint.col(2), R_rotation, t_trans);
					hconcat(R_rotation, t_trans, RT_r2v);
					startRegistration = false;
				}	
			}

			//按Q下一步
			int key_wait = waitKey(15);
			if (key_wait == 81)
			{
				process = 2;
			}
			//Esc 退出
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
				cout << "请输入锉头的半径大小（预设21）：" << endl;
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
				//读取model点
				vector<Vec3d> samplePts;  //模型的采样点
				vector<Vec4d> tipPts;  //模型的圆点和笔尖点
				Vec3d vec_keypoint;
				LoadSamplePts("S_P.model", tipPts, samplePts, vec_keypoint);
				Mat samPts_xyz = vec2mat(samplePts);
				//求RT
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

				//用两个点求得RT来投影笔和椭圆
				MatRT RT1 = state2RT(kf_sys_key_points, tipPts);
				Mat1d modelIDxyz = vec2mat(tipPts);
				Mat1d modelxyz;
				modelIDxyz.rowRange(1, 4).copyTo(modelxyz);
				Mat1d sys_key_points = RT1 * add_homo(modelxyz);

				//采样点转到相机坐标系下
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
					if (num <= 20)  //一次Q采集次数为20
					{
						normalxyz.push_back(PinLocCT);
						num++;
					}
					else
					{
						isPush = false;
						num = 1;
						cout << "本次采集结束" << num << endl;
					}
				}
				else
				{
					cout << "请重新采集验证点" << endl;
				}

				/// <summary>
				/// 算角度
				/// </summary>
			
				Mat1d v_direc = PinLocCT - EndLocCT;
				double dotP = v_direc.dot(normal_Y);
				double norm1 = sqrt(pow(v_direc.at<double>(0,0), 2) + pow(v_direc.at<double>(1,0), 2) + pow(v_direc.at<double>(2,0), 2));
				double norm2 = sqrt(pow(normal_Y[0], 2) + pow(normal_Y[1], 2) + pow(normal_Y[2], 2));
				rake = acos(dotP/(norm1* norm2)); 
				abduction = atan2(v_direc.at<double>(0, 0), v_direc.at<double>(0, 2));
				cout << v_direc.at<double>(0, 0) << endl;

				//线片角度显示
				stringstream ss1, ss2;
				rake = round((90 - rake * 180 / CV_PI) * pow(10, 1)) * pow(10, -1);
				abduction = -round((abduction * 180 / CV_PI) * pow(10, 1)) * pow(10, -1);

				//角度值保留一位小数点
				ss1 << setiosflags(ios::fixed) << setprecision(1) << rake;
				ss2 << setiosflags(ios::fixed) << setprecision(1) << abduction;

				//线片显示器械投影转换关系
				Mat1d T1 = (Mat1d(2, 4) << 2, 0, 0, 250.305404663086,
					0, 0, -2, 3066.39526367188);//(正位片*有*股骨的转换关系)

				Mat1d T3 = (Mat1d(2, 4) << 2, 0, 0, 277.042999267578,
					0, 0, -2, -445.814575195313); //(正位片*无*股骨的转换关系)

				Mat1d T5 = (Mat1d(2, 4) << 0, 2, 0, -25.4366149902344,
					0, 0, -2, 3066.39526367188);//(侧位*有*股骨的转换关系)

				Mat1d T6 = (Mat1d(2, 4) << 0, -2, 0, -15.1028137207031,
					0, 0, -2, -445.814575195313);//(侧位*无*股骨的转换关系)

				if (drill_center_r_vector.size() > 0)
				{
					/////正位片-有股骨/////
					Mat Xray_img_ZhengWei_new = imread(X_rayImg_filename + "Xray_img_ZhengWei_1.jpg");
					resize(Xray_img_ZhengWei_new, Xray_img_ZhengWei_new, Size(541, 408), INTER_LINEAR);
					monitor::XrayImg_show(Xray_img_ZhengWei_new, T1, PinLocCT, EndLocCT, "lin1", samplePtsCT, radius_drill);
					//正位3D
					Mat Xray_img_ZhengWei_new3D = imread(X_rayImg_filename + "Xray_img_ZhengWei3D_1.png");
					resize(Xray_img_ZhengWei_new3D, Xray_img_ZhengWei_new3D, Size(541, 408), INTER_LINEAR);
					monitor::XrayImg_show(Xray_img_ZhengWei_new3D, T1, PinLocCT, EndLocCT, "lin1", samplePtsCT, radius_drill);
					//侧位 有股骨
					Mat Xray_img_ceWei = imread(X_rayImg_filename + "HCT_YouGuGuCeiW3D.jpg");
					monitor::ceWeiXrayImg_show(Xray_img_ceWei, T5, PinLocCT, EndLocCT, "lin5", samplePtsCT, radius_drill);

					resize(Xray_img_ZhengWei_new, Xray_img_ZhengWei_new, Size(541 * 2, 408 * 2), INTER_LINEAR);
					resize(Xray_img_ZhengWei_new3D, Xray_img_ZhengWei_new3D, Size(541 * 2, 408 * 2), INTER_LINEAR);
					resize(Xray_img_ceWei, Xray_img_ceWei, Size(305 * 2, 408 * 2), INTER_LINEAR);

					//有股骨显示
					Mat image_X;
					hconcat(Xray_img_ZhengWei_new, Xray_img_ZhengWei_new3D, image_X);
					hconcat(image_X, Xray_img_ceWei, image_X);
					string angle1 = "前倾角:";// 角度显示
					monitor::putTextInChinese(image_X, angle1.c_str(), Point2f(20, 500), Scalar(255, 255, 255), 100, "微软雅黑");
					putText(image_X, ss1.str(), Point2f(360, 550), cv::FONT_HERSHEY_PLAIN, 8, Scalar(255, 255, 255), 3);

					string angle2 = "外展角:";
					monitor::putTextInChinese(image_X, angle2.c_str(), Point2f(20, 650), Scalar(255, 255, 255), 100, "微软雅黑");
					putText(image_X, ss2.str(), Point2f(360, 700), cv::FONT_HERSHEY_PLAIN, 8, Scalar(255, 255, 255), 3);
					namedWindow("Line3_Xray", WINDOW_NORMAL);
					imshow("Line3_Xray", image_X);

					////正位片-无股骨////

					Mat Xray_img_ZhengWeiW_new = imread(X_rayImg_filename + "Xray_img_ZhengWeiW_1.jpg");
					monitor::XrayImg_show(Xray_img_ZhengWeiW_new, T3, PinLocCT, EndLocCT, "lin3", samplePtsCT, radius_drill);

					Mat Xray_img_ZhengWeiW3D_new = imread(X_rayImg_filename + "Xray_img_ZhengWeiW3D_1.png");
					monitor::XrayImg_show(Xray_img_ZhengWeiW3D_new, T3, PinLocCT, EndLocCT, "lin3", samplePtsCT, radius_drill);

					Mat Xray_img_ceWeiW = imread(X_rayImg_filename + "HCT_WuGuGuCeiW3D.jpg");
					//monitor::ceWeiXrayImg_show(Xray_img_ceWeiW, T6, PinLocCT, EndLocCT, "lin6");
					monitor::ceWeiXrayImg_show(Xray_img_ceWeiW, T6, PinLocCT, EndLocCT, "lin6", samplePtsCT, radius_drill);

					//无股骨显示
					Mat image_Xw;
					hconcat(Xray_img_ZhengWeiW_new, Xray_img_ZhengWeiW3D_new, image_Xw);
					hconcat(image_Xw, Xray_img_ceWeiW, image_Xw);
					Mat resize_image_Xw;
					resize(image_Xw, resize_image_Xw, Size(2392, 248 * 2), INTER_LINEAR);

					monitor::putTextInChinese(resize_image_Xw, angle1.c_str(), Point2f(50, 300), Scalar(255, 255, 255), 50, "微软雅黑");
					putText(resize_image_Xw, ss1.str(), Point2f(230, 360), cv::FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 2);
					monitor::putTextInChinese(resize_image_Xw, angle2.c_str(), Point2f(50, 400), Scalar(255, 255, 255), 50, "微软雅黑");
					putText(resize_image_Xw, ss2.str(), Point2f(230, 460), cv::FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 2);
					namedWindow("Line4_Xray", WINDOW_NORMAL);
					imshow("Line4_Xray", resize_image_Xw);

				}
				else
				{
					//林-正位片 
					Mat Xray_img_ZhengWei = imread(X_rayImg_filename + "zhengWei_1.bmp");
					monitor::XrayImg_show(Xray_img_ZhengWei, T1, PinLocCT, EndLocCT, "lin1", samplePtsCT, radius_drill);
					//林-正位片 3D
					Mat Xray_img_ZhengWei3D = imread(X_rayImg_filename + "zhengwei3D_1.png");
					//Mat1d T2 = (Mat1d(2, 4) << 2, 0, 0, 275.167388916016,
					//	0, 0, -2, -1575.20507812500);
					monitor::XrayImg_show(Xray_img_ZhengWei3D, T1, PinLocCT, EndLocCT, "lin2", samplePtsCT, radius_drill);

					////林-无股骨正位片 
					Mat Xray_img_ZhengWeiW = imread(X_rayImg_filename + "ZhengW_123.jpg");
					//Mat1d T3 = (Mat1d(2, 4) << 2, 0, 0, 221.907806396484,
					//	0, 0, -2, -1575.01318359375);
					monitor::XrayImg_show(Xray_img_ZhengWeiW, T3, PinLocCT, EndLocCT, "lin3", samplePtsCT, radius_drill);
					////林-无股骨正位片3D 
					Mat Xray_img_ZhengWeiW3D = imread(X_rayImg_filename + "ZhengW_3D.jpg");
					//Mat1d T4 = (Mat1d(2, 4) << 2, 0, 0, 221.907806396484,
					//	0, 0, -2, -1575.01318359375);
					monitor::XrayImg_show(Xray_img_ZhengWeiW3D, T3, PinLocCT, EndLocCT, "lin4", samplePtsCT, radius_drill);

					//林-侧位3D
					Mat Xray_img_ceWei = imread(X_rayImg_filename + "cewei_1.png");
					monitor::ceWeiXrayImg_show(Xray_img_ceWei, T5, PinLocCT, EndLocCT, "lin5", samplePtsCT, radius_drill);
					//林-侧位无股骨3D
					Mat Xray_img_ceWeiW = imread(X_rayImg_filename + "CeiW_123.jpg");
					//monitor::ceWeiXrayImg_show(Xray_img_ceWeiW, T6, PinLocCT, EndLocCT, "lin6");
					monitor::ceWeiXrayImg_show(Xray_img_ceWeiW, T6, PinLocCT, EndLocCT, "lin6", samplePtsCT, radius_drill);

					resize(Xray_img_ZhengWei, Xray_img_ZhengWei, Size(556 * 2, 441 * 2), INTER_LINEAR);
					resize(Xray_img_ZhengWei3D, Xray_img_ZhengWei3D, Size(556 * 2, 441 * 2), INTER_LINEAR);
					resize(Xray_img_ceWei, Xray_img_ceWei, Size(319 * 2, 441 * 2), INTER_LINEAR);

					//有股骨显示
					Mat image_X;
					hconcat(Xray_img_ZhengWei, Xray_img_ZhengWei3D, image_X);
					hconcat(image_X, Xray_img_ceWei, image_X);
					string angle1 = "前倾角:";// 角度显示
					monitor::putTextInChinese(image_X, angle1.c_str(), Point2f(20, 500), Scalar(255, 255, 255), 70, "微软雅黑");
					putText(image_X, ss1.str(), Point2f(320, 600), cv::FONT_HERSHEY_PLAIN, 8, Scalar(255, 255, 255), 3);

					string angle2 = "外展角:";
					monitor::putTextInChinese(image_X, angle2.c_str(), Point2f(20, 650), Scalar(255, 255, 255), 70, "微软雅黑");
					putText(image_X, ss2.str(), Point2f(320, 750), cv::FONT_HERSHEY_PLAIN, 8, Scalar(255, 255, 255), 3);
					namedWindow("Line1_Xray", WINDOW_NORMAL);
					imshow("Line1_Xray", image_X);

					//无股骨显示
					Mat image_Xw;
					hconcat(Xray_img_ZhengWeiW, Xray_img_ZhengWeiW3D, image_Xw);
					hconcat(image_Xw, Xray_img_ceWeiW, image_Xw);
					Mat resize_image_Xw;
					resize(image_Xw, resize_image_Xw, Size(2882, 340 * 2), INTER_LINEAR);

					monitor::putTextInChinese(resize_image_Xw, angle1.c_str(), Point2f(50, 300), Scalar(255, 255, 255), 50, "微软雅黑");
					putText(resize_image_Xw, ss1.str(), Point2f(230, 360), cv::FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 2);
					monitor::putTextInChinese(resize_image_Xw, angle2.c_str(), Point2f(50, 400), Scalar(255, 255, 255), 50, "微软雅黑");
					putText(resize_image_Xw, ss2.str(), Point2f(230, 460), cv::FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 2);
					namedWindow("Line2_Xray", WINDOW_NORMAL);
					imshow("Line2_Xray", resize_image_Xw);
				}

				int key_wait = waitKey(15);
				if (key_wait == 82)//R
				{
					//正位有股骨
					Mat Xray_img_ZhengWei_1 = imread(X_rayImg_filename + "HCT_ZhengWYouGuGu.jpg");
					Mat Xray_img_ZhengWei3D_1 = imread(X_rayImg_filename + "HCT_ZhengWYouGuGu3D.jpg");

					//正位无股骨
					Mat Xray_img_ZhengWeiW_1 = imread(X_rayImg_filename + "HCT_WuGuGuZhengW.jpg");
					Mat Xray_img_ZhengWeiW3D_1 = imread(X_rayImg_filename + "HCT_WuGuGuZhengW3D.jpg");

					Mat1d drill_center_r;
					//笔尖点 笔末端点 球头半径
					PinLocCT.copyTo(drill_center_r);
					drill_center_r.push_back(EndLocCT);
					drill_center_r.push_back(radius_drill);
					drill_center_r_vector.push_back(drill_center_r); //记录挫头的大小
					cout << "请输入新锉头的半径大小：" << endl;
					cin >> radius_drill;

					//记录上一次的位置，并存成图片
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

	// 读 model
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
	// 定义模型对象
	pcl::PolygonMesh mesh;

	// 加载STL模型文件
	pcl::io::loadPolygonFileSTL(filename, mesh);

	// 将模型转换为点云对象
	pcl::fromPCLPointCloud2(mesh.cloud, *grid);
}

//显示3D点云模型
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

	// 创建可视化对象
	visualization::PCLVisualizer viewer("Point Cloud Viewer");

	// 设置点云渲染颜色为绿色
	visualization::PointCloudColorHandlerCustom<PointXYZ> color_handler(cloud, 0, 255, 0);
	viewer.addPointCloud<PointXYZ>(cloud, color_handler, "cloud");
	// 设置可视化参数
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	//添加坐标轴
	viewer.addCoordinateSystem(50.0, "cloud", 0);
	viewer.setCameraPosition(0, -100, -300, 1, 0, 0, 0, 0, 1);
	viewer.addLine<PointXYZ>(point1, point2, "line");

	viewer.spinOnce();

}