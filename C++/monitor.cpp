#include "monitor.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
using namespace cv;

void monitor::debug_show(const Data::detector& data)
{
	Mat3b imgShow = get_imgShow(data.in_img);

	// 展示耗时
	put_timecost(imgShow, data.d_time_cost);

	// 展示不同阶段的检测结果
	float scale = (float)imgShow.rows / 480;
	for (Point2f p : data.d_ptPre)
		drawMarker(imgShow, p, Scalar(0, 0, 255), MARKER_CROSS, scale * 3, scale * 1, LINE_AA);
	for (Point2f p : data.o_ptList)
		circle(imgShow, pow(2, monitor::shift) * p, pow(2, monitor::shift) * scale * 3, Scalar(0, 255, 0), scale * 1, LINE_AA, monitor::shift);

	// 绘制
	namedWindow("monitor detector", WINDOW_NORMAL);
	resizeWindow("monitor detector", 500, 400);
	imshow("monitor detector", imgShow);
}

void monitor::debug_show(const Data::constructor& data)
{
	Mat3b imgShow = 0.5*get_imgShow(data.in_img);

	// 展示耗时
	put_timecost(imgShow, data.d_time_cost);

	// 展示检测结果
	float scale = min((float)imgShow.cols / 960, (float)imgShow.rows / 480);
	if (!data.d_hexInfo.empty())
	{
		//// 绘制三角形
		//for (int i = 0; i < data.d_triList.size(); i++)
		//{
		//	// 绘制三角形轮廓
		//	Vec3i tri = data.d_triList[i];
		//	Point2f p0 = data.in_ptList[tri[0]];
		//	Point2f p1 = data.in_ptList[tri[1]];
		//	Point2f p2 = data.in_ptList[tri[2]];
		//	line(imgMark, pow(2, monitor::shift) * p0, pow(2, monitor::shift) * p1, Scalar(0, 0, 255), max(1, (int)(scale * 0.5)), LINE_AA, monitor::shift);
		//	line(imgMark, pow(2, monitor::shift) * p0, pow(2, monitor::shift) * p2, Scalar(0, 0, 255), max(1, (int)(scale * 0.5)), LINE_AA, monitor::shift);
		//	line(imgMark, pow(2, monitor::shift) * p1, pow(2, monitor::shift) * p2, Scalar(0, 0, 255), max(1, (int)(scale * 0.5)), LINE_AA, monitor::shift);

		//	// 标记三角形状态
		//	Point2f c = (data.in_ptList[tri[0]] + data.in_ptList[tri[1]] + data.in_ptList[tri[2]]) / 3;
		//	int state = data.d_tri_IdxtoState.find(tri)->second;
		//	if (state == 0)
		//		drawMarker(imgMark, c, Scalar(0, 0, 255), MARKER_TILTED_CROSS, scale * 5, max(1, (int)scale), LINE_AA);
		//	if (state == 2)
		//		fillPoly(imgMark, vector<vector<Point>>{vector<Point>{pow(2, monitor::shift)* p0, pow(2, monitor::shift)* p1, pow(2, monitor::shift)* p2}}, Scalar(0, 255, 255), LINE_AA, monitor::shift);
		//	if (state == 3)
		//		fillPoly(imgMark, vector<vector<Point>>{vector<Point>{pow(2, monitor::shift)* p0, pow(2, monitor::shift)* p1, pow(2, monitor::shift)* p2}}, Scalar(255, 255, 0), LINE_AA, monitor::shift);
		//}

		// 绘制hexPair
		for (Vec11i info : data.d_hexInfo)
		{
			auto it = data.map_key2idx.find(info[10]);
			if (it == data.map_key2idx.end())	continue;
			Vec10i sub = it->second;

			Point2f p[10];
			for (int j = 0; j < 10; j++)
				p[j] = data.in_ptList[info[j]];

			if (data.d_IDList[info[0]] == sub[0] && data.d_IDList[info[1]] == sub[1])
				line(imgShow, pow(2, shift) * p[0], pow(2, shift) * p[1], Scalar(0, 255, 0), max(1, (int)(scale * 2)), LINE_AA, shift);

			for (int j = 2; j < 7; j++)
				if (data.d_IDList[info[0]]==sub[0]&& data.d_IDList[info[j]] == sub[j])
					line(imgShow, pow(2, shift) * p[0], pow(2, shift) * p[j], Scalar(0, 255, 0), max(1, (int)(scale * 2)), LINE_AA, shift);
			
			for (int j = 6; j < 9; j++)
				if (data.d_IDList[info[1]] == sub[1] && data.d_IDList[info[j]] == sub[j])
					line(imgShow, pow(2, shift) * p[1], pow(2, shift) * p[j], Scalar(0, 255, 0), max(1, (int)(scale * 2)), LINE_AA, shift);
			
			if (data.d_IDList[info[1]] == sub[1] && data.d_IDList[info[2]] == sub[2])
			line(imgShow, pow(2, shift) * p[1], pow(2, shift) * p[2], Scalar(0, 255, 0), max(1, (int)(scale * 2)), LINE_AA, shift);

			if (data.d_IDList[info[2]] == sub[2] && data.d_IDList[info[9]] == sub[9])
			line(imgShow, pow(2, shift) * p[2], pow(2, shift) * p[9], Scalar(0, 255, 0), max(1, (int)(scale * 2)), LINE_AA, shift);
			
			for (int j = 2; j < 9; j++)
				if (data.d_IDList[info[j]] == sub[j] && data.d_IDList[info[j + 1]] == sub[j + 1])
					line(imgShow, pow(2, shift) * p[j], pow(2, shift) * p[j + 1], Scalar(0, 255, 0), max(1, (int)(scale * 2)), LINE_AA, shift);
		}
	}
	
	//// 标记点
	//for (size_t i = 0; i < data.in_ptList.size(); i++)
	//	if (data.d_IDList[i] != -1)
	//		putText(imgShow, to_string(data.d_IDList[i]), data.in_ptList[i], FONT_ITALIC, scale * 0.7, Scalar(0, 0, 255), scale * 0.8, LINE_AA);
	
	// 绘制
	namedWindow("monitor constructor", WINDOW_NORMAL);
    resizeWindow("monitor constructor", 500, 400);
	imshow("monitor constructor", imgShow);
}

void monitor::debug_show(const vector<Mat1b>& img, const Data::poseSolver& data)
{
	int camNum = img.size();
	for (int cam = 0; cam < camNum; cam++)
	{
		Mat3b imgShow = get_imgShow(img[cam]);
		float scale = (float)imgShow.rows / 480;

		put_timecost(imgShow, data.d_time_cost);

		if (!data.d_repro_fList.empty())
			for (Vec3f vec : data.d_repro_fList[cam])
				circle(imgShow, pow(2, monitor::shift) * Point(vec[1], vec[2]), pow(2, monitor::shift) * scale * 3, Scalar(0, 255, 0), scale * 3, LINE_AA, monitor::shift);

		// 绘制
		namedWindow("mPnP Cam " + to_string(cam), WINDOW_NORMAL);
		resizeWindow("mPnP Cam " + to_string(cam), 1000, 800);
		imshow("mPnP Cam " + to_string(cam), imgShow);	
	}
}

Mat3b monitor::get_imgShow(const Mat& img)
{
	assert(!img.empty());
	assert(img.type() == CV_8UC1 || img.type() == CV_8UC3);

	Mat3b imgShow;
	if (img.type() == CV_8UC1)		cvtColor(img, imgShow, COLOR_GRAY2RGB);
	else							img.copyTo(imgShow);

	return imgShow;
}

void monitor::put_timecost(Mat3b& img, const vector<Data::time_cost>& info)
{
	int num = info.size();
	float scale = min((float)img.cols / 960, (float)img.rows / 480);

	ostringstream s;
	for (int i = 0; i < num; i++)
	{
		s.str("");	s.clear();
		s << setprecision(1) << fixed << info[i].time;
		putText(img, info[i].name + ":" + s.str() + "ms", scale * Point(5, (i + 1) * 15), FONT_ITALIC, 0.5 * scale, Scalar(0, 255, 255), scale, LINE_AA);
	}
}

void monitor::put_number(Mat3b& img, const vector<String>& name, const vector<double>& times, const String suffix)
{
	assert(name.size() == times.size());
	
	int num = name.size();
	float scale = (float)img.rows / 480;

	ostringstream s;
	for (int i = 0; i < num; i++)
	{
		s.str("");	s.clear();
		s << setprecision(1) << fixed << times[i];
		putText(img, name[i] + ":" + s.str() + suffix, scale * Point(5, (i + 1) * 15), FONT_ITALIC, 0.5 * scale, Scalar(0, 255, 255), scale, LINE_AA);
	}
}

void monitor::put_number(Mat3b& img, const vector<String>& name, const vector<double>& times, const vector<String>& suffix)
{
	TRY_THROW(name.size() == times.size(), "the size of names and times must be the same!");
	TRY_THROW(name.size() == suffix.size(), "the size of names and suffix must be the same!");

	int num = name.size();
	float scale = (float)img.rows / 480;

	ostringstream s;
	for (int i = 0; i < num; i++)
	{
		s.str("");	s.clear();
		s << setprecision(1) << fixed << times[i];
		putText(img, name[i] + ":" + s.str() + suffix[i], scale * Point(5, (i + 1) * 15), FONT_ITALIC, 0.5 * scale, Scalar(0, 255, 255), scale, LINE_AA);
	}
}

void monitor::DrawArc(Mat& src, Point2f ArcCenter, Point2f StartPoint, Point2f EndPoint, int Fill, Scalar color)
{
	if (Fill <= 0) return;

	vector<Point2f> Dots;
	double Angle1 = atan2((StartPoint.y - ArcCenter.y), (StartPoint.x - ArcCenter.x));
	double Angle2 = atan2((EndPoint.y - ArcCenter.y), (EndPoint.x - ArcCenter.x));
	double Angle = Angle1 - Angle2;
	Angle = Angle * 180.0 / CV_PI;

	if (Angle < 0) Angle = 360 + Angle;
	if (Angle == 0) Angle = 360;
	int  brim = floor(Angle / 10); // 向下取整

	Dots.push_back(StartPoint);
	for (int i = 0; i < brim; i++)
	{
		double dSinRot = sin(-(10 * (i + 1)) * CV_PI / 180);
		double dCosRot = cos(-(10 * (i + 1)) * CV_PI / 180);
		int x = ArcCenter.x + dCosRot * (StartPoint.x - ArcCenter.x) - dSinRot * (StartPoint.y - ArcCenter.y);
		int y = ArcCenter.y + dSinRot * (StartPoint.x - ArcCenter.x) + dCosRot * (StartPoint.y - ArcCenter.y);
		Dots.push_back(Point2f(x, y));
	}
	Dots.push_back(EndPoint);
	RNG& rng = theRNG();
	//Scalar color = Scalar(0, 0, 255);
	//Scalar color = Scalar(rng.uniform(100, 255), rng.uniform(100, 255), rng.uniform(100, 255));
	for (int i = 0; i < Dots.size() - 1; i++) {
		line(src, Dots[i], Dots[i + 1], color, Fill);
	}
	Dots.clear();
}

string monitor::ToString(double val)
{
	stringstream ss;
	ss << setiosflags(ios::fixed) << setprecision(1) << val;
	string str = ss.str();
	return str;
}

void monitor::Xray_show(const Mat& img, const Mat& RT, const Mat1d& T, const vector<Mat1d>& sys_key_points)
{
	vector<Point2d>Xray_points;
	Point2d Xray_point;
	MatRT XRT = RT;
	for (int i = 0; i < sys_key_points.size(); i++)
	{
		Mat1d Xray_key_point=T*add_homo(RT* add_homo(sys_key_points[i]));	
		Xray_point.x = Xray_key_point(0, 0);
		Xray_point.y = Xray_key_point(1, 0);
		Xray_points.push_back(Xray_point);
	}
	line(img, Xray_points[0], Xray_points[1],  Scalar(0, 0, 255), 5);
	//line(img, Xray_points[2], Xray_points[3], Scalar(0, 255, 0), 5);
	namedWindow("monitor Xray", WINDOW_NORMAL);
	imshow("monitor Xray", img);
}

void  monitor::XrayImg_show( Mat& img, const Mat1d& T,const Mat1d& PinLocCT, const Mat1d& EndLocCT,String name, const Mat1d& samplePtsCT, double radius_drill)
{
	vector<Point2f>Xray_points;
	Point2f Xray_point_pin;
	Point2f Xray_point_end;

	Mat1d Xray_key_point = T * add_homo(PinLocCT);
	Xray_point_pin.x = Xray_key_point(0, 0);
	Xray_point_pin.y = Xray_key_point(1, 0);
	Xray_points.push_back(Xray_point_pin);

	Mat1d Xray_key_point1 = T * add_homo(EndLocCT);
	Xray_point_end.x = Xray_key_point1(0, 0);
	Xray_point_end.y = Xray_key_point1(1, 0);
	Xray_points.push_back(Xray_point_end);

	//draw arc 画挫头  有左右之分
	//左边的
	float r = radius_drill *2;
	double angle = atan2(abs(Xray_point_pin.y - Xray_point_end.y), abs(Xray_point_pin.x - Xray_point_end.x));
	float x1 = Xray_point_pin.x - r * sin(angle);
	float y1 = Xray_point_pin.y + r * cos(angle);
	float x2 = Xray_point_pin.x + r * sin(angle);
	float y2 = Xray_point_pin.y - r * cos(angle);
	
	////右边的
	//float x1 = Xray_point_pin.x - r * sin(angle);
	//float y1 = Xray_point_pin.y - r * cos(angle);
	//float x2 = Xray_point_pin.x + r * sin(angle);
	//float y2 = Xray_point_pin.y + r * cos(angle);

	Mat1d samplePtsCT_2D = T * add_homo(samplePtsCT);
	vector<Point2f> sampPts_vect;
	Point2f samPts;
	for (int i = 0; i < samplePtsCT_2D.cols; i++)
	{
		samPts.x = samplePtsCT_2D(0, i);
		samPts.y = samplePtsCT_2D(1, i);
		sampPts_vect.push_back(samPts);
	}

	DrawArc(img, Xray_points[0],  Point2f(x2, y2), Point2f(x1, y1), 2, Scalar(255,255,255));
	line(img, Point2f(x1, y1), Point2f(x2, y2), Scalar(255, 255, 255), 2);

	//画椭圆
	RotatedRect box;
	box = fitEllipse(sampPts_vect);
	if (isnan(box.size.width) || isnan(box.size.height))
		return;
	else
		ellipse(img, box, Scalar(255, 255, 255), 3, LINE_AA);

	line(img, Xray_points[0], Xray_points[1], Scalar(255, 255, 255), 5);

}

void monitor::registration_show(const vector<Mat1d>& sys_key_points, const vector<Mat1b>& img, const vector<Vec3d> & normalxyz, const Data::poseSolver& data)
{
	Mat img0,img1;
	//float rake, abduction;
	Point2f comp1end, comp2end;
	img[0].copyTo(img0);
	img[1].copyTo(img1);
	Mat1d v = sys_key_points[1] - sys_key_points[0];
	Vec3d v_cast(v.dot(normalxyz[0]), v.dot(normalxyz[1]), v.dot(normalxyz[2]));
	double rake = atan2(v_cast[1], v_cast[0]);
	double abduction = atan2(v_cast[2], v_cast[0]);
	cvtColor(img0, img0, COLOR_GRAY2RGB);
	cvtColor(img1, img1, COLOR_GRAY2RGB);
	
	// display compass
	float r = 300;
	Point2f comC1(100 + r, 150+100 + r);
	Point2f comC2(100 + r, 150 + 2 * r + 200 + r);
	circle(img0, comC1, r, Scalar(255, 0, 0), 5);
	circle(img0, comC2, r, Scalar(255, 0, 0), 5);

	//左右腿罗盘有差异
	//左腿
	line(img0, comC1, comC1 + Point2f(r, 0), Scalar(255, 0, 0), 10);
	line(img0, comC2, comC2 + Point2f(r, r) / sqrt(2), Scalar(255, 0, 0), 10);

	//右腿
	//line(img0, comC1, comC1 - Point2f(r, 0), Scalar(255, 0, 0), 10);
	//line(img0, comC2, comC2 - Point2f(r, -r) / sqrt(2), Scalar(255, 0, 0), 10);
	// 
	//putText(img0, "rake", comC1 + Point2f(100, 0), cv::FONT_HERSHEY_PLAIN, 8, Scalar(0, 255, 0), 4);
	//putText(img0, "abduction", comC2 + Point2f(100, 0), cv::FONT_HERSHEY_PLAIN, 5, Scalar(0, 255, 0), 4);
	putTextInChinese(img0, "前倾角", Point2f(450, 400), Scalar(255, 255, 255), 100, "微软雅黑");
	putTextInChinese(img0, "外展角", Point2f(450, 1100), Scalar(255, 255, 255), 100, "微软雅黑");

	//putText(img0, ToString(round((rake * 180 / CV_PI) * pow(10, 1)) * pow(10, -1)), comC1 + Point2f(100, 100), cv::FONT_HERSHEY_PLAIN, 8, Scalar(0, 255, 0), 4);
	putText(img0, ToString(-round((rake * 180 / CV_PI - 180) * pow(10,1))*pow(10,-1)), comC1 + Point2f(100, 100), cv::FONT_HERSHEY_PLAIN, 8, Scalar(0, 255, 0), 4);
	putText(img0, ToString(round((abduction * 180 / CV_PI - 90) * pow(10, 1)) * pow(10, -1)), comC2 + Point2f(100, 100), cv::FONT_HERSHEY_PLAIN, 8, Scalar(0, 255, 0), 4);


	comp1end = comC1 - r * Point2f(cos(rake), -sin(rake));
	comp2end = comC2 + r * Point2f(-cos(abduction), sin(abduction));

	arrowedLine(img0, comC1, comp1end, Scalar(0, 255, 0), 8);
	arrowedLine(img0, comC2, comp2end, Scalar(0, 255, 0), 8);

	// pin
	int repropoint_number = data.d_repro_fList[0].size();
	if (repropoint_number != 0)
	{
		Vec3f r_uv = data.d_repro_fList[0][repropoint_number - 1];
		circle(img0, Point(r_uv[1], r_uv[2]), 20, Scalar(0, 0, 255), 15);
	}
	Mat img_show = img0;
	int L = 800;
	resize(img_show, img_show, Size(L, L));
	//img_show(Rect(img0.cols - L - 1, 0, L, L)) = 0.5 * img_show(Rect(img0.cols - L - 1, 0, L, L)) + 0.5 * img1;
	putText(img_show, "registering...", Point(50, 50), cv::FONT_HERSHEY_PLAIN, 4, Scalar(0, 255, 0), 4);
	namedWindow("view", WINDOW_NORMAL);
	imshow("view", img_show);
}

void monitor::CT_img_show(vector<Mat>& CT_imgs, const Mat& RT, const vector <Mat1d> CT_img_scal, const vector<Mat1d>& sys_key_points)
{
	Mat Coronal_img = CT_imgs[0];
	Mat Axial_img = CT_imgs[1];
	Mat Sagittal_img = CT_imgs[2];
	vector<Mat1d> CT_Keypoints;
	vector<Point2d>Coronal_points, Axial_points, Sagittal_points;
	int A = 800;    //CT图片的长宽高
	int B = 304;

	// 计算笔尖位子在三视CT图中的二维投影坐标
	for (int i = 0; i < sys_key_points.size(); i++)
	{
		Mat1d CT_Key_point = RT * add_homo(sys_key_points[i]);
		CT_Keypoints.push_back(CT_Key_point);
		Point2d Coronal_point, Axial_point, Sagittal_point;

		Coronal_point.x = CT_img_scal[0](0) * CT_Key_point(0);
		Coronal_point.y = CT_img_scal[0](1) * CT_Key_point(1) + B;
		Coronal_points.push_back(Coronal_point);

		Axial_point.x = CT_img_scal[1](0) * CT_Key_point(0);
		Axial_point.y = CT_img_scal[1](1) * CT_Key_point(2) + A;
		Axial_points.push_back(Axial_point);

		Sagittal_point.x = CT_img_scal[2](0) * CT_Key_point(2);
		Sagittal_point.y = CT_img_scal[2](1) * CT_Key_point(1) + B;
		Sagittal_points.push_back(Sagittal_point);
	}

	// 计算路径的投影
	Point2d Coroanl_route_star, Coroanl_route_end, Axial_route_star, Axial_route_end, Sagittal_route_star, Sagittal_route_end;
	Coroanl_route_star.x = 285.9558818912190;
	Coroanl_route_star.y = 196.3235124103258;
	Coroanl_route_end.x = 283.7723772970758;
	Coroanl_route_end.y = 252.6607283186777;

	Axial_route_star.x = 285.9558818912190;
	Axial_route_star.y = 242.3280843316667;
	Axial_route_end.x = 283.7723772970758;
	Axial_route_end.y = 237.4575455285924;

	Sagittal_route_star.x = 557.6719156683333;
	Sagittal_route_star.y = 196.3235124103258;
	Sagittal_route_end.x = 562.5424544714076;
	Sagittal_route_end.y = 252.6607283186777;

	//在CT图上画线
	line(CT_imgs[0], Coronal_points[0], Coronal_points[1], Scalar(0, 0, 255), 5);
	line(CT_imgs[0], Coroanl_route_star, Coroanl_route_end, Scalar(0, 255, 0), 8);
	line(CT_imgs[1], Axial_points[0], Axial_points[1], Scalar(0, 0, 255), 5);
	line(CT_imgs[1], Axial_route_star, Axial_route_end, Scalar(0, 255, 0), 8);
	line(CT_imgs[2], Sagittal_points[0], Sagittal_points[1], Scalar(0, 0, 255), 5);
	line(CT_imgs[2], Sagittal_route_star, Sagittal_route_end, Scalar(0, 255, 0), 10);
	Mat CT_img_show;
	vconcat(CT_imgs[0], CT_imgs[2], CT_img_show);
	int L = 608;
	resize(CT_imgs[1], CT_imgs[1], Size(L, L));
	hconcat(CT_img_show, CT_imgs[1], CT_img_show);
	imshow("view", CT_img_show);
}

void  monitor::ceWeiXrayImg_show(Mat& img, const Mat1d& T, const Mat1d& PinLocCT, const Mat1d& EndLocCT, String name, const Mat1d& samplePtsCT, double radius_drill)
{
	vector<Point2f>Xray_points;
	Point2f Xray_point_pin;
	Point2f Xray_point_end;

	Mat1d Xray_key_point = T * add_homo(PinLocCT);
	Xray_point_pin.x = Xray_key_point(0, 0);
	Xray_point_pin.y = Xray_key_point(1, 0);
	Xray_points.push_back(Xray_point_pin);

	Mat1d Xray_key_point1 = T * add_homo(EndLocCT);
	Xray_point_end.x = Xray_key_point1(0, 0);
	Xray_point_end.y = Xray_key_point1(1, 0);
	Xray_points.push_back(Xray_point_end);

	//转换到2维上
	Mat1d samplePtsCT_2D = T * add_homo(samplePtsCT);
	vector<Point2f> sampPts_vect;
	Point2f samPts;
	for (int i = 0; i < samplePtsCT_2D.cols; i++)
	{
		samPts.x = samplePtsCT_2D(0, i);
		samPts.y = samplePtsCT_2D(1, i);
		sampPts_vect.push_back(samPts);
	}
	RotatedRect box;
	box = fitEllipse(sampPts_vect);

	//draw arc 画挫头  有左右之分
	float r =  radius_drill * 2;
	double angle = atan2(abs(Xray_point_pin.y - Xray_point_end.y), abs(Xray_point_pin.x - Xray_point_end.x));
	float x1 = Xray_point_pin.x - r * sin(angle);
	float y1 = Xray_point_pin.y + r * cos(angle);
	float x2 = Xray_point_pin.x + r * sin(angle);
	float y2 = Xray_point_pin.y - r * cos(angle);

	if(isnan(box.size.width) || isnan(box.size.height))
		return;
	else
		ellipse(img, box, Scalar(255, 255, 255), 3, LINE_AA);
	line(img, Xray_points[0], Xray_points[1], Scalar(255, 255, 255), 5);

}

void monitor::putTextInChinese(Mat& dst, const char* str, Point org, Scalar color, int fontsize, const char* fn, bool italic, bool underline)
{
	CV_Assert(dst.data != 0 && (dst.channels() == 1 || dst.channels() == 3));


	int x, y, r, b;
	//坐标点大于图像宽和高直接返回
	if (org.x > dst.cols || org.y > dst.rows) return;
	x = org.x < 0 ? -org.x : 0;
	y = org.y < 0 ? -org.y : 0;


	LOGFONTA lf;
	lf.lfHeight = -fontsize; //字体高度
	lf.lfWidth = 0;          //平均宽度
	lf.lfEscapement = 0;   //字符排列角度
	lf.lfOrientation = 0;   //字符本身旋转的角度
	lf.lfWeight = 5;        //设置字体线条的宽度
	lf.lfItalic = italic; //斜体
	lf.lfUnderline = underline; //下划线
	lf.lfStrikeOut = 0;     //是否字符中央加横线
	lf.lfCharSet = DEFAULT_CHARSET;   //字符集
	lf.lfOutPrecision = 0;  //字体的精确度
	lf.lfQuality = PROOF_QUALITY;  //字体质量
	lf.lfPitchAndFamily = 0; //选择字体的间距和字体家族
	strcpy_s(lf.lfFaceName, fn); //字体的名称


	HFONT hf = CreateFontIndirectA(&lf);
	HDC hdc = CreateCompatibleDC(0);
	HFONT holdfont = (HFONT)SelectObject(hdc, hf);


	int strBaseW = 0, strBaseH = 0;
	int singleRow = 0;
	char buf[1 << 12];
	strcpy_s(buf, str);
	char* bufT[1 << 12];   //这个用于分隔字符串后剩余的字符，可能会超出。


	//处理多行
	{
		int nnh = 0;
		int cw, ch;


		const char* ln = strtok_s(buf, "\n", bufT);
		while (ln != 0)
		{
			GetStringSize(hdc, ln, &cw, &ch);
			strBaseW = max(strBaseW, cw);
			strBaseH = max(strBaseH, ch);


			ln = strtok_s(0, "\n", bufT);
			nnh++;
		}
		singleRow = strBaseH;
		strBaseH *= nnh;
	}


	if (org.x + strBaseW < 0 || org.y + strBaseH < 0)
	{
		SelectObject(hdc, holdfont);
		DeleteObject(hf);
		DeleteObject(hdc);
		return;
	}


	r = org.x + strBaseW > dst.cols ? dst.cols - org.x - 1 : strBaseW - 1;
	b = org.y + strBaseH > dst.rows ? dst.rows - org.y - 1 : strBaseH - 1;
	org.x = org.x < 0 ? 0 : org.x;
	org.y = org.y < 0 ? 0 : org.y;


	BITMAPINFO bmp = { 0 };
	BITMAPINFOHEADER& bih = bmp.bmiHeader;
	int strDrawLineStep = strBaseW * 3 % 4 == 0 ? strBaseW * 3 : (strBaseW * 3 + 4 - ((strBaseW * 3) % 4));


	bih.biSize = sizeof(BITMAPINFOHEADER);
	bih.biWidth = strBaseW;
	bih.biHeight = strBaseH;
	bih.biPlanes = 1;
	bih.biBitCount = 24;
	bih.biCompression = BI_RGB;
	bih.biSizeImage = strBaseH * strDrawLineStep;
	bih.biClrUsed = 0;
	bih.biClrImportant = 0;


	void* pDibData = 0;
	HBITMAP hBmp = CreateDIBSection(hdc, &bmp, DIB_RGB_COLORS, &pDibData, 0, 0);


	CV_Assert(pDibData != 0);
	HBITMAP hOldBmp = (HBITMAP)SelectObject(hdc, hBmp);


	//color.val[2], color.val[1], color.val[0]
	SetTextColor(hdc, RGB(255, 255, 255));
	SetBkColor(hdc, 0);
	//SetStretchBltMode(hDC, COLORONCOLOR);


	strcpy_s(buf, str);
	const char* ln = strtok_s(buf, "\n", bufT);
	int outTextY = 0;
	while (ln != 0)
	{
		TextOutA(hdc, 0, outTextY, ln, strlen(ln));
		outTextY += singleRow;
		ln = strtok_s(0, "\n", bufT);
	}
	uchar* dstData = (uchar*)dst.data;
	int dstStep = dst.step / sizeof(dstData[0]);
	unsigned char* pImg = (unsigned char*)dst.data + org.x * dst.channels() + org.y * dstStep;
	unsigned char* pStr = (unsigned char*)pDibData + x * 3;
	for (int tty = y; tty <= b; ++tty)
	{
		unsigned char* subImg = pImg + (tty - y) * dstStep;
		unsigned char* subStr = pStr + (strBaseH - tty - 1) * strDrawLineStep;
		for (int ttx = x; ttx <= r; ++ttx)
		{
			for (int n = 0; n < dst.channels(); ++n) {
				double vtxt = subStr[n] / 255.0;
				int cvv = vtxt * color.val[n] + (1 - vtxt) * subImg[n];
				subImg[n] = cvv > 255 ? 255 : (cvv < 0 ? 0 : cvv);
			}


			subStr += 3;
			subImg += dst.channels();
		}
	}


	SelectObject(hdc, hOldBmp);
	SelectObject(hdc, holdfont);
	DeleteObject(hf);
	DeleteObject(hBmp);
	DeleteDC(hdc);
}

void monitor::GetStringSize(HDC hdc, const char* str, int* w, int* h)
{
		SIZE size;
		GetTextExtentPoint32A(hdc, str, strlen(str), &size);
		if (w != 0) *w = size.cx;
		if (h != 0) *h = size.cy;
}

void monitor::recordChange(Mat& img, const Mat1d& T, vector<Mat1d>& drill_center_r_vector,string name,string mainFilename)
{
	
	for (int j = 0;  j < drill_center_r_vector.size(); j++)
	{
		Mat1d drill_center_r = drill_center_r_vector[j];
		vector<Point2f>Xray_points;
		Point2f Xray_point_pin;
		Point2f Xray_point_end;

		Mat1d Xray_key_point = T * add_homo(drill_center_r.rowRange(0, 3));
		Xray_point_pin.x = Xray_key_point(0, 0);
		Xray_point_pin.y = Xray_key_point(1, 0);
		Xray_points.push_back(Xray_point_pin);

		Mat1d Xray_key_point1 = T * add_homo(drill_center_r.rowRange(3, 6));
		Xray_point_end.x = Xray_key_point1(0, 0);
		Xray_point_end.y = Xray_key_point1(1, 0);
		Xray_points.push_back(Xray_point_end);

		//draw arc 画挫头  有左右之分
		//左边的
		float r = drill_center_r(0, 6) * 2;
		double angle = atan2(abs(Xray_point_pin.y - Xray_point_end.y), abs(Xray_point_pin.x - Xray_point_end.x));
		float x1 = Xray_point_pin.x - r * sin(angle);
		float y1 = Xray_point_pin.y + r * cos(angle);
		float x2 = Xray_point_pin.x + r * sin(angle);
		float y2 = Xray_point_pin.y - r * cos(angle);

		//右边的
		//float x1 = Xray_point_pin.x - r * sin(angle);
		//float y1 = Xray_point_pin.y - r * cos(angle);
		//float x2 = Xray_point_pin.x + r * sin(angle);
		//float y2 = Xray_point_pin.y + r * cos(angle);

		DrawArc(img, Xray_points[0], Point2f(x2, y2), Point2f(x1, y1), 2, Scalar(255, 0, 0));
	}
		imwrite(mainFilename+name, img);
}

//void monitor::drawEllipseWithBox(cv::RotatedRect box, cv::Scalar color, int lineThickness)
//{
//	//if (img.empty()) {
//	//	stretch(box);
//	//	img = cv::Mat::zeros(rows, cols, CV_8UC3);
//	//}
//	box.center = scale * cv::Point2f(box.center.x - origin.x, box.center.y - origin.y);
//	box.size.width = (float)(scale * box.size.width);
//	box.size.height = (float)(scale * box.size.height);
//	ellipse(img, box, color, lineThickness, LINE_AA);
//	Point2f vtx[4];
//	box.points(vtx);
//	for (int j = 0; j < 4; j++) {
//		line(img, vtx[j], vtx[(j + 1) % 4], color, lineThickness, LINE_AA);
//	}
////}
//void drawPoints(vector<Point2f> pts, cv::Scalar color)
//{
//	if (img.empty()) {
//		stretch(pts);
//		img = cv::Mat::zeros(rows, cols, CV_8UC3);
//	}
//	for (size_t i = 0; i < pts.size(); i++) {
//		Point2f pnt = scale * cv::Point2f(pts[i].x - origin.x, pts[i].y - origin.y);
//		img.at<cv::Vec3b>(int(pnt.y), int(pnt.x))[0] = (uchar)color[0];
//		img.at<cv::Vec3b>(int(pnt.y), int(pnt.x))[1] = (uchar)color[1];
//		img.at<cv::Vec3b>(int(pnt.y), int(pnt.x))[2] = (uchar)color[2];
//	};
//}