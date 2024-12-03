#include "config.h"

bool check_suffix(const string path, const string suffix)
{
	int idx = path.find(suffix, path.size() - suffix.size());
	return idx != string::npos;
}

vector<Vec3d> remove_homo(const vector<Vec4d>& homo)
{
	vector<Vec3d> inhomo;
	for (Vec4d vec : homo)
		inhomo.push_back(Vec3d(vec[0], vec[1], vec[2]) / vec[3]);
	return inhomo;
}

vector<Vec2d> remove_homo(const vector<Vec3d>& homo)
{
	vector<Vec2d> inhomo;
	for (Vec3d vec : homo)
		inhomo.push_back(Vec2d(vec[0], vec[1]) / vec[3]);
	return inhomo;
}

Mat remove_homo(const Mat& homo)
{
	Mat H, inhomo;
	repeat(homo.row(homo.rows-1), homo.rows-1, 1, H);
	divide(homo.rowRange(0, homo.rows - 1), H, inhomo);
	return inhomo;
}

vector<Vec4d> add_homo(const vector<Vec3d>& inhomo)
{
	vector<Vec4d> homo;
	for (Vec3d vec : inhomo)
		homo.push_back(Vec4d(vec[0], vec[1], vec[2], 1));
	return homo;
}

vector<Vec3d> add_homo(const vector<Vec2d>& inhomo)
{
	vector<Vec3d> homo;
	for (Vec2d vec : inhomo)
		homo.push_back(Vec3d(vec[0], vec[1], 1));
	return homo;
}

Mat add_homo(const Mat& inhomo)
{
	Mat homo;
	vconcat(inhomo, Mat1d::ones(1, inhomo.cols), homo);
	return homo;
}