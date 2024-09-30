#include "detector_cross.h"

inline bool cmp_score(const ptList_score& a, const ptList_score& b)
{
	return a.score < b.score;
}

void detector_cross::apply(Data::detector& data)
{
	// ���ִ��
	data.reset_output();

	// �����Ԥ��
	if (data.in_img.empty())		return;

	this->parallel = data.p_parallel;
	this->mask_R = data.p_mask_R;
	this->mask_r = data.p_mask_r;
	if (this->A_tri.empty() || this->pA_tri.empty())	createA();

	// �㷨��ʱ
	TIME_START(time_all);

	// Ԥ��⣬ʹ��FAST���������������ؼ��ķ���ƽ�ơ��ϲ�
	TIME_START(time_pre);
	preFilter(data.in_img, data.p_FAST_T, data.p_subPix, data.d_ptPre);
	TIME_END(time_pre);

	// �б��������ض�λ�����ںﰰ��
	TIME_START(time_ref);
	data.o_ptList = data.d_ptPre;
	refine_tri(data.in_img, data.o_ptList);
	TIME_END(time_ref);

	TIME_END(time_all);
	
	// ��¼���������
	data.d_time_cost.push_back(Data::time_cost("pre", time_pre));
	data.d_time_cost.push_back(Data::time_cost("refine", time_ref));
	data.d_time_cost.insert(data.d_time_cost.begin(), Data::time_cost("detector", time_all));
}

void detector_cross::meshgrid(const Range xgv, const Range ygv, Mat1f& X, Mat1f& Y)
{
	vector<float> t_x, t_y;
	for (int i = xgv.start; i <= xgv.end; i++)
		t_x.push_back(i);
	for (int i = ygv.start; i <= ygv.end; i++)
		t_y.push_back(i);
	repeat(Mat(t_x).reshape(1, 1), Mat(t_y).total(), 1, X);
	repeat(Mat(t_y).reshape(1, 1).t(), 1, Mat(t_x).total(), Y);
}

void detector_cross::createA()
{
	Range range = Range(-this->mask_r, this->mask_r);

	//����meshgrid: mx,my
	Mat1f mx, my;
	meshgrid(range, range, mx, my);

	int S = this->mask_R * this->mask_R;
	Mat1f x = mx.reshape(1, S);    //������
	Mat1f y = my.reshape(1, S);

	// f(x,y) = c0 * x^3 + c1 * x^2y + c2 * xy^2 + c3 * y^3 + 
	//	        c4 * x^2 + c5 * xy + c6 * y^2 + c7 * x + c8 * y + c9
	Mat1f X = Mat1f::ones(S, 10);
	X.col(0) = (x.mul(x)).mul(x);	// A
	X.col(1) = (x.mul(x)).mul(y);	// B
	X.col(2) = (x.mul(y)).mul(y);	// C
	X.col(3) = (y.mul(y)).mul(y);	// D
	X.col(4) = x.mul(x);
	X.col(5) = x.mul(y);
	X.col(6) = y.mul(y);
	x.copyTo(X.col(7));
	y.copyTo(X.col(8));

	X.copyTo(this->A_tri);
	invert(X, this->pA_tri, DECOMP_SVD);
}

void detector_cross::preFilter(const Mat1b& img, const int FAST_T, const TermCriteria subPix, vector<Point2f>& ptList)
{
	ptList.clear();
	if (img.empty())	return;

	// FAST
	vector<KeyPoint> keyList;
	FAST(img, keyList, FAST_T, true, FastFeatureDetector::TYPE_9_16);
	KeyPoint::convert(keyList, ptList);

	// ʹ��opencv�Ľ���������ط�����FAST��⵽������λ���򽻲�㴦�ۼ�
	if (this->parallel)
		parallel_for_(Range(0, ptList.size()), p_cornerSubPix(this, img, subPix, ptList));
	else					
		cornerSubPix(img, ptList, Size(this->mask_R, this->mask_R), Size(-1, -1), subPix);

	// ʹ��kdTreeѰ�Ҿۼ��ĵ㣬������ɾ������
	vector<bool> remove;
	merge_kdTree(ptList, img.size(), this->mask_r, remove);

	// ɾ��kdTree����ĵ㣬�Լ�������ӽ�ͼ���Ե�ĵ�
	vector<Point2f>::iterator pt_iter = ptList.begin();
	vector<bool>::iterator r_iter = remove.begin();
	for (; pt_iter != ptList.end();) {
		Point2f p = (*pt_iter);
		if ((*r_iter) || p.x < this->mask_r || p.x > img.cols - this->mask_r - 1 || p.y < this->mask_r || p.y > img.rows - this->mask_r - 1)
		{
			pt_iter = ptList.erase(pt_iter);
			++r_iter;
			continue;
		}
		++pt_iter;
		++r_iter;
	}
}

void detector_cross::merge_kdTree(const vector<Point2f>& ptList, const Size pt_scope, const float r, vector<bool>& remove)
{
	remove.clear();
	if (ptList.empty())	return;

	vector<PointForTree> points(ptList.size());
	for (int i = 0; i < ptList.size(); i++) {
		points[i] = PointForTree(ptList[i].x, ptList[i].y);
		remove.push_back(false);
	}
	kdt::KDTree<PointForTree> kdtree(points);

	//// ��Բ�η�Χ��Ѱ�ҽ��ڵ�
	//for (int i = 0; i < ptList.size(); i++) {
	//	if (!remove[i]) {
	//		vector<int> neighbor = kdtree.radiusSearch(points[i], r);
	//		for (int j = 0; j < neighbor.size(); j++)
	//			remove[neighbor[j]] = true;
	//		remove[i] = false;
	//	}
	//}

	vector<ptList_score> ptScoreList;
	for (int i = 0; i < ptList.size(); i++) {
		Point2f pCenter = ptList[i];
		vector<int> neighbor = kdtree.radiusSearch(points[i], r);
		for (int j = 0; j < neighbor.size(); j++)
			pCenter += ptList[neighbor[j]];
		pCenter /= (float)(1 + neighbor.size());
		float dis = abs(pCenter.x - ptList[i].x) + abs(pCenter.y - ptList[i].y);
		ptList_score ptScore;
		ptScore.score = dis;
		ptScore.p = ptList[i];
		ptScoreList.push_back(ptScore);
	}

	//���㰴�����÷�����
	stable_sort(ptScoreList.begin(), ptScoreList.end(), cmp_score);

	// ��Բ�η�Χ��Ѱ�ҽ��ڵ�
	for (int i = 0; i < ptScoreList.size(); i++) {
		if (!remove[i]) {
			vector<int> neighbor = kdtree.radiusSearch(points[i], r);
			for (int j = 0; j < neighbor.size(); j++)
				remove[neighbor[j]] = true;
			remove[i] = false;
		}
	}
}

void detector_cross::refine_tri(const Mat1b& img, vector<Point2f>& ptList)
{
	if (ptList.empty()) return;
	
	if (this->parallel)	parallel_for_(Range(0, ptList.size()), p_refine_tri(this, img, ptList));
	else				for (Point2f& p : ptList)	refine_tri_single(img, p);

	// �Ƴ��б�Ϊ�ﰰ�������
	vector<Point2f>::iterator pt_iter = ptList.begin();
	for (; pt_iter != ptList.end();){
		if ((*pt_iter).x == -1){
			pt_iter = ptList.erase(pt_iter);
			continue;
		}
		++pt_iter;
	}
}

void detector_cross::refine_tri_single(const Mat1b& img, Point2f& p)
{
	// ��������ӽ�ͼ���Ե�ĵ���Ϊ1
	if (p.x < this->mask_r || p.x > img.cols - this->mask_r - 1 || p.y < this->mask_r || p.y >img.rows - this->mask_r - 1){
		p = Point2f(-1, -1);
		return;
	}

	// ����ʼλ���ܱ߲�ֵ
	Mat1f subpix_buf(this->mask_R, this->mask_R);
	getRectSubPix(img, Size(this->mask_R, this->mask_R), p, subpix_buf, subpix_buf.type());
	Mat1f Y = subpix_buf.reshape(1, this->mask_R * this->mask_R);

	// �������ϵ����f(x,y) = c0 * x^3 + c1 * x^2y + c2 * xy^2 + c3 * y^3 + c4 * x^2 + c5 * xy + c6 * y^2 + c7 * x + c8 * y + c9
	Mat1f c = this->pA_tri * Y;
	
	// �ﰰ���б�
	float est_J = 3 * (c(0) * c(2) + c(1) * c(3)) - (c(1) * c(1) + c(2) * c(2));
	if (est_J >= -5) {
		p = Point2f(-1, -1);
		return;
	}

	// ����ﰰ�����ģ��ƶ���p
	Mat1f D = (Mat1f(3, 2) << 6 * c(0), 2 * c(1), 2 * c(1), 2 * c(2), 2 * c(2), 6 * c(3));
	Mat1f b = (Mat1f(3, 1) << -2 * c(4), -c(5), -2 * c(6));
	invert(D, D, DECOMP_SVD);
	Mat1f shift = D * b;
	p = p + Point2f(shift);

	// ִ�����ϲ���5�Σ���ֱ��ĳ���ƶ�����仯����0.03
	for (int cyc = 0; cyc < 5; cyc++)
	{
		// ��������ӽ�ͼ���Ե�ĵ���Ϊ1
		if (p.x < this->mask_r || p.x > img.cols - this->mask_r - 1 || p.y < this->mask_r || p.y >img.rows - this->mask_r - 1) {
			p = Point2f(-1, -1);
			return;
		}
		getRectSubPix(img, Size(this->mask_R, this->mask_R), p, subpix_buf, subpix_buf.type());
		Y = subpix_buf.reshape(1, this->mask_R * this->mask_R);
		c = this->pA_tri * Y;
		D = (Mat1f(3, 2) << 6 * c(0), 2 * c(1), 2 * c(1), 2 * c(2), 2 * c(2), 6 * c(3));
		Mat1f b = (Mat1f(3, 1) << -2 * c(4), -c(5), -2 * c(6));
		invert(D, D, DECOMP_SVD);
		shift = D * b;
		p = p + Point2f(shift);
		if (abs(shift(0)) < 0.03 && abs(shift(1)) < 0.03)	break;
	}

	// ��������ӽ�ͼ���Ե�ĵ���Ϊ1
	if (p.x < this->mask_r || p.x > img.cols - this->mask_r - 1 || p.y < this->mask_r || p.y >img.rows - this->mask_r - 1) {
		p = Point2f(-1, -1);
		return;
	}

	// �ﰰ���б�
	est_J = 3 * (c(0) * c(2) + c(1) * c(3)) - (c(1) * c(1) + c(2) * c(2));
	/*Scalar Ymean, Ydev;
	meanStdDev(Y, Ymean, Ydev);*/
	//-0.0001 * pow(Ydev(0), 2)
	Mat1f powY;
	cv::pow(Y - mean(Y), 2, powY);
	float J_T = -0.015 * mean(powY)(0);	
	if (est_J >= J_T) {
		p = Point2f(-1, -1);
		return;
	}
}
