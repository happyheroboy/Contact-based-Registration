#include "constructor_hexPair.h"

constructor_hexPair::constructor_hexPair(const string& path)
{
	load_from_file(path);
	build_map();
	this->max_ID = (this->state.rows + 1) * (this->state.cols / 2 + 1);
}

constructor_hexPair::constructor_hexPair(const Mat1i& set_state)
{
	this->state = set_state;
	build_map();
	this->max_ID = (this->state.rows + 1) * (this->state.cols / 2 + 1);
}

void constructor_hexPair::load_from_file(const string path)
{
	// 打开文件
	ifstream input_file(path);
	TRY_THROW(input_file.is_open(), "could not open the file\n");
	TRY_THROW(check_suffix(path, ".marker"), "only support .marker file\n");

	// 读 marker
	int hexM, hexN, triM, triN;
	input_file >> hexM >> hexN;
	triM = hexM + 1;
	triN = 2 * (hexN + 1);
	this->state = Mat1i(triM, triN);
	for (int& i : this->state)
		input_file >> i;
}

void constructor_hexPair::draw(Mat1b& img, const int scale)
{
	int triM = this->state.rows;
	int triN = this->state.cols;
	int hexM = triM - 1;
	int hexN = triN / 2 - 1;
	float H = sqrt(3) / 2;			// 正三角形的高
	float h = 1 / (2 * sqrt(3));	// 正三角形中心到边的垂线长度
	float pad = 0.3;				// 边界的延伸长度
	float r = h / 2;				// 前景三角形边长与背景三角形边长的比

	// 创建 img
	int imgM = ceil(scale * (H * triM + 2 * pad));
	int imgN = ceil(scale * (hexN + 1.5 + 2 * pad));
	img = Mat1b(imgM, imgN, 255);

	// 绘制三角形
	vector<vector<Point>> black_poly;
	vector<Point> black_inner, white_inner;
	int shift = 12;
	int resolution = pow(2, shift);

	for (int y = -1; y <= triM; y++)
	{
		for (int x = -2; x <= triN + 1; x++)
		{
			Point center(scale * resolution * (0.5 * (x + 1) + pad), scale * resolution * (H * y + ((x + y) % 2 == 0 ? h : H - h) + pad));
			vector<Point> black_vertex, white_vertex;
			if ((x + y) % 2 == 0)
			{
				black_vertex.push_back(center + Point(-scale * resolution * 0.5, -scale * resolution * h));
				black_vertex.push_back(center + Point(scale * resolution * 0.5, -scale * resolution * h));
				black_vertex.push_back(center + Point(0, scale * resolution * (H - h)));
				black_poly.push_back(black_vertex);
				if (x >= 0 && x < triN && y >= 0 && y < triM && this->state.at<int>(y, x) == 1)
				{
					white_inner.push_back(center);
				}
			}
			else
			{
				if (x >= 0 && x < triN && y >= 0 && y < triM && this->state.at<int>(y, x) == 1)
				{
					black_inner.push_back(center);
				}
			}
		}
	}

	fillPoly(img, black_poly, 0, LINE_AA, shift);

	for (Point p : black_inner)
		circle(img, p, r * scale * resolution, 0, -1, LINE_AA, shift);

	for (Point p : white_inner)
		circle(img, p, r * scale * resolution, 255, -1, LINE_AA, shift);
}

void constructor_hexPair::save(const string& path)
{
	// 创建或打开文件
	ofstream output_file(path);
	output_file << this->state.rows - 1 << endl;
	output_file << this->state.cols / 2 - 1 << endl;
	for (int i : this->state) {
		output_file << i << " ";
	}
}

void constructor_hexPair::build_map()
{
	// 速查表
	if (!this->map_key2idx.empty())	this->map_key2idx.clear();

	int vexM = this->state.rows + 1;
	int vexN = this->state.cols / 2 + 1;

	// 横向的 delpair
	for (int m = 1; m < vexM - 1; m++)
	{
		for (int n = 1; n < vexN - 2; n++)
		{
			Point sub_end(m, n);
			Point sub_start(m, n + 1);
			Point sub_up(m - 1, m % 2 == 0 ? n : n + 1);
			Point sub_bottom = sub_up + Point(2, 0);
			Vec<Point, 10> GTsub_seq = { sub_start,
								sub_end,
				sub_up, sub_up + Point(0, 1), sub_start + Point(0, 1), sub_bottom + Point(0, 1), sub_bottom,
				sub_bottom + Point(0, -1), sub_end + Point(0, -1), sub_up + Point(0, -1) };

			Point sta_start(m - 1, 2 * n + (m % 2 == 0 ? 0 : 1));
			vector<Point> STAsub_seq = { sta_start, sta_start + Point(0, 1), sta_start + Point(0, 2),
			sta_start + Point(1, 2), sta_start + Point(1, 1), sta_start + Point(1, 0),
			sta_start + Point(1, -1), sta_start + Point(1, -2), sta_start + Point(0, -2),
			sta_start + Point(0, -1) };

			Vec10i sta_seq;
			for (int i = 0; i < STAsub_seq.size(); i++)
			{
				sta_seq[i] = (this->state.at<int>(STAsub_seq[i].x, STAsub_seq[i].y));
			}
			int key = 0;
			Vec10i idx;
			for (int i = 0; i < 10; i++)
			{
				key += (bool)(sta_seq[i]) * pow(2, 9 - i);
				idx[i] = GTsub_seq[i].x * vexN + GTsub_seq[i].y;
			}

			this->map_key2idx.insert(make_pair(key, idx));
		}
	}

	// 斜向右下的 delpair
	for (int m = 1; m < vexM - 2; m++)
	{
		for (int n = 1; n < vexN - (m % 2 == 0 ? 1 : 2); n++)
		{
			Point sub_start(m, n);
			Point sub_end(m + 1, m % 2 == 0 ? n : n + 1);
			Point sub_up(m - 1, m % 2 == 0 ? n - 1 : n);
			Point sub_bottom(m + 2, n + 1);
			Vec<Point, 10> GTsub_seq = { sub_start, sub_end, sub_end + Point(0, -1), sub_start + Point(0, -1),
				sub_up, sub_up + Point(0, 1), sub_start + Point(0, 1), sub_end + Point(0, 1),
				sub_bottom, sub_bottom + Point(0, -1) };

			Point sta_start(m, 2 * n - (m % 2 == 0 ? 1 : 0));
			vector<Point> STAsub_seq = { sta_start, sta_start + Point(0, -1), sta_start + Point(-1, -1),
				sta_start + Point(-1, 0), sta_start + Point(-1, 1), sta_start + Point(0, 1),
				sta_start + Point(0, 2), sta_start + Point(1, 2), sta_start + Point(1, 1),
				sta_start + Point(1, 0) };

			Vec10i sta_seq;
			for (int i = 0; i < STAsub_seq.size(); i++)
			{
				sta_seq[i] = (this->state.at<int>(STAsub_seq[i].x, STAsub_seq[i].y));
			}
			int key = 0;
			Vec10i idx;
			for (int i = 0; i < 10; i++)
			{
				key += (bool)(sta_seq[i]) * pow(2, 9 - i);
				idx[i] = GTsub_seq[i].x * vexN + GTsub_seq[i].y;
			}

			this->map_key2idx.insert(make_pair(key, idx));
		}
	}

	// 斜向左下的 delpair
	for (int m = 1; m < vexM - 2; m++)
	{
		for (int n = (m % 2 == 0 ? 2 : 1); n < vexN - 1; n++)
		{
			Point sub_end(m, n);
			Point sub_start(m + 1, m % 2 == 0 ? n - 1 : n);
			Point sub_up(m - 1, m % 2 == 0 ? n : n + 1);
			Point sub_bottom(m + 2, n - 1);
			Vec<Point, 10> GTsub_seq = { sub_start, sub_end, sub_start + Point(0, 1), sub_bottom + Point(0, 1),
				sub_bottom, sub_start + Point(0, -1), sub_end + Point(0, -1),
				sub_up + Point(0, -1), sub_up, sub_end + Point(0, 1) };

			Point sta_start(m, 2 * n - (m % 2 == 0 ? 1 : 0));
			vector<Point> STAsub_seq = { sta_start, sta_start + Point(1, 0), sta_start + Point(1, -1),
				sta_start + Point(1, -2), sta_start + Point(0, -2), sta_start + Point(0, -1),
				sta_start + Point(-1, -1), sta_start + Point(-1, 0), sta_start + Point(-1, 1),
				sta_start + Point(0, 1) };

			Vec10i sta_seq;
			for (int i = 0; i < STAsub_seq.size(); i++)
			{
				sta_seq[i] = (this->state.at<int>(STAsub_seq[i].x, STAsub_seq[i].y));
			}
			int key = 0;
			Vec10i idx;
			for (int i = 0; i < 10; i++)
			{
				key += (bool)(sta_seq[i]) * pow(2, 9 - i);
				idx[i] = GTsub_seq[i].x * vexN + GTsub_seq[i].y;
			}

			this->map_key2idx.insert(make_pair(key, idx));
		}
	}

	// 构造edge2idx查询表
	for (auto it : this->map_key2idx)
	{
		this->map_edg2idx.insert(make_pair(Vec2i(it.second[0],it.second[1]),
		Vec8i(it.second[2], it.second[3], it.second[4], it.second[5], it.second[6], it.second[7], it.second[8], it.second[9])));
	}
}

void constructor_hexPair::apply(Data::constructor& data)
{
	// 标记执行
	data.reset_output();

	// 检查与预备
	if (data.in_ptList.size() < 10 || data.in_img.empty())		return;
	this->parallel = data.p_parallel;
	
	// 算法计时
	TIME_START(time_all);

	// 构造各种查询表
	initial(data);

	// 构造hexPair
	construct_hexInfo(data.d_hexInfo);

	get_IDList(data.in_ptList, data.d_hexInfo, data.d_coreIDList, data.d_IDList);

	// fList
	for (int i = 0; i < data.d_IDList.size(); i++)
		if (data.d_IDList[i] != -1)
			data.o_fList.push_back(Vec3f(data.d_IDList[i], data.in_ptList[i].x, data.in_ptList[i].y));

	TIME_END(time_all);

	// 记录调试用输出
	data.d_time_cost.insert(data.d_time_cost.begin(), Data::time_cost("constructor", time_all));
	data.map_key2idx = this->map_key2idx;
}

void constructor_hexPair::initial(Data::constructor& data)
{
	// 三角剖分
	Subdiv2D subdiv(Rect(0, 0, data.in_img.cols, data.in_img.rows));
	subdiv.insert(data.in_ptList);

	// 构造
	build_pt_XYtoIdx(data.in_ptList, this->pt_XYtoIdx);
	build_triList(subdiv, this->pt_XYtoIdx, this->triList);
	build_linkList(subdiv, data.in_ptList.size(), this->linkList);
	build_tri_IdxtoState(data.in_img, this->triList, data.in_ptList, data.p_distort_T, data.p_gray_T, this->tri_IdxtoState);
	build_hexEdgeList(this->linkList, this->hexEdgeList);

	// 记录
	data.d_triList = this->triList;
	data.d_tri_IdxtoState = this->tri_IdxtoState;
}

inline int constructor_hexPair::getPointID(map<Point2f, int, ComparePoint2f>& pt_XYtoIdx, const Point2f& pt)
{
	map<Point2f, int, ComparePoint2f>::iterator it = pt_XYtoIdx.find(pt);
	if (it == pt_XYtoIdx.end())
		return -1;
	return it->second;
}
inline int constructor_hexPair::getTriState(const Vec3i tri, map<Vec3i, int, CompareVec3i>& tri_IdxtoState)
{
	Vec3i tri_sort;
	cv::sort(tri, tri_sort, SORT_EVERY_COLUMN + SORT_ASCENDING);
	map<Vec3i, int, CompareVec3i>::iterator it = tri_IdxtoState.find(tri_sort);
	if (it == tri_IdxtoState.end())
		return -1;
	return it->second;
}
inline float constructor_hexPair::getLength(const Point2f& pt0, const Point2f& pt1)
{
	float x = pt0.x - pt1.x;
	float y = pt0.y - pt1.y;
	return sqrt(pow(x, 2) + pow(y, 2));
}
inline bool constructor_hexPair::checkTriLens(const Vec3f& triLens, const float distort_T)
{
	float min, max;
	min = triLens[0] < triLens[1] ? triLens[0] : triLens[1];
	min = min < triLens[2] ? min : triLens[2];
	max = triLens[0] > triLens[1] ? triLens[0] : triLens[1];
	max = max > triLens[2] ? max : triLens[2];
	if (min == 0 || (max / min) > distort_T)	return false;
	else return true;
}
inline void constructor_hexPair::round_IdxtoState(const int A, const int B, const Vec8i& round_Idx, map<Vec3i, int, CompareVec3i>& tri_IdxtoState, Vec10i& state)
{
	state = Vec10i(
		getTriState(Vec3i(round_Idx[0], A, B), tri_IdxtoState),
		getTriState(Vec3i(round_Idx[0], round_Idx[1], A), tri_IdxtoState),
		getTriState(Vec3i(round_Idx[1], round_Idx[2], A), tri_IdxtoState),
		getTriState(Vec3i(round_Idx[2], round_Idx[3], A), tri_IdxtoState),
		getTriState(Vec3i(round_Idx[3], round_Idx[4], A), tri_IdxtoState),
		getTriState(Vec3i(round_Idx[4], A, B), tri_IdxtoState),
		getTriState(Vec3i(round_Idx[4], round_Idx[5], B), tri_IdxtoState),
		getTriState(Vec3i(round_Idx[5], round_Idx[6], B), tri_IdxtoState),
		getTriState(Vec3i(round_Idx[6], round_Idx[7], B), tri_IdxtoState),
		getTriState(Vec3i(round_Idx[7], round_Idx[0], B), tri_IdxtoState));
}
inline int constructor_hexPair::round_StatetoKey(const Vec10i& state)
{
	int key = 0;
	for (int i = 0; i < 10; i++)
	{
		key += (state[i] > 1) * pow(2, 9 - i);
	}
	return key;
}
inline Vec3i constructor_hexPair::findJoint(const vector<int>& A, const vector<int>& B)
{
	for (int i = 0; i < A.size(); i++)
	{
		for (int j = 0; j < B.size(); j++)
		{
			if (A[i] == B[j])
			{
				return Vec3i(i, j, A[i]);
			}
		}
	}
	return Vec3i(-1, -1, -1);
}

void constructor_hexPair::build_pt_XYtoIdx(const vector<Point2f> ptList, map<Point2f, int, ComparePoint2f>& pt_XYtoIdx)
{
	this->pt_XYtoIdx.clear();
	for (int i = 0; i < ptList.size(); i++)
		pt_XYtoIdx.insert(pair<Point2f, int>(ptList[i], i));
}
void constructor_hexPair::build_triList(const Subdiv2D& subdiv, map<Point2f, int, ComparePoint2f>& pt_XYtoIdx, vector<Vec3i>& triList)
{
	this->triList.clear();
	vector<Vec6f> triXYList;
	subdiv.getTriangleList(triXYList);
	for (Vec6f v : triXYList)
	{
		// 构建TriIdx
		vector<int> point_ID;
		int ID_0 = getPointID(pt_XYtoIdx, Point2f(v[0], v[1]));
		int ID_1 = getPointID(pt_XYtoIdx, Point2f(v[2], v[3]));
		int ID_2 = getPointID(pt_XYtoIdx, Point2f(v[4], v[5]));
		if (ID_0 < 0 || ID_1 < 0 || ID_2 < 0)
			continue;

		point_ID.push_back(ID_0);
		point_ID.push_back(ID_1);
		point_ID.push_back(ID_2);

		sort(point_ID.begin(), point_ID.end());

		// 加入triList
		triList.push_back(Vec3i(point_ID[0], point_ID[1], point_ID[2]));
	}
}
void constructor_hexPair::build_linkList(const Subdiv2D& subdiv, const int ptNum, vector<vector<int>>& linkList)
{
	this->linkList.clear();

	// 初始化linkList
	for (int i = 0; i < ptNum; i++){
		this->linkList.push_back(vector<int>());
	}

	// 填充linkList
	for (int pt_ID = 0; pt_ID < ptNum; pt_ID++)
	{
		// 在opencv的subdiv中，由于多了四个边界点，点的索引从头开始多4个
		int pt_subdivID = pt_ID + 4;

		// 找到与当前点相连的一条边，将另一头的端点存入linkList
		int firstEdgeID;
		
		subdiv.getVertex(pt_subdivID, &firstEdgeID);
		int dst_subdivID = subdiv.edgeDst(firstEdgeID);
		if (dst_subdivID > 3)
			linkList[pt_ID].push_back(dst_subdivID - 4);

		// 从firstEdge开始，顺时针依次找出与当前点相连的边，将另一头的端点存入linkList
		int edgeID = firstEdgeID;
		do
		{
			edgeID = subdiv.nextEdge(edgeID);
			if (firstEdgeID == edgeID)		// 如果已遍历所有边，跳出循环
				break;
			dst_subdivID = subdiv.edgeDst(edgeID);
			if (dst_subdivID > 3)
				linkList[pt_ID].push_back(dst_subdivID - 4);
		} while (firstEdgeID != edgeID);	// 若未遍历所有边，则继续
	}
}
void constructor_hexPair::build_tri_IdxtoState(const Mat1b& img, const vector<Vec3i>& triList, const vector<Point2f>& ptList, const float distort_T, const uchar grayT, map<Vec3i, int, CompareVec3i>& tri_IdxtoState)
{
	this->tri_IdxtoState.clear();
	for (int i = 0; i < triList.size(); i++)
	{
		Vec3i tri = triList[i];

		// 三个角点
		Point2f v0 = ptList[tri[0]];
		Point2f v1 = ptList[tri[1]];
		Point2f v2 = ptList[tri[2]];

		// 检查三角形的形状
		Vec3f triLens;
		triLens[0] = getLength(v0, v1);
		triLens[1] = getLength(v0, v2);
		triLens[2] = getLength(v1, v2);
 
		// state: 0-变形过大
		if (!checkTriLens(triLens, distort_T)){
			tri_IdxtoState.insert(pair<Vec3i, int>(tri, 0));
			continue;
		}

		// 找到3个偏内侧的交点v0/v1/v2，和中心点c
		Point2f c = (v0 + v1 + v2) / 3;
		v0 = 0.625f * v0 + 0.375f * c;
		v1 = 0.625f * v1 + 0.375f * c;
		v2 = 0.625f * v2 + 0.375f * c;

		// 读4个关键点的灰度
		float v_c = (float)img(round(c.y), round(c.x));
		float v_v0 = (float)img(round(v0.y), round(v0.x));
		float v_v1 = (float)img(round(v1.y), round(v1.x));
		float v_v2 = (float)img(round(v2.y), round(v2.x));
		float back_v = (v_v0 + v_v1 + v_v2) / 3;

		// state: 1-纯色（黑白未定），2-黑底白点，3-白底黑点
		if (v_c - back_v > grayT)
			tri_IdxtoState.insert(pair<Vec3i, int>(tri, 2));
		else if (back_v - v_c > grayT)
			tri_IdxtoState.insert(pair<Vec3i, int>(tri, 3));
		else
			tri_IdxtoState.insert(pair<Vec3i, int>(tri, 1));
	}
}
void constructor_hexPair::build_hexEdgeList(const vector<vector<int>>& linkList, vector<Vec2i>& hexEdgeList)
{
	this->hexEdgeList.clear();
	for (int start_ID = 0; start_ID < linkList.size(); start_ID++)
	{
		if (linkList[start_ID].size() == 6)
		{
			for (int i = 0; i < 6; i++)
			{
				int end_ID = linkList[start_ID][i];
				if (end_ID > start_ID && linkList[end_ID].size() == 6)
				{
					hexEdgeList.push_back(Vec2i{ start_ID, end_ID });
				}
			}
		}
	}
}

void constructor_hexPair::construct_hexInfo(vector<Vec11i>& hexInfo)
{
	hexInfo.clear();
	if (this->hexEdgeList.empty())	return;

	for (const Vec2i& v : this->hexEdgeList)
	{
		// 找到两个圈的点索引序列
		int A = v[0], B = v[1];
		vector<int> round_A = this->linkList[A];
		vector<int> round_B = this->linkList[B];

		// 找到其中一个接壤点
		Vec3i jointInfo = findJoint(round_A, round_B);
		if (jointInfo[0] == -1)	continue;

		int jointA = jointInfo[0];
		int jointB = jointInfo[1];
		int jointIdx = jointInfo[2];
		int jointState = getTriState(Vec3i(A, B, jointIdx), this->tri_IdxtoState);

		// state可能为0（三角形变形过大）或-1（找不到这个三角形）
		if (jointState <= 0)
		{
			//hexInfo.push_back(Vec11i{ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 });
			continue;
		}

		// 找到两个环的顺时针起点
		int start_A, start_B;
		if (round_A[(jointA + 2) % 6] == round_B[(jointB + 4) % 6])
		{
			start_A = (jointA + 2) % 6;
			start_B = jointB;
		}
		else
		{
			start_A = jointA;
			start_B = (jointB + 2) % 6;
		}

		// 找到大环的一圈点在ptList中的索引，起点为两个共同点之一，读state
		Vec8i round_Idx = Vec8i(round_A[start_A],
			round_A[(start_A + 1) % 6],
			round_A[(start_A + 2) % 6],
			round_A[(start_A + 3) % 6],
			round_B[start_B],
			round_B[(start_B + 1) % 6],
			round_B[(start_B + 2) % 6],
			round_B[(start_B + 3) % 6]);
		Vec10i round_state;
		round_IdxtoState(A, B, round_Idx, this->tri_IdxtoState, round_state);

		/*flag == -1 , 包含找不到或变形过大的三角形
				   0 ，正常
				   1 ，奇偶的背景色不对，需要调换起点终点*/
		int flag = 0;

		// 不能包含无法读取状态的，或变形过大的三角形
		for (int i = 0; i < 10; i++)
			if (round_state[i] <= 0)
				flag = -1;
		if (flag == -1)
		{
			//hexInfo.push_back(Vec11i{ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 });
			continue;
		}

		for (int i = 0; i < 10; i++)
		{
			if (i % 2 && round_state[i] == 3)
			{
				flag = 1;
				break;
			}
			if ((i + 1) % 2 && round_state[i] == 2)
			{
				flag = 1;
				break;
			}
		}
		if (flag == 0)	hexInfo.push_back(Vec11i{ A, B, round_Idx[0], round_Idx[1], round_Idx[2], round_Idx[3],
												  round_Idx[4], round_Idx[5], round_Idx[6], round_Idx[7], round_StatetoKey(round_state) });
		else
		{
			round_Idx = Vec8i(round_Idx[4], round_Idx[5], round_Idx[6], round_Idx[7],
				round_Idx[0], round_Idx[1], round_Idx[2], round_Idx[3]);
			round_IdxtoState(B, A, round_Idx, this->tri_IdxtoState, round_state);
			hexInfo.push_back(Vec11i{ B, A, round_Idx[0], round_Idx[1], round_Idx[2], round_Idx[3],
												  round_Idx[4], round_Idx[5], round_Idx[6], round_Idx[7], round_StatetoKey(round_state) });
		}
	}
}
void constructor_hexPair::get_IDList(const vector<Point2f>& ptList, vector<Vec11i>& hexInfo, vector<int>& coreIDList, vector<int>& IDList)
{
	IDList.clear();
	coreIDList.clear();
	if (hexInfo.empty())	return;

	struct IDSta
	{
		vector<Vec2i> IDandNum;
		int maxNum = 0;
	};

	vector<IDSta> VecID(ptList.size(),IDSta());
	vector<IDSta> SurID(ptList.size(), IDSta());

	int vexM = this->state.rows + 1;
	int vexN = this->state.cols / 2 + 1;
	int vexNum = vexM * vexN;

	// 统计VecID与SurID, SurID包含VecID
	for (int i = 0; i < hexInfo.size(); i++)
	{
		int key = hexInfo[i][10];
		map<int, Vec10i>::iterator it = this->map_key2idx.find(key);
		if (it == map_key2idx.end())	continue;
		Vec10i sub = it->second;
		
		for (int a = 0; a < 2; a++)
		{
			bool has_same = false;
			for (Vec2i& v : VecID[hexInfo[i][a]].IDandNum)
			{
				if (sub[a] == v[0])
				{
					v[1]++;
					if (VecID[hexInfo[i][a]].maxNum < v[1])	VecID[hexInfo[i][a]].maxNum = v[1];
					has_same = true;
					break;
				}
			}
			if (!has_same)
			{
				VecID[hexInfo[i][a]].IDandNum.push_back(Vec2i(sub[a], 1));
				if (VecID[hexInfo[i][a]].maxNum < 1)	VecID[hexInfo[i][a]].maxNum = 1;
			}
		}
		
		for (int a = 0; a < 10; a++)
		{
			bool has_same = false;
			for (Vec2i& v : SurID[hexInfo[i][a]].IDandNum)
			{
				if (sub[a] == v[0])
				{
					v[1]++;
					if (SurID[hexInfo[i][a]].maxNum < v[1])	SurID[hexInfo[i][a]].maxNum = v[1];
					has_same = true;
					break;
				}
			}
			if (!has_same)
			{
				SurID[hexInfo[i][a]].IDandNum.push_back(Vec2i(sub[a], 1));
				if (SurID[hexInfo[i][a]].maxNum < 1)	SurID[hexInfo[i][a]].maxNum = 1;
			}
		}
	}

	// 过滤SurID，只保存众数
	for (IDSta& sta : SurID)
	{
		vector<Vec2i>::iterator iter_v = sta.IDandNum.begin();
		for (;iter_v!=sta.IDandNum.end();)
		{
			if ((*iter_v)[1]<sta.maxNum)
			{
				iter_v = sta.IDandNum.erase(iter_v);
				continue;
			}
			iter_v++;
		}
	}

	// 过滤VecID，只保留SurID众数支持的ID
	for (int i = 0; i < VecID.size(); i++)
	{
		vector<Vec2i>::iterator iter_v = VecID[i].IDandNum.begin();
		for (; iter_v != VecID[i].IDandNum.end();)
		{
			bool is_in = false;
			for (Vec2i v : SurID[i].IDandNum)
			{
				if ((*iter_v)[0] == v[0])
				{
					is_in = true;
					break;
				}
			}
			if (!is_in)
			{
				iter_v = VecID[i].IDandNum.erase(iter_v);
				continue;
			}
			iter_v++;
		}
	}

	// 过滤VecID，仅保留有唯一ID的成员到settle_ID
	for (IDSta& sta : VecID)
	{
		if (sta.IDandNum.size() != 1)
			coreIDList.push_back(-1);
		else
			coreIDList.push_back(sta.IDandNum[0][0]);
	}

	// 扩散settle_ID，仅保留无争议点
	vector<vector<int>> spread_ID(ptList.size(), vector<int>());
	for (int i = 0; i < hexInfo.size(); i++)
	{
		int start = hexInfo[i][0];
		int end = hexInfo[i][1];
		if (coreIDList[start] != -1 && coreIDList[end] != -1)
		{
			Vec2i key = Vec2i(coreIDList[start], coreIDList[end]);
			map<Vec2i, Vec8i>::iterator it = this->map_edg2idx.find(key);
			if (it == map_edg2idx.end())	continue;
			Vec8i sub = it->second;
			for (int a = 0; a < 8; a++)
				if (coreIDList[hexInfo[i][a + 2]] == -1)
					spread_ID[hexInfo[i][a + 2]].push_back(sub[a]);
		}
	}

	// 移除spread_ID中的重复点
	for (vector<int>& v : spread_ID)
	{
		std::sort(v.begin(), v.end());
		v.erase(unique(v.begin(), v.end()), v.end());
	}
	
	// 移除spread_ID中的争议点，构造IDList
	IDList = vector<int>(ptList.size(), -1);
	for (int i = 0; i < coreIDList.size(); i++)
	{
		if (coreIDList[i] != -1)	IDList[i] = coreIDList[i];
		else if (spread_ID[i].size() == 1)	IDList[i] = spread_ID[i][0];
	}
}