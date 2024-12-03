#pragma once
#include "config.h"
#include "Data.h"

class constructor_hexPair
{
public:

	constructor_hexPair(const string& path);			// 构造函数，读取“.marker”文件
	constructor_hexPair(const Mat1i& set_state);		// 构造函数，手动填写 marker 的状态矩阵

	// 从ptList中提取hexInfo
	void apply(Data::constructor& data);

	// 绘制当前 marker, scale 为绘制三角形边长的像素数
	void draw(Mat1b& img, const int scale = 100);

	// 保存当前 marker
	void save(const string& path);

private:
	
	// 全局参数	
	bool parallel;
	Mat1i state;					// 状态矩阵
	int max_ID;						// 状态矩阵中ID的最大索引
	map<int, Vec10i> map_key2idx;	// key-vertexIdx 查询表
	map<Vec2i, Vec8i, CompareVec2i> map_edg2idx;	// edge-vertexIdx 查询表

	// 相关查询表
	vector<Vec3i>	triList;		// 三角形列表，节点编号升序排列
	vector<vector<int>> linkList;	// 点与点相连关系的列表
	vector<Vec2i>	hexEdgeList;	// 六边形对的中心边缘列表，以边缘两个端点在ptList中的索引表达
	map<Point2f, int, ComparePoint2f> pt_XYtoIdx;		// 表，点坐标->点索引
	map<Vec3i, int, CompareVec3i> tri_IdxtoState;		// 表，三角形顶点编号->三角形状态，state: 0-变形，1-纯色（黑白未知），2-黑底白点，3-白底黑点

	// 初始化，构造大部分查询表
	void initial(Data::constructor& data);
	void load_from_file(const string path);
	void build_map();

	// 工具
	inline int getPointID(map<Point2f, int, ComparePoint2f>& pt_XYtoIdx, const Point2f& pt);
	inline int getTriState(const Vec3i tri, map<Vec3i, int, CompareVec3i>& tri_IdxtoState);
	inline float getLength(const Point2f& p0, const Point2f& p1);
	inline bool checkTriLens(const Vec3f& triLens, const float distort_T);
	inline void round_IdxtoState(const int A, const int B, const Vec8i& round_Idx, map<Vec3i, int, CompareVec3i>& tri_IdxtoState, Vec10i& state);
	inline int round_StatetoKey(const Vec10i& state);
	inline Vec3i findJoint(const vector<int>& A, const vector<int>& B);

	// 各种表格的构造
	void build_pt_XYtoIdx(const vector<Point2f> ptList, map<Point2f, int, ComparePoint2f>& pt_XYtoIdx);
	void build_triList(const Subdiv2D& subdiv, map<Point2f, int, ComparePoint2f>& pt_XYtoIdx, vector<Vec3i>& triList);
	void build_linkList(const Subdiv2D& subdiv, const int ptNum, vector<vector<int>>& linkList);
	void build_tri_IdxtoState(const Mat1b& img, const vector<Vec3i>& triList, const vector<Point2f>& ptList, const float distort_T, const uchar grayT, map<Vec3i, int, CompareVec3i>& tri_IdxtoState);
	void build_hexEdgeList(const vector<vector<int>>& linkList, vector<Vec2i>& hexEdgeList);

	// hexInfo的构造
	void construct_hexInfo(vector<Vec11i>& hexInfo);

	// IDList的构造
	void get_IDList(const vector<Point2f>& ptList, vector<Vec11i>& hexInfo, vector<int>& coreIDList, vector<int>& IDList);
};

