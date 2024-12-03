#pragma once
#include "config.h"
#include "Data.h"

class constructor_hexPair
{
public:

	constructor_hexPair(const string& path);			// ���캯������ȡ��.marker���ļ�
	constructor_hexPair(const Mat1i& set_state);		// ���캯�����ֶ���д marker ��״̬����

	// ��ptList����ȡhexInfo
	void apply(Data::constructor& data);

	// ���Ƶ�ǰ marker, scale Ϊ���������α߳���������
	void draw(Mat1b& img, const int scale = 100);

	// ���浱ǰ marker
	void save(const string& path);

private:
	
	// ȫ�ֲ���	
	bool parallel;
	Mat1i state;					// ״̬����
	int max_ID;						// ״̬������ID���������
	map<int, Vec10i> map_key2idx;	// key-vertexIdx ��ѯ��
	map<Vec2i, Vec8i, CompareVec2i> map_edg2idx;	// edge-vertexIdx ��ѯ��

	// ��ز�ѯ��
	vector<Vec3i>	triList;		// �������б��ڵ�����������
	vector<vector<int>> linkList;	// �����������ϵ���б�
	vector<Vec2i>	hexEdgeList;	// �����ζԵ����ı�Ե�б��Ա�Ե�����˵���ptList�е��������
	map<Point2f, int, ComparePoint2f> pt_XYtoIdx;		// ��������->������
	map<Vec3i, int, CompareVec3i> tri_IdxtoState;		// �������ζ�����->������״̬��state: 0-���Σ�1-��ɫ���ڰ�δ֪����2-�ڵװ׵㣬3-�׵׺ڵ�

	// ��ʼ��������󲿷ֲ�ѯ��
	void initial(Data::constructor& data);
	void load_from_file(const string path);
	void build_map();

	// ����
	inline int getPointID(map<Point2f, int, ComparePoint2f>& pt_XYtoIdx, const Point2f& pt);
	inline int getTriState(const Vec3i tri, map<Vec3i, int, CompareVec3i>& tri_IdxtoState);
	inline float getLength(const Point2f& p0, const Point2f& p1);
	inline bool checkTriLens(const Vec3f& triLens, const float distort_T);
	inline void round_IdxtoState(const int A, const int B, const Vec8i& round_Idx, map<Vec3i, int, CompareVec3i>& tri_IdxtoState, Vec10i& state);
	inline int round_StatetoKey(const Vec10i& state);
	inline Vec3i findJoint(const vector<int>& A, const vector<int>& B);

	// ���ֱ��Ĺ���
	void build_pt_XYtoIdx(const vector<Point2f> ptList, map<Point2f, int, ComparePoint2f>& pt_XYtoIdx);
	void build_triList(const Subdiv2D& subdiv, map<Point2f, int, ComparePoint2f>& pt_XYtoIdx, vector<Vec3i>& triList);
	void build_linkList(const Subdiv2D& subdiv, const int ptNum, vector<vector<int>>& linkList);
	void build_tri_IdxtoState(const Mat1b& img, const vector<Vec3i>& triList, const vector<Point2f>& ptList, const float distort_T, const uchar grayT, map<Vec3i, int, CompareVec3i>& tri_IdxtoState);
	void build_hexEdgeList(const vector<vector<int>>& linkList, vector<Vec2i>& hexEdgeList);

	// hexInfo�Ĺ���
	void construct_hexInfo(vector<Vec11i>& hexInfo);

	// IDList�Ĺ���
	void get_IDList(const vector<Point2f>& ptList, vector<Vec11i>& hexInfo, vector<int>& coreIDList, vector<int>& IDList);
};

