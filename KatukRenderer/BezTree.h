#ifndef H_BEZTREE
#define H_BEZTREE

#include <opencv2/core.hpp>
#include <vector>

class Vector2f;
class Vector3f;
class Matrix3f;
class QuadBezierCurve2f;
class QuadBezierPatch2f;
class BezTreeNode;

class BezTree
{
public:
	BezTree(){ root = NULL; }
	~BezTree();
	BezTree(const std::vector<cv::Mat>& _trfPnts, const std::vector<cv::Point2f>& projPnts, int _maxlv);
	void subdivide();
	void draw(int lv);
	std::vector<cv::Mat> transformedPoints;
	std::vector<cv::Point2f> projPoints;
	int maxLv;
	BezTreeNode *root;
};

class BezTreeNode
{
public:
	BezTreeNode();
	BezTreeNode(int _lv, float us, float ue, float vs, float ve,
		BezTree* _tree, BezTreeNode* p, int* _idx);
	~BezTreeNode();
	void draw();
	void subdivide();

	int lv;
	int idx[9];
	float u_s, u_e, v_s, v_e;
	BezTree* tree;
	BezTreeNode* parent;
	BezTreeNode* child[4];
	QuadBezierPatch2f* surface;
	std::vector<Vector2f> bezPatch;
	std::vector<std::vector<unsigned>> bezPatchIdx;
};

//inline cv::Point2f estimateControlPoint(const cv::Point2f& cp0, const cv::Point2f& cp2, const cv::Point2f& curvePoint, double t)
//{
//	double X, Y;
//	double diff = 1.0 - t;
//	double sqrT = t*t, sqrDiff = diff*diff;
//	X = (curvePoint.x - sqrDiff*cp0.x - sqrT*cp2.x) / (2 * diff*t);
//	Y = (curvePoint.y - sqrDiff*cp0.y - sqrT*cp2.y) / (2 * diff*t);
//
//	return cv::Point2f(X, Y);
//}
inline Vector2f operator-(const cv::Point2f lhs, const Vector2f rhs)
{
	double _x, _y;
	_x = lhs.x - rhs.x;
	_y = lhs.y - rhs.y;
	return Vector2f(_x, _y);
}

inline Vector2f estimateControlPoint(const Vector2f& cp0, const Vector2f& cp2, const cv::Point2f& curvePoint, double t)
{
	double X, Y;
	double diff = 1.0 - t;
	double sqrT = t*t, sqrDiff = diff*diff;
	X = (curvePoint.x - sqrDiff*cp0.x - sqrT*cp2.x) / (2 * diff*t);
	Y = (curvePoint.y - sqrDiff*cp0.y - sqrT*cp2.y) / (2 * diff*t);

	return Vector2f(X, Y);
}

#endif