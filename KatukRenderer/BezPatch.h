#ifndef H_BEZPATCH
#define H_BEZPATCH
#include <iostream>
#include <opencv2/core.hpp>
#include <vector>

class Vector2f;
class Vector3f;
class Matrix3f;
class QuadBezierCurve2f;
class QuadBezierPatch2f;
class BezTreeNode;
class GeoCorrection;
struct LUT;

using std::vector;
using std::cout;
using std::endl;
using cv::Mat;
using cv::Point2f;
using cv::Size2f;

struct LUT
{
	Point2f* coord;
	int row;
	LUT(int _maxlv)
	{
		row = static_cast<int>(pow(2, _maxlv + 1) + 1);
		coord = new Point2f[row*row];
		float invr = 1.f / static_cast<float>(row-1);
		for (size_t j = 0; j < row; ++j)
		{
			for (size_t i = 0; i < row; ++i)
			{
				coord[j*row + i].x = static_cast<float>(i)*invr;
				coord[j*row + i].y = static_cast<float>(j)*invr;
			}
		}
	}
	LUT(const LUT& rhs)
	{
		row = rhs.row;
		coord = new Point2f[row*row];
		for (size_t i = 0; i < row*row; ++i)
		{
			coord[i] = rhs.coord[i];
		}
	}
	void operator()(const Point2f* _coord)
	{
		for (size_t j = 0; j < row; ++j)
		{
			for (size_t i = 0; i < row; ++i)
			{
				coord[j*row + i].x = _coord[j*row + 1].x;
				coord[j*row + i].y = _coord[j*row + 1].y;
			}
		}
	}
	~LUT(){ delete[] coord; }
};

class BezPatch
{
private:
	vector<Mat> transformedPoints;
	vector<Point2f> projPoints;
	int maxlv;
	//int** setIdx;
	/*vector<QuadBezierPatch2f > surface;
	vector<vector<Vector2f> > bezPatch;
	vector<vector<unsigned> > bezPatchIdx;*/
	LUT tex;
	BezTreeNode* root;
	GeoCorrection* geoCorrection;

public:
	void draw(int lv);
	void getMinMax(int lv, Size2f& minsize, Size2f& maxsize);
	void subdivide();
	const vector<Mat>& getTransformedPoints() const;
	const vector<Point2f>& getProjPoints() const;
	void updatePoints(vector<Mat>& _trfPts, vector<Point2f>& _projPts);
	void updateLUT(int lv);
	BezPatch(const vector<Mat>& _trfPts, const vector<Point2f>& _projPts, int _lv, GeoCorrection* _geoc);
	LUT& getLUT();
	int getMaxLevel() const;
	~BezPatch();
};

class BezTreeNode
{
public:
	//enum Quadrant{LEFTTOP, RIGHTTOP, RIGHTBTM, LEFTBTM};
	BezTreeNode();
	//BezTreeNode(int _lv, int _ninterp, BezPatch* _tree, BezTreeNode* p, int* _idx);
	BezTreeNode(int _lv, int _ninterp, int _redun, BezPatch& _tree, BezTreeNode* _parent, int* _idx);
	~BezTreeNode();
	void draw();
	void subdivide();
	void getMinMax(Size2f& minsize, Size2f& maxsize);
	void updateLUT();

	int redun;
	int lv;
	int ninterp;
	int idx[9];
	//float u_s, u_e, v_s, v_e;
	BezPatch& tree;
	BezTreeNode* parent;
	//BezTreeNode* parent;
	BezTreeNode* child[4];
	QuadBezierPatch2f* surface;
	vector<Vector3f> BernsteinVal;
	vector<Vector2f> bezPatch;
	vector<vector<unsigned> > bezPatchIdx;
};

inline int getColumnMajorIdx(int idx, int row)
{
	return (idx%row)*row + static_cast<int>(idx / row);
}

inline Vector2f operator-(const Point2f lhs, const Vector2f rhs)
{
	double _x, _y;
	_x = lhs.x - rhs.x;
	_y = lhs.y - rhs.y;
	return Vector2f(_x, _y);
}

inline Vector2f estimateControlPoint(const Vector2f& cp0, const Vector2f& cp2, const Point2f& curvePoint, double t)
{
	double X, Y;
	double diff = 1.0 - t;
	double sqrT = t*t, sqrDiff = diff*diff;
	X = (curvePoint.x - sqrDiff*cp0.x - sqrT*cp2.x) / (2 * diff*t);
	Y = (curvePoint.y - sqrDiff*cp0.y - sqrT*cp2.y) / (2 * diff*t);

	return Vector2f(X, Y);
}

void genBernsVal(std::vector<Vector3f>& container, const Matrix3f& coffMat, unsigned int steps);

#endif