#include <GL/freeglut.h>
#include <opencv2/core.hpp>
#include "../QuadBezLib/QuadBezier.h"
#include "GeoCorrection.h"
#include "BezPatch.h"
#include <vector>
#include <iostream>

const Matrix3f BernCoff(1.0, -2.0, 1.0, 0.0, 2.0, -2.0, 0.0, 0.0, 1.0);

BezPatch::BezPatch(const vector<Mat>& _trfPts, const vector<Point2f>& _projPts, int _lv) : transformedPoints(_trfPts), projPoints(_projPts), tex(_lv), maxlv(_lv)
{
	// index for initial bezier patch
	int row = static_cast<int>(pow(2, maxlv + 1) + 1);
	int rootIdx[9];
	rootIdx[0] = 0;
	rootIdx[2] = row - 1;
	rootIdx[6] = row*(row - 1);
	rootIdx[8] = row*row - 1;
	rootIdx[1] = (rootIdx[0] + rootIdx[2]) / 2;
	rootIdx[7] = (rootIdx[6] + rootIdx[8]) / 2;
	rootIdx[3] = (rootIdx[0] + rootIdx[6]) / 2;
	rootIdx[5] = (rootIdx[2] + rootIdx[8]) / 2;
	rootIdx[4] = (rootIdx[3] + rootIdx[5]) / 2;

	/*cout << "Index array" << endl;
	for (size_t i = 0; i < 9; ++i)
		cout << rootIdx[i] << " ";
	cout << endl;

	cout << "tex.row " << tex.row << endl;
	cout << "Print LUT" << endl;
	for (size_t i = 0; i < tex.row*row; ++i)
	{
		cout << tex.coord[i].x << "," << tex.coord[i].y << " ";
		if (i % (tex.row) == tex.row-1)
			cout << endl;
	}*/

	// and last, texture coordinates should be updated
	root = new BezTreeNode(0, row-1, 0, *this, nullptr, rootIdx);
}

int BezPatch::getMaxLevel() const
{
	return maxlv;
}

BezPatch::~BezPatch()
{
	delete root;
}

LUT& BezPatch::getLUT()
{
	return tex;
}

void BezPatch::draw(int lv)
{
	if (lv == 0)
	{
		root->draw();
		return;
	}

	BezTreeNode* p;
	for (int i = 0; i < 4; i++)
	{
		p = root;
		while (p->child[i] != NULL && p->child[i]->lv != lv)
		{
			p = p->child[i];
		}
		if (p->child[i] == NULL)
		{
			std::cerr << "Subdivision is not operated yet, run subdivision first" << std::endl;
			return;
		}
		p->child[0]->draw();
		//p->child[1]->draw();
		//p->child[2]->draw();
		//p->child[3]->draw();
	}
}

void BezPatch::getMinMax(int lv, Size2f& minsize, Size2f& maxsize)
{
	if (lv == 0)
	{
		root->getMinMax(minsize, maxsize);
		return;
	}
	float _minx = 1e8, _miny = 1e8, _maxx = -1e8, _maxy = -1e8;

	BezTreeNode* p;
	for (int i = 0; i < 4; i++)
	{
		p = root;
		while (p->child[i] != nullptr && p->child[i]->lv != lv)
		{
			p = p->child[i];
		}
		if (p->child[i] == nullptr)
		{
			std::cerr << "BezPatch::getMinMax() > subdivision is not run yet" << std::endl;
			return;
		}
		Size2f smin, smax;
		
		p->child[0]->getMinMax(smin, smax);
		_minx = fminf(_minx, smin.width); 		_miny = fminf(_miny, smin.height);
		_maxx = fmaxf(_maxx, smax.width); 		_maxy = fmaxf(_maxy, smax.height);
		p->child[1]->getMinMax(smin, smax);
		_minx = fminf(_minx, smin.width); 		_miny = fminf(_miny, smin.height);
		_maxx = fmaxf(_maxx, smax.width); 		_maxy = fmaxf(_maxy, smax.height);
		p->child[2]->getMinMax(smin, smax);
		_minx = fminf(_minx, smin.width); 		_miny = fminf(_miny, smin.height);
		_maxx = fmaxf(_maxx, smax.width); 		_maxy = fmaxf(_maxy, smax.height);
		p->child[3]->getMinMax(smin, smax);
		_minx = fminf(_minx, smin.width); 		_miny = fminf(_miny, smin.height);
		_maxx = fmaxf(_maxx, smax.width); 		_maxy = fmaxf(_maxy, smax.height);
	}
	minsize.width = _minx; minsize.height = _miny;
	maxsize.width = _maxx; maxsize.height = _maxy;
}

const vector<Mat>& BezPatch::getTransformedPoints() const
{
	return transformedPoints;
}

const vector<Point2f>& BezPatch::getProjPoints() const
{
	return projPoints;
}

void BezPatch::updatePoints(vector<Mat>& _trfPts, vector<Point2f>& _projPts)
{
	cout << "BezPatch::updatePoints() > update " << endl;
	transformedPoints.swap(_trfPts);
	projPoints.swap(_projPts);
	cout << "BezPatch::updatePoints() > update complete" << endl;
}

void BezPatch::subdivide()
{
	cout << "BezPatch::subdivide() > subdivide " << endl;
	root->subdivide();
	cout << "BezPatch::subdivide() > subdivide complete" << endl;

}

void BezPatch::updateLUT(int lv)
{
	cout << "BezPatch::updateLUT() > updateLUT " << endl;
	if (lv == 0)
	{
		root->updateLUT();
		return;
	}

	BezTreeNode* p;
	for (int i = 0; i < 4; i++)
	{
		p = root;
		while (p->child[i] != nullptr && p->child[i]->lv != lv)
		{
			p = p->child[i];
		}
		if (p->child[i] == nullptr)
		{
			std::cerr << "BezPatch::updateLUT() > Subdivision is not operated yet, run subdivision first" << std::endl;
			return;
		}
		p->child[0]->updateLUT();
		p->child[1]->updateLUT();
		p->child[2]->updateLUT();
		p->child[3]->updateLUT();
	}
	cout << "BezPatch::updateLUT() > updateLUT complete" << endl;

}

void BezTreeNode::updateLUT()
{
	// update Texture Coordinate
	Size2f _min, _max;
	getMinMax(_min, _max);
	float width = _max.width - _min.width;
	float height = _max.height - _min.height;

	LUT& updatedLUT = tree.getLUT();
	int row = ninterp + 1;
	float u, v;
	Point2f tmpLUT;
	for (size_t i = 0; i < bezPatch.size(); ++i)
	{
		u = bezPatch[i].x + _min.width;
		v = bezPatch[i].y + _min.height;
		tmpLUT.x = u / width;
		tmpLUT.y = v / height;
		updatedLUT.coord[redun + i] = tmpLUT;
	}
}

BezTreeNode::BezTreeNode(int _lv, int _ninterp, int _redun, BezPatch& _tree, BezTreeNode* _parent, int* _idx) :lv(_lv), ninterp(_ninterp), redun(_redun), tree(_tree)
{
	parent = _parent;
	const vector<Mat>& trfPts = tree.getTransformedPoints();
	const vector<Point2f>& projPts = tree.getProjPoints();
	memcpy(idx, _idx, sizeof(int) * 9);

	// interpolate Bernstein polynomial
	child[0] = child[1] = child[2] = child[3] = nullptr;
	Vector2f p00(trfPts[idx[0]].at<double>(0, 0), trfPts[idx[0]].at<double>(1, 0)),
		p02(trfPts[idx[2]].at<double>(0, 0), trfPts[idx[2]].at<double>(1, 0)),
		p01 = estimateControlPoint(p00, p02, cv::Point2f(trfPts[idx[1]].at<double>(0, 0), trfPts[idx[1]].at<double>(1, 0)), 0.5),
		p20(trfPts[idx[6]].at<double>(0, 0), trfPts[idx[6]].at<double>(1, 0)),
		p22(trfPts[idx[8]].at<double>(0, 0), trfPts[idx[8]].at<double>(1, 0)),
		p21 = estimateControlPoint(p20, p22, cv::Point2f(trfPts[idx[7]].at<double>(0, 0), trfPts[idx[7]].at<double>(1, 0)), 0.5),
		p10 = estimateControlPoint(p00, p20, cv::Point2f(trfPts[idx[3]].at<double>(0, 0), trfPts[idx[3]].at<double>(1, 0)), 0.5),
		p12 = estimateControlPoint(p02, p22, cv::Point2f(trfPts[idx[5]].at<double>(0, 0), trfPts[idx[5]].at<double>(1, 0)), 0.5),
		p11 = estimateControlPoint(p10, p12, cv::Point2f(trfPts[idx[4]].at<double>(0, 0), trfPts[idx[4]].at<double>(1, 0)), 0.5);

	Vector2f glProjPoint0(projPts[idx[0]].x, projPts[idx[0]].y),
		glProjPoint1(projPts[idx[1]].x, projPts[idx[1]].y),
		glProjPoint2(projPts[idx[2]].x, projPts[idx[2]].y),
		glProjPoint3(projPts[idx[3]].x, projPts[idx[3]].y),
		glProjPoint4(projPts[idx[4]].x, projPts[idx[4]].y),
		glProjPoint5(projPts[idx[5]].x, projPts[idx[5]].y),
		glProjPoint6(projPts[idx[6]].x, projPts[idx[6]].y),
		glProjPoint7(projPts[idx[7]].x, projPts[idx[7]].y),
		glProjPoint8(projPts[idx[8]].x, projPts[idx[8]].y);

	if (_lv == 0)
	{
		cout << "Level zero Bezier patch" << endl;
		surface = new QuadBezierPatch2f(p00,
			2.0*glProjPoint1 - p01,
			p02,
			2.0*glProjPoint3 - p10,
			2.0*glProjPoint4 - p11,
			2.0*glProjPoint5 - p12,
			p20,
			2.0*glProjPoint7 - p21,
			p22
			);
	}
	else
	{
		cout << "Level " << lv << " Bezier Patch" << endl;
		cout << "BezTreeNode::Constructor > parent->bezPatch value" << endl;
		cout << parent->bezPatch[idx[0]].x << ',' << parent->bezPatch[idx[0]].y << endl;
		cout << parent->bezPatch[idx[2]].x << ',' << parent->bezPatch[idx[2]].y << endl;
		cout << parent->bezPatch[idx[6]].x << ',' << parent->bezPatch[idx[6]].y << endl;
		cout << parent->bezPatch[idx[8]].x << ',' << parent->bezPatch[idx[8]].y << endl;
		
		cout << endl;
		cout << p00.x << ',' << p00.y << endl;
		cout << p02.x << ',' << p02.y << endl;
		cout << p20.x << ',' << p20.y << endl;
		cout << p22.x << ',' << p22.y << endl;
		surface = new QuadBezierPatch2f(p00, //parent->bezPatch[idx[0]],
			2.0*glProjPoint1 - p01,
			p02, //parent->bezPatch[idx[2]],
			2.0*glProjPoint3 - p10,
			2.0*glProjPoint4 - p11,
			2.0*glProjPoint5 - p12,
			p20, //parent->bezPatch[idx[6]],
			2.0*glProjPoint7 - p21,
			p22 //parent->bezPatch[idx[8]]
			);
	}

	genBernsVal(BernsteinVal, BernCoff, ninterp);
	surface->interpolate(bezPatch, bezPatchIdx, BernsteinVal, 0, BernsteinVal.size());
}

void BezTreeNode::draw()
{
	std::vector<unsigned> tmpIdx;
	Point2f tmpUV;
	unsigned int steps = BernsteinVal.size(), rows = steps + 1;
	const LUT& lut = tree.getLUT();
	glBegin(GL_TRIANGLES);

	//cout << "Print Index" << endl;
	for (unsigned int i = 0; i < bezPatchIdx.size(); i++)
	{
		tmpIdx = bezPatchIdx[i];
		tmpUV = lut.coord[redun+tmpIdx[0]];
		//cout << tmpIdx[0] << " ";
		glTexCoord2f(tmpUV.x, tmpUV.y);
		glVertex2d(bezPatch[tmpIdx[0]].x, bezPatch[tmpIdx[0]].y);
		
		tmpUV = lut.coord[redun + tmpIdx[1]];
		//cout << tmpIdx[1] << " ";
		glTexCoord2f(tmpUV.x, tmpUV.y);
		glVertex2d(bezPatch[tmpIdx[1]].x, bezPatch[tmpIdx[1]].y);
		
		tmpUV = lut.coord[redun + tmpIdx[2]];
		//cout << tmpIdx[2] << endl;
		glTexCoord2f(tmpUV.x, tmpUV.y);
		glVertex2d(bezPatch[tmpIdx[2]].x, bezPatch[tmpIdx[2]].y);
	}
	glEnd();
}

void BezTreeNode::subdivide()
{
	if (child[0] == nullptr)
	{
		if (lv >= tree.getMaxLevel())
		{
			std::cerr << "BezTreeNode::subdivide() > Exceed the max level of subidivision" << std::endl;
			return;
		}
		int childlv = lv + 1;
		// subdivide
		// clock wise order
		int id0[9], id1[9], id2[9], id3[9];
		// child[0]
		id0[0] = idx[0];
		id0[2] = idx[1];
		id0[6] = idx[3];
		id0[8] = idx[4];
		id0[1] = static_cast<int>((id0[0] + id0[2]) / 2.0);
		id0[7] = static_cast<int>((id0[6] + id0[8]) / 2.0);
		id0[3] = static_cast<int>((id0[0] + id0[6]) / 2.0);
		id0[5] = static_cast<int>((id0[2] + id0[8]) / 2.0);
		id0[4] = static_cast<int>((id0[3] + id0[5]) / 2.0);
		cout << "First child subdivison" << endl;
		child[0] = new BezTreeNode(childlv, ninterp/2, 0, tree, this, id0);

		// child[1]
		id1[0] = idx[1];
		id1[2] = idx[2];
		id1[6] = idx[4];
		id1[8] = idx[5];
		id1[1] = static_cast<int>((id1[0] + id1[2]) / 2.0);
		id1[7] = static_cast<int>((id1[6] + id1[8]) / 2.0);
		id1[3] = static_cast<int>((id1[0] + id1[6]) / 2.0);
		id1[5] = static_cast<int>((id1[2] + id1[8]) / 2.0);
		id1[4] = static_cast<int>((id1[3] + id1[5]) / 2.0);
		cout << "Second child subdivison" << endl;
		child[1] = new BezTreeNode(childlv, ninterp/2,	4, tree, this, id1);

		// child[2] right bottom
		id2[0] = idx[4];
		id2[2] = idx[5];
		id2[6] = idx[7];
		id2[8] = idx[8];
		id2[1] = static_cast<int>((id2[0] + id2[2]) / 2.0);
		id2[7] = static_cast<int>((id2[6] + id2[8]) / 2.0);
		id2[3] = static_cast<int>((id2[0] + id2[6]) / 2.0);
		id2[5] = static_cast<int>((id2[2] + id2[8]) / 2.0);
		id2[4] = static_cast<int>((id2[3] + id2[5]) / 2.0);
		cout << "Thrid child subdivison" << endl;
		child[2] = new BezTreeNode(childlv, ninterp/2,  40, tree, this, id2);

		// child[3] left bottom
		id3[0] = idx[3];
		id3[2] = idx[4];
		id3[6] = idx[6];
		id3[8] = idx[7];
		id3[1] = static_cast<int>((id3[0] + id3[2]) / 2.0);
		id3[7] = static_cast<int>((id3[6] + id3[8]) / 2.0);
		id3[3] = static_cast<int>((id3[0] + id3[6]) / 2.0);
		id3[5] = static_cast<int>((id3[2] + id3[8]) / 2.0);
		id3[4] = static_cast<int>((id3[3] + id3[5]) / 2.0);
		cout << "Fourth child subdivison" << endl;
		child[3] = new BezTreeNode(childlv, ninterp/2,  36, tree, this, id3);
	}
	else
	{
		child[0]->subdivide();
		child[1]->subdivide();
		child[2]->subdivide();
		child[3]->subdivide();
	}
}

void BezTreeNode::getMinMax(Size2f& minsize, cv::Size2f& maxsize)
{
	float minx = 1e8, miny = 1e8, maxx = -1e8, maxy = -1e8;
	std::vector<Vector2f>::iterator iter = bezPatch.begin();
	for (; iter != bezPatch.end(); ++iter)
	{
		if (iter->x < minx)
			minx = iter->x;
		if (iter->x > maxx)
			maxx = iter->x;
		if (iter->y < miny)
			miny = iter->y;
		if (iter->y > maxy)
			maxy = iter->y;
	}
	minsize.width = minx;
	minsize.height = miny;
	maxsize.width = maxx;
	maxsize.height = maxy;
}

BezTreeNode::~BezTreeNode()
{
	delete surface;
	// should delete all child node 
}

void genBernsVal(std::vector<Vector3f>& container, const Matrix3f& coffMat, unsigned int steps)
{
	Vector3f tmp;
	tmp.x = 1.0;
	double u;
	for (unsigned int i = 1; i <= steps; i++)
	{
		u = i / (double)steps;
		tmp.y = u;
		tmp.z = u*u;
		container.push_back(coffMat * tmp);
	}
}