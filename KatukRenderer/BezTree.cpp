#include <GL/freeglut.h>
#include <opencv2/core.hpp>
#include "../QuadBezLib/QuadBezier.h"
#include "GeoCorrection.h"
#include "BezTree.h"
#include <vector>
#include <iostream>

extern std::vector<Vector3f> BernsVal;

BezTree::BezTree(const std::vector<cv::Mat>& _trfPnts, const std::vector<cv::Point2f>& projPnts, int _maxlv)
{
	transformedPoints = _trfPnts;
	projPoints = projPnts;
	maxLv = _maxlv;

	// generate index
	int row = static_cast<int>(sqrt(projPnts.size()));
	int rootIdx[9];
	rootIdx[0] = 0;
	rootIdx[2] = row - 1;
	rootIdx[8] = row*row - 1;
	rootIdx[6] = row*(row - 1);
	rootIdx[1] = (rootIdx[0] + rootIdx[2]) / 2;
	rootIdx[7] = (rootIdx[6] + rootIdx[8]) / 2;
	rootIdx[3] = (rootIdx[0] + rootIdx[6]) / 2;
	rootIdx[5] = (rootIdx[2] + rootIdx[8]) / 2;
	rootIdx[4] = (rootIdx[3] + rootIdx[5]) / 2;

	root = new BezTreeNode(0, 0.f, 1.f, 0.f, 1.f, this, NULL, rootIdx);
}

BezTree::~BezTree()
{
	delete root;
}

void BezTree::subdivide()
{
	root->subdivide();
}

void BezTree::draw(int lv)
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
		p->child[1]->draw();
		p->child[2]->draw();
		p->child[3]->draw();
	}
}

void BezTreeNode::subdivide()
{
	if (child[0] == NULL)
	{
		if (lv >= tree->maxLv)
		{
			std::cerr << "Exceed the max level of subidivision" << std::endl;
			return;
		}
		int childlv = lv + 1;
		// subdivide
		// clock wise order
		int id0[9], id1[9], id2[9], id3[9];
		float u_mid = 0.5f*(u_s + u_e);
		float v_mid = 0.5f*(v_s + v_e);
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
		child[0] = new BezTreeNode(childlv, u_s, u_mid, v_s, v_mid,
			tree, this, id0);
	
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
		child[1] = new BezTreeNode(childlv, u_mid, u_e, v_s, v_mid,
			tree, this, id1);

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
		child[2] = new BezTreeNode(childlv, u_mid, u_e, v_mid, v_e,
			tree, this, id2);

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
		child[3] = new BezTreeNode(childlv, u_s, u_mid, v_mid, v_e,
			tree, this, id3);
	}
	else
	{
		child[0]->subdivide();
		child[1]->subdivide();
		child[2]->subdivide();
		child[3]->subdivide();
	}
}

BezTreeNode::BezTreeNode() :lv(0), u_s(0), u_e(0), v_s(0), v_e(0), surface()
{
	parent = NULL;
	child[0] = child[1] = child[2] = child[3] = NULL;
}

BezTreeNode::BezTreeNode(int _lv, float us, float ue, float vs, float ve, 
	BezTree* _tree, BezTreeNode* p, int* _idx)
	: lv(_lv), u_s(us), u_e(ue), v_s(vs), v_e(ve)
{
	tree = _tree;
	parent = p;
	child[0] = child[1] = child[2] = child[3] = NULL;
	memcpy(idx, _idx, sizeof(int) * 9);

	const double Height = static_cast<double>(GeoCorrection::gridY) - 1;
	// compute bezier control points
	Vector2f p00(tree->transformedPoints[idx[0]].at<double>(0, 0), Height-tree->transformedPoints[idx[0]].at<double>(1, 0)),
		p02(tree->transformedPoints[idx[2]].at<double>(0, 0), Height-tree->transformedPoints[idx[2]].at<double>(1, 0)),
		p01 = estimateControlPoint(p00, p02, cv::Point2f(tree->transformedPoints[idx[1]].at<double>(0, 0), Height-tree->transformedPoints[idx[1]].at<double>(1, 0)), 0.5),
		p20(tree->transformedPoints[idx[6]].at<double>(0, 0), Height-tree->transformedPoints[idx[6]].at<double>(1, 0)),
		p22(tree->transformedPoints[idx[8]].at<double>(0, 0), Height-tree->transformedPoints[idx[8]].at<double>(1, 0)),
		p21 = estimateControlPoint(p20, p22, cv::Point2f(tree->transformedPoints[idx[7]].at<double>(0, 0), Height - tree->transformedPoints[idx[7]].at<double>(1, 0)), 0.5),
		p10 = estimateControlPoint(p00, p20, cv::Point2f(tree->transformedPoints[idx[3]].at<double>(0, 0), Height - tree->transformedPoints[idx[3]].at<double>(1, 0)), 0.5),
		p12 = estimateControlPoint(p02, p22, cv::Point2f(tree->transformedPoints[idx[5]].at<double>(0, 0), Height - tree->transformedPoints[idx[5]].at<double>(1, 0)), 0.5),
		p11 = estimateControlPoint(p10, p12, cv::Point2f(tree->transformedPoints[idx[4]].at<double>(0, 0), Height - tree->transformedPoints[idx[4]].at<double>(1, 0)), 0.5);

	Vector2f glProjPoint0(tree->projPoints[idx[0]].x, Height - tree->projPoints[idx[0]].y),
		glProjPoint1(tree->projPoints[idx[1]].x, Height - tree->projPoints[idx[1]].y),
		glProjPoint2(tree->projPoints[idx[2]].x, Height - tree->projPoints[idx[2]].y),
		glProjPoint3(tree->projPoints[idx[3]].x, Height - tree->projPoints[idx[3]].y),
		glProjPoint4(tree->projPoints[idx[4]].x, Height - tree->projPoints[idx[4]].y),
		glProjPoint5(tree->projPoints[idx[5]].x, Height - tree->projPoints[idx[5]].y),
		glProjPoint6(tree->projPoints[idx[6]].x, Height - tree->projPoints[idx[6]].y),
		glProjPoint7(tree->projPoints[idx[7]].x, Height - tree->projPoints[idx[7]].y),
		glProjPoint8(tree->projPoints[idx[8]].x, Height - tree->projPoints[idx[8]].y);
	// actual bezier control points, need to differntiate delta
	surface = new QuadBezierPatch2f(2.0f*glProjPoint0 - p00,
		2.0f*glProjPoint1 - p01,
		2.0f*glProjPoint2 - p02,
		2.0f*glProjPoint3 - p10,
		2.0f*glProjPoint4 - p11,
		2.0f*glProjPoint5 - p12,
		2.0f*glProjPoint6 - p20,
		2.0f*glProjPoint7 - p21,
		2.0f*glProjPoint8 - p22
		);

	// generate vertices and mesh indices
	surface->interpolate(bezPatch, bezPatchIdx, BernsVal, 0, BernsVal.size());
}

void BezTreeNode::draw()
{
	std::vector<unsigned> tmpIdx;
	float s, t;
	float u_m = u_e-u_s, v_m = v_e - v_s;
	unsigned int steps = BernsVal.size(), rows = steps + 1;
	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < bezPatchIdx.size(); i++)
	{
		tmpIdx = bezPatchIdx[i];
		s = u_s + u_m*(tmpIdx[0] / rows) / (float)steps;
		t = v_s + v_m*(tmpIdx[0] % rows) / (float)steps;
		glTexCoord2d(s, t);
		glVertex2d(bezPatch[tmpIdx[0]].x, bezPatch[tmpIdx[0]].y);
		s = u_s + u_m*(tmpIdx[1] / rows) / (float)steps;
		t = v_s + v_m*(tmpIdx[1] % rows) / (float)steps;
		glTexCoord2d(s, t);
		glVertex2d(bezPatch[tmpIdx[1]].x, bezPatch[tmpIdx[1]].y);
		s = u_s + u_m*(tmpIdx[2] / rows) / (float)steps;
		t = v_s + v_m*(tmpIdx[2] % rows) / (float)steps;
		glTexCoord2d(s, t);
		glVertex2d(bezPatch[tmpIdx[2]].x, bezPatch[tmpIdx[2]].y);
	}
	glEnd();
}


BezTreeNode::~BezTreeNode()
{
	delete surface;
	// should delete all child node 
}