#include "QuadBezierCurve2f.h"
#include "QuadBezierPatch2f.h"

QuadBezierPatch2f::QuadBezierPatch2f()
{
}

QuadBezierPatch2f::QuadBezierPatch2f(const QuadBezierCurve2f& curv0, const QuadBezierCurve2f& curv1, const QuadBezierCurve2f& curv2)
{
	cp[0] = curv0(0); cp[1] = curv0(1); cp[2] = curv0(2);
	cp[3] = curv1(0); cp[4] = curv1(1); cp[5] = curv1(2);
	cp[6] = curv2(0); cp[7] = curv2(1); cp[8] = curv2(2);
}

QuadBezierPatch2f::QuadBezierPatch2f(const Vector2f& p00, const Vector2f& p01, const Vector2f& p02,
	const Vector2f& p10, const Vector2f& p11, const Vector2f& p12,
	const Vector2f& p20, const Vector2f& p21, const Vector2f& p22)
{
	cp[0] = p00; cp[1] = p01; cp[2] = p02;
	cp[3] = p10; cp[4] = p11; cp[5] = p12;
	cp[6] = p20; cp[7] = p21; cp[8] = p22;
}

void QuadBezierPatch2f::setControlPoint(unsigned int r, unsigned int c, const Vector2f& point)
{
	cp[r * 3 + c] = point;
}

void QuadBezierPatch2f::setControlPoint(unsigned int r, unsigned int c, float p1, float p2)
{
	cp[r * 3 + c] = Vector2f(p1, p2);
}

void QuadBezierPatch2f::setControlPoint(unsigned int r, unsigned int c, float* p)
{
	cp[r * 3 + c] = Vector2f(p[0], p[1]);
}

Vector2f& QuadBezierPatch2f::operator()(unsigned int r, unsigned int c)
{
	return cp[r * 3 + c];
}

const Vector2f& QuadBezierPatch2f::operator()(unsigned int r, unsigned int c) const
{
	return cp[r * 3 + c];
}

void QuadBezierPatch2f::interpolate(std::vector<Vector2f>& container, std::vector<std::vector<unsigned>>& idx,
	std::vector<Vector3f>& Bernsteins, unsigned int begin, unsigned int end)
{
	// interpolate bezier patch
	// container: vertices, idx: vector of indices of trinagle meshes
	unsigned int i = begin;
	unsigned int rows = end - begin + 1;
	if (begin == 0)
	{
		// blend the first curve on the V direction using control points
		QuadBezierCurve2f::interpolate((*this)(0, 0), (*this)(1, 0), (*this)(2, 0), container, Bernsteins, begin, end);
		i++;
	}
	// generate new control points
	std::vector<Vector2f> nCp0, nCp1, nCp2;
	for (; i <= end; i++)
	{
		QuadBezierCurve2f::interpolate((*this)(0, 0), (*this)(0, 1), (*this)(0, 2), nCp0, Bernsteins, i, i);
		QuadBezierCurve2f::interpolate((*this)(1, 0), (*this)(1, 1), (*this)(1, 2), nCp1, Bernsteins, i, i);
		QuadBezierCurve2f::interpolate((*this)(2, 0), (*this)(2, 1), (*this)(2, 2), nCp2, Bernsteins, i, i);

		// blending
		QuadBezierCurve2f::interpolate(nCp0.back(), nCp1.back(), nCp2.back(), container, Bernsteins, begin, end);
	}

	// generate indices of triangle meshes
	for (unsigned int s = 0; s < rows - 1; s++)
	{
		for (unsigned int t = 0; t < rows - 1; t++)
		{
			std::vector<unsigned> index,index2;
			index.push_back(s*rows + t);
			index.push_back(s*rows + t + 1);
			index.push_back((s + 1)*rows + t);
			idx.push_back(index);
			index2.push_back(s*rows + t + 1);
			index2.push_back((s + 1)*rows + t + 1);
			index2.push_back((s + 1)*rows + t);
			idx.push_back(index2);
		}
	}
}

void QuadBezierPatch2f::interpolate(const QuadBezierCurve2f& c0, const QuadBezierCurve2f& c1, const QuadBezierCurve2f& c2,
	std::vector<Vector2f>& container, std::vector<std::vector<unsigned>>& idx, std::vector<Vector3f>& Bernsteins,
	unsigned int begin, unsigned int end)
{
	QuadBezierPatch2f patch(c0, c1, c2);
	patch.interpolate(container, idx, Bernsteins, begin, end);
}

QuadBezierPatch2f::~QuadBezierPatch2f()
{
}
