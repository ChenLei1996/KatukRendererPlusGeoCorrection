#include "QuadBezierCurve2f.h"

QuadBezierCurve2f::QuadBezierCurve2f()
{
}

QuadBezierCurve2f::QuadBezierCurve2f(const Vector2f& p0, const Vector2f& p1, const Vector2f& p2)
{
	cp[0] = p0;
	cp[1] = p1;
	cp[2] = p2;
}

void QuadBezierCurve2f::setControlPoint(unsigned int idx, const Vector2f& point)
{
	if ( idx < 3 )
		cp[idx] = point;
}

void QuadBezierCurve2f::setControlPoint(unsigned int idx, float p1, float p2)
{
	if ( idx < 3 )
		cp[idx] = Vector2f(p1, p2);
}

void QuadBezierCurve2f::setControlPoint(unsigned int idx, float* p)
{
	if ( idx < 3 && p != NULL)
	{
		cp[idx] = Vector2f(p[0], p[1]);
	}
}

Vector2f& QuadBezierCurve2f::operator()(unsigned int idx)
{
	return cp[idx];
}

const Vector2f& QuadBezierCurve2f::operator()(unsigned int idx) const
{
	return cp[idx];
}

void QuadBezierCurve2f::interpolate(std::vector<Vector2f>& container, std::vector<Vector3f>& Bernsteins, unsigned int begin, unsigned int end)
{
	// interpolate bezier curves
	if (begin == 0)
	{
		container.push_back(cp[0]);
		begin++;
	}
	Vector2f tmp;
	for (unsigned int i = begin; i <= end; i++)
	{
		tmp.x = cp[0].x * Bernsteins[i-1].x + cp[1].x * Bernsteins[i-1].y + cp[2].x * Bernsteins[i-1].z;
		tmp.y = cp[0].y * Bernsteins[i-1].x + cp[1].y * Bernsteins[i-1].y + cp[2].y * Bernsteins[i-1].z;
		container.push_back(tmp);
	}
}

void QuadBezierCurve2f::interpolate(const Vector2f& p0, const Vector2f& p1, const Vector2f& p2,
	std::vector<Vector2f>& container, std::vector<Vector3f>& Bernsteins,
	unsigned int begin, unsigned int end)
{
	QuadBezierCurve2f curv(p0, p1, p2);
	curv.interpolate(container, Bernsteins, begin, end);
}

QuadBezierCurve2f::~QuadBezierCurve2f()
{
}
