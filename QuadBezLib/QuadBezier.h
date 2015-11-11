#ifndef H_CUBICBEZIER
#define H_CUBICBEZIER

#include "QuadBezierCurve2f.h"
#include "QuadBezierPatch2f.h"

class Matrix3f
{
public:
	Matrix3f()
	{
		m[0][0] = m[1][1] = m[2][2] = 1.f;
		m[0][1] = m[0][2] = m[1][0] = m[1][2] = m[2][0] = m[2][1] = 0.f;
	}
	Matrix3f(float m00, float m01, float m02,
		float m10, float m11, float m12,
		float m20, float m21, float m22)
	{
		m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
		m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
		m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
	}
	//Vector3f operator*(const Vector3f& rhs);

	float m[3][3];
};

//inline Vector3f Matrix3f::operator*(const Vector3f& rhs)
inline Vector3f operator*(const Matrix3f& lhs, const Vector3f& rhs)
{
	float _x, _y, _z;
	_x = lhs.m[0][0] * rhs.x + lhs.m[0][1] * rhs.y + lhs.m[0][2] * rhs.z;
	_y = lhs.m[1][0] * rhs.x + lhs.m[1][1] * rhs.y + lhs.m[1][2] * rhs.z;
	_z = lhs.m[2][0] * rhs.x + lhs.m[2][1] * rhs.y + lhs.m[2][2] * rhs.z;
	return Vector3f(_x, _y, _z);
}

#endif