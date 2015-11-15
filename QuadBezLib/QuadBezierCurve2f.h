#ifndef H_QUAD_BEZ_CURVE2D
#define H_QUAD_BEZ_CURVE2D
#include <vector>

class Vector2f
{
public:
	Vector2f(float _x=0.f, float _y=0.f){ x = _x; y = _y; }
	Vector2f& operator=(const Vector2f& rhs){ x = rhs.x; y = rhs.y; return *this; }
	float x, y;
};

inline Vector2f operator*(float m, const Vector2f& rhs)
{
	return Vector2f(m*rhs.x, m*rhs.y);
}

inline Vector2f operator-(const Vector2f& lhs, const Vector2f& rhs)
{
	return Vector2f(lhs.x - rhs.x, lhs.y - rhs.y);
}

inline Vector2f operator+(const Vector2f& lhs, const Vector2f& rhs)
{
	return Vector2f(lhs.x + rhs.x, lhs.y + rhs.y);
}

class Vector3f
{
public:
	Vector3f(float _x = 0.f, float _y = 0.f, float _z = 0.f){ x = _x; y = _y; z = _z; }
	Vector3f& operator=(const Vector3f& rhs){ x = rhs.x; y = rhs.y; z = rhs.z; return *this; }
	float x, y, z;
};

class QuadBezierCurve2f
{
public:
	QuadBezierCurve2f();
	QuadBezierCurve2f(const Vector2f& p0, const Vector2f& p1, const Vector2f& p2);

	// get, set functions
	void setControlPoint(unsigned int idx, const Vector2f& point);
	void setControlPoint(unsigned int idx, float p1, float p2);
	void setControlPoint(unsigned int idx, float* p);

	Vector2f& operator()(unsigned int idx);
	const Vector2f& operator()(unsigned int idx) const;
	
	// interpolation functions
	void interpolate(std::vector<Vector2f>& container, std::vector<Vector3f>& Bernsteins, unsigned int begin, unsigned int end);
	static void interpolate(const Vector2f& p0, const Vector2f& p1, const Vector2f& p2,
		std::vector<Vector2f>& container, std::vector<Vector3f>& Bernsteins,
		unsigned int begin, unsigned int end);

	~QuadBezierCurve2f();
private:
	Vector2f cp[3];
};

#endif