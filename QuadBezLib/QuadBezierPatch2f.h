#ifndef H_QUAD_BEZ_PATCH2D
#define H_QUAD_BEZ_PATCH2D
#include <vector>

class Vector2f;
class Vector3f;
class QuadBezierCurve2f;

class QuadBezierPatch2f
{
public:
	QuadBezierPatch2f();
	QuadBezierPatch2f(const QuadBezierCurve2f& curv0, const QuadBezierCurve2f& curv1, const QuadBezierCurve2f& curv2);
	QuadBezierPatch2f(const Vector2f& p00, const Vector2f& p01, const Vector2f& p02,
		const Vector2f& p10, const Vector2f& p11, const Vector2f& p12,
		const Vector2f& p20, const Vector2f& p21, const Vector2f& p22);

	// get, set functions
	void setControlPoint(unsigned int r, unsigned int c, const Vector2f& point);
	void setControlPoint(unsigned int r, unsigned int c, float p1, float p2);
	void setControlPoint(unsigned int r, unsigned int c, float* p);

	Vector2f& operator()(unsigned int r, unsigned int c);
	const Vector2f& operator()(unsigned int r, unsigned int c) const;

	// interpolation functions
	void interpolate(std::vector<Vector2f>& container, std::vector<std::vector<unsigned>>& idx,
		std::vector<Vector3f>& Bernsteins, unsigned int begin, unsigned int end);
	static void interpolate(const QuadBezierCurve2f& c0, const QuadBezierCurve2f& c1, const QuadBezierCurve2f& c2,
		std::vector<Vector2f>& container, std::vector<std::vector<unsigned>>& idx, std::vector<Vector3f>& Bernsteins,
		unsigned int begin, unsigned int end);

	~QuadBezierPatch2f();

private:
	Vector2f cp[9];
};

#endif