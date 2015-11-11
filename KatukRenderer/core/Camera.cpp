#include "geometry.h"
#include "Transform.h"
#include "Sampler.h"
#include "Camera.h"

void PerspectiveCamera::genRay(const Point2& sample, Ray& ret)
{
	Transform invT = Inverse(toCamera);
	Point o(0.f, 0.f, 0.f);
	Vector d(sample.x, sample.y, -focal);
	Ray r(o, d, 0.f);
	ret = invT(r);
}