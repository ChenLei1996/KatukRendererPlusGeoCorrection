#ifndef H_CAMERA
#define H_CAMERA
#include "../global.h"
#include "geometry.h"
#include "Transform.h"

class Camera
{
public:
	Camera(){}
	virtual ~Camera(){}
	virtual void genRay(const Point2& sample, Ray& ret){ std::cerr << "This virtual Camera::genRay() shouldn't be called" << std::endl; }
};

class PerspectiveCamera : public Camera
{
public:
	PerspectiveCamera():focal(0.f){};
	PerspectiveCamera(const Point& _e, const Point& lookAt, const Vector& up, float fovy) :e(_e)
	{
		toCamera = LookAt(e, lookAt, up);
		focal = 1 / tan(Radians(0.5f*fovy));
	}
	void genRay(const Point2& sample, Ray& ret);
	
	Transform toCamera;
	Point e;// , u, v, w;
	float focal;
};

#endif