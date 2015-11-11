#include "geometry.h"
#include "Transform.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "Light.h"

ProjectionLight::ProjectionLight(const Transform& lightToWorld, const string& texname, float fov) : Light(lightToWorld)
{
	lightPos = LightToWorld(Point(0, 0, 0));
	cv::Mat image = cv::imread(texname, cv::IMREAD_COLOR);
	//cv::flip(image, image, 1);
	texture = new cv::Mat[1];
	image.copyTo(*texture);
	float aspect = float(texture->cols) / float(texture->rows);
	if (aspect > 1.f)
	{
		screenX0 = -aspect; screenX1 = aspect;
		screenY0 = -1.f; screenY1 = 1.f;
	}
	else
	{
		screenX0 = -1.f; screenX1 = 1.f;
		screenY0 = -1.f / aspect; screenY1 = 1.f / aspect;
	}
	hither = 1e-3f;
	yon = 1e30f;
	lightProjection = Perspective(fov, hither, yon);

	// compute cosine of cone surrounding projection directions
	float opposite = tanf(Radians(fov) / 2.f);
	float tanDiag = opposite * sqrtf(1.f + 1.f / (aspect*aspect));
	cosTotalWidth = cosf(atanf(tanDiag));
}

bool ProjectionLight::switchProjection(const string& anotherTex)
{
	cv::Mat newTex = cv::imread(anotherTex, cv::IMREAD_COLOR);
	texture->release();
	newTex.copyTo(*texture);
	//cv::flip(*texture, *texture, 1);
	
	float aspect = float(texture->cols) / float(texture->rows);
	if (aspect > 1.f)
	{
		screenX0 = -aspect; screenX1 = aspect;
		screenY0 = -1.f; screenY1 = 1.f;
	}
	else
	{
		screenX0 = -1.f; screenX1 = 1.f;
		screenY0 = -1.f / aspect; screenY1 = 1.f / aspect;
	}
	return true;
}

bool ProjectionLight::switchProjection(const cv::Mat& anotherTex)
{
	texture->release();
	anotherTex.copyTo(*texture);
	//cv::flip(*texture, *texture, 1);
	float aspect = float(texture->cols) / float(texture->rows);
	if (aspect > 1.f)
	{
		screenX0 = -aspect; screenX1 = aspect;
		screenY0 = -1.f; screenY1 = 1.f;
	}
	else
	{
		screenX0 = -1.f; screenX1 = 1.f;
		screenY0 = -1.f / aspect; screenY1 = 1.f / aspect;
	}
	return true;
}

Vector ProjectionLight::Projection(const Point& p) const
{
	Vector w = Normalize(lightPos - p);
	Vector wl = WorldToLight(w);
	// Discard directions behind projection light
	if (wl.z < hither) return Vector();

	// Project point onto projection plane and compute light
	Point Pl = lightProjection(Point(wl.x, wl.y, wl.z));
	if (Pl.x < screenX0 || Pl.x > screenX1 ||
		Pl.y < screenY0 || Pl.y > screenY1) return Vector();

	float s = (Pl.x - screenX0) / (screenX1 - screenX0);
	float t = (Pl.y - screenY0) / (screenY1 - screenY0);
	
	// change s,t to image coordinate
	int u = texture->cols - int(texture->cols * s);
	int v = int(texture->rows * t);
	
	cv::Vec3b texLight = texture->at<cv::Vec3b>(v, u);
	return Vector(float(texLight[0])/256.f, float(texLight[1])/256.f, float(texLight[2])/256.f);
}

ProjectionLight::~ProjectionLight(){ texture->release(); }