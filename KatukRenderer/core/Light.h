#ifndef H_LIGHT
#define H_LIGHT
#include "../global.h"
#include <opencv2/core.hpp>

class Light
{
public:
	Light(){}
	Light(const Transform& l2w) : LightToWorld(l2w), WorldToLight(Inverse(l2w)){
		if (WorldToLight.HasScale())
			std::cerr << "Scaling is in world to light transformation" << std::endl;
	}
	virtual ~Light(){}
	const Transform LightToWorld, WorldToLight;
};

class ProjectionLight : public Light{
public:
	ProjectionLight(const Transform& lightToWorld, const string& texname, float fov);
	~ProjectionLight();
	Vector Projection(const Point& p) const;
	bool switchProjection(const string& anotherTex);
	bool switchProjection(const cv::Mat& anotherTex);


	cv::Mat *texture;
	Point lightPos;
	Transform lightProjection;
	float screenX0, screenX1, screenY0, screenY1; // projetion plane
	float hither, yon;
	float cosTotalWidth;
};

#endif