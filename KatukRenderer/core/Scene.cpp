#include "../global.h"
#include "geometry.h"
#include "Transform.h"
#include "shape.h"
#include "Sampler.h"
#include "Camera.h"
#include "Light.h"
#include "Scene.h"

Scene::~Scene()
{
	delete plight;
}

void Scene::loop(float* image) const
{
	int i = 0, j = 0;
	Point2 sample;
	Ray ray;
	Vector intensity;
	for (; j < h; j++)
	{
		for (i=0; i < w; i++)
		{
			// get sample point on the pixel
			sampler->getPoint(i, j, sample);

			// generate ray using the point
			cam->genRay(sample, ray);

			// now check whether a ray intersects any object.
			intensity = rayTracer(*this, ray);

			// save intensity into the image
			image[3 * (j*w + i) + 0] = intensity.x;
			image[3 * (j*w + i) + 1] = intensity.y;
			image[3 * (j*w + i) + 2] = intensity.z;
		}
		fprintf(stdout, "\r Ray tracing %.1f%% complete", static_cast<float>(j) / h * 100.f);
	}
	fprintf(stdout, "\r Ray tracing %.1f%% complete", static_cast<float>(j) / h * 100.f);
}

Vector rayTracer(const Scene& scene, const Ray& ray)
{
	float tHit = FLOAT_INFINITY, tTmp = FLOAT_INFINITY;
	int idx;
	Localgeo geoHit, geoTmp;

	// find if there's any intersections
	for (int i = 0; i < scene.objs->size(); i++)
	{
		if (!(*scene.objs)[i]->intersect(ray, &tTmp, geoTmp))
			continue;	
		if (tHit > tTmp)
		{
			tHit = tTmp;
			idx = i;
			geoHit = geoTmp;
		}
	}

	// if there's none..
	if (tHit == FLOAT_INFINITY)
		return Vector();

	// if it hits anything, we need to make a shadow ray
	Point shadowOrigin = (*(*scene.objs)[idx]->T)(geoHit.p);
	Vector shadowDir = scene.lightp - shadowOrigin;
	Ray shadowRay(shadowOrigin, shadowDir, 1.0e-4);
	bool isLightHit= true;
	for (int i = 0; i < scene.objs->size(); i++)
	{
		if (!(*scene.objs)[i]->intersectP(shadowRay))
			continue;
		else
		{
			isLightHit = false;
			break;
		}
	}
	if (!isLightHit)
		return Vector();
	
	// if it hits the light
	Normal wrdNormal = Normalize( (*(*scene.objs)[idx]->T)(geoHit.n) );
	Normal invDir(-ray.d);
	invDir = Normalize(invDir);
	float lamb = Dot(wrdNormal, invDir);
	//return Vector(lamb, lamb, lamb);
	Vector projIntensity = scene.plight->Projection(shadowOrigin);
	if (projIntensity.Length() == 0.f)
		return Vector(.3f, .3f, .3f);
	else
		return projIntensity;
}

void Scene::writeImage(cv::Mat& result, float* intensity) const
{
	//float gamma = 1/2.2f;
	unsigned int w = result.cols, h = result.rows;
	for (unsigned int j = 0; j < h; j++)
	{
		for (unsigned int i = 0; i < w; i++)
		{
			/*result.at<cv::Vec3b>(j, i)[0] = unsigned char(256 * pow(intensity[3 * (j*w + i) + 0], gamma));
			result.at<cv::Vec3b>(j, i)[1] = unsigned char(256 * pow(intensity[3 * (j*w + i) + 1], gamma));
			result.at<cv::Vec3b>(j, i)[2] = unsigned char(256 * pow(intensity[3 * (j*w + i) + 2], gamma));*/
			result.at<cv::Vec3b>(j, i)[0] = unsigned char(256 * intensity[3 * (j*w + i) + 0]);
			result.at<cv::Vec3b>(j, i)[1] = unsigned char(256 * intensity[3 * (j*w + i) + 1]);
			result.at<cv::Vec3b>(j, i)[2] = unsigned char(256 * intensity[3 * (j*w + i) + 2]);
		}
	}
}