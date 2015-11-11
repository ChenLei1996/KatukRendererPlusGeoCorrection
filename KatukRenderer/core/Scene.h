#ifndef H_SCENE
#define H_SCENE
#include "../global.h"
#include <opencv2/core.hpp>

class Scene
{
public:
	Scene(){}
	Scene(Sampler* _sampler, Camera* _cam, vector<Shape*>* _objs, ProjectionLight* l, int _w, int _h) :w(_w), h(_h)
	{
		sampler = _sampler; cam = _cam; objs = _objs;
		plight = l;
		lightp.x = 0.f; lightp.y = 0.f; lightp.z = 7.f; //temporarily fixed
	}
	void loop(float* image) const;
	~Scene();
	friend Vector rayTracer(const Scene& scene, const Ray& ray);
	void writeImage(cv::Mat& result, float* intensity) const;

	int w, h;
	Sampler* sampler;
	Camera* cam;
	vector<Shape*> *objs;
	Point lightp;
	ProjectionLight *plight;

};



#endif