#ifndef H_SAMPLER
#define H_SAMPLER
#include "../global.h"

typedef struct _point2{
	float x, y;
}Point2;

class Sampler
{
public:
	Sampler(){ }
	virtual ~Sampler(){ }
	virtual void getPoint(int x, int y, Point2& ret){ std::cerr << "Virtual Sampler::getPoint is called" << std::endl; }
};

class SimpleSampler : public Sampler
{
public:
	SimpleSampler() : w(0), h(0) { }
	SimpleSampler(int width, int height) : w(width), h(height){ }
	~SimpleSampler(){}
	void getPoint(int x, int y, Point2& ret);
	int w, h;
};

#endif