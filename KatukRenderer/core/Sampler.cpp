#include "../global.h"
#include "geometry.h"
#include "Transform.h"
#include "Sampler.h"

void SimpleSampler::getPoint(int x, int y, Point2& ret)
{
	// change (x,y)th pixel fit in [-1, 1]
	// and return the center of the pixel
	float _x = (float(x) + 0.5f) / w;
	float _y = (float(y) + 0.5f) / h;
	
	ret.x = -1.f + 2.f*_x;
	ret.y = -1.f + 2.f*_y;
}