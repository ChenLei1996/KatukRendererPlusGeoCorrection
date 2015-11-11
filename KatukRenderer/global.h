#ifndef H_GLOBAL
#define H_GLOBAL
#include <iostream>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#ifdef M_PI
#undef M_PI
#endif
#define M_PI       3.14159265358979323846f
#define INV_PI     0.31830988618379067154f
#define INV_TWOPI  0.15915494309189533577f
#define INV_FOURPI 0.07957747154594766788f

const float FLOAT_INFINITY = std::numeric_limits<float>::infinity();
extern const int WINX, WINY;

using std::vector;
using std::string;
using std::min;
using std::max;
using std::swap;
using std::sort;

class Vector;
class Point;
class Normal;
class Ray;
class RayDifferential;
class Transform;
class Shape;
class Sampler;
class Camera;
class Mesh;
class MeshTriangle;
class Light;
class ProjectionLight;

// Global Inline Functions
inline float Lerp(float t, float v1, float v2) {
	return (1.f - t) * v1 + t * v2;
}


inline float Clamp(float val, float low, float high) {
	if (val < low) return low;
	else if (val > high) return high;
	else return val;
}


inline int Clamp(int val, int low, int high) {
	if (val < low) return low;
	else if (val > high) return high;
	else return val;
}


inline int Mod(int a, int b) {
	int n = int(a / b);
	a -= n*b;
	if (a < 0) a += b;
	return a;
}

inline float Radians(float deg)
{
	return (M_PI / 180.0f) * deg;
}
inline float Degrees(float rad)
{
	return (180.0f / M_PI) * rad;
}
inline bool isPowerOf2(int v)
{
	return (v & (v - 1)) == 0;
}

#endif