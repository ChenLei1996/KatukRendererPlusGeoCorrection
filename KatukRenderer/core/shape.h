#ifndef H_SHAPE
#define H_SHAPE
#include "../global.h"
#include "geometry.h"
#include "Transform.h"

typedef struct _localgeo{
	Point p;
	Normal n;
}Localgeo;

class Shape
{
public:
	Shape(){ T = NULL; }
	virtual ~Shape(){}
	virtual bool intersect(const Ray& ray, float* tHit, Localgeo& local){ std::cerr << "Virtual shape::intersect is called" << std::endl; return false; }
	virtual bool intersectP(const Ray& ray){ std::cerr << "Virtual shape::intersectP is called" << std::endl; return false; }
	
	Transform* T; // saves from local to world
};

class Sphere : public Shape
{
public:
	Sphere() : Shape(), r(0.0) {}
	Sphere(Transform* _t, float _r) : r(_r) { T = _t; }
	Sphere& operator=(const Sphere& rhs){ T = rhs.T; r = rhs.r; return *this; }
	~Sphere(){}
	bool intersect(const Ray& ray, float* tHit, Localgeo& local);
	bool intersectP(const Ray& ray);

	float r; // radius
};

class Triangle : public Shape
{
public:
	Triangle() : Shape(), p0(), p1(), p2(){}
	Triangle(Transform* _t, const Point& _p0, const Point& _p1, const Point& _p2)
		: p0(_p0), p1(_p1), p2(_p2){ T = _t; }
	Triangle& operator=(const Triangle& rhs){ T = rhs.T;  p0 = rhs.p0; p1 = rhs.p1; p2 = rhs.p2; return *this; }
	~Triangle(){}
	bool intersect(const Ray& ray, float* tHit, Localgeo& local);
	bool intersectP(const Ray& ray);

	Point p0, p1, p2;
};

class MeshTriangle
{
public:
	MeshTriangle(){}
	~MeshTriangle(){}

	bool intersect(const Mesh& mesh, const Ray& ray, float* tHit, Localgeo& local);
	bool intersectP(const Mesh& mesh, const Ray& ray);
	int& v(unsigned int idx);
	int& n(unsigned int idx);

	int vert[3];
	int norm[3];

};

class Mesh : public Shape
{
public:
	Mesh() : Shape(){ v = NULL; tris = NULL; }
	Mesh& operator=(const Mesh& rhs){ T = rhs.T; v = rhs.v; tris = rhs.tris; return *this; }
	Mesh(Transform* _t, vector<Vector> *vertices, vector<MeshTriangle> *triangles){ T = _t; v = vertices; n = NULL; tris = triangles; }
	Mesh(Transform* _t, vector<Vector> *vertices, vector<Normal> *normals, vector<MeshTriangle> *triangles)
	{ 
		T = _t; v = vertices;
		if (normals->size() != 0)
			n = normals;
		else
			n = NULL;
		tris = triangles;
	}
	~Mesh(){}
	friend class MeshTriangle;
	
	bool intersect(const Ray& ray, float* tHit, Localgeo& local);
	bool intersectP(const Ray& ray);

	vector<Vector> *v;
	vector<Normal> *n;
	vector<MeshTriangle> *tris;
};

#endif