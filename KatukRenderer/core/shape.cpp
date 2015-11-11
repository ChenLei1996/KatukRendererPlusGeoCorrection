#include "geometry.h"
#include "Transform.h"
#include "shape.h"

bool Sphere::intersect(const Ray& ray, float* tHit, Localgeo& local)
{
	// transform ray to local coordinate
	Transform inv = Inverse(*T);
	Ray rayp = inv(ray);
	float rSquare = r* r;

	// origin to center, origin is always at 0,0,0 in local coordinate
	Vector oc(rayp.o.x, rayp.o.y, rayp.o.z);
	float ocsqr = Dot(oc, oc);
	float dsqr = Dot(ray.d, ray.d);
	float dDotOc = Dot(ray.d, oc);
	float det = dDotOc*dDotOc - dsqr*(ocsqr - rSquare);
	if (det < 0)
		return false;
	det = sqrt(det);
	*tHit = (-dDotOc - det) / dsqr;
	if (*tHit < rayp.mint || *tHit > rayp.maxt)
		return false;

	//Vector oc = -Vector(rayp.o); // center at origin
	//float sqr_oc = Dot(oc, oc);

	//if (sqr_oc < rSquare)
	//	return false;

	//float t_ca = Dot(oc, rayp.d);
	//if (t_ca < 0.0)
	//	return false;

	//float sqr_hc = rSquare - sqr_oc + t_ca * t_ca;
	//if (sqr_hc < 0)
	//	return false;

	//*tHit = t_ca - sqrtf(sqr_hc);
	//if (*tHit < rayp.mint || *tHit > rayp.maxt)
	//	return false;

	local.p = rayp(*tHit);
	local.n = Normal(local.p.x, local.p.y,local.p.z);

	return true;
}

bool Sphere::intersectP(const Ray& ray)
{
	float hit; 	Localgeo loc;
	return intersect(ray, &hit, loc);
}

bool Triangle::intersect(const Ray& ray, float* tHit, Localgeo& local)
{
	// transform ray to local coordinate
	Transform inv = Inverse(*T);
	Ray rayp = inv(ray);

	Vector a = p0 - p1, b = p0 - p2, d = p0 - rayp.o;
	float m00 = a.x, m01 = b.x, m02 = rayp.d.x,
		  m10 = a.y, m11 = b.y, m12 = rayp.d.y,
		  m20 = a.z, m21 = b.z, m22 = rayp.d.z;
	float det = m00*(m11*m22 - m12*m21) - m01*(m10*m22 - m12*m20) + m02*(m10*m21 - m11*m20);
	if (det == 0.f)
		return false;
	float invDet = 1.f / det;
	float beta = invDet*(d.x*(m11*m22 - m12*m21) - d.y*(m01*m22 - m02*m21) + d.z*(m01*m12 - m02*m11));
	if (beta < 0.f)
		return false;
	float gamma = invDet*(-d.x*(m10*m22 - m12*m20) + d.y*(m00*m22 - m02*m20) - d.z*(m00*m12 - m02*m10));
	if (gamma < 0.f || beta + gamma > 1.f)
		return false;
	*tHit = invDet*(d.x*(m10*m21 - m11*m20) - d.y*(m00*m21 - m01*m20) + d.z*(m00*m11 - m01 * m10));
	if (*tHit < rayp.mint || *tHit > rayp.maxt)
		return false;

	local.p = rayp(*tHit);
	local.n = Normal(Cross(a, b));

	return true;
}

bool Triangle::intersectP(const Ray& ray)
{
	float hit; 	Localgeo loc;
	return intersect(ray, &hit, loc);
}

bool Mesh::intersect(const Ray& ray, float* tHit, Localgeo& local)
{
	// transform ray to local coordinate
	Transform inv = Inverse(*T);
	Ray rayp = inv(ray);

	float hit = FLOAT_INFINITY, tTmp = FLOAT_INFINITY;
	Localgeo locTmp;
	// intersection test
	for (unsigned int i = 0; i < tris->size(); i++)
	{
		if (!tris->at(i).intersect(*this, rayp, &tTmp, locTmp))
			continue;
		if (hit > tTmp)
		{
			hit = tTmp;
			local = locTmp;
		}
	}
	// no hits
	if (hit == FLOAT_INFINITY)
		return false;

	*tHit = hit;
	return true;
}

bool Mesh::intersectP(const Ray& ray)
{
	float hit; Localgeo loc;
	return intersect(ray, &hit, loc);
}

bool MeshTriangle::intersect(const Mesh& mesh, const Ray& ray, float* tHit, Localgeo& local)
{
	// transform ray to local coordinate
	Transform inv = Inverse(*(mesh.T));
	Ray rayp = inv(ray);

	Vector a = mesh.v->at(vert[0]-1) - mesh.v->at(vert[1]-1), b = mesh.v->at(vert[0]-1) - mesh.v->at(vert[2]-1), d = mesh.v->at(vert[0]-1) - Vector(rayp.o);
	float m00 = a.x, m01 = b.x, m02 = rayp.d.x,
		m10 = a.y, m11 = b.y, m12 = rayp.d.y,
		m20 = a.z, m21 = b.z, m22 = rayp.d.z;
	float det = m00*(m11*m22 - m12*m21) - m01*(m10*m22 - m12*m20) + m02*(m10*m21 - m11*m20);
	if (det == 0.f)
		return false;
	float invDet = 1.f / det;
	float beta = invDet*(d.x*(m11*m22 - m12*m21) - d.y*(m01*m22 - m02*m21) + d.z*(m01*m12 - m02*m11));
	if (beta < 0.f)
		return false;
	float gamma = invDet*(-d.x*(m10*m22 - m12*m20) + d.y*(m00*m22 - m02*m20) - d.z*(m00*m12 - m02*m10));
	if (gamma < 0.f || beta + gamma > 1.f)
		return false;
	*tHit = invDet*(d.x*(m10*m21 - m11*m20) - d.y*(m00*m21 - m01*m20) + d.z*(m00*m11 - m01 * m10));
	if (*tHit < rayp.mint || *tHit > rayp.maxt)
		return false;

	local.p = rayp(*tHit);
	if (mesh.n == NULL)
		local.n = Normal(Cross(a, b));
	else
	{
		float alpha = 1.f - beta - gamma;
		local.n = Normalize(alpha*mesh.n->at(norm[0] - 1) + beta*mesh.n->at(norm[1] - 1) + gamma*mesh.n->at(norm[2] - 1));
	}
	return true;
}

bool MeshTriangle::intersectP(const Mesh& mesh, const Ray& ray)
{
	float hit; 	Localgeo loc;
	return intersect(mesh, ray, &hit, loc);
}

int& MeshTriangle::v(unsigned int idx)
{
	if (idx > 3)
	{
		std::cerr << "Error: MeshTriangle::v() out of bound" << std::endl;
		exit(1);
	}
	return vert[idx];
}

int& MeshTriangle::n(unsigned int idx)
{
	if (idx > 3)
	{
		std::cerr << "Error: MeshTriangle::n() out of bound" << std::endl;
		exit(1);
	}
	return norm[idx];
}