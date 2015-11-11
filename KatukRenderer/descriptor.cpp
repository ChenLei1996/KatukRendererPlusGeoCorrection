#include "global.h"
#include "core/geometry.h"
#include "core/Transform.h"
#include "core/shape.h"
#include "core/Sampler.h"
#include "core/Camera.h"
#include "core/Scene.h"
#include "core/Light.h"
#include "descriptor.h"
#include "objloader.h"

// global variables
const int WINX = 800, WINY = 800;
float* intensity;
vector<Shape*> objList;
vector<Transform*> trfList;
Camera* camera;
Sampler* sampler;
Scene* scene;

// obj name
const string objName("nurbs_surface4.obj");
vector<Vector> vertices, texcoord;
vector<Normal> normals;
vector<MeshTriangle> mesh;
Mesh cube;

// initilize scene
// load obj files, set lights, camera
void initScene(const std::string& textureName)
{
	// for loading triangle mesh
	Matrix4x4 matrix;
	loadObj(objName.c_str(), vertices, normals, texcoord, mesh);
	matrix.m[2][3] = -9.f;
	matrix.m[1][3] = 0.f;

	Transform* tr = new Transform(matrix);
	trfList.push_back(tr);
	Mesh* newmesh = new Mesh(tr, &vertices, &normals, &mesh);
	objList.push_back(newmesh);

	// object initialize, for sphere
	/*Matrix4x4 mat;
	mat.m[2][3] = -16.f;
	Transform* tr2 = new Transform(mat);
	trfList.push_back(tr2);
	Sphere *sphere1 = new Sphere(tr2, 6.f);
	objList.push_back(sphere1);*/

	// initialize image plane
	intensity = new float[WINX * WINY * 3];
	memset(intensity, 0, WINX*WINY * 3 * sizeof(float));

	// set up the camera
	Point camCenter, camLookAt(0.f, 0.f, -1.f);
	Vector camUp(0.f, 1.f, 0.f);
	float fovy = 60.f;
	camera = new PerspectiveCamera(camCenter, camLookAt, camUp, fovy);

	// set up the projectionLight
	Point e(-1.5f, 3.f, 15.f), gaze(-1.2f, 1.3f, 0.f);
	Vector up(0.f, 0.9f, 0.43589f);
	/* For sphere ray tracing test */
	/*Point e(0.f, 0.f, 7.f), gaze(.0f, 0.f, -1.f);
	Vector up(0.f, 1.f, 0.f);*/
	Transform light2world = LookAt(e, gaze, up);
	
	//string texname("grid.jpg");
	float projFovy = 20.0;
	ProjectionLight *projector = new ProjectionLight(Inverse(light2world), textureName, projFovy);

	// set up the sampler
	sampler = new SimpleSampler(WINX, WINY);

	scene = new Scene(sampler, camera, &objList, projector, WINX, WINY);
}

void releaseScene()
{
	// release transformations and objects
	for (unsigned int i = 0; i < trfList.size(); i++)
		delete trfList[i];
	for (unsigned int i = 0; i < objList.size(); i++)
		delete objList[i];
	delete scene;
	delete sampler;
	delete camera;
	delete[] intensity;
}