#include <GL/glew.h>
#include <GL/freeglut.h>
#include <opencv2/core.hpp>
#include <iostream>
#include <vector>
#include "../core/geometry.h"
#include "../core/Transform.h"
#include "../core/shape.h"
#include "../core/Sampler.h"
#include "../core/Camera.h"
#include "../core/Light.h"
#include "../core/Scene.h"
#include "../descriptor.h"
#include "QuadBezier.h"
#include "bezfit.h"

using std::vector;

QuadBezierPatch2f surface;
const Matrix3f BernCoff(1.0, -2.0, 1.0, 0.0, 2.0, -2.0, 0.0, 0.0, 1.0);
vector<Vector3f> BernsVal;
vector<Vector2f> BezPatch;
vector<vector<unsigned>> bezPatchIdx;

int PROJWINX = 400, PROJWINY = 400;
const unsigned int steps = 10;

GLuint tex; // texture
cv::Mat outputImage;

// from cvMethod.cpp
extern vector<cv::Point2f> bezControlPoints;
extern cv::Mat passedFromCv;

// from descriptor.cpp
extern Scene* scene;

void genBernsVal(std::vector<Vector3f>& container, const Matrix3f& coffMat, unsigned int steps)
{
	Vector3f tmp;
	tmp.x = 1.0;
	double u;
	for (unsigned int i = 1; i <= steps; i++)
	{
		u = i / (double)steps;
		tmp.y = u;
		tmp.z = u*u;
		container.push_back(coffMat * tmp);
	}
}

void bezInit()
{
	cv::Mat texmat = *scene->plight->texture;
	
	// estimate control points
	// especially edge and mid points
	Vector2f p00(bezControlPoints[0].x, bezControlPoints[0].y),
		p01(bezControlPoints[1].x, bezControlPoints[1].y),
		p02(bezControlPoints[2].x, bezControlPoints[2].y),
		p10(bezControlPoints[3].x, bezControlPoints[3].y),
		p11(bezControlPoints[4].x, bezControlPoints[4].y),
		p12(bezControlPoints[5].x, bezControlPoints[5].y),
		p20(bezControlPoints[6].x, bezControlPoints[6].y),
		p21(bezControlPoints[7].x, bezControlPoints[7].y),
		p22(bezControlPoints[8].x, bezControlPoints[8].y);
		
	surface = QuadBezierPatch2f(p00,p01,p02,p10,p11,p12,p20,p21,p22);
	genBernsVal(BernsVal, BernCoff, steps);

	// opengl initialization!
	glutInitWindowSize(PROJWINX, PROJWINY);
	glutCreateWindow("Bezier patch warping");
	glutSetOption(GLUT_RENDERING_CONTEXT, GLUT_CREATE_NEW_CONTEXT);

	//glPolygonMode(GL_FRONT, GL_LINE);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, texmat.step / texmat.elemSize());
	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texmat.rows, texmat.cols, 0, GL_BGR, GL_UNSIGNED_BYTE, texmat.ptr());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D, 0);

	glutDisplayFunc(bezfitDisplay);
	glutKeyboardFunc(bezfitKeyboard);
}

void bezfitDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, PROJWINX, 0, PROJWINY, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	BezPatch.clear();
	bezPatchIdx.clear();
	surface.interpolate(BezPatch, bezPatchIdx, BernsVal, 0, steps);
	std::vector<unsigned> tmpIdx;
	double s, t;
	unsigned int rows = steps + 1;
	glBindTexture(GL_TEXTURE_2D, tex);
	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < bezPatchIdx.size(); i++)
	{

		tmpIdx = bezPatchIdx[i];
		s = (tmpIdx[0] / rows) / (double)steps;
		t = (tmpIdx[0] % rows) / (double)steps;
		glTexCoord2d(s, t);
		glVertex2d(BezPatch[tmpIdx[0]].x, BezPatch[tmpIdx[0]].y);
		s = (tmpIdx[1] / rows) / (double)steps;
		t = (tmpIdx[1] % rows) / (double)steps;
		glTexCoord2d(s, t);
		glVertex2d(BezPatch[tmpIdx[1]].x, BezPatch[tmpIdx[1]].y);
		s = (tmpIdx[2] / rows) / (double)steps;
		t = (tmpIdx[2] % rows) / (double)steps;
		glTexCoord2d(s, t);
		glVertex2d(BezPatch[tmpIdx[2]].x, BezPatch[tmpIdx[2]].y);
	}
	glEnd();
	glBindTexture(GL_TEXTURE_2D, 0);

	glutSwapBuffers();
}

void bezfitKeyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(1);
		break;

	case 'p':
	case 'P':
		glReadPixels(0, 0, PROJWINX, PROJWINY, GL_BGR, GL_UNSIGNED_BYTE, passedFromCv.data);
		scene->plight->texture->release();
		passedFromCv.copyTo(*scene->plight->texture);

		break;
	}
}