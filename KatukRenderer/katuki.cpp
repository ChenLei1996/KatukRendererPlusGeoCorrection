#include "global.h"
#include "core/geometry.h"
#include "core/Transform.h"
#include "core/shape.h"
#include "core/Sampler.h"
#include "core/Camera.h"
#include "core/Scene.h"
#include "core/Light.h"
#include "descriptor.h"
#include "glheader.h"
#include "GeoCorrection.h"
#include <opencv2/highgui.hpp>
#include <GL/glew.h>
#include <GL/freeglut.h>

#pragma comment(lib,"glew32.lib")

extern float* intensity;
extern Scene* scene;
static cv::Mat passToCV;
cv::Mat passedFromCv;

const string windowName("Ray tracing with Geometric Correction");

int main(int argc, char* argv[])
{
	// Ray tracing initialization
	if (argc == 1)
		return 0;
	initScene(string(argv[1]));

	// GLUT initlization and run
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WINX, WINY);
	glutCreateWindow(windowName.c_str());
	glInit();
	glutKeyboardFunc(keyboard);
	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	glutMainLoop();
	
	// Release resources
	releaseScene();

	return 0;
}

void glInit()
{
	glewInit();
	glClearColor(0.0, 0.0, 0.0, 1.0);

	// use fast 4-byte alignment if possible
	// glPixelStorei(GL_PACK_ALIGNMENT, (copy2CV.step & 3) ? 1 : 4);
	// set length of one complete row in destination data (doesn't need to equal img.cols)
	// glPixelStorei(GL_UNPACK_ROW_LENGTH, copy2CV.step / copy2CV.elemSize());
}

void reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
}

void display()
{
	// link image array to the frame buffer object
	glClear(GL_COLOR_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	/*glDrawPixels(WINX, WINY, GL_RGB, GL_UNSIGNED_BYTE, rayTracedImage);*/
	glDrawPixels(WINX, WINY, GL_BGR, GL_FLOAT, intensity);
	glFlush();
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(1);
		break;

	case 's':
	case 'S':
		std::cout << " Ray tracing operated." << std::endl;
		scene->loop(intensity);
		break;

	case 'r':
	case 'R':
		// replace ProjectionLight with corrected image
		std::cout << " Replace projection light with the image passed from CV" << std::endl;
		scene->plight->switchProjection(passedFromCv);

		break;
	case 'c':
	case 'C':
		std::cout << "\n Pass rendering to geometric correction path." << std::endl;
		// calibration part
		passToCV.create(scene->w, scene->h, CV_8UC3);
		scene->writeImage(passToCV, intensity);
		cv::flip(passToCV, passToCV, 0);
		cv::imshow("passToCV", passToCV);
		GeoCorrection geoCorrection(*scene->plight->texture, passToCV);
		
		/*cv::Mat image = cv::imread("rendering-checker-lv2.jpg", cv::IMREAD_COLOR);
		GeoCorrection geoCorrection(*scene->plight->texture, image);*/
		geoCorrection.runCorrection(2);
		geoCorrection.initTexWindow();
		break;
	
	}
	glutPostRedisplay();
}