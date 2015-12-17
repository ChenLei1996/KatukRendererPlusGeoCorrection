#include <GL/glew.h>
#include <GL/freeglut.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/flann.hpp>
#include "../QuadBezLib/QuadBezier.h"
#include "GeoCorrection.h"
#include "BezTree.h"
#include <memory>

using std::cout;
using std::endl;
using std::vector;

int GeoCorrection::gridX;
int GeoCorrection::gridY;
int GeoCorrection::rtX;
int GeoCorrection::rtY;
extern cv::Mat passedFromCv;

QuadBezierPatch2f surface;
const Matrix3f BernCoff(1.0, -2.0, 1.0, 0.0, 2.0, -2.0, 0.0, 0.0, 1.0);
vector<Vector3f> BernsVal;
vector<Vector2f> BezPatch;
vector<vector<unsigned>> bezPatchIdx;
vector<cv::Point2f> bezControlPoints;
unsigned int steps;
BezTree *quadBezTree;
int patchDrawLevel = 0;

cv::Point2f _trfReference[] = {
	//cv::Point2f(198, 218), cv::Point2f(508, 183), cv::Point2f(174, 532), cv::Point2f(514, 545)
	cv::Point2f(0, 0), cv::Point2f(310, -35), cv::Point2f(-24, 314), cv::Point2f(316, 327)
};

cv::Point2f _squareReference[] = {
	//cv::Point2f(198, 218), cv::Point2f(508, 218), cv::Point2f(198, 532), cv::Point2f(508, 532)
	cv::Point2f(0, 0), cv::Point2f(310, 0), cv::Point2f(0, 314), cv::Point2f(310, 314)
};

GLuint tex;

GeoCorrection::GeoCorrection(const cv::Mat _grid, const cv::Mat _rtImage, int _lv)
	: subdivLv(_lv), corrected(false), gridDetects(), rtDetects()
{
	_grid.copyTo(grid);
	_rtImage.copyTo(rtImage);
	gridX = grid.cols;
	gridY = grid.rows;
	rtX = rtImage.cols;
	rtY = rtImage.rows;

}

void GeoCorrection::updateImages(const Mat& _grid, const Mat& _rtImage)
{
	grid = _grid;
	rtImage = _rtImage;
	gridX = grid.cols;
	gridY = grid.rows;
	rtX = rtImage.cols;
	rtY = rtImage.rows;
}

// run geometric correction
void GeoCorrection::runCorrection(int level)
{
	// clear detected points
	rtDetects.clear();
	gridDetects.clear();

	// find reference points in ray-traced image(camera) and projection image(projector)
	findGrids(RENDER, level);
	findGrids(PROJECTION, level);
	cout << "Detected points in camera: " << rtDetects.size() << endl
		<< "Detected points in projection: " << gridDetects.size() << endl;

	// find homography
	int row = static_cast<int>(sqrt(rtDetects.size()));
	vector<cv::Point2f> projcorners;
	
	projcorners.push_back(gridDetects[0]);
	projcorners.push_back(gridDetects[row - 1]);
	projcorners.push_back(gridDetects[row*row-1]);
	projcorners.push_back(gridDetects[row*(row - 1)]);;

	vector<cv::Point2f> camcorners;
	camcorners.push_back(rtDetects[0]);
	camcorners.push_back(rtDetects[row - 1]);
	camcorners.push_back(rtDetects[row*row - 1]);
	camcorners.push_back(rtDetects[row*(row - 1)]);;
	
	cv::Mat H = cv::findHomography(projcorners, camcorners, 0);
	std::cout << H << std::endl;
	
	// apply homography to move camera points to projector space
	cv::Mat invH = H.inv();
	vector<cv::Mat> toProjector;
	for (unsigned int i = 0; i < rtDetects.size(); i++)
	{
		cv::Mat tmp(3, 1, CV_64F);	// CV_64F
		tmp.at<double>(0, 0) = static_cast<double>(rtDetects[i].x);
		tmp.at<double>(1, 0) = static_cast<double>(rtDetects[i].y);
		tmp.at<double>(2, 0) = 1.0;
		tmp = invH*tmp;
		tmp = tmp / tmp.at<double>(2, 0);
		toProjector.push_back(tmp);
	}

	steps = row - 1;
	if (BernsVal.size() == 0)
		genBernsVal(BernsVal, BernCoff, steps);
	if (quadBezTree)
	{
		cout << "Delete quadBezTree and initialize new quadBezTree" << endl;
		delete quadBezTree;

	}
	quadBezTree = new BezTree(toProjector, gridDetects, level);
	
	corrected = true;
}

// after geometric correction is done,
// produce corrected image and pass to ray tracer
void GeoCorrection::bakeCorrection(const cv::Mat& source, cv::Mat& destination)
{
	if (!corrected)
	{
		std::cerr << "Geometric correction should be done to call this." << std::endl;
		return;
	}

}

// find reference points in specified type
void GeoCorrection::findGrids(Geotype type, int level)
{
	int row = pow(2, level + 1) + 1;
	int numCorners = row * row;

	std::vector<cv::Point2f>& target = (type == PROJECTION) ? gridDetects : rtDetects;
	cv::Mat& targetImage = (type == PROJECTION) ? grid : rtImage;
	
	cv::Mat gray(targetImage.size(), CV_8UC1);
	cv::cvtColor(targetImage, gray, CV_BGR2GRAY);
	cv::Mat filtered, harrisDst, dstNorm, dstScaled;
	cv::GaussianBlur(gray, filtered, cv::Size(5, 5), 0.0, 0.0);
	/*harrisDst = cv::Mat::zeros(gray.size(), CV_32FC1);
	int blockSize = 3;
	int apertureSize = 3;
	double k = 0.05;*/
	
	cv::Mat m_corners;
	cv::goodFeaturesToTrack(filtered, m_corners, numCorners, 0.01, 15.0, cv::noArray(), 5);
	cv::Mat cornerDisplayed;
	targetImage.copyTo(cornerDisplayed);
	for (std::size_t i = 0; i < numCorners; i++)
	{
		//std::cout << m_corners.at<cv::Point2f>(i, 0) << std::endl;
		circle(cornerDisplayed, m_corners.at<cv::Point2f>(i, 0), 5, cv::Scalar(255.0, 255.0, .0, 255.0), 2, 8, 0);
	}
	std::string winTitle = (type == PROJECTION) ? "Projector": "Camera";
	cv::imshow(winTitle, cornerDisplayed);
	cv::waitKey(0);

	// change Y coordinate for OpenGL
	for (std::size_t i = 0; i < numCorners; i++)
	{
		m_corners.at<cv::Point2f>(i, 0).y = static_cast<float>(targetImage.rows - 1) - m_corners.at<cv::Point2f>(i, 0).y;
	}
		
	sort(m_corners, level);
	for (std::size_t i = 0; i < numCorners; i++)
		target.push_back(m_corners.at<cv::Point2f>(i, 0));
	//std::cout << m_corners << std::endl;

}

GeoCorrection::~GeoCorrection()
{
	grid.release();
	rtImage.release();
}

void GeoCorrection::sort(cv::Mat& arr, int level)
{
	int row = pow(2, level + 1) + 1;
	// sorting
	for (unsigned int i = 1; i < row*row; i++)
	{
		int j = i - 1;
		cv::Point2f target = arr.at<cv::Point2f>(i, 0);
		while (j >= 0 && target.x < arr.at<cv::Point2f>(j, 0).x)
		{
			arr.at<cv::Point2f>(j + 1, 0) = arr.at<cv::Point2f>(j, 0);
			j--;
		}
		arr.at<cv::Point2f>(j + 1, 0) = target;
	}

	for (unsigned int i = 0; i < row; i++)
	{
		for (unsigned int k = i*row+1; k < i*row + row; k++)
		{
			int j = k - 1;
			int lim = row*i;
			cv::Point2f target = arr.at<cv::Point2f>(k, 0);
			while ((j >= lim)
				&& (target.y < arr.at<cv::Point2f>(j, 0).y))
			{
				arr.at<cv::Point2f>(j + 1, 0) = arr.at<cv::Point2f>(j, 0);
				j--;
			}
			arr.at<cv::Point2f>(j + 1, 0) = target;
		}
	}
}

void GeoCorrection::initTexWindow()
{
	// opengl initialization!
	glutInitWindowSize(gridX, gridY);
	glutCreateWindow("Bezier patch warping");
	glutSetOption(GLUT_RENDERING_CONTEXT, GLUT_CREATE_NEW_CONTEXT);

	//glPolygonMode(GL_FRONT, GL_LINE);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, grid.step / grid.elemSize());
	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, grid.rows, grid.cols, 0, GL_BGR, GL_UNSIGNED_BYTE, grid.ptr());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D, 0);

	glutDisplayFunc(GeoCorrection::bezfitDisplay);
	glutKeyboardFunc(GeoCorrection::bezfitKeyboard);
}

// opengl functions
void GeoCorrection::bezfitDisplay()
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glOrtho(-10, gridX+10, -10, gridX+10, -1, 1);
	cv::Size mins, maxs;
	quadBezTree->root->getMinMax(mins, maxs);
	cout << "quadBezTree::root::getMinMax: " << mins << maxs << endl;
	
	glOrtho(mins.width, maxs.width, mins.height, maxs.height, -1, 1);
	//glOrtho(0, gridX, 0, gridX, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	std::vector<unsigned> tmpIdx;
	double s, t;
	unsigned int rows = steps + 1;
	glBindTexture(GL_TEXTURE_2D, tex);
	quadBezTree->draw(patchDrawLevel);
	glBindTexture(GL_TEXTURE_2D, 0);

	glutSwapBuffers();
}

void GeoCorrection::bezfitKeyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(0);
		break;

	case '0':
		patchDrawLevel = 0;
		break;
	case '1':
		patchDrawLevel = 1;
		break;
	case '2':
		patchDrawLevel = 2;
		break;

	case 'd':
	case 'D':
		// bezier patch subdivide
		std::cout << " Subdivide Bezier Patch " << std::endl;
		quadBezTree->subdivide();
		break;

	case 'p':
	case 'P':
		// runCorrection
		cv::Mat bezPatchMat;
		std::cout << " Pass Bezier patch to Ray-tracer" << std::endl;
		bezPatchMat.create(cv::Size(gridX, gridY), CV_8UC3);
		glReadPixels(0, 0, gridX, gridY, GL_BGR, GL_UNSIGNED_BYTE, bezPatchMat.data);
		cv::flip(bezPatchMat, bezPatchMat, 0);
		cv::imshow("before warping", bezPatchMat);
		// _trfReference, _squareReference
		/*cv::Mat lastTransform = cv::getPerspectiveTransform(_squareReference, _trfReference);
		std::cout << lastTransform << std::endl;
		cv::warpPerspective(bezPatchMat, passedFromCv, lastTransform, cv::Size(gridX, gridY), CV_WARP_INVERSE_MAP);
		cv::imshow("passedFromCV", passedFromCv);*/
		bezPatchMat.copyTo(passedFromCv);
		break;
	}
	glutPostRedisplay();
}

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