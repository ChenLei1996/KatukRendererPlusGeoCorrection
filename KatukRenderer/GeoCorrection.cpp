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
const unsigned int steps = 10;
BezTree *quadBezTree;
int patchDrawLevel = 0;
//std::auto_ptr<BezTree> quadBezTree;

int NCP[] = { 9, 25, 81 };

// level2 grid reference points
cv::Point2f _rtGrids[] = { 
	cv::Point2f(193, 212), cv::Point2f(228, 206), cv::Point2f(266, 199), cv::Point2f(306, 193), cv::Point2f(346, 187), cv::Point2f(386, 183), cv::Point2f(430, 179), cv::Point2f(473, 177), cv::Point2f(518, 176),
	cv::Point2f(186, 244), cv::Point2f(221, 238), cv::Point2f(261, 232), cv::Point2f(302, 227), cv::Point2f(343, 224), cv::Point2f(384, 221), cv::Point2f(429, 219), cv::Point2f(474, 219), cv::Point2f(518, 221),
	cv::Point2f(179, 280), cv::Point2f(216, 276), cv::Point2f(257, 272), cv::Point2f(301, 271), cv::Point2f(343, 270), cv::Point2f(385, 270), cv::Point2f(431, 273), cv::Point2f(476, 274), cv::Point2f(520, 277),
	cv::Point2f(173, 319), cv::Point2f(212, 317), cv::Point2f(256, 316), cv::Point2f(301, 316), cv::Point2f(344, 318), cv::Point2f(387, 320), cv::Point2f(433, 323), cv::Point2f(478, 327), cv::Point2f(521, 330),
	cv::Point2f(169, 359), cv::Point2f(209, 358), cv::Point2f(254, 358), cv::Point2f(300, 360), cv::Point2f(345, 363), cv::Point2f(388, 366), cv::Point2f(435, 371), cv::Point2f(479, 374), cv::Point2f(522, 377),
	cv::Point2f(166, 401), cv::Point2f(208, 400), cv::Point2f(253, 401), cv::Point2f(300, 404), cv::Point2f(345, 408), cv::Point2f(389, 411), cv::Point2f(435, 414), cv::Point2f(480, 418), cv::Point2f(524, 422),
	cv::Point2f(164, 447), cv::Point2f(205, 445), cv::Point2f(252, 447), cv::Point2f(300, 451), cv::Point2f(345, 454), cv::Point2f(389, 457), cv::Point2f(436, 461), cv::Point2f(481, 464), cv::Point2f(524, 467),
	cv::Point2f(165, 494), cv::Point2f(204, 492), cv::Point2f(251, 495), cv::Point2f(299, 497), cv::Point2f(345, 501), cv::Point2f(389, 503), cv::Point2f(437, 507), cv::Point2f(481, 510), cv::Point2f(525, 512),
	cv::Point2f(170, 542), cv::Point2f(206, 540), cv::Point2f(251, 540), cv::Point2f(300, 543), cv::Point2f(346, 546), cv::Point2f(390, 548), cv::Point2f(437, 551), cv::Point2f(482, 554), cv::Point2f(525, 555)
};

cv::Point2f _texGrids[] = {
	cv::Point2f(0, 0),   cv::Point2f(49, 0),   cv::Point2f(99, 0),   cv::Point2f(151, 0),   cv::Point2f(199, 0),   cv::Point2f(247, 0),   cv::Point2f(298, 0),   cv::Point2f(349, 0),   cv::Point2f(399, 0),
	cv::Point2f(0, 49),  cv::Point2f(49, 49),  cv::Point2f(99, 49),  cv::Point2f(151, 49),  cv::Point2f(199, 49),  cv::Point2f(247, 49),  cv::Point2f(298, 49),  cv::Point2f(349, 49),  cv::Point2f(399, 49),
	cv::Point2f(0, 100), cv::Point2f(49, 100), cv::Point2f(99, 100), cv::Point2f(151, 100), cv::Point2f(199, 100), cv::Point2f(247, 100), cv::Point2f(298, 100), cv::Point2f(349, 100), cv::Point2f(399, 100),
	cv::Point2f(0, 150), cv::Point2f(49, 150), cv::Point2f(99, 150), cv::Point2f(151, 150), cv::Point2f(199, 150), cv::Point2f(247, 150), cv::Point2f(298, 150), cv::Point2f(349, 150), cv::Point2f(399, 150),
	cv::Point2f(0, 200), cv::Point2f(49, 200), cv::Point2f(99, 200), cv::Point2f(151, 200), cv::Point2f(199, 200), cv::Point2f(247, 200), cv::Point2f(298, 200), cv::Point2f(349, 200), cv::Point2f(399, 200),
	cv::Point2f(0, 248), cv::Point2f(49, 248), cv::Point2f(99, 248), cv::Point2f(151, 248), cv::Point2f(199, 248), cv::Point2f(247, 248), cv::Point2f(298, 248), cv::Point2f(349, 248), cv::Point2f(399, 248),
	cv::Point2f(0, 300), cv::Point2f(49, 300), cv::Point2f(99, 300), cv::Point2f(151, 300), cv::Point2f(199, 300), cv::Point2f(247, 300), cv::Point2f(298, 300), cv::Point2f(349, 300), cv::Point2f(399, 300),
	cv::Point2f(0, 349), cv::Point2f(49, 349), cv::Point2f(99, 349), cv::Point2f(151, 349), cv::Point2f(199, 349), cv::Point2f(247, 349), cv::Point2f(298, 349), cv::Point2f(349, 349), cv::Point2f(399, 349),
	cv::Point2f(0, 399), cv::Point2f(49, 399), cv::Point2f(99, 399), cv::Point2f(151, 399), cv::Point2f(199, 399), cv::Point2f(247, 399), cv::Point2f(298, 399), cv::Point2f(349, 399), cv::Point2f(399, 399)
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

// run geometric correction
void GeoCorrection::runCorrection(int level)
{
	// find reference points in ray-traced image(camera) and projection image(projector)
	
	/* assign reference points and display image */
	rtDetects.assign(_rtGrids, _rtGrids + 81);
	/*cv::Mat tmpDisplayGrid;
	rtImage.copyTo(tmpDisplayGrid);
	for (int i = 0; i < rtDetects.size(); i++)
	{
		circle(tmpDisplayGrid, rtDetects[i], 5, cv::Scalar(255.0, 255.0, .0, 255.0), 2, 8, 0);
	}
	cv::imshow("check reference points RT", tmpDisplayGrid);*/
	gridDetects.assign(_texGrids, _texGrids + 81);
	/*cv::Mat tmpDisplayTex;
	grid.copyTo(tmpDisplayTex);
	for (int i = 0; i < gridDetects.size(); i++)
	{
		circle(tmpDisplayTex, gridDetects[i], 5, cv::Scalar(255.0, 255.0, .0, 255.0), 2, 8, 0);
	}
	cv::imshow("check reference points TEX", tmpDisplayTex);
	cv::waitKey(0);*/

	//findGrids(RENDER, level);
	//findGrids(PROJECTION, level);

	// find homography
	int row = static_cast<int>(sqrt(rtDetects.size()));
	vector<cv::Point2f> projcorners;
	projcorners.push_back(gridDetects[0]);
	projcorners.push_back(gridDetects[row - 1]);
	projcorners.push_back(gridDetects[row*row-1]);
	projcorners.push_back(gridDetects[row*(row - 1)]);;
	/* debug */
	/*cv::Mat tmpDisplayGrid;
	grid.copyTo(tmpDisplayGrid);
	for (int i = 0; i < projcorners.size(); i++)
	{
		circle(tmpDisplayGrid, projcorners[i], 5, cv::Scalar(255.0, 255.0, .0, 255.0), 2, 8, 0);
	}
	cv::imshow("check reference points RT", tmpDisplayGrid); 
	cv::waitKey(0);*/
	vector<cv::Point2f> camcorners;
	camcorners.push_back(rtDetects[0]);
	camcorners.push_back(rtDetects[row - 1]);
	camcorners.push_back(rtDetects[row*row - 1]);
	camcorners.push_back(rtDetects[row*(row - 1)]);;
	/*cv::Mat H = cv::findHomography(projcorners, camcorners, 0);*/
	cv::Mat H = cv::findHomography(gridDetects, rtDetects, 0);
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
	/* debug */
	for (unsigned int i = 0; i < toProjector.size(); i++)
		std::cout << toProjector[i] << std::endl;
	
	genBernsVal(BernsVal, BernCoff, steps);
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
	std::vector<cv::Point2f>& target = (type == PROJECTION) ? gridDetects : rtDetects;
	cv::Mat& targetImage = (type == PROJECTION) ? grid : rtImage;
	
	cv::Mat gray(targetImage.size(), CV_8UC1);
	cv::cvtColor(targetImage, gray, CV_BGR2GRAY);
	cv::Mat filtered, harrisDst, dstNorm, dstScaled;
	cv::GaussianBlur(gray, filtered, cv::Size(5, 5), 0.0, 0.0);
	harrisDst = cv::Mat::zeros(gray.size(), CV_32FC1);
	int blockSize = 3;
	int apertureSize = 3;
	double k = 0.05;

	cv::cornerHarris(filtered, harrisDst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);

	// Normalizing
	cv::normalize(harrisDst, dstNorm, 0, 255, cv::NORM_MINMAX, CV_32FC1);
	cv::convertScaleAbs(dstNorm, dstScaled);
	// draw a circle around corners
	
	//cv::Mat copyOri;
	//image.copyTo(copyOri);
	float thresh = 110.f;
	vector<cv::Point2f> corners;

	for (int j = 0; j < dstNorm.rows; j++)
	{
		for (int i = 0; i < dstNorm.cols; i++)
		{
			if (dstNorm.at<float>(j, i) > thresh)
			{
				circle(dstScaled, cv::Point(i, j), 5, cv::Scalar(255.0, 255.0, .0, 255.0), 2, 8, 0);
				corners.push_back(cv::Point2f(i, j));
			}
		}
	}
	cv::imshow("feature Points", dstScaled);
	cv::waitKey(0);
	cv::Mat label, centers;
	// find corner points using kmeans algorithm and display

	//cv::flann::kd
	cv::kmeans(corners, NCP[level], label, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 15, 0.1), 100, cv::KMEANS_RANDOM_CENTERS, centers);

	// sorting
	sort(centers);
	std::cout << centers << std::endl;
	corners.clear();

	// corner
	target.push_back(centers.at<cv::Point2f>(0, 0));
	target.push_back(centers.at<cv::Point2f>(6, 0));
	target.push_back(centers.at<cv::Point2f>(8, 0));
	target.push_back(centers.at<cv::Point2f>(2, 0));
	// edge
	target.push_back(centers.at<cv::Point2f>(1, 0));
	target.push_back(centers.at<cv::Point2f>(3, 0));
	target.push_back(centers.at<cv::Point2f>(7, 0));
	target.push_back(centers.at<cv::Point2f>(5, 0));
	// middle
	target.push_back(centers.at<cv::Point2f>(4, 0));
}

GeoCorrection::~GeoCorrection()
{
	grid.release();
	rtImage.release();
	/*if (quadBezTree != NULL)
		delete quadBezTree;*/
}

void GeoCorrection::sort(cv::Mat& arr)
{
	// sorting
	for (unsigned int i = 1; i < 9; i++)
	{
		int j = i - 1;
		cv::Point2f target = arr.at<cv::Point2f>(i, 0);
		while (j >= 0 && target.y < arr.at<cv::Point2f>(j, 0).y)
		{
			arr.at<cv::Point2f>(j + 1, 0) = arr.at<cv::Point2f>(j, 0);
			j--;
		}
		arr.at<cv::Point2f>(j + 1, 0) = target;
	}
	for (unsigned int i = 1; i < 3; i++)
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
	for (unsigned int i = 4; i < 6; i++)
	{
		int j = i - 1;
		cv::Point2f target = arr.at<cv::Point2f>(i, 0);
		while (j >= 3 && target.x < arr.at<cv::Point2f>(j, 0).x)
		{
			arr.at<cv::Point2f>(j + 1, 0) = arr.at<cv::Point2f>(j, 0);
			j--;
		}
		arr.at<cv::Point2f>(j + 1, 0) = target;
	}
	for (unsigned int i = 7; i < 9; i++)
	{
		int j = i - 1;
		cv::Point2f target = arr.at<cv::Point2f>(i, 0);
		while (j >= 6 && target.x < arr.at<cv::Point2f>(j, 0).x)
		{
			arr.at<cv::Point2f>(j + 1, 0) = arr.at<cv::Point2f>(j, 0);
			j--;
		}
		arr.at<cv::Point2f>(j + 1, 0) = target;
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
	glOrtho(-20, gridX+20, -20, gridX+20, -1, 1);
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
		std::cout << " Pass Bezier patch to Ray-tracer" << std::endl;
		passedFromCv.create(cv::Size(gridX,gridY), CV_8UC3);
		glReadPixels(0, 0, gridX, gridY, GL_BGR, GL_UNSIGNED_BYTE, passedFromCv.data);
		cv::flip(passedFromCv, passedFromCv, 0);
		cv::imshow("passedFromCV", passedFromCv);
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