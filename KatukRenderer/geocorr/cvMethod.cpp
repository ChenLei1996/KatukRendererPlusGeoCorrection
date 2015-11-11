#include <GL/glew.h>
#include <GL/freeglut.h>
#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/flann.hpp>
#include "cvMethod.h"
#include "bezfit.h"

using std::vector;

vector<cv::Point2f> cameraCorner;
vector<cv::Point2f> cameraEdge;
cv::Point2f cameraMid, projMid;
vector<cv::Point2f> projCorner, projEdge;
vector<cv::Mat> toProjector;
vector<cv::Point2f> bezControlPoints, projPoints;
cv::Mat passToCv;
cv::Mat passedFromCv;

void runCVProcedure(const cv::Mat& image,cv::Mat& projection)
{
	// initialize passedFromCv
	passedFromCv.create(projection.rows, projection.cols, CV_8UC3);
	cv::Mat temp1;
	findGrids(image, cameraCorner, cameraEdge, cameraMid);
	findGrids(projection, projCorner, projEdge, projMid);

	
	// find homography
	cv::Mat H = cv::findHomography(projCorner, cameraCorner, 0);
	std::cout << H << std::endl;

	cv::Mat invH = H.inv();
	for (unsigned int i = 0; i < cameraCorner.size(); i++)
		toProjector.push_back(invH*cv::Mat(cv::Point3d(cameraCorner[i].x,cameraCorner[i].y,1.0)));
	for (unsigned int i = 0; i < cameraEdge.size(); i++)
		toProjector.push_back(invH*cv::Mat(cv::Point3d(cameraEdge[i].x, cameraEdge[i].y, 1.0)));
	toProjector.push_back(invH*cv::Mat(cv::Point3d(cameraMid.x, cameraMid.y, 1.0)));
	
	// put all projector points in one vector
	projPoints.insert(projPoints.end(), projCorner.begin(), projCorner.end());
	projPoints.insert(projPoints.end(), projEdge.begin(), projEdge.end());
	projPoints.push_back(projMid);

	// divide by homogeneous coordinate
	for (unsigned int i = 0; i < toProjector.size(); i++)
	{
		toProjector[i] = toProjector[i] / toProjector[i].at<double>(2);
	}
	
	// Compute Bezier control points
	cv::Point2f p00(toProjector[1].at<double>(0), toProjector[1].at<double>(1)),
		p02(toProjector[2].at<double>(0), toProjector[2].at<double>(1)),
		p01 = estimateControlPoint(p00, p02, cv::Point2f(toProjector[6].at<double>(0), toProjector[6].at<double>(1)), 0.5),
		p20(toProjector[0].at<double>(0), toProjector[0].at<double>(1)),
		p22(toProjector[3].at<double>(0), toProjector[3].at<double>(1)),
		p21 = estimateControlPoint(p20, p22, cv::Point2f(toProjector[4].at<double>(0), toProjector[4].at<double>(1)), 0.5),
		p10 = estimateControlPoint(p00, p20, cv::Point2f(toProjector[5].at<double>(0), toProjector[5].at<double>(1)), 0.5),
		p12 = estimateControlPoint(p02, p22, cv::Point2f(toProjector[7].at<double>(0), toProjector[7].at<double>(1)), 0.5),
		p11 = estimateControlPoint(p10, p12, cv::Point2f(toProjector[8].at<double>(0), toProjector[8].at<double>(1)), 0.5);

	/*std::cout << p20 << std::endl << p00 << std::endl << p02 << std::endl <<
		p22 << std::endl << p21 << std::endl << p10 << std::endl <<
		p01 << std::endl << p12 << std::endl << p11 << std::endl;
	
	for (unsigned int i = 0; i < projCorner.size(); i++)
		std::cout << projCorner[i] << std::endl;
	for (unsigned int i = 0; i < projEdge.size(); i++)
		std::cout << projEdge[i] << std::endl;
	std::cout << projMid << std::endl;*/

	bezControlPoints.push_back(2.0f*projCorner[1] - p00);
	bezControlPoints.push_back(2.0f*projEdge[2] - p01);
	bezControlPoints.push_back(2.0f*projCorner[2] - p02);
	bezControlPoints.push_back(2.0f*projEdge[1] - p10);
	bezControlPoints.push_back(2.0f*projMid - p11);
	bezControlPoints.push_back(2.0f*projEdge[3] - p12);
	bezControlPoints.push_back(2.0f*projCorner[0] - p20);
	bezControlPoints.push_back(2.0f*projEdge[0] - p21);
	bezControlPoints.push_back(2.0f*projCorner[3] - p22);
	
	for (unsigned int i = 0; i < bezControlPoints.size(); i++)
		std::cout << bezControlPoints[i] << std::endl;
	
	bezInit();
		
}

// so far we only consider level 0 bezier patch!!
void findGrids(const cv::Mat& image, std::vector<cv::Point2f>& corners, std::vector<cv::Point2f>& edges, cv::Point2f& mid)
{
	cv::Mat gray(image.size(), CV_8UC1);
	cv::cvtColor(image, gray, CV_BGR2GRAY);
	cv::Mat filtered,harrisDst, dstNorm, dstScaled;
	cv::GaussianBlur(gray, filtered, cv::Size(3, 3), 1.0, 0.0);
	harrisDst = cv::Mat::zeros(gray.size(), CV_32FC1);
	int blockSize = 2;
	int apertureSize = 3;
	double k = 0.04;
	
	cv::cornerHarris(filtered, harrisDst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);

	// Normalizing
	cv::normalize(harrisDst, dstNorm, 0, 255, cv::NORM_MINMAX, CV_32FC1);
	cv::convertScaleAbs(dstNorm, dstScaled);
	// draw a circle around corners

	//cv::Mat copyOri;
	//image.copyTo(copyOri);
	int thresh = 165;
	for (int j = 0; j < dstNorm.rows; j++)
	{
		for (int i = 0; i < dstNorm.cols; i++)
		{
			if ((int)dstNorm.at<float>(j, i) > thresh)
			{
				circle(dstScaled , cv::Point(i, j), 5, cv::Scalar(255.0, 255.0, .0, 255.0), 2, 8, 0);
				corners.push_back(cv::Point2f(i, j));
			}
		}
	}
	cv::imshow("feature Points", dstScaled);
	cv::waitKey(0);
	cv::Mat label, centers;
	// find corner points using kmeans algorithm and display

	//cv::flann::kd
	cv::kmeans(corners, 9, label, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 15, 0.1), 100, cv::KMEANS_RANDOM_CENTERS, centers);
	
	// sorting
	sort(centers);
	std::cout << centers << std::endl;
	corners.clear();
	
	corners.push_back(centers.at<cv::Point2f>(0,0));
	corners.push_back(centers.at<cv::Point2f>(6,0));
	corners.push_back(centers.at<cv::Point2f>(8,0));
	corners.push_back(centers.at<cv::Point2f>(2,0));

	edges.push_back(centers.at<cv::Point2f>(1,0));
	edges.push_back(centers.at<cv::Point2f>(3,0));
	edges.push_back(centers.at<cv::Point2f>(7,0));
	edges.push_back(centers.at<cv::Point2f>(5,0));

	mid = centers.at<cv::Point2f>(4,0);
	
}

void sort(cv::Mat& arr)
{
	// sorting
	for (unsigned int i = 1; i < 9; i++)
	{
		int j = i - 1;
		cv::Point2f target = arr.at<cv::Point2f>(i,0);
		while (j >= 0 && target.y < arr.at<cv::Point2f>(j,0).y)
		{
			arr.at<cv::Point2f>(j + 1,0) = arr.at<cv::Point2f>(j,0);
			j--;
		}
		arr.at<cv::Point2f>(j + 1,0) = target;
	}
	for (unsigned int i = 1; i < 3; i++)
	{
		int j = i - 1;
		cv::Point2f target = arr.at<cv::Point2f>(i,0);
		while (j >= 0 && target.x < arr.at<cv::Point2f>(j,0).x)
		{
			arr.at<cv::Point2f>(j + 1,0) = arr.at<cv::Point2f>(j,0);
			j--;
		}
		arr.at<cv::Point2f>(j + 1,0) = target;
	}
	for (unsigned int i = 4; i < 6; i++)
	{
		int j = i - 1;
		cv::Point2f target = arr.at<cv::Point2f>(i,0);
		while (j >= 3 && target.x < arr.at<cv::Point2f>(j,0).x)
		{
			arr.at<cv::Point2f>(j + 1,0) = arr.at<cv::Point2f>(j,0);
			j--;
		}
		arr.at<cv::Point2f>(j + 1,0) = target;
	}
	for (unsigned int i = 7; i < 9; i++)
	{
		int j = i - 1;
		cv::Point2f target = arr.at<cv::Point2f>(i,0);
		while (j >= 6 && target.x < arr.at<cv::Point2f>(j,0).x)
		{
			arr.at<cv::Point2f>(j + 1,0) = arr.at<cv::Point2f>(j,0);
			j--;
		}
		arr.at<cv::Point2f>(j + 1,0) = target;
	}
}