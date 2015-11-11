#ifndef H_BEZIER_FITTING
#define H_BEZIER_FITTING
#include "QuadBezier.h"

void genBernsVal(std::vector<Vector3f>& container, const Matrix3f& coffMat, unsigned int steps);
void bezInit();
inline Vector2f estimateControlPoint(const Vector2f& cp0, const Vector2f& cp2, const Vector2f& curvePoint, double t)
{
	double X, Y;
	double diff = 1.0 - t;
	double sqrT = t*t, sqrDiff = diff*diff;
	X = (curvePoint.x - sqrDiff*cp0.x - sqrT*cp2.x) / (2 * diff*t);
	Y = (curvePoint.y - sqrDiff*cp0.y - sqrT*cp2.y) / (2 * diff*t);

	return Vector2f(X, Y);
}

inline cv::Point2d estimateControlPoint(const cv::Point2d& cp0, const cv::Point2d& cp2, const cv::Point2d& curvePoint, double t)
{
	double X, Y;
	double diff = 1.0 - t;
	double sqrT = t*t, sqrDiff = diff*diff;
	X = (curvePoint.x - sqrDiff*cp0.x - sqrT*cp2.x) / (2 * diff*t);
	Y = (curvePoint.y - sqrDiff*cp0.y - sqrT*cp2.y) / (2 * diff*t);

	return cv::Point2d(X, Y);
}

inline Vector2f matToVec(const cv::Mat& src)
{
	return Vector2f(src.at<double>(0), src.at<double>(1));
}

void bezfitDisplay();
void bezfitKeyboard(unsigned char key, int x, int y);

#endif