#ifndef H_GEOMETRIC_CORRECTION
#define H_GEOMETRIC_CORRECTION
#include <vector>

class Vector3f;
class Matrix3f;

class GeoCorrection
{
public:
	enum Geotype{PROJECTION, RENDER };
	GeoCorrection() : grid(), rtImage(), subdivLv(0), corrected(false), gridDetects(), rtDetects(){}
	GeoCorrection(const cv::Mat _grid, const cv::Mat _rtImage, int _lv = 0);
	~GeoCorrection();
	void runCorrection(int level);
	void bakeCorrection(const cv::Mat& source, cv::Mat& destination);

	// opengl functions
	void initTexWindow();
	
	static int gridX, gridY, rtX, rtY;
private:
	static void bezfitDisplay();
	static void bezfitKeyboard(unsigned char key, int x, int y);

	void findGrids(Geotype type, int level);
	void sort(cv::Mat& arr, int level);

	cv::Mat grid, rtImage;
	int subdivLv;
	bool corrected;
	std::vector<cv::Point2f> gridDetects, rtDetects; // grids found in projection image(projector) and ray tracing image(camera)
};

void genBernsVal(std::vector<Vector3f>& container, const Matrix3f& coffMat, unsigned int steps);
//inline cv::Point2f estimateControlPoint(const cv::Point2f& cp0, const cv::Point2f& cp2, const cv::Point2f& curvePoint, double t)
//{
//	double X, Y;
//	double diff = 1.0 - t;
//	double sqrT = t*t, sqrDiff = diff*diff;
//	X = (curvePoint.x - sqrDiff*cp0.x - sqrT*cp2.x) / (2 * diff*t);
//	Y = (curvePoint.y - sqrDiff*cp0.y - sqrT*cp2.y) / (2 * diff*t);
//
//	return cv::Point2f(X, Y);
//}

#endif