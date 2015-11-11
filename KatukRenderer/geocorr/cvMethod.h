#ifndef H_CV_METHOD
#define H_CV_METHOD

void runCVProcedure(const cv::Mat& image, cv::Mat& projection);
void findGrids(const cv::Mat& image, std::vector<cv::Point2f>& corners, std::vector<cv::Point2f>& edges, cv::Point2f& mid);
void sort(cv::Mat& arr);

#endif