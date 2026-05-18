#ifndef _SADDLE_FIT_H_
#define _SADDLE_FIT_H_
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

vector<int> ReallinkSaddlePointFit(const Mat& gray, vector<Point2f>& corners, int r = 10);

#endif