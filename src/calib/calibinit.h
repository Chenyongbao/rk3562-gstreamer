#ifndef _CALIB_INIT_H_
#define _CALIB_INIT_H_

#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
int ReallinkFindChessboardCorners(InputArray image_, Size pattern_size,
                           OutputArray corners_, int flags);
int getCancelFlag();
void setCancelFlag(int cancel);
#endif