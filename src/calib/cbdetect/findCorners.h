#pragma once
#ifndef _CBDETECT_FIND_CORNERS_H_
#define _CBDETECT_FIND_CORNERS_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "config.h"

namespace ReallinkCB {

void find_corners(const Mat& img, Corner& corners,
                                       const Params& params = Params());

}

#endif
