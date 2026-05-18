#pragma once
#ifndef _CBDETECT_BOARD_H_
#define _CBDETECT_BOARD_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "config.h"

namespace ReallinkCB {

void boards_from_corners(const Mat& img, const Corner& corners,
                                              std::vector<Board>& boards, const Params& params);

}

#endif
