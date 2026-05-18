
#pragma once
#ifndef LIBCBDETECT_PLOT_CORNERS_H
#define LIBCBDETECT_PLOT_CORNERS_H

#include "config.h"

namespace ReallinkCB {

LIBCBDETECT_DLL_DECL void plot_corners(const cv::Mat& img, const std::vector<cv::Point2d>& corners, const char* str);

LIBCBDETECT_DLL_DECL void plot_corners(const cv::Mat& img, const Corner& corners);

} // namespace cbdetect

#endif //LIBCBDETECT_PLOT_CORNERS_H
