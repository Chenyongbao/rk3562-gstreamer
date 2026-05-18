
#pragma once
#ifndef LIBCBDETECT_PLOT_DELTILLES_H
#define LIBCBDETECT_PLOT_DELTILLES_H

#include "config.h"
namespace ReallinkCB {

LIBCBDETECT_DLL_DECL void plot_boards(const cv::Mat& img, const Corner& corners,
                                      const std::vector<Board>& boards, const Params& params);

}

#endif //LIBCBDETECT_PLOT_DELTILLES_H
