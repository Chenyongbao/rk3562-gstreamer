#include "yolo_preprocess.h"
#include <cmath>
#include <algorithm>

namespace Preprocess {

cv::Mat letterboxBGRtoRGB(const cv::Mat& bgr,
                          int targetW, int targetH,
                          LetterBox& lb,
                          cv::Scalar padColor) {
    float r = std::min((float)targetH / bgr.rows, (float)targetW / bgr.cols);
    int nw = (int)std::round(bgr.cols * r);
    int nh = (int)std::round(bgr.rows * r);

    lb.scale    = r;
    lb.resize_w = nw;
    lb.resize_h = nh;
    lb.x_pad    = (targetW - nw) / 2;
    lb.y_pad    = (targetH - nh) / 2;

    cv::Mat resized;
    if (bgr.cols != nw || bgr.rows != nh)
        cv::resize(bgr, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    else
        resized = bgr;

    int top    = (int)std::round((targetH - nh) * 0.5f - 0.1f);
    int bottom = (int)std::round((targetH - nh) * 0.5f + 0.1f);
    int left   = (int)std::round((targetW - nw) * 0.5f - 0.1f);
    int right  = (int)std::round((targetW - nw) * 0.5f + 0.1f);

    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, top, bottom, left, right,
                       cv::BORDER_CONSTANT, padColor);

    cv::Mat rgb;
    cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
    return rgb;
}

cv::Mat normalizeFloat32(const cv::Mat& rgb, float scale) {
    cv::Mat f32;
    rgb.convertTo(f32, CV_32FC3, scale);
    return f32;
}

} // namespace Preprocess
