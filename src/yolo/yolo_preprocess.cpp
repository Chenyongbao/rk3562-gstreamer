#include "yolo_preprocess.h"
#include <cmath>
#include <algorithm>

namespace Preprocess {

// 将 BGR 图像等比缩放并填充至目标尺寸，并转为 RGB 格式
cv::Mat letterboxBGRtoRGB(const cv::Mat& bgr,
                          int targetW, int targetH,
                          LetterBox& lb,
                          cv::Scalar padColor) {
    // 计算缩放比例 (取宽高的最小缩放比以保证完整显示)
    float r = std::min((float)targetH / bgr.rows, (float)targetW / bgr.cols);
    int nw = (int)std::round(bgr.cols * r);
    int nh = (int)std::round(bgr.rows * r);

    // 记录 LetterBox 参数
    lb.scale    = r;
    lb.resize_w = nw;
    lb.resize_h = nh;
    lb.x_pad    = (targetW - nw) / 2;
    lb.y_pad    = (targetH - nh) / 2;

    // 等比缩放
    cv::Mat resized;
    if (bgr.cols != nw || bgr.rows != nh)
        cv::resize(bgr, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    else
        resized = bgr;

    // 缩放后正好等于目标尺寸（如 BEV 1280×1280 → 640×640 方形等比），
    // 无需填充，直接转色返回。
    if (nw == targetW && nh == targetH) {
        cv::Mat rgb;
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
        return rgb;
    }

    // 非等比输入才需要 letterbox 填充（通用路径）
    int top    = (int)std::round((targetH - nh) * 0.5f - 0.1f);
    int bottom = (int)std::round((targetH - nh) * 0.5f + 0.1f);
    int left   = (int)std::round((targetW - nw) * 0.5f - 0.1f);
    int right  = (int)std::round((targetW - nw) * 0.5f + 0.1f);

    // 添加边界填充
    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, top, bottom, left, right,
                       cv::BORDER_CONSTANT, padColor);

    // 转换颜色空间：BGR -> RGB
    cv::Mat rgb;
    cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
    return rgb;
}

} // namespace Preprocess
