#ifndef _REALLINK_YOLO_PREPROCESS_H_
#define _REALLINK_YOLO_PREPROCESS_H_

#include "yolo_common.h"

namespace Preprocess {

cv::Mat letterboxBGRtoRGB(const cv::Mat& bgr,
                          int targetW, int targetH,
                          LetterBox& lb,
                          cv::Scalar padColor = cv::Scalar(114, 114, 114));

cv::Mat normalizeFloat32(const cv::Mat& rgb, float scale = 1.0f / 255.0f);

} // namespace Preprocess

#endif
