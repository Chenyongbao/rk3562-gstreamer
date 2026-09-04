#ifndef _REALLINK_YOLO_PREPROCESS_H_
#define _REALLINK_YOLO_PREPROCESS_H_

#include "yolo_common.h"

namespace Preprocess {

// 将 BGR 图像等比例缩放并填充至目标尺寸 (LetterBox 操作)，同时转换为 RGB 格式
// 填充颜色默认使用灰色 (114, 114, 114)
cv::Mat letterboxBGRtoRGB(const cv::Mat& bgr,
                          int targetW, int targetH,
                          LetterBox& lb,
                          cv::Scalar padColor = cv::Scalar(114, 114, 114));

} // namespace Preprocess

#endif
