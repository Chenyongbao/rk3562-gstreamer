#ifndef _REALLINK_YOLO_POSTPROCESS_H_
#define _REALLINK_YOLO_POSTPROCESS_H_

#include "yolo_common.h"
#include <vector>
#include <string>

namespace Postprocess {

struct NMSConfig {
    float confThreshold = 0.05f;
    float nmsThreshold = 0.7f;
};

void decode(const float* data, int numAnchors, int infoDim,
            int numClasses, float confThreshold,
            std::vector<ObjectDetectResult>& candidates);

float iou(const cv::Rect& a, const cv::Rect& b);

void nms(std::vector<ObjectDetectResult>& candidates,
         float nmsThreshold,
         ObjectDetectResultList& out,
         int modelWidth, int modelHeight);

std::vector<SegmentationResult> extractMasks(
    const ObjectDetectResultList& dets,
    const cv::Mat& protoTensor,
    const cv::Size& originalSize,
    const LetterBox& lb,
    const std::vector<std::string>& classNames);

std::vector<cv::Point> extractContour(const cv::Mat& binMask, float epsFactor = 0.003f);

} // namespace Postprocess

#endif
