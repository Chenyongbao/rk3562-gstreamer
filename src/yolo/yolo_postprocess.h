#ifndef _REALLINK_YOLO_POSTPROCESS_H_
#define _REALLINK_YOLO_POSTPROCESS_H_

#include "yolo_common.h"
#include <vector>
#include <string>

namespace Postprocess {

// 解码 YOLO 输出张量，提取候选框
void decode(const float* data, int numAnchors, int infoDim,
            int numClasses, float confThreshold,
            std::vector<ObjectDetectResult>& candidates);

// 计算两个矩形框的交并比 (Intersection over Union, IoU)
float iou(const cv::Rect& a, const cv::Rect& b);

// 非极大值抑制 (NMS)，过滤重叠框
void nms(std::vector<ObjectDetectResult>& candidates,
         float nmsThreshold,
         ObjectDetectResultList& out,
         int modelWidth, int modelHeight);

// 每个类别只保留分数最高的一个检测（per-class top-1）
void keepTop1PerClass(ObjectDetectResultList& dets);

// 提取实例分割的 Mask 掩膜（结合检测框和原型张量）
// 流程对标官方 rknn_model_zoo yolov8_seg: 掩码系数×proto → resize → 裁框(>0) → 还原原图
std::vector<SegmentationResult> extractMasks(
    const ObjectDetectResultList& dets,
    const cv::Mat& protoTensor,
    const cv::Size& originalSize,
    const LetterBox& lb,
    int modelWidth, int modelHeight,
    const std::vector<std::string>& classNames);

// 提取二值化掩膜的外轮廓
std::vector<cv::Point> extractContour(const cv::Mat& binMask, float epsFactor = 0.003f);

// 把检测结果绘制到 BGR 图上：mask 半透明叠加 + 轮廓 + 框 + 类别标签。
// 用于人工验证模型推理是否正常（原图尺寸）。
void drawResults(cv::Mat& frame, const std::vector<SegmentationResult>& results);

} // namespace Postprocess

#endif
