#ifndef _REALLINK_YOLO_COMMON_H_
#define _REALLINK_YOLO_COMMON_H_

#include <opencv2/opencv.hpp>

// 掩码系数数量（YOLOv5-seg/YOLOv8-seg 输出张量每行末尾的 32 个分割系数）
static constexpr int kMaskCoeffCount = 32;

// 图像等比例缩放并填充 (LetterBox) 的参数记录
struct LetterBox {
    float scale = 1.0f;     // 缩放比例
    int x_pad = 0;          // X方向填充的像素数
    int y_pad = 0;          // Y方向填充的像素数
    int resize_w = 0;       // 缩放后的宽度
    int resize_h = 0;       // 缩放后的高度
};

// 目标检测结果
struct ObjectDetectResult {
    cv::Rect box;           // 边界框
    float confidence = 0.0f;// 置信度得分
    int classId = -1;       // 类别ID
    float maskCoefs[kMaskCoeffCount] = {0.0f}; // 掩码系数（供分割掩码解码使用）
};

// 目标检测结果列表
struct ObjectDetectResultList {
    static constexpr int kMaxCount = 128; // 最大检测目标数量
    int count = 0;                        // 当前检测到的目标数量
    ObjectDetectResult results[kMaxCount];// 目标结果数组

    void clear() { count = 0; }           // 清空结果
};

// 实例分割结果
struct SegmentationResult {
    int classId = -1;                     // 类别ID
    std::string className;                // 类别名称
    float confidence = 0.0f;              // 置信度得分
    cv::Rect boundingBox;                 // 边界框
    cv::Mat mask;                         // 分割掩膜 (Mask)
    std::vector<cv::Point> contour;       // 简化后的轮廓点集
    std::vector<cv::Point> contour_full;  // 完整的轮廓点集
};

#endif
