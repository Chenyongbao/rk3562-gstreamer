#ifndef _REALLINK_YOLO_COMMON_H_
#define _REALLINK_YOLO_COMMON_H_

#include <opencv2/opencv.hpp>

struct LetterBox {
    float scale = 1.0f;
    int x_pad = 0;
    int y_pad = 0;
    int resize_w = 0;
    int resize_h = 0;
};

struct ObjectDetectResult {
    cv::Rect box;
    float confidence = 0.0f;
    int classId = -1;
};

struct ObjectDetectResultList {
    static constexpr int kMaxCount = 128;
    int count = 0;
    ObjectDetectResult results[kMaxCount];

    void clear() { count = 0; }
};

struct SegmentationResult {
    int classId = -1;
    std::string className;
    float confidence = 0.0f;
    cv::Rect boundingBox;
    cv::Mat mask;
    std::vector<cv::Point> contour;
    std::vector<cv::Point> contour_full;
};

#endif
