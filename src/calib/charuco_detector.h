#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

struct CharucoBoardDetection {
    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point3f> objectPoints;
    std::vector<int> ids;
    cv::Rect roi;
};

// Multi-board Charuco detector for calibration (6 boards, DICT_5X5_250)
class CharucoMultiBoardDetector {
public:
    // Detect Charuco corners and return corresponding world coordinates
    // Returns: number of detected corners, 0 on failure
    static int detect(const cv::Mat& image,
                      std::vector<cv::Point2f>& imagePoints,
                      std::vector<cv::Point3f>& objectPoints,
                      float squareSize = 1.0f);

    static int detectPerBoard(const cv::Mat& image,
                              std::vector<CharucoBoardDetection>& boards,
                              float squareSize = 1.0f);
};
