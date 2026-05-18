#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// Detection result for a single chessboard
struct BoardDetectionResult {
    std::vector<cv::Point2f> corners;       // Detected 2D corners
    std::vector<cv::Point3f> objectPoints;  // Corresponding 3D world coordinates
    cv::Rect roi;                           // Bounding box
    int rows = 0;
    int cols = 0;
    int pointCount = 0;
    bool patternMatched = false;
};

// Multi-board chessboard detector for calibration
class ChessboardDetector {
public:
    ChessboardDetector(const cv::Size& patternSize, float squareSize = 1.0f, int expectedBoardsPerHalf = 3);
    
    // Detect all chessboards in the image, returns boards with world coordinates
    std::vector<BoardDetectionResult> detect(const cv::Mat& grayImage) const;

private:
    cv::Size patternSize_;
    float squareSize_;
    int expectedBoardsPerHalf_;

    std::vector<BoardDetectionResult> detectInRegion(const cv::Mat& img, const cv::Rect& roi, float scale) const;
    static bool refineBoardROI(const cv::Mat& img, const cv::Size& pattern, BoardDetectionResult& board);
    static std::vector<BoardDetectionResult> mergeAndFilter(std::vector<BoardDetectionResult> boards, 
                                                             float iouThresh, const cv::Size& pattern, const cv::Size& imgSize);
};
