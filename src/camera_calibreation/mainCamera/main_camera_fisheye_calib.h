#ifndef MAIN_CAMERA_FISHEYE_CALIB_H
#define MAIN_CAMERA_FISHEYE_CALIB_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "../../calib/ChessboardDetector.h"
#include "../../calib/camToolKit/calibData.h"

struct MainCameraFisheyeCalibResult {
    bool success = false;
    std::string error;
};

class MainCameraFisheyeCalibrator {
public:
    MainCameraFisheyeCalibrator();

    MainCameraFisheyeCalibResult calibrateFromNv12(const unsigned char* nv12_data,
    std::size_t nv12_size, int width, int height,
    const std::string& output_dir, std::uint64_t frame_id);

private:
    struct FrameQualityStats;

    bool validateCalibrationQuality(double rms,
                                    const cv::Mat& cameraMatrix,
                                    const cv::Mat& distCoeffs,
                                    const std::vector<cv::Vec3d>& rvecs,
                                    const std::vector<cv::Vec3d>& tvecs,
                                    int imageWidth,
                                    int imageHeight,
                                    std::string& reason) const;
    static bool isEepromDistortionAcceptable(const CalibData& calibData);
    bool saveNv12ToFile(const std::string& path,
                        const unsigned char* nv12_data,
                        std::size_t nv12_size) const;
    FrameQualityStats analyzeFrameQuality(const cv::Mat& gray) const;
    bool rejectBadFrame(const FrameQualityStats& stats, std::string& reason) const;
    int collectCalibrationPoints(const std::vector<BoardDetectionResult>& boards,
                                 std::vector<cv::Point3f>& objectPoints,
                                 std::vector<cv::Point2f>& imagePoints) const;
};

#endif // MAIN_CAMERA_FISHEYE_CALIB_H
