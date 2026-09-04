#ifndef MAIN_CAMERA_FISHEYE_CALIB_H
#define MAIN_CAMERA_FISHEYE_CALIB_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "../../calib/ChessboardDetector.h"
#include "../../calib/camToolKit/calibData.h"

// 主相机单帧鱼眼标定入口的返回结果。
struct MainCameraFisheyeCalibResult {
    bool success = false;
    std::string error;
};

// 封装主相机单帧标定流程：质检、角点检测、内参选择、外参求解和落盘。
class MainCameraFisheyeCalibrator {
public:
    MainCameraFisheyeCalibrator();

    MainCameraFisheyeCalibResult calibrateFromNv12(const unsigned char* nv12_data,
    std::size_t nv12_size, int width, int height,
    const std::string& output_dir, std::uint64_t frame_id);

private:
    struct FrameQualityStats;

    // 对当前候选内外参做结果验收，避免把明显异常值写入配置。
    bool validateCalibrationQuality(double rms,
                                    const cv::Mat& cameraMatrix,
                                    const cv::Mat& distCoeffs,
                                    const std::vector<cv::Vec3d>& rvecs,
                                    const std::vector<cv::Vec3d>& tvecs,
                                    int imageWidth,
                                    int imageHeight,
                                    std::string& reason) const;
    // EEPROM 畸变超出经验阈值时，切换到默认内参兜底流程。
    static bool isEepromDistortionAcceptable(const CalibData& calibData);
    // 保存原始 NV12 帧，便于离线复现现场问题。
    bool saveNv12ToFile(const std::string& path,
                        const unsigned char* nv12_data,
                        std::size_t nv12_size) const;
    // 从亮度、对比度和行跳变等指标粗筛异常帧。
    FrameQualityStats analyzeFrameQuality(const cv::Mat& gray) const;
    // 根据帧质量统计值拦截花屏、低纹理或强噪声帧。
    bool rejectBadFrame(const FrameQualityStats& stats, std::string& reason) const;
    // 汇总所有有效棋盘的二维/三维角点，作为后续外参求解输入。
    int collectCalibrationPoints(const std::vector<BoardDetectionResult>& boards,
                                 std::vector<cv::Point3f>& objectPoints,
                                 std::vector<cv::Point2f>& imagePoints) const;
};

#endif // MAIN_CAMERA_FISHEYE_CALIB_H
