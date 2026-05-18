#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <thread>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <limits>
#include <curl/curl.h>
#include "../../calib/ChessboardDetector.h"
#include "../../calib/saddleFit.h"
#include "main_camera_fisheye_calib.h"
#include "main_camera_default_intrinsics.h"
#include "../../calib/camToolKit/calibData.h"
#include "../../calib/camToolKit/calib_eeprom.h"
#include "../../reallink_ogles/camera.h"
#include "../../reallink_ogles/file_utils.h"
#include "../../config.h"

namespace {

constexpr double kMainCameraEepromK3Min = -0.2;
constexpr double kMainCameraEepromK4Max = 0.2;

bool computeFisheyeReprojectionRms(const std::vector<cv::Point3f>& objectPoints,
                                   const std::vector<cv::Point2f>& imagePoints,
                                   const cv::Mat& cameraMatrix,
                                   const cv::Mat& distCoeffs,
                                   const cv::Vec3d& rvec,
                                   const cv::Vec3d& tvec,
                                   double& outRms,
                                   std::string* error)
{
    outRms = std::numeric_limits<double>::quiet_NaN();

    if (objectPoints.empty() || imagePoints.empty()) {
        if (error) {
            *error = "reprojection-rms: empty input points";
        }
        return false;
    }
    if (objectPoints.size() != imagePoints.size()) {
        if (error) {
            *error = "reprojection-rms: object/image point count mismatch";
        }
        return false;
    }

    std::vector<cv::Point2f> projectedPoints;
    cv::fisheye::projectPoints(objectPoints,
                               projectedPoints,
                               rvec,
                               tvec,
                               cameraMatrix,
                               distCoeffs);
    if (projectedPoints.size() != imagePoints.size()) {
        if (error) {
            *error = "reprojection-rms: projected/image point count mismatch";
        }
        return false;
    }

    double squaredErrorSum = 0.0;
    for (size_t i = 0; i < imagePoints.size(); ++i) {
        const double dx = static_cast<double>(projectedPoints[i].x) - static_cast<double>(imagePoints[i].x);
        const double dy = static_cast<double>(projectedPoints[i].y) - static_cast<double>(imagePoints[i].y);
        squaredErrorSum += dx * dx + dy * dy;
    }

    outRms = std::sqrt(squaredErrorSum / static_cast<double>(imagePoints.size()));
    if (!std::isfinite(outRms)) {
        if (error) {
            *error = "reprojection-rms: computed RMS is not finite";
        }
        return false;
    }
    return true;
}

} // namespace

MainCameraFisheyeCalibrator::MainCameraFisheyeCalibrator() = default;

bool MainCameraFisheyeCalibrator::isEepromDistortionAcceptable(const CalibData& calibData)
{
    return calibData.distCoeffs[2] > kMainCameraEepromK3Min &&
           calibData.distCoeffs[3] < kMainCameraEepromK4Max;
}

// 将 NV12 原始帧落盘，便于离线复现问题。
bool MainCameraFisheyeCalibrator::saveNv12ToFile(const std::string& path,
                                                 const unsigned char* nv12_data,
                                                 std::size_t nv12_size) const
{
    if (!nv12_data || nv12_size == 0) {
        return false;
    }
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open()) {
        std::cerr << "[MainCameraCalib] Failed to open for NV12 write: " << path << std::endl;
        return false;
    }
    ofs.write(reinterpret_cast<const char*>(nv12_data), static_cast<std::streamsize>(nv12_size));
    if (!ofs.good()) {
        std::cerr << "[MainCameraCalib] Failed to write NV12 to: " << path << std::endl;
        return false;
    }
    std::cout << "[MainCameraCalib] Saved NV12 to " << path << " (size=" << nv12_size << ")" << std::endl;
    return true;
}
//图像质量
struct MainCameraFisheyeCalibrator::FrameQualityStats {
    double mean = 0.0;
    double stddev = 0.0;
    double gradientMean = 0.0;
    double rowJumpRatio = 0.0;
    int sampleCount = 0;
};
//计算亮度，对比度，清晰度的指标拦截花屏
MainCameraFisheyeCalibrator::FrameQualityStats MainCameraFisheyeCalibrator::analyzeFrameQuality(
    const cv::Mat& gray) const
{
    FrameQualityStats stats;
    if (gray.empty()) {
        return stats;
    }

    const int step = std::max(4, std::min(gray.cols, gray.rows) / 256);
    double sum = 0.0;
    double sum_sq = 0.0;
    double grad_sum = 0.0;
    int grad_count = 0;
    std::vector<double> row_means;
    row_means.reserve(gray.rows / step + 1);

    for (int y = 0; y < gray.rows; y += step) {
        const uint8_t* row = gray.ptr<uint8_t>(y);
        const uint8_t* next_row = gray.ptr<uint8_t>(std::min(y + step, gray.rows - 1));
        double row_sum = 0.0;
        int row_samples = 0;

        for (int x = 0; x < gray.cols; x += step) {
            const double value = row[x];
            sum += value;
            sum_sq += value * value;
            row_sum += value;
            row_samples++;
            stats.sampleCount++;

            if (x + step < gray.cols) {
                grad_sum += std::abs(int(value) - int(row[x + step]));
                grad_count++;
            }
            if (y + step < gray.rows) {
                grad_sum += std::abs(int(value) - int(next_row[x]));
                grad_count++;
            }
        }

        if (row_samples > 0) {
            row_means.push_back(row_sum / row_samples);
        }
    }

    if (stats.sampleCount == 0) {
        return stats;
    }

    stats.mean = sum / stats.sampleCount;
    const double variance = std::max(0.0, (sum_sq / stats.sampleCount) - stats.mean * stats.mean);
    stats.stddev = std::sqrt(variance);
    if (grad_count > 0) {
        stats.gradientMean = grad_sum / grad_count;
    }

    int row_jump_count = 0;
    for (size_t i = 1; i < row_means.size(); ++i) {
        if (std::abs(row_means[i] - row_means[i - 1]) > 28.0) {
            row_jump_count++;
        }
    }
    if (row_means.size() > 1) {
        stats.rowJumpRatio = static_cast<double>(row_jump_count) /
                             static_cast<double>(row_means.size() - 1);
    }

    return stats;
}
//拒绝坏帧
bool MainCameraFisheyeCalibrator::rejectBadFrame(const FrameQualityStats& stats, std::string& reason) const
{
    if (stats.sampleCount < 32) {
        reason = "bad-frame-gate:insufficient-samples";
        return true;
    }
    if (stats.stddev < 6.0) {
        reason = "bad-frame-gate:low-variance";
        return true;
    }
    if (stats.gradientMean > 42.0 && stats.rowJumpRatio > 0.18) {
        reason = "bad-frame-gate:heavy-row-jumps";
        return true;
    }
    if (stats.gradientMean > 60.0 && stats.stddev > 35.0) {
        reason = "bad-frame-gate:high-noise";
        return true;
    }
    return false; 
}
//收集有效标定板的角点
int MainCameraFisheyeCalibrator::collectCalibrationPoints(
    const std::vector<BoardDetectionResult>& boards,
    std::vector<cv::Point3f>& objectPoints,
    std::vector<cv::Point2f>& imagePoints) const
{
    objectPoints.clear();
    imagePoints.clear();

    int validBoards = 0;
    for (const auto& board : boards) {
        if (!board.patternMatched) {
            continue;
        }
        if (board.pointCount > 20) {
            validBoards++;
            imagePoints.insert(imagePoints.end(), board.corners.begin(), board.corners.end());
            objectPoints.insert(objectPoints.end(), board.objectPoints.begin(), board.objectPoints.end());
        }
    }
    return validBoards;
}
MainCameraFisheyeCalibResult MainCameraFisheyeCalibrator::calibrateFromNv12(const unsigned char* nv12_data,
                                                                            std::size_t nv12_size,
                                                                            int imageWidth,
                                                                            int imageHeight,
                                                                            const std::string& output_dir,
                                                                            std::uint64_t frame_id)
{
    // 1) 输入合法性检查：空指针/尺寸异常直接失败。
    if (!nv12_data) {
        std::cerr << "[MainCameraCalib] Invalid NV12 buffer" << std::endl;
        return {false, "INVALID_NV12_BUFFER"};
    }
    const size_t yPlaneSize = static_cast<size_t>(imageWidth) * static_cast<size_t>(imageHeight);
    if (nv12_size < yPlaneSize) {
        std::cerr << "[MainCameraCalib] NV12 buffer too small: " << nv12_size
                  << " bytes, expected at least " << yPlaneSize << std::endl;
        return {false, "NV12_BUFFER_TOO_SMALL"};
    }
    try {
        const std::string final_output_dir = output_dir.empty() ? "/home/linaro" : output_dir;
        std::filesystem::create_directories(final_output_dir);

        std::cout << "[MainCameraCalib] frame_id=" << frame_id
                  << " start (" << imageWidth << "x" << imageHeight
                  << ", nv12_size=" << nv12_size << ")" << std::endl;

        cv::Mat gray(imageHeight, imageWidth, CV_8UC1);
        std::memcpy(gray.data, nv12_data, yPlaneSize);

        const float squareSize = 1280.0f * 80.0f / 3200.0f;
        const cv::Size patternSize(7, 7);
        // 批量标定参数（与单帧流程保持一致，便于效果对齐）。
        constexpr int kExpectedBoardsPerHalf = 3;

        // 失败时仅保存 NV12 原始帧，便于离线复现。
        auto saveDebugArtifacts = [&](double rmsValue,
                                      const std::string& reason,
                                      const std::vector<BoardDetectionResult>& boards) {
            (void)boards;
            const std::string debug_nv12_path = final_output_dir + "/debug_frame.nv12";
            const bool ok = saveNv12ToFile(debug_nv12_path, nv12_data, nv12_size);
            std::cout << "[MainCameraCalib] Saved NV12 debug frame to " << debug_nv12_path
                      << " (status=" << (ok ? "OK" : "FAIL")
                      << ", reason=" << reason << ", RMS=" << rmsValue << ")"
                      << std::endl;
        };

        //计算亮度，对比度，清晰度的指标拦截花屏
        const FrameQualityStats qualityStats = analyzeFrameQuality(gray);
        std::cout << "[MainCameraCalib] frame_id=" << frame_id
                  << " frame_quality mean=" << qualityStats.mean
                  << " stddev=" << qualityStats.stddev
                  << " gradientMean=" << qualityStats.gradientMean
                  << " rowJumpRatio=" << qualityStats.rowJumpRatio
                  << " sampleCount=" << qualityStats.sampleCount
                  << std::endl;

        //开始坏帧拦截并落盘
        std::string badFrameReason;
        if (rejectBadFrame(qualityStats, badFrameReason)) {
            std::cerr << "[MainCameraCalib] frame_id=" << frame_id
                      << " rejected by " << badFrameReason << std::endl;
            saveDebugArtifacts(std::numeric_limits<double>::quiet_NaN(), badFrameReason, {});
            return {false, badFrameReason};
        }

        // 2) 检测阶段：直接使用原始分辨率，避免缩放角点导致 RMS 偏高。
        std::cout << "[MainCameraCalib] calibration START" << std::endl;

        ChessboardDetector detector(patternSize, squareSize, kExpectedBoardsPerHalf);
        std::vector<BoardDetectionResult> chessboardResults = detector.detect(gray);

        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;
        int validBoards = collectCalibrationPoints(chessboardResults, objectPoints, imagePoints);
        int totalCorners = static_cast<int>(imagePoints.size());

        constexpr int kExpectedValidBoardsPerFrame = 6;
        constexpr int kExpectedTotalCornersPerFrame = 294;
        if (chessboardResults.empty() && validBoards == 0 && totalCorners == 0) {
            std::cerr << "[MainCameraCalib] frame_id=" << frame_id
                      << " empty detection" << std::endl;
            const std::string gateReason = "gate: empty-detection";
            saveDebugArtifacts(std::numeric_limits<double>::quiet_NaN(), gateReason, chessboardResults);
            return {false, "EMPTY_DETECTION"};
        }

        std::cout << "[MainCameraCalib] frame_id=" << frame_id
                  << " boards_detected=" << chessboardResults.size()
                  << " validBoards=" << validBoards
                  << " totalCorners=" << totalCorners
                  << std::endl;

        // 3) 数据闸门：必须恰好检测到 6 个有效棋盘、294 个角点，否则交给上层重抓新帧重试。
        if (validBoards != kExpectedValidBoardsPerFrame || totalCorners != kExpectedTotalCornersPerFrame) {
            std::cerr << "[MainCameraCalib] frame_id=" << frame_id
                      << " rejected by gate: validBoards=" << validBoards
                      << " (need == " << kExpectedValidBoardsPerFrame << "), totalCorners=" << totalCorners
                      << " (need == " << kExpectedTotalCornersPerFrame << ")"
                      << std::endl;
            const std::string gateReason =
                "gate: validBoards=" + std::to_string(validBoards) +
                ", totalCorners=" + std::to_string(totalCorners) +
                ", expectedValidBoards=" + std::to_string(kExpectedValidBoardsPerFrame) +
                ", expectedTotalCorners=" + std::to_string(kExpectedTotalCornersPerFrame);
            saveDebugArtifacts(std::numeric_limits<double>::quiet_NaN(), gateReason, chessboardResults);
            return {false, "DETECTION_COUNT_MISMATCH"};
        }

        //==================开始求外参==================
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        cv::Vec3d rvec;
        cv::Vec3d tvec;
        double reprojectionRms = std::numeric_limits<double>::quiet_NaN();

        // 4) 鞍点优化：角点亚像素精修，提高标定稳定性。
        std::cout << "[MainCameraCalib] frame_id=" << frame_id << " saddle_begin" << std::endl;
        ReallinkSaddlePointFit(gray, imagePoints, 10);
        std::cout << "[MainCameraCalib] frame_id=" << frame_id << " saddle_done" << std::endl;

        // 5) 从 EEPROM 读取固定内参；若畸变不满足阈值则切默认内参，仅求当前帧外参。
        try {
            CalibEEPROM eeprom;
            CalibData calibData{};
            if (!eeprom.read(calibData)) {
                std::cerr << "[MainCameraCalib] frame_id=" << frame_id
                          << " failed to read intrinsics from EEPROM" << std::endl;
                saveDebugArtifacts(std::numeric_limits<double>::quiet_NaN(),
                                   "eeprom-read-failed",
                                   chessboardResults);
                return {false, "EEPROM_READ_FAILED"};
            }

            const CalibData* activeCalibData = &calibData;
            CalibData defaultCalibData{};
            if (!isEepromDistortionAcceptable(calibData)) {
                defaultCalibData = makeDefaultMainCameraCalibData(imageWidth, imageHeight);
                activeCalibData = &defaultCalibData;
                std::cout << "[MainCameraCalib] frame_id=" << frame_id
                          << " EEPROM distortion rejected, using default intrinsics"
                          << " k3=" << calibData.distCoeffs[2]
                          << " k4=" << calibData.distCoeffs[3]
                          << std::endl;

                if (!eeprom.write(defaultCalibData)) {
                    std::cerr << "[MainCameraCalib] frame_id=" << frame_id
                              << " failed to write default intrinsics to EEPROM" << std::endl;
                    saveDebugArtifacts(std::numeric_limits<double>::quiet_NaN(),
                                       "eeprom-write-default-failed",
                                       chessboardResults);
                    return {false, "EEPROM_WRITE_DEFAULT_FAILED"};
                }
                std::cout << "[MainCameraCalib] frame_id=" << frame_id
                          << " default intrinsics written to EEPROM" << std::endl;
            } else {
                std::cout << "[MainCameraCalib] frame_id=" << frame_id
                          << " EEPROM distortion accepted, using EEPROM intrinsics"
                          << " k3=" << calibData.distCoeffs[2]
                          << " k4=" << calibData.distCoeffs[3]
                          << std::endl;
            }

            cameraMatrix = (cv::Mat_<double>(3, 3) <<
                activeCalibData->fx, 0.0, activeCalibData->cx,
                0.0, activeCalibData->fy, activeCalibData->cy,
                0.0, 0.0, 1.0);
            distCoeffs = (cv::Mat_<double>(4, 1) <<
                activeCalibData->distCoeffs[0],
                activeCalibData->distCoeffs[1],
                activeCalibData->distCoeffs[2],
                activeCalibData->distCoeffs[3]);

            std::vector<cv::Point2f> undistortedImagePoints;
            cv::fisheye::undistortPoints(imagePoints,
                                         undistortedImagePoints,
                                         cameraMatrix,
                                         distCoeffs);

            const bool solved = cv::solvePnP(
                objectPoints,
                undistortedImagePoints,
                cv::Mat::eye(3, 3, CV_64F),
                cv::noArray(),
                rvec,
                tvec,
                false,
                cv::SOLVEPNP_ITERATIVE);
            if (!solved) {
                std::cerr << "[MainCameraCalib] frame_id=" << frame_id
                          << " fisheye solvePnP returned false" << std::endl;
                saveDebugArtifacts(std::numeric_limits<double>::quiet_NaN(),
                                   "solvepnp-failed",
                                   chessboardResults);
                return {false, "SOLVEPNP_FAILED"};
            }
        } catch (const cv::Exception& e) {
            std::cerr << "[MainCameraCalib] frame_id=" << frame_id
                      << " fisheye solvePnP exception: " << e.what()
                      << std::endl;
            saveDebugArtifacts(std::numeric_limits<double>::quiet_NaN(),
                               std::string("solvepnp-exception: ") + e.what(),
                               chessboardResults);
            return {false, "SOLVEPNP_EXCEPTION"};
        }

        std::cout << "[MainCameraCalib] frame_id=" << frame_id
                  << " solvepnp_done"
                  << " validBoards=" << validBoards
                  << " totalCorners=" << totalCorners
                  << std::endl;

        try {
            std::string rmsError;
            if (!computeFisheyeReprojectionRms(objectPoints,
                                               imagePoints,
                                               cameraMatrix,
                                               distCoeffs,
                                               rvec,
                                               tvec,
                                               reprojectionRms,
                                               &rmsError)) {
                std::cerr << "[MainCameraCalib] frame_id=" << frame_id
                          << " failed to compute reprojection RMS: " << rmsError
                          << std::endl;
                saveDebugArtifacts(reprojectionRms, rmsError, chessboardResults);
                return {false, "REPROJECTION_RMS_INVALID"};
            }
        } catch (const cv::Exception& e) {
            std::cerr << "[MainCameraCalib] frame_id=" << frame_id
                      << " reprojection RMS exception: " << e.what()
                      << std::endl;
            saveDebugArtifacts(reprojectionRms,
                               std::string("reprojection-rms-exception: ") + e.what(),
                               chessboardResults);
            return {false, "REPROJECTION_RMS_EXCEPTION"};
        }

        std::cout << "[MainCameraCalib] rvec=" << rvec << std::endl;
        std::cout << "[MainCameraCalib] tvec=" << tvec << std::endl;
        std::cout << "[MainCameraCalib] reprojection_rms=" << reprojectionRms
                  << " px, points=" << imagePoints.size() << std::endl;
        std::cout << "[MainCameraCalib] cameraMatrix=" << cameraMatrix << std::endl;
        std::cout << "[MainCameraCalib] distCoeffs=" << distCoeffs << std::endl;
        std::cout << "[MainCameraCalib] intrinsics source=EEPROM, extrinsics solved only" << std::endl;

        {
            ReallinkCVConfig config{};
            const std::string conf_path = "/home/linaro/reallinkCV.conf";
            readReallinkCVConf(conf_path, config);
            config.cam0.rvec[0] = rvec[0];
            config.cam0.rvec[1] = rvec[1];
            config.cam0.rvec[2] = rvec[2];
            config.cam0.tvec[0] = tvec[0];
            config.cam0.tvec[1] = tvec[1];
            config.cam0.tvec[2] = tvec[2];
            if (writeReallinkCVConf(conf_path, config)) {
                std::cout << "[MainCameraCalib] Extrinsics saved to " << conf_path << std::endl;
                std::cout << "[MainCameraCalib]   rvec=" << rvec << " tvec=" << tvec << std::endl;
            } else {
                std::cerr << "[MainCameraCalib] Failed to save extrinsics to " << conf_path << std::endl;
            }
        }

        return {true, ""};
    } catch (const std::exception& e) {
        std::cerr << "[MainCameraCalib] frame_id=" << frame_id
                  << " Exception (NV12 flow): " << e.what()
                  << std::endl;
        return {false, "NV12_FLOW_EXCEPTION"};
    }
}

