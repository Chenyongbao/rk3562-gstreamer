#include "main_camera_default_intrinsics_refinement.h"

#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>

namespace {

// 用于检查固定参数是否在优化后保持不变，避免误放开 fx/fy 或畸变系数。
bool nearlyEqual(double a, double b, double tolerance)
{
    return std::abs(a - b) <= tolerance;
}

// 基于鱼眼投影模型计算当前位姿下的重投影 RMS。
bool computeFisheyeReprojectionRms(const std::vector<cv::Point3f>& objectPoints,
                                   const std::vector<cv::Point2f>& imagePoints,
                                   const cv::Mat& cameraMatrix,
                                   const cv::Mat& distCoeffs,
                                   const cv::Vec3d& rvec,
                                   const cv::Vec3d& tvec,
                                   double& outRms)
{
    outRms = std::numeric_limits<double>::quiet_NaN();
    if (objectPoints.empty() || imagePoints.empty() || objectPoints.size() != imagePoints.size()) {
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
        return false;
    }

    double squaredErrorSum = 0.0;
    for (size_t i = 0; i < imagePoints.size(); ++i) {
        const double dx = static_cast<double>(projectedPoints[i].x) - static_cast<double>(imagePoints[i].x);
        const double dy = static_cast<double>(projectedPoints[i].y) - static_cast<double>(imagePoints[i].y);
        squaredErrorSum += dx * dx + dy * dy;
    }

    outRms = std::sqrt(squaredErrorSum / static_cast<double>(imagePoints.size()));
    return std::isfinite(outRms);
}

} // namespace

bool refineMainCameraDefaultIntrinsicsCxCyOnly(
    const std::vector<cv::Point3f>& objectPoints,
    const std::vector<cv::Point2f>& imagePoints,
    int imageWidth,
    int imageHeight,
    const CalibData& defaultCalibData,
    MainCameraDefaultIntrinsicsRefinementResult& result)
{
    result = MainCameraDefaultIntrinsicsRefinementResult{};

    // 单帧微调要求二维/三维点严格一一对应。
    if (objectPoints.empty() || imagePoints.empty() || objectPoints.size() != imagePoints.size()) {
        result.error = "point-count-mismatch";
        return false;
    }

    // 从默认标定值构造初始内参，只允许后续优化主点位置。
    result.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        defaultCalibData.fx, 0.0, defaultCalibData.cx,
        0.0, defaultCalibData.fy, defaultCalibData.cy,
        0.0, 0.0, 1.0);
    result.distCoeffs = (cv::Mat_<double>(4, 1) <<
        defaultCalibData.distCoeffs[0],
        defaultCalibData.distCoeffs[1],
        defaultCalibData.distCoeffs[2],
        defaultCalibData.distCoeffs[3]);

    const cv::Mat expectedDistCoeffs = result.distCoeffs.clone();

    std::vector<std::vector<cv::Point3f>> objectPointsList{objectPoints};
    std::vector<std::vector<cv::Point2f>> imagePointsList{imagePoints};
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;

    // 固定焦距、斜切和全部鱼眼畸变，仅重算当前帧外参并微调 cx/cy。
    const int flags =
        cv::fisheye::CALIB_USE_INTRINSIC_GUESS |
        cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
        cv::fisheye::CALIB_FIX_SKEW |
        cv::fisheye::CALIB_FIX_K1 |
        cv::fisheye::CALIB_FIX_K2 |
        cv::fisheye::CALIB_FIX_K3 |
        cv::fisheye::CALIB_FIX_K4;

    try {
        result.rms = cv::fisheye::calibrate(objectPointsList,
                                            imagePointsList,
                                            cv::Size(imageWidth, imageHeight),
                                            result.cameraMatrix,
                                            result.distCoeffs,
                                            rvecs,
                                            tvecs,
                                            flags,
                                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6));
    } catch (const cv::Exception& e) {
        result.error = e.what();
        return false;
    }

    if (rvecs.empty() || tvecs.empty()) {
        result.error = "empty-extrinsics";
        return false;
    }

    result.rvec = rvecs.front();
    result.tvec = tvecs.front();
    result.refinedCx = result.cameraMatrix.at<double>(0, 2);
    result.refinedCy = result.cameraMatrix.at<double>(1, 2);

    // 回读一次结果，确认固定参数没有被 OpenCV 优化过程悄悄改动。
    result.fixedDistortionPreserved =
        nearlyEqual(result.cameraMatrix.at<double>(0, 0), defaultCalibData.fx, 1e-9) &&
        nearlyEqual(result.cameraMatrix.at<double>(1, 1), defaultCalibData.fy, 1e-9) &&
        nearlyEqual(result.distCoeffs.at<double>(0, 0), expectedDistCoeffs.at<double>(0, 0), 1e-9) &&
        nearlyEqual(result.distCoeffs.at<double>(1, 0), expectedDistCoeffs.at<double>(1, 0), 1e-9) &&
        nearlyEqual(result.distCoeffs.at<double>(2, 0), expectedDistCoeffs.at<double>(2, 0), 1e-9) &&
        nearlyEqual(result.distCoeffs.at<double>(3, 0), expectedDistCoeffs.at<double>(3, 0), 1e-9);

    double reprojectionRms = std::numeric_limits<double>::quiet_NaN();
    if (!computeFisheyeReprojectionRms(objectPoints,
                                       imagePoints,
                                       result.cameraMatrix,
                                       result.distCoeffs,
                                       result.rvec,
                                       result.tvec,
                                       reprojectionRms)) {
        result.error = "reprojection-rms-invalid";
        return false;
    }

    // 对单帧主点微调流程而言，更关心实际重投影误差而不是 calibrate 返回值。
    result.rms = reprojectionRms;
    result.success = true;
    return true;
}
