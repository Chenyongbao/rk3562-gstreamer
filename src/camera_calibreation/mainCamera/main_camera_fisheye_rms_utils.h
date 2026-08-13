#ifndef MAIN_CAMERA_FISHEYE_RMS_UTILS_H
#define MAIN_CAMERA_FISHEYE_RMS_UTILS_H

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

// 使用鱼眼模型把三维点重投影回图像，计算当前内外参组合的 RMS 误差。
inline double computeFisheyeReprojectionRms(const std::vector<cv::Point3f>& objectPoints,
                                            const std::vector<cv::Point2f>& imagePoints,
                                            const cv::Mat& cameraMatrix,
                                            const cv::Mat& distCoeffs,
                                            const cv::Vec3d& rvec,
                                            const cv::Vec3d& tvec)
{
    if (objectPoints.empty() || imagePoints.empty() || objectPoints.size() != imagePoints.size()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    try {
        std::vector<cv::Point2f> projectedPoints;
        cv::fisheye::projectPoints(objectPoints, projectedPoints, rvec, tvec, cameraMatrix, distCoeffs);
        if (projectedPoints.size() != imagePoints.size()) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        double squaredErrorSum = 0.0;
        for (size_t i = 0; i < imagePoints.size(); ++i) {
            const double dx = projectedPoints[i].x - imagePoints[i].x;
            const double dy = projectedPoints[i].y - imagePoints[i].y;
            squaredErrorSum += dx * dx + dy * dy;
        }
        return std::sqrt(squaredErrorSum / static_cast<double>(imagePoints.size()));
    } catch (const cv::Exception&) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

// 先做鱼眼去畸变，再在单位内参平面上求解外参。
inline bool solveFisheyePose(const std::vector<cv::Point3f>& objectPoints,
                             const std::vector<cv::Point2f>& imagePoints,
                             const cv::Mat& cameraMatrix,
                             const cv::Mat& distCoeffs,
                             cv::Vec3d& rvec,
                             cv::Vec3d& tvec,
                             std::string* failureReason = nullptr)
{
    if (failureReason) {
        failureReason->clear();
    }

    if (objectPoints.empty() || imagePoints.empty() || objectPoints.size() != imagePoints.size()) {
        if (failureReason) {
            *failureReason = "invalid input points: empty or size mismatch";
        }
        return false;
    }

    try {
        std::vector<cv::Point2f> undistortedPoints;
        cv::fisheye::undistortPoints(imagePoints, undistortedPoints, cameraMatrix, distCoeffs);
        if (undistortedPoints.size() != imagePoints.size()) {
            if (failureReason) {
                *failureReason = "fisheye::undistortPoints size mismatch";
            }
            return false;
        }

        const bool solved = cv::solvePnP(objectPoints,
                                         undistortedPoints,
                                         cv::Mat::eye(3, 3, CV_64F),
                                         cv::noArray(),
                                         rvec,
                                         tvec,
                                         false,
                                         cv::SOLVEPNP_ITERATIVE);
        if (!solved) {
            if (failureReason) {
                *failureReason = "cv::solvePnP returned false";
            }
            return false;
        }
        return true;
    } catch (const cv::Exception& e) {
        if (failureReason) {
            *failureReason = e.what();
        }
        return false;
    }
}

// 将外参求解与 RMS 评估打包，便于调用方一次拿到完整诊断结果。
inline bool solveFisheyePoseAndComputeRms(const std::vector<cv::Point3f>& objectPoints,
                                          const std::vector<cv::Point2f>& imagePoints,
                                          const cv::Mat& cameraMatrix,
                                          const cv::Mat& distCoeffs,
                                          int imageWidth,
                                          int imageHeight,
                                          cv::Vec3d& rvec,
                                          cv::Vec3d& tvec,
                                          double& rms,
                                          std::string* failureReason = nullptr)
{
    (void)imageWidth;
    (void)imageHeight;
    rms = std::numeric_limits<double>::quiet_NaN();

    if (!solveFisheyePose(objectPoints,
                          imagePoints,
                          cameraMatrix,
                          distCoeffs,
                          rvec,
                          tvec,
                          failureReason)) {
        return false;
    }

    rms = computeFisheyeReprojectionRms(objectPoints,
                                        imagePoints,
                                        cameraMatrix,
                                        distCoeffs,
                                        rvec,
                                        tvec);
    if (!std::isfinite(rms)) {
        if (failureReason) {
            *failureReason = "reprojection RMS is not finite";
        }
        return false;
    }
    return true;
}

#endif // MAIN_CAMERA_FISHEYE_RMS_UTILS_H
