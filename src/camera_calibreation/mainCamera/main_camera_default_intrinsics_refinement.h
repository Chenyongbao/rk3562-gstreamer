#ifndef MAIN_CAMERA_DEFAULT_INTRINSICS_REFINEMENT_H
#define MAIN_CAMERA_DEFAULT_INTRINSICS_REFINEMENT_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "../../calib/camToolKit/calibData.h"

struct MainCameraDefaultIntrinsicsRefinementResult {
    bool success = false;
    bool fixedDistortionPreserved = false;
    double refinedCx = 0.0;
    double refinedCy = 0.0;
    double rms = 0.0;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    std::string error;
};

bool refineMainCameraDefaultIntrinsicsCxCyOnly(
    const std::vector<cv::Point3f>& objectPoints,
    const std::vector<cv::Point2f>& imagePoints,
    int imageWidth,
    int imageHeight,
    const CalibData& defaultCalibData,
    MainCameraDefaultIntrinsicsRefinementResult& result);

#endif // MAIN_CAMERA_DEFAULT_INTRINSICS_REFINEMENT_H
