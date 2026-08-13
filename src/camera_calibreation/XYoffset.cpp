#include "XYoffset.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "../config.h"
#include "../reallink_ogles/camera.h"
#include "../reallink_ogles/file_utils.h"
#include "../pipeline/3_consumers/common/LatestNv12FrameBuffer.h"
#include "../app/capture_state.h"
#include "../klipper/klipper_manager.h"

XYoffsetService::XYoffsetService(CaptureLoopState* captureState,
                                 double targetX,
                                 double targetY,
                                 double mmPerPixel,
                                 const char* confPath)
    : target_x_(targetX > 0 ? targetX : kDefaultTargetX),
      target_y_(targetY > 0 ? targetY : kDefaultTargetY),
      mm_per_pixel_(mmPerPixel > 0 ? mmPerPixel : kDefaultMmPerPixel),
      conf_path_(confPath ? confPath : REALLINK_CV_CONF_PATH),
      capture_state_(captureState) {}

//截图nv12转jpg
bool XYoffsetService::captureImage(cv::Mat& out, std::string& errorMsg) const {
    const int width = INPUT_WIDTH;
    const int height = INPUT_HEIGHT;
    const size_t frameSize = static_cast<size_t>(width) * height * 3 / 2;

    std::vector<uint8_t> buffer(frameSize);
    size_t outSize = 0;
    uint64_t frameId = 0;

    const bool copied = capture_state_
        ? main_camera_frame_buffer_request_fresh_copy(capture_state_,
                                                      buffer.data(),
                                                      frameSize,
                                                      &outSize,
                                                      &frameId,
                                                      kMainCameraRefreshTimeoutMs,
                                                      kMainCameraRefreshPollIntervalMs)
        : main_camera_frame_buffer_copy(buffer.data(), frameSize, &outSize, &frameId);
    if (!copied) {
        errorMsg = "Failed to copy frame from main camera buffer";
        return false;
    }
    if (outSize < frameSize) {
        errorMsg = "Frame too small: " + std::to_string(outSize);
        return false;
    }

    cv::Mat yPlane(height, width, CV_8UC1, buffer.data());
    cv::Mat uvPlane(height / 2, width / 2, CV_8UC2, buffer.data() + static_cast<size_t>(width) * height);

    cv::Mat bgr;
    cv::cvtColorTwoPlane(yPlane, uvPlane, bgr, cv::COLOR_YUV2BGR_NV12);
    if (bgr.empty()) {
        errorMsg = "NV12 -> BGR conversion failed";
        return false;
    }

    out = bgr.clone();
    return true;
}

bool XYoffsetService::detectLaserSpot(const cv::Mat& image,
                                      double& px,
                                      double& py,
                                      std::string& errorMsg) const {
    if (image.empty()) {
        errorMsg = "Empty image";
        return false;
    }

    const cv::Rect fixedRoi(kSearchX, kSearchY, kSearchW, kSearchH);
    const cv::Rect imageRect(0, 0, image.cols, image.rows);
    const cv::Rect searchRoi = fixedRoi & imageRect;

    std::cout << "[XYoffset] Strict ROI fixed=("
              << fixedRoi.x << "," << fixedRoi.y << "," << fixedRoi.width << "," << fixedRoi.height
              << ") clipped=("
              << searchRoi.x << "," << searchRoi.y << "," << searchRoi.width << "," << searchRoi.height
              << ")" << std::endl;

    if (searchRoi.width <= 0 || searchRoi.height <= 0) {
        errorMsg = "strict ROI miss: fixed ROI out of image bounds";
        return false;
    }

    cv::Mat searchBgr = image(searchRoi);
    cv::Mat hsv;
    cv::Mat gray;
    cv::cvtColor(searchBgr, hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(searchBgr, gray, cv::COLOR_BGR2GRAY);

    cv::Mat blueMask;
    cv::inRange(hsv, cv::Scalar(90, 50, 20), cv::Scalar(135, 255, 255), blueMask);

    cv::Mat whiteMask;
    cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 60, 255), whiteMask);

    cv::Mat spotMask;
    cv::bitwise_or(blueMask, whiteMask, spotMask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(spotMask, spotMask, cv::MORPH_CLOSE, kernel);

    cv::imwrite("/tmp/xyoffset_mask_roi.png", spotMask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(spotMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        cv::imwrite("/tmp/xyoffset_raw_roi.png", searchBgr);
        errorMsg = "strict ROI miss: no laser region in fixed ROI (raw saved to /tmp/xyoffset_raw_roi.png)";
        return false;
    }

    int bestIdx = -1;
    double bestArea = 0.0;
    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        const double area = cv::contourArea(contours[i]);
        if (area >= 2.0 && area <= 80000.0 && area > bestArea) {
            bestArea = area;
            bestIdx = i;
        }
    }
    if (bestIdx < 0) {
        errorMsg = "strict ROI miss: no valid contour in fixed ROI (area out of range 2-80000 px²)";
        return false;
    }

    cv::Rect roi = cv::boundingRect(contours[bestIdx]);
    roi.x = std::max(0, roi.x - 8);
    roi.y = std::max(0, roi.y - 8);
    roi.width = std::min(searchBgr.cols - roi.x, roi.width + 16);
    roi.height = std::min(searchBgr.rows - roi.y, roi.height + 16);

    cv::Mat grayRoi = gray(roi);

    double minVal = 0.0;
    double maxVal = 0.0;
    cv::minMaxLoc(grayRoi, &minVal, &maxVal);
    const double coreRatio = 0.70;
    const uint8_t coreThresh = static_cast<uint8_t>(maxVal * coreRatio);

    cv::Mat coreMask;
    cv::threshold(grayRoi, coreMask, coreThresh, 255, cv::THRESH_BINARY);

    if (cv::countNonZero(coreMask) > 0) {
        cv::Mat dist;
        cv::distanceTransform(coreMask, dist, cv::DIST_L2, 3);
        double distMax = 0.0;
        cv::Point maxLoc;
        cv::minMaxLoc(dist, nullptr, &distMax, nullptr, &maxLoc);
        const double localPx = roi.x + static_cast<double>(maxLoc.x);
        const double localPy = roi.y + static_cast<double>(maxLoc.y);
        px = searchRoi.x + localPx;
        py = searchRoi.y + localPy;
        std::cout << "[XYoffset] White core: thresh=" << static_cast<int>(coreThresh)
                  << " maxVal=" << maxVal
                  << " corePixels=" << cv::countNonZero(coreMask)
                  << " peakDist=" << distMax << std::endl;
        std::cout << "[XYoffset] Spot local=(" << localPx << ", " << localPy
                  << ") global=(" << px << ", " << py << ")" << std::endl;
    } else {
        std::cout << "[XYoffset] White core empty, fallback to local ROI gray-weighted centroid" << std::endl;
        double sumW = 0.0;
        double sumWxGlobal = 0.0;
        double sumWyGlobal = 0.0;
        double sumWxLocal = 0.0;
        double sumWyLocal = 0.0;
        for (int y = 0; y < grayRoi.rows; ++y) {
            const uint8_t* grayRow = grayRoi.ptr<uint8_t>(y);
            for (int x = 0; x < grayRoi.cols; ++x) {
                const double weight = static_cast<double>(grayRow[x]);
                const double localX = roi.x + x;
                const double localY = roi.y + y;
                const double globalX = searchRoi.x + localX;
                const double globalY = searchRoi.y + localY;
                sumW += weight;
                sumWxLocal += weight * localX;
                sumWyLocal += weight * localY;
                sumWxGlobal += weight * globalX;
                sumWyGlobal += weight * globalY;
            }
        }
        if (sumW < 1e-6) {
            errorMsg = "strict ROI miss: zero intensity in local laser ROI";
            return false;
        }
        const double localPx = sumWxLocal / sumW;
        const double localPy = sumWyLocal / sumW;
        px = sumWxGlobal / sumW;
        py = sumWyGlobal / sumW;
        std::cout << "[XYoffset] Spot local=(" << localPx << ", " << localPy
                  << ") global=(" << px << ", " << py << ")" << std::endl;
    }

    std::cout << "[XYoffset] Spot: (" << px << ", " << py
              << ")  haloArea=" << bestArea
              << " fixedROI=(" << searchRoi.x << "," << searchRoi.y
              << "," << searchRoi.width << "," << searchRoi.height << ")"
              << std::endl;
    return true;
}

bool XYoffsetService::detectLaserSpotAvg(const cv::Mat& mapX,
                                         const cv::Mat& mapY,
                                         double& outPx,
                                         double& outPy,
                                         std::string& errorMsg) const {
    std::vector<double> xs;
    std::vector<double> ys;

    for (int i = 0; i < kDetectFrames; ++i) {
        if (i > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kDetectIntervalMs));
        }

        cv::Mat raw;
        if (!captureImage(raw, errorMsg)) {
            return false;
        }

        cv::Mat overhead;
        if (mapX.empty() || mapY.empty()) {
            overhead = raw;
        } else {
            cv::remap(raw, overhead, mapX, mapY, cv::INTER_LINEAR);
        }

        if (i == 0) {
            const double expectedPx = target_x_ / mm_per_pixel_;
            const double expectedPy = target_y_ / mm_per_pixel_;
            cv::imwrite("/tmp/blobdetect.jpg", overhead);
            std::cout << "[XYoffset] blobdetect.jpg saved" << std::endl;
            std::cout << "[XYoffset] measurement_context target_x=" << target_x_
                      << " target_y=" << target_y_
                      << " mm_per_pixel=" << mm_per_pixel_
                      << " expectedPx=" << expectedPx
                      << " expectedPy=" << expectedPy << std::endl;
        }

        double px = 0.0;
        double py = 0.0;
        std::string detectError;
        if (!detectLaserSpot(overhead, px, py, detectError)) {
            std::cout << "[XYoffset] Frame " << i << " skipped: " << detectError << std::endl;
            continue;
        }

        xs.push_back(px);
        ys.push_back(py);
        std::cout << "[XYoffset] Frame " << i << ": (" << px << ", " << py << ")" << std::endl;
    }

    if (xs.empty()) {
        errorMsg = "Laser spot not detected in any of " + std::to_string(kDetectFrames) + " frames";
        return false;
    }

    if (xs.size() >= 3) {
        auto tmp = xs;
        std::sort(tmp.begin(), tmp.end());
        const double medX = tmp[tmp.size() / 2];

        tmp = ys;
        std::sort(tmp.begin(), tmp.end());
        const double medY = tmp[tmp.size() / 2];

        std::vector<double> filteredX;
        std::vector<double> filteredY;
        for (size_t i = 0; i < xs.size(); ++i) {
            if (std::abs(xs[i] - medX) < 5.0 && std::abs(ys[i] - medY) < 5.0) {
                filteredX.push_back(xs[i]);
                filteredY.push_back(ys[i]);
            }
        }
        if (!filteredX.empty()) {
            xs = filteredX;
            ys = filteredY;
        }
    }

    outPx = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
    outPy = std::accumulate(ys.begin(), ys.end(), 0.0) / ys.size();
    std::cout << "[XYoffset] Averaged " << xs.size() << " frames: ("
              << outPx << ", " << outPy << ")" << std::endl;
    return true;
}
//入口函数
bool XYoffsetService::calibrate(double& outDxPx, double& outDyPx, std::string& errorMsg) {
    KlipperManager& klipper = KlipperManager::instance();

    std::cout << "[XYoffset] Step 1: Move to Z=0, then X=" << target_x_ << " Y=" << target_y_ << std::endl;
    {
        klipper.setFillLight(0, &errorMsg);

        std::ostringstream script;
        script.setf(std::ios::fixed);
        script.precision(3);
        script << "G90\nG1 Z0 \nM400\n"
               << "G1 X" << target_x_ << " Y" << target_y_ << " F4000\nM400\n";

        if (!klipper.sendGcode(script.str(), nullptr, 40L, &errorMsg)) {
            return false;
        }
    }

    std::cout << "[XYoffset] Step 2: Move to fixed Z=" << kFixedCalibZ
              << " (no height probing)" << std::endl;
    {
        std::ostringstream script;
        script.setf(std::ios::fixed);
        script.precision(3);
        script << "G90\nG1 Z" << kFixedCalibZ << " F2000\nM400\n";

        if (!klipper.sendGcode(script.str(), nullptr, 40L, &errorMsg)) {
            errorMsg = "Move to fixed Z failed: " + errorMsg;
            return false;
        }
    }

    std::cout << "[XYoffset] Step 3: Laser on" << std::endl;
    if (!klipper.laserOn(&errorMsg)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout << "[XYoffset] Step 4: Build overhead maps (offset=0)" << std::endl;
    cameraInit();
    cameraSetOverheadXYOffset(0.0, 0.0);
    std::cout << "[XYoffset] using_zero_offset_overhead_for_measurement" << std::endl;
    cv::Mat mapX;
    cv::Mat mapY;
    getOverHeadMaps(mapX, mapY);
    if (mapX.empty() || mapY.empty()) {
        std::cout << "[XYoffset] Warning: overhead maps unavailable, using raw image" << std::endl;
    } else {
        std::cout << "[XYoffset] map_ready width=" << mapX.cols
                  << " height=" << mapX.rows
                  << " mapX_type=" << mapX.type()
                  << " mapY_type=" << mapY.type() << std::endl;
    }

    std::cout << "[XYoffset] Step 5: Detect laser spot (" << kDetectFrames << " frames)" << std::endl;
    double pixelX = 0.0;
    double pixelY = 0.0;
    if (!detectLaserSpotAvg(mapX, mapY, pixelX, pixelY, errorMsg)) {
        std::string offErr;
        klipper.laserOff(&offErr);
        return false;
    }

    std::cout << "[XYoffset] Step 6: Laser off" << std::endl;
    {
        std::string offErr;
        if (!klipper.laserOff(&offErr)) {
            std::cout << "[XYoffset] WARNING: laserOff failed: " << offErr << std::endl;
        }
    }

    const double expectedPx = target_x_ / mm_per_pixel_;
    const double expectedPy = target_y_ / mm_per_pixel_;
    const double dxPx = std::round((pixelX - expectedPx) * 10.0) / 10.0;
    const double dyPx = std::round((pixelY - expectedPy) * 10.0) / 10.0;

    const std::streamsize oldPrecision = std::cout.precision();
    const std::ios::fmtflags oldFlags = std::cout.flags();
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "[XYoffset] Expected pixel: (" << expectedPx << ", " << expectedPy << ")" << std::endl;
    std::cout << "[XYoffset] Detected pixel: (" << pixelX << ", " << pixelY << ")" << std::endl;
    std::cout << "[XYoffset] Offset:  dxPx=" << dxPx << "  dyPx=" << dyPx << " px" << std::endl;
    std::cout.precision(oldPrecision);
    std::cout.flags(oldFlags);

    klipper.setFillLight(128, &errorMsg);

    std::cout << "[XYoffset] Step 9: Write config to " << conf_path_ << std::endl;
    ReallinkCVConfig cfgData;
    readReallinkCVConf(conf_path_, cfgData);
    cfgData.cam0.dxPx = dxPx;
    cfgData.cam0.dyPx = dyPx;
    if (!writeReallinkCVConf(conf_path_, cfgData)) {
        errorMsg = "Failed to write config file: " + conf_path_;
        return false;
    }

    cameraReloadConfAndRebuildMaps();
    std::cout << "[XYoffset] Config saved. Calibration complete." << std::endl;

    outDxPx = dxPx;
    outDyPx = dyPx;
    return true;
}
