#ifndef XYOFFSET_H
#define XYOFFSET_H

#include <string>
#include <opencv2/core/mat.hpp>

#include "../config.h"

class FrameProvider;
class KlipperManager;

class XYoffsetService {
public:
    explicit XYoffsetService(KlipperManager* klipper = nullptr,
                             FrameProvider* frameProvider = nullptr,
                             double targetX = 150.0,
                             double targetY = 150.0,
                             double mmPerPixel = 0.25,
                             const char* confPath = REALLINK_CV_CONF_PATH);
    ~XYoffsetService() = default;

    bool calibrate(double& outDxPx, double& outDyPx, std::string& errorMsg);

private:
    bool captureImage(cv::Mat& out, std::string& errorMsg) const;
    bool detectLaserSpot(const cv::Mat& image,
                         double& px,
                         double& py,
                         std::string& errorMsg) const;
    bool detectLaserSpotAvg(const cv::Mat& mapX,
                            const cv::Mat& mapY,
                            double& outPx,
                            double& outPy,
                            std::string& errorMsg) const;

    double target_x_;
    double target_y_;
    double mm_per_pixel_;
    std::string conf_path_;
    KlipperManager* klipper_ = nullptr;
    FrameProvider* frame_provider_ = nullptr;

    static constexpr double kDefaultTargetX = 150.0;
    static constexpr double kDefaultTargetY = 150.0;
    static constexpr double kDefaultMmPerPixel = 0.25;
    static constexpr double kFixedCalibZ = 32.0;
    static constexpr int kMainCameraRefreshTimeoutMs = 5000;

    static constexpr int kDetectFrames = 3;
    static constexpr int kDetectIntervalMs = 200;
    static constexpr int kSearchX = 400;
    static constexpr int kSearchY = 400;
    static constexpr int kSearchW = 400;
    static constexpr int kSearchH = 400;
};

#endif // XYOFFSET_H
