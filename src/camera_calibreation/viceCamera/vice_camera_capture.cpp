#include "vice_camera_internal.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include <sys/stat.h>

#include <opencv2/opencv.hpp>

#include "../../calib/charuco_detector.h"
#include "../../calib/saddleFit.h"
#include "../../config.h"

namespace {

class V4L2CtlImageCapturer final : public IImageCapturer {
public:
    bool capture(const std::string& camera_device,
                 const std::string& filepath,
                 std::string& error) override {
        // 通过 v4l2-ctl 抓取单帧图像，避免引入额外采集依赖。
        std::ostringstream cmd;
        cmd << "timeout 3s " << CALIB_V4L2_CTL_PATH
            << " -d " << shellQuote(camera_device)
            << " --set-fmt-video=width=" << CALIB_FRAME_WIDTH
            << ",height=" << CALIB_FRAME_HEIGHT
            << ",pixelformat=" << CALIB_PIXEL_FORMAT
            << " --stream-mmap --stream-count=1"
            << " --stream-to=" << shellQuote(filepath);

        const int ret = std::system(cmd.str().c_str());
        if (ret != 0) {
            std::ostringstream oss;
            oss << "capture command failed (ret=" << ret << ")";
            error = oss.str();
            return false;
        }

        struct stat st;
        if (stat(filepath.c_str(), &st) != 0) {
            error = "capture output file not found";
            return false;
        }
        return true;
    }
};

} // namespace

std::unique_ptr<IImageCapturer> makeV4L2CtlImageCapturer() {
    return std::make_unique<V4L2CtlImageCapturer>();
}

bool ViceCameraService::measureHeightAtCurrentPose(double& out_height, std::string& error_msg) {
    // 先触发一次激光测距，再在短时间窗口内轮询结果。
    std::string trigger_error;
    if (!triggerLaserRange(trigger_error)) {
        error_msg = "trigger laser failed: " + trigger_error;
        return false;
    }

    std::string last_error;
    for (int retry = 0; retry < kLaserQueryRetries; ++retry) {
        double distance = 0.0;
        bool valid = false;
        std::string query_error;
        const bool ok = queryLaserDistance(distance, valid, query_error);
        if (ok && valid) {
            std::cout << "[ViceCamera] Height query hit at retry "
                      << (retry + 1) << "/" << kLaserQueryRetries
                      << ", value=" << distance << std::endl;
            out_height = distance;
            return true;
        }
        if (!query_error.empty()) {
            last_error = query_error;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    error_msg = last_error.empty()
        ? ("laser distance invalid after retries (" + std::to_string(kLaserQueryRetries) + ")")
        : ("laser distance query failed after retries: " + last_error);
    return false;
}

bool ViceCameraService::captureLoop(std::string& error_msg) {
    const int num_fixed = static_cast<int>(fixed_coordinates_.size());

    for (int i = 0; i < num_fixed; ++i) {
        const double x = fixed_coordinates_[i].first;
        const double y = fixed_coordinates_[i].second;

        SampleTxn txn;
        txn.step_index = i + 1;
        txn.gx = x;
        txn.gy = y;
        txn.gz = kCaptureZMm;

        std::ostringstream sid;
        sid << "S" << (i + 1) << "_"
            << std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
        txn.sample_id = sid.str();

        ++capture_stats_.attempted;

        // 采用绝对坐标移动到预设采样点，并在到位后短暂停顿。
        std::ostringstream script;
        script.setf(std::ios::fixed);
        script.precision(3);
        script << "G90\n";
        script << "G1 X" << x << " Y" << y << " Z" << kCaptureZMm << " F" << config_.feedrate << "\n";
        script << "G4 P500\n";

        std::cout << "[ViceCamera] Move " << txn.step_index << "/" << num_fixed
                  << " sample_id=" << txn.sample_id
                  << " to X=" << x << " Y=" << y << " Z=" << kCaptureZMm << std::endl;

        if (!sendScript(script.str(), error_msg)) {
            return false;
        }

        std::string height_error;
        double measured_height = 0.0;
        if (measureHeightAtCurrentPose(measured_height, height_error)) {
            txn.h = measured_height;
            txn.height_valid = true;
            std::cout << "[ViceCamera] sample_id=" << txn.sample_id
                      << " height(measured per-frame)=" << txn.h << std::endl;
        } else {
            txn.fail_reason = "height_fail: " + height_error;
            ++capture_stats_.height_fail;
            std::cout << "[ViceCamera] sample_id=" << txn.sample_id
                      << " height measure failed: " << height_error << std::endl;
        }

        std::ostringstream filename;
        filename << latest_capture_dir_ << "/img_"
                 << std::setw(3) << std::setfill('0') << (i + 1)
                 << "_" << txn.sample_id << ".jpg";
        const std::string filepath = filename.str();

        std::cout << "[ViceCamera] Capturing image: " << filepath << std::endl;
        std::string capture_error;
        if (!image_capturer_->capture(config_.camera_device, filepath, capture_error)) {
            txn.fail_reason = "capture_fail: " + capture_error;
            ++capture_stats_.capture_fail;
            sample_txns_.push_back(txn);
            std::cout << "[ViceCamera] Skip sample_id=" << txn.sample_id
                      << " because " << txn.fail_reason << std::endl;
            continue;
        }

        txn.image_ok = true;
        txn.image_path = filepath;

        std::cout << "[ViceCamera] Saved image: " << filepath << std::endl;
        cv::Mat image = cv::imread(filepath, cv::IMREAD_GRAYSCALE);
        if (image.empty()) {
            txn.fail_reason = "image_read_fail";
            std::cout << "[ViceCamera] sample_id=" << txn.sample_id
                      << " failed to load captured image for charuco detect" << std::endl;
        } else {
            std::vector<cv::Point2f> image_points;
            std::vector<cv::Point3f> object_points;
            // 先做 ChArUco 检测，再对角点做亚像素优化。
            const int detected = CharucoMultiBoardDetector::detect(
                image, image_points, object_points, kCharucoSquareSizeMm);
            if (detected >= kMinCharucoPointCount &&
                image_points.size() >= static_cast<size_t>(kMinSolvePnpPointCount) &&
                object_points.size() >= static_cast<size_t>(kMinSolvePnpPointCount) &&
                image_points.size() == object_points.size()) {
                ReallinkSaddlePointFit(image, image_points, kSaddleFitRadius);
                txn.charuco_ok = true;
                txn.image_points = image_points;
                txn.object_points = object_points;
                calibration_engine_->ingestDetection(image_points, object_points, image.size(), i + 1);
                std::cout << "[ViceCamera] Charuco detected on image " << (i + 1)
                          << " with " << image_points.size() << " points" << std::endl;
            } else {
                // 角点不足时记录失败原因，后续统一做统计。
                txn.fail_reason = "pose_fail: charuco points insufficient";
                std::cout << "[ViceCamera] Charuco NOT detected or too few points on image "
                          << (i + 1) << " (detected=" << detected << ")" << std::endl;
            }
        }

        sample_txns_.push_back(std::move(txn));
    }

    return true;
}
