#include "vice_camera_internal.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "../klipper/klipper_manager.h"

namespace {

class LocalSessionStore final : public ISessionStore {
public:
    std::string createSessionDir(const std::string& capture_dir) override {
        if (mkdirRecursive(capture_dir) != 0) {
            std::cerr << "[ViceCamera] Failed to create capture dir: " << capture_dir << std::endl;
            return {};
        }
        return capture_dir;
    }

    void removeFile(const std::string& file_path) override {
        if (std::remove(file_path.c_str()) != 0) {
            std::cerr << "[ViceCamera] Warning: Failed to remove temp image: "
                      << file_path << std::endl;
        }
    }
};

} // namespace

std::unique_ptr<ISessionStore> makeLocalSessionStore() {
    return std::make_unique<LocalSessionStore>();
}

// 预设 16 个采样点，覆盖平台边缘与角落区域，兼顾标定稳定性与运动安全。
const std::vector<std::pair<double, double>> ViceCameraService::fixed_coordinates_ = {
    {0.0, 75.0}, {0.0, 90.0}, {30.0, 90.0}, {30.0, 75.0},
    {257.0, 75.0}, {290.0, 75.0}, {290.0, 90.0}, {257.0, 90.0},
    {295.0, 300.0}, {284.0, 300.0}, {273.0, 300.0}, {262.0, 300.0},
    {30.0, 300.0}, {22.0, 300.0}, {11.0, 300.0}, {0.0, 300.0}
};

ViceCameraService::ViceCameraService(ViceCameraConfig config,
                                     std::unique_ptr<IImageCapturer> image_capturer,
                                     std::unique_ptr<ICalibrationEngine> calibration_engine,
                                     std::unique_ptr<ISessionStore> session_store)
    : config_(std::move(config)) {
    image_capturer_ = image_capturer ? std::move(image_capturer)
                                     : makeV4L2CtlImageCapturer();
    calibration_engine_ = calibration_engine ? std::move(calibration_engine)
                                             : makeOpenCvCalibrationEngine();
    session_store_ = session_store ? std::move(session_store)
                                   : makeLocalSessionStore();
}

ViceCameraService::~ViceCameraService() = default;

ViceCameraResult ViceCameraService::run(bool skip_homing) {
    ViceCameraResult result;
    std::string error_msg;
    bool has_captured_images = false;

    try {
        if (!createSessionDir(error_msg)) {
            result.error = error_msg;
            return result;
        }

        result.latest_dir = latest_capture_dir_;
        std::cout << "[ViceCamera] Created session directory: " << latest_capture_dir_ << std::endl;

        clearSessionState();
        calibration_engine_->reset();

        if (!skip_homing) {
            std::string homing_error;
            if (!forceHome(homing_error)) {
                result.error = "Homing failed: " + homing_error;
                return result;
            }
        }

        if (!setFillLight(25, error_msg)) {
            result.error = error_msg;
            return result;
        }

        if (!captureLoop(error_msg)) {
            cleanupCapturedImages();
            result.error = error_msg;
            return result;
        }
        has_captured_images = true;

        // 采集完成后统一执行内参标定、位姿整理和外参落盘。
        if (!finalizeCalibration(error_msg)) {
            if (has_captured_images) {
                cleanupCapturedImages();
            }
            result.error = error_msg;
            return result;
        }

        if (!setFillLight(128, error_msg)) {
            result.error = error_msg;
            return result;
        }

        result.success = true;
        return result;
    } catch (const std::exception& e) {
        if (has_captured_images) {
            cleanupCapturedImages();
        }
        result.error = e.what();
        result.latest_dir = latest_capture_dir_;
        return result;
    }
}

bool ViceCameraService::createSessionDir(std::string& error_msg) {
    latest_capture_dir_ = session_store_->createSessionDir(config_.capture_dir);
    if (latest_capture_dir_.empty()) {
        error_msg = "Failed to create session directory";
        return false;
    }
    return true;
}

bool ViceCameraService::forceHome(std::string& error_msg) {
    return KlipperManager::instance().forceHome(&error_msg);
}

bool ViceCameraService::sendScript(const std::string& script, std::string& error_msg) {
    return KlipperManager::instance().sendGcode(script, nullptr, 300L, &error_msg);
}

bool ViceCameraService::triggerLaserRange(std::string& error_msg) {
    return sendScript("LASER_RANGE_SENSOR SENSOR=my_range_sensor", error_msg);
}

bool ViceCameraService::queryLaserDistance(double& out_distance, bool& out_valid, std::string& error_msg) {
    // 读取一次激光测距状态，由调用方决定是否继续重试。
    const bool ok = KlipperManager::instance().queryLaserDistance(
        out_distance, out_valid, &error_msg, 1);
    if (!ok) {
        std::cerr << "[ViceCamera] query_laser_distance failed: " << error_msg << std::endl;
    }
    return ok;
}

bool ViceCameraService::setFillLight(int brightness, std::string& error_msg) {
    return KlipperManager::instance().setFillLight(brightness, &error_msg);
}

void ViceCameraService::cleanupCapturedImages() {
    // 失败路径下删除临时图片，避免目录中残留不完整采样数据。
    for (auto& txn : sample_txns_) {
        if (!txn.image_path.empty()) {
            session_store_->removeFile(txn.image_path);
            txn.image_path.clear();
        }
    }
}

void ViceCameraService::clearSessionState() {
    // 新会话开始前重置所有缓存和统计计数。
    cleanupCapturedImages();
    sample_txns_.clear();
    sample_records_.clear();
    capture_stats_ = CaptureStats{};
}
