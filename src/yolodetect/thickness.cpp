#include "thickness.h"

#include "../camera_calibreation/totalHigh.h"
#include "../klipper/klipper_manager.h"
#include "../reallink_ogles/camera.h"
#include "../reallink_ogles/file_utils.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace {

// totalHigh 判零时允许的浮点误差，避免把极小噪声当成有效值。
constexpr double kTotalHighZeroEps = 1e-9;
// 无检测目标时的兜底移动坐标（用于保证流程可继续执行）。
constexpr double kFallbackMoveX = 160.0;
constexpr double kFallbackMoveY = 160.0;
// 厚度测高从 Z=2 起始；若无数据，则回退到固定的 Z=10/20/30 点位探测。
constexpr double kInitialProbeZ = 2.0;
constexpr std::array<double, 3> kFallbackProbeZPoints = {10.0, 20.0, 30.0};
constexpr double kTargetDistanceMm = 55.0;
constexpr double kTargetDistanceToleranceMm = 2.0;
constexpr int kFixedSampleQueryRetries = 3;
constexpr int kFixedSampleQueryRetryDelayMs = 120;
constexpr int kFixedSampleSettleMs = 150;
constexpr long kFixedSampleMoveTimeoutSec = 20L;
constexpr long kFixedSampleTriggerTimeoutSec = 20L;

enum class FixedZSampleStatus {
    Valid,
    NoValidData,
    MoveFailed,
    TriggerFailed,
    QueryFailed
};

struct FixedZSampleResult {
    FixedZSampleStatus status = FixedZSampleStatus::NoValidData;
    double z_mm = 0.0;
    double distance_mm = 0.0;
};

bool isDistanceWithinTargetRange(double distance_mm) {
    return std::fabs(distance_mm - kTargetDistanceMm) <= kTargetDistanceToleranceMm;
}

FixedZSampleResult sampleDistanceAtFixedZ(double z_target_mm, double feedrate, const std::string& log_prefix) {
    FixedZSampleResult result;
    result.z_mm = z_target_mm;

    std::ostringstream z_script;
    z_script.setf(std::ios::fixed);
    z_script.precision(3);
    z_script << "G90\n";
    z_script << "G1 Z" << z_target_mm << " F" << feedrate << "\n";
    z_script << "M400\n";

    std::string move_error;
    if (!KlipperManager::instance().sendGcode(z_script.str(), nullptr, kFixedSampleMoveTimeoutSec, &move_error)) {
        result.status = FixedZSampleStatus::MoveFailed;
        return result;
    }

    std::string trigger_error;
    if (!KlipperManager::instance().sendGcode("LASER_RANGE_SENSOR SENSOR=my_range_sensor",
                                              nullptr,
                                              kFixedSampleTriggerTimeoutSec,
                                              &trigger_error)) {
        result.status = FixedZSampleStatus::TriggerFailed;
        return result;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kFixedSampleSettleMs));

    std::string last_query_error;
    for (int attempt = 0; attempt < kFixedSampleQueryRetries; ++attempt) {
        double distance_mm = 0.0;
        bool valid = false;
        std::string query_error;
        if (KlipperManager::instance().queryLaserDistance(distance_mm, valid, &query_error, 1)) {
            result.distance_mm = distance_mm;
            if (valid) {
                result.status = FixedZSampleStatus::Valid;
                if (!log_prefix.empty()) {
                    std::cout << log_prefix << " Fixed-Z sample success: Z=" << z_target_mm
                              << " distance=" << distance_mm << std::endl;
                }
                return result;
            }
        } else {
            last_query_error = query_error;
        }

        if (attempt + 1 < kFixedSampleQueryRetries) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kFixedSampleQueryRetryDelayMs));
        }
    }

    result.status = last_query_error.empty() ? FixedZSampleStatus::NoValidData : FixedZSampleStatus::QueryFailed;
    return result;
}

std::string buildFixedZSampleError(const FixedZSampleResult& sample_result, const std::string& phase_label) {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);

    switch (sample_result.status) {
    case FixedZSampleStatus::Valid:
        return "";
    case FixedZSampleStatus::NoValidData:
        oss << phase_label << " invalid laser distance at Z=" << sample_result.z_mm;
        break;
    case FixedZSampleStatus::MoveFailed:
        oss << phase_label << " failed to move to Z=" << sample_result.z_mm;
        break;
    case FixedZSampleStatus::TriggerFailed:
        oss << phase_label << " failed to trigger laser sensor at Z=" << sample_result.z_mm;
        break;
    case FixedZSampleStatus::QueryFailed:
        oss << phase_label << " failed to query laser distance at Z=" << sample_result.z_mm;
        break;
    }
    return oss.str();
}

} // namespace

ThicknessService::ThicknessService(const ThicknessConfig& config)
    : feedrate_(config.feedrate),
      z_step_(config.z_step),
      z_max_(config.z_max),
      max_attempts_(config.max_attempts),
      conf_path_(config.conf_path),
      // pixel_ratio 非法时回退到默认值，避免像素坐标换算异常。
      pixel_ratio_(config.pixel_ratio > 0.0 ? config.pixel_ratio : 4.0) {}
//面积检测逻辑
bool ThicknessService::selectTargetDetection(const YOLOFrameResult& yolo_result,
                                             YOLODetection& selected_det,
                                             bool& selected_by_max_area,
                                             std::string& errorMsg) const {
    if (yolo_result.detection_count <= 0) {
        errorMsg = "No object detected";
        return false;
    }

    selected_by_max_area = false;
    selected_det = yolo_result.detections[0];

    if (yolo_result.detection_count == 1) {
        return true; // 单目标：不触发“最大面积选择”
    }

    // 多目标场景下按面积最大策略选主目标，提升稳定性。
    long long best_area = std::numeric_limits<long long>::min();
    int best_index = 0;
    for (int i = 0; i < yolo_result.detection_count; ++i) {
        const YOLODetection& det = yolo_result.detections[i];
        const long long w = std::max(det.w, 0);
        const long long h = std::max(det.h, 0);
        const long long area = w * h;
        if (area > best_area) {
            best_area = area;
            best_index = i;
        }
    }

    selected_det = yolo_result.detections[best_index];
    selected_by_max_area = true;
    return true;
}

bool ThicknessService::moveToXY(double x, double y, std::string& errorMsg) const {
    // 构造绝对坐标运动指令并等待运动完成。
    std::ostringstream script;
    script.setf(std::ios::fixed);
    script.precision(3);
    script << "G90\n";
    // 先仅移动 XY，避免 XYZ 同时联动
    script << "G1 X" << x << " Y" << y << " F" << feedrate_ << "\n";
    script << "M400\n";

    std::string move_error;
    if (!KlipperManager::instance().sendGcode(script.str(), nullptr, 20L, &move_error)) {
        errorMsg = move_error.empty() ? "Failed to move to target XY" : move_error;
        return false;
    }
    return true;
}

bool ThicknessService::probeDistanceAndZDrop(double& out_distance_mm,
                                             double& out_z_drop_mm,
                                             std::string& errorMsg) const {
    auto forceHomeAfterNoData = [&](const std::string& reason) {
        errorMsg = reason;
        std::string home_error;
        if (!KlipperManager::instance().forceHome(&home_error)) {
            std::cout << "[Thickness] WARNING: Auto home after no-data probe failed: "
                      << home_error << std::endl;
            if (!home_error.empty()) {
                errorMsg += " | auto home failed: " + home_error;
            }
        } else {
            std::cout << "[Thickness] No valid probe data, forced homing completed" << std::endl;
        }
    };

    auto acceptSample = [&](const FixedZSampleResult& sample_result, const std::string& reason) {
        out_distance_mm = sample_result.distance_mm;
        out_z_drop_mm = sample_result.z_mm;
        std::cout << "[Thickness] " << reason
                  << " final_distance=" << out_distance_mm
                  << " final_z_drop=" << out_z_drop_mm << std::endl;
        return true;
    };

    auto adjustSampleIntoTargetRange = [&](FixedZSampleResult sample_result,
                                           const std::string& phase_label) -> bool {
        if (sample_result.distance_mm < kTargetDistanceMm) {
            return acceptSample(sample_result, phase_label + " valid but below target 55, keep current reading");
        }

        int adjustment_attempts = 0;
        while (true) {
            if (isDistanceWithinTargetRange(sample_result.distance_mm)) {
                return acceptSample(sample_result, phase_label + " reached target range 55±2");
            }

            if (sample_result.distance_mm < kTargetDistanceMm) {
                return acceptSample(sample_result, phase_label + " adjusted below 55, keep current reading");
            }

            if (adjustment_attempts >= std::max(max_attempts_, 1)) {
                std::ostringstream oss;
                oss << phase_label << " exceeded max adjustment attempts while targeting 55±2";
                errorMsg = oss.str();
                return false;
            }

            const double delta_z_mm = sample_result.distance_mm - kTargetDistanceMm;
            const double next_z_mm = sample_result.z_mm + delta_z_mm;
            if (next_z_mm > z_max_) {
                std::ostringstream oss;
                oss.setf(std::ios::fixed);
                oss.precision(3);
                oss << phase_label << " adjusted Z target " << next_z_mm
                    << " exceeds limit " << z_max_;
                errorMsg = oss.str();
                return false;
            }

            std::cout << "[Thickness] " << phase_label
                      << " distance=" << sample_result.distance_mm
                      << " above target, descend additional " << delta_z_mm
                      << " to Z=" << next_z_mm << std::endl;

            sample_result = sampleDistanceAtFixedZ(next_z_mm, feedrate_, "[Thickness]");
            if (sample_result.status != FixedZSampleStatus::Valid) {
                errorMsg = buildFixedZSampleError(sample_result, phase_label + " adjustment");
                return false;
            }

            ++adjustment_attempts;
        }
    };

    const FixedZSampleResult initial_sample = sampleDistanceAtFixedZ(kInitialProbeZ, feedrate_, "[Thickness]");
    if (initial_sample.status == FixedZSampleStatus::Valid) {
        return adjustSampleIntoTargetRange(initial_sample, "Initial Z=2 probe");
    }

    if (initial_sample.status != FixedZSampleStatus::NoValidData) {
        errorMsg = buildFixedZSampleError(initial_sample, "Initial Z=2 probe");
        return false;
    }

    for (double fallback_z_mm : kFallbackProbeZPoints) {
        const FixedZSampleResult fallback_sample = sampleDistanceAtFixedZ(fallback_z_mm, feedrate_, "[Thickness]");
        if (fallback_sample.status == FixedZSampleStatus::Valid) {
            return adjustSampleIntoTargetRange(
                fallback_sample,
                "Fallback probe at Z=" + std::to_string(static_cast<int>(fallback_z_mm)));
        }
        if (fallback_sample.status != FixedZSampleStatus::NoValidData) {
            errorMsg = buildFixedZSampleError(
                fallback_sample,
                "Fallback probe at Z=" + std::to_string(static_cast<int>(fallback_z_mm)));
            return false;
        }
    }

    forceHomeAfterNoData("No valid probe data from Z=2 or fallback Z=10/20/30");
    return false;
}

bool ThicknessService::measureFromYolo(const YOLOFrameResult& yolo_result,
                                       ThicknessResult& result,
                                       std::string& errorMsg) {
    // 统一初始化，避免调用方读取到脏数据。
    result = ThicknessResult{};
    result.triggered = true;
    result.detection_count = yolo_result.detection_count;

    bool success = false;
    try {
        do {
            if (yolo_result.detection_count > 0) {
                // 有检测结果时：先选定目标，再从 ROI 计算机械坐标。
                YOLODetection det{};
                bool selected_by_max_area = false;
                if (!selectTargetDetection(yolo_result, det, selected_by_max_area, errorMsg)) {
                    break;
                }
                result.selected_by_max_area = selected_by_max_area;
                result.class_id = det.class_id;
                result.confidence = det.confidence;
                result.roi_x = det.x;
                result.roi_y = det.y;
                result.roi_w = det.w;
                result.roi_h = det.h;

                result.center_x = static_cast<double>(det.x) + static_cast<double>(det.w) / 2.0;
                result.center_y = static_cast<double>(det.y) + static_cast<double>(det.h) / 2.0;

                // 像素坐标转机械坐标（含机构偏移补偿）。
                result.move_x = (result.center_x / pixel_ratio_) + 5.0;   //机械设计
                result.move_y = (result.center_y / pixel_ratio_) + 48.0;
            } else {
                // 无检测目标时走兜底点位，便于继续测高链路。
                result.move_x = kFallbackMoveX;
                result.move_y = kFallbackMoveY;
                std::cout << "[Thickness] No object detected, fallback move_xy=("
                          << result.move_x << "," << result.move_y << ")" << std::endl;
            }

            if (!moveToXY(result.move_x, result.move_y, errorMsg)) {
                break;
            }

            if (!probeDistanceAndZDrop(result.distance_mm, result.z_drop_mm, errorMsg)) {
                break;
            }
            // 物料表面高度 = 探测距离 + Z 轴下探量。
            result.measured_height_mm = result.distance_mm + result.z_drop_mm;

            const std::string bin_path =
                std::string(CALIB_RESULT_DIR) + "/" + std::string(CALIB_BIN_NAME);
            if (!readPersistedTotalHigh(conf_path_, bin_path, result.total_high_mm, &errorMsg)) {
                break;
            }

            if (std::fabs(result.total_high_mm) <= kTotalHighZeroEps) {
                errorMsg = "Invalid totalHigh (0), abort thickness calculation";
                break;
            }

            // 厚度 = 设备总高 - 探测距离 - Z 下探量。
            result.thickness_high_mm = result.total_high_mm - result.distance_mm - result.z_drop_mm;
            if (!setMaterialThickness(result.thickness_high_mm)) {
                errorMsg = "Failed to apply runtime thickness compensation";
                break;
            }

            std::cout << "[Thickness] det_count=" << result.detection_count
                      << " roi_xywh=[" << result.roi_x << "," << result.roi_y
                      << "," << result.roi_w << "," << result.roi_h << "]"
                      << " center_xy=(" << result.center_x << "," << result.center_y << ")"
                      << " move_xy=(" << result.move_x << "," << result.move_y << ")"
                      << " distance=" << result.distance_mm
                      << " z_drop=" << result.z_drop_mm
                      << " totalHigh=" << result.total_high_mm
                      << " thicknesshigh=" << result.thickness_high_mm
                      << " selected_by_max_area=" << (result.selected_by_max_area ? "true" : "false")
                      << std::endl;

            success = true;
        } while (false);
    } catch (const std::exception& e) {
        errorMsg = e.what();
        success = false;
    }
    return success;
}
