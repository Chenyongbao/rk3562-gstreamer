#include "totalHigh.h"
#include "../reallink_ogles/file_utils.h"
#include "../tools/WRbin.h"
#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <cstring>
#include <thread>
#include <chrono>
#include <cerrno>
#include <cmath>

#include "../config.h"
#include "../calib/camToolKit/calibData.h"
#include "../klipper/klipper_manager.h"
static constexpr double kRdcnRatio = 1.0;

static constexpr double kTotalHighMeasureZ = 25.0;
static constexpr int kTotalHighQueryRetries = 3;
static constexpr int kTotalHighTriggerSettleMs = 150;

namespace {

bool isPositiveFinite(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool readTotalHighFromBin(const std::string& bin_path,
                          double& out_height,
                          std::string* error_msg) {
    std::vector<uint8_t> blob;
    if (!WRbin::instance().readAll(bin_path, blob)) {
        if (error_msg) {
            *error_msg = "Failed to read totalHigh bin: " + bin_path;
        }
        return false;
    }
    if (blob.size() < sizeof(CalibData)) {
        if (error_msg) {
            *error_msg = "totalHigh bin too small: " + bin_path;
        }
        return false;
    }

    CalibData calib {};
    std::memcpy(&calib, blob.data(), sizeof(CalibData));
    out_height = calib.distCoeffs[4];
    if (!isPositiveFinite(out_height)) {
        if (error_msg) {
            *error_msg = "Invalid totalHigh in bin";
        }
        return false;
    }
    return true;
}

bool readTotalHighFromConf(const std::string& conf_path,
                           double& out_height,
                           std::string* error_msg) {
    ReallinkCVConfig cv_config;
    if (!readReallinkCVConf(conf_path, cv_config)) {
        if (error_msg) {
            *error_msg = "Failed to read totalHigh conf: " + conf_path;
        }
        return false;
    }

    out_height = cv_config.totalHigh;
    if (!isPositiveFinite(out_height)) {
        if (error_msg) {
            *error_msg = "Invalid totalHigh in reallinkCV.conf";
        }
        return false;
    }
    return true;
}

} // namespace

bool readPersistedTotalHigh(const std::string& confPath,
                            const std::string& binPath,
                            double& outHeight,
                            std::string* errorMsg) {
    double conf_height = 0.0;
    double bin_height = 0.0;
    std::string conf_error;
    std::string bin_error;
    const bool has_conf = readTotalHighFromConf(confPath, conf_height, &conf_error);
    const bool has_bin = readTotalHighFromBin(binPath, bin_height, &bin_error);

    if (has_bin) {
        outHeight = bin_height;
        if (errorMsg) {
            errorMsg->clear();
        }
        return true;
    }

    if (has_conf) {
        outHeight = conf_height;
        if (errorMsg) {
            errorMsg->clear();
        }
        return true;
    }

    if (errorMsg) {
        if (!conf_error.empty() && !bin_error.empty()) {
            *errorMsg = conf_error + " | " + bin_error;
        } else if (!conf_error.empty()) {
            *errorMsg = conf_error;
        } else if (!bin_error.empty()) {
            *errorMsg = bin_error;
        } else {
            *errorMsg = "Failed to read totalHigh from both bin and conf";
        }
    }
    return false;
}

TotalHighService::TotalHighService(double feedrate, const char* confPath)
    : feedrate_(feedrate > 0 ? feedrate : CALIB_FEEDRATE),
      conf_path_(confPath ? confPath : REALLINK_CV_CONF_PATH) {}

bool TotalHighService::measure(double& outHeight, std::string& errorMsg) {
    try {
        // 1. Z轴下降测量
        std::cout << "[TotalHigh] Step 1: Move directly to Z=" << kTotalHighMeasureZ
                  << "mm and measure once..." << std::endl;

        {
            std::ostringstream script;
            script.setf(std::ios::fixed);
            script.precision(3);
            script << "G90\n";
            script << "G1 Z" << kTotalHighMeasureZ << " F" << feedrate_ << "\n";
            script << "M400\n";
            sendGcodeScript(script.str());
        }

        std::string trigger_error;
        if (!KlipperManager::instance().sendGcode(
                "LASER_RANGE_SENSOR SENSOR=my_range_sensor", nullptr, 20L, &trigger_error)) {
            errorMsg = trigger_error.empty() ? "Failed to trigger laser range sensor" : trigger_error;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kTotalHighTriggerSettleMs));

        double distance = 0.0;
        bool valid = false;
        std::string query_error;
        if (!KlipperManager::instance().queryLaserDistance(
                distance, valid, &query_error, kTotalHighQueryRetries)) {
            errorMsg = query_error.empty() ? "Failed to query laser range sensor" : query_error;
            return false;
        }

        if (!valid) {
            errorMsg = "Laser range sensor returned invalid measurement at Z=25mm";
            return false;
        }

        const double currentZ = kTotalHighMeasureZ;
        outHeight = currentZ * kRdcnRatio + distance;
        std::cout << "[TotalHigh] Measurement successful! "
                  << "Z=" << currentZ << "mm * kRdcnRatio=" << kRdcnRatio
                  << " + distance=" << distance << "mm"
                  << " => totalHigh=" << outHeight << "mm" << std::endl;
        if (!saveTotalHigh(outHeight, errorMsg)) {
            return false;
        }

        const std::string bin_path = std::string(CALIB_RESULT_DIR) + "/" + std::string(CALIB_BIN_NAME);
        std::vector<uint8_t> blob;
        if (!WRbin::instance().readAll(bin_path, blob)) {
            std::cerr << "[TotalHigh] 读取 " << bin_path << " 失败，在写入 totalHigh" << std::endl;
        } else if (blob.size() < sizeof(CalibData)) {
            std::cerr << "[TotalHigh] " << bin_path << " 太小 (" << blob.size()
                      << "), 期望大于等于 " << sizeof(CalibData) << std::endl;
        } else {
            CalibData calib{};
            std::memcpy(&calib, blob.data(), sizeof(CalibData));
            calib.distCoeffs[4] = outHeight;
            calib.checksum = calibdata_calc_checksum(&calib);

            if (!WRbin::instance().write(bin_path, &calib, sizeof(CalibData))) {
                std::cerr << "[TotalHigh] 更新 " << bin_path << " 中的 totalHigh 失败" << std::endl;
            } else {
                std::cout << "[TotalHigh] 更新 " << bin_path << " 中的 totalHigh 到 CalibData.distCoeffs[4]" << std::endl;
            }
        }

        {   //归零
            std::ostringstream script;
            script.setf(std::ios::fixed);
            script.precision(3);
            script << "G90\n";
            script << "G1 Z10 F" << feedrate_ << "\n";
            script << "M400\n";
            sendGcodeScript(script.str());
        }
        return true;

    } catch (const std::exception& e) {
        errorMsg = e.what();
        return false;
    }
}

void TotalHighService::sendGcodeScript(const std::string& script) {
    std::string err;
    if (!KlipperManager::instance().sendGcode(script, nullptr, 20L, &err)) {
        throw std::runtime_error(err);
    }
}

bool TotalHighService::saveTotalHigh(double height, std::string& errorMsg) {
    ReallinkCVConfig cvConfig;
    if (!readReallinkCVConf(conf_path_, cvConfig)) {
        std::cout << "[TotalHigh] Config not readable before save, will create fresh file: "
                  << conf_path_ << std::endl;
    }
    cvConfig.totalHigh = height;
    if (writeReallinkCVConf(conf_path_, cvConfig)) {
        std::cout << "[TotalHigh] Saved totalHigh=" << height << "mm to " << conf_path_ << std::endl;
        errorMsg.clear();
        return true;
    } else {
        std::cerr << "[TotalHigh] Failed to save totalHigh to " << conf_path_ << std::endl;
        errorMsg = "Failed to save totalHigh to config: " + conf_path_;
        return false;
    }
}

bool TotalHighService::readTotalHigh(double& outHeight, std::string& errorMsg) const {
    const std::string bin_path = std::string(CALIB_RESULT_DIR) + "/" + std::string(CALIB_BIN_NAME);
    if (!readPersistedTotalHigh(conf_path_, bin_path, outHeight, &errorMsg)) {
        return false;
    }
    std::cout << "[TotalHigh] Read totalHigh=" << outHeight
              << "mm using persisted source resolution" << std::endl;
    return true;
}
