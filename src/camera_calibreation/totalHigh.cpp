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

#include "../config.h"
#include "../calib/camToolKit/calibData.h"
#include "klipper/klipper_manager.h"
#define RdcnRatio 1

static constexpr double kTotalHighMeasureZ = 25.0;
static constexpr int kTotalHighQueryRetries = 3;
static constexpr int kTotalHighTriggerSettleMs = 150;

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
        outHeight = currentZ * RdcnRatio + distance;
        std::cout << "[TotalHigh] Measurement successful! "
                  << "Z=" << currentZ << "mm * RdcnRatio=" << RdcnRatio
                  << " + distance=" << distance << "mm"
                  << " => totalHigh=" << outHeight << "mm" << std::endl;
        saveTotalHigh(outHeight);

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

void TotalHighService::saveTotalHigh(double height) {
    ReallinkCVConfig cvConfig;
    readReallinkCVConf(conf_path_, cvConfig);
    cvConfig.totalHigh = height;
    if (writeReallinkCVConf(conf_path_, cvConfig)) {
        std::cout << "[TotalHigh] Saved totalHigh=" << height << "mm to " << conf_path_ << std::endl;
    } else {
        std::cerr << "[TotalHigh] Failed to save totalHigh to " << conf_path_ << std::endl;
    }
}

bool TotalHighService::readTotalHigh(double& outHeight, std::string& errorMsg) const {
    const std::string bin_path = std::string(CALIB_RESULT_DIR) + "/" + std::string(CALIB_BIN_NAME);
    if (WRbin::instance().read(bin_path, &outHeight, sizeof(outHeight))) {
        errorMsg.clear();
        return true;
    }

    ReallinkCVConfig cvConfig;
    if (!readReallinkCVConf(conf_path_, cvConfig)) {
        errorMsg = "Failed to read totalHigh from both bin and reallinkCV.conf";
        return false;
    }

    outHeight = cvConfig.totalHigh;
    if (outHeight <= 0.0) {
        errorMsg = "Invalid totalHigh in reallinkCV.conf";
        return false;
    }

    std::cout << "[TotalHigh] Fallback read totalHigh=" << outHeight
              << "mm from " << conf_path_ << std::endl;
    errorMsg.clear();
    return true;
}
