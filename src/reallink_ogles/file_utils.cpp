#include "file_utils.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <iomanip>

// 去除字符串首尾空格
static std::string trim(const std::string& str) {
    auto notSpace = [](int ch) { return !std::isspace(ch); };
    std::string result = str;
    result.erase(result.begin(), std::find_if(result.begin(), result.end(), notSpace));
    result.erase(std::find_if(result.rbegin(), result.rend(), notSpace).base(), result.end());
    return result;
}

// 解析形如 "key = value" 的行
static bool parseDoubleLine(const std::string& line, const char* key, double& out) {
    std::string trimmed = trim(line);
    std::string prefix = std::string(key) + " =";
    if (trimmed.compare(0, prefix.size(), prefix) != 0) {
        return false;
    }
    std::istringstream iss(trimmed.substr(prefix.size()));
    return static_cast<bool>(iss >> out);
}

// 解析形如 "key = v0 v1 v2" 的行（3个double）
static bool parseVec3Line(const std::string& line, const char* key, double out[3]) {
    std::string trimmed = trim(line);
    std::string prefix = std::string(key) + " =";
    if (trimmed.compare(0, prefix.size(), prefix) != 0) {
        return false;
    }
    std::istringstream iss(trimmed.substr(prefix.size()));
    return static_cast<bool>(iss >> out[0] >> out[1] >> out[2]);
}

// 解析 cam1 下 calib1~calib4 子块起始行
static bool parseCalibBlockIndex(const std::string& line, int& out_index) {
    const std::string trimmed = trim(line);
    for (int i = 1; i <= 4; ++i) {
        const std::string block_with_space = "calib" + std::to_string(i) + " {";
        const std::string block_no_space = "calib" + std::to_string(i) + "{";
        if (trimmed == block_with_space || trimmed == block_no_space) {
            out_index = i - 1;
            return true;
        }
    }
    return false;
}


// 写入完整配置到 reallinkCV.conf
bool writeReallinkCVConf(const std::string& path, const ReallinkCVConfig& config) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        std::cerr << "[reallinkCV] Failed to open for write: " << path
                  << " (errno=" << errno << ", " << std::strerror(errno) << ")" << std::endl;
        return false;
    }

    // 写入 cam0（主摄）
    ofs << "cam0 {\n";
    ofs << "    rvec = " << config.cam0.rvec[0] << " " 
        << config.cam0.rvec[1] << " " 
        << config.cam0.rvec[2] << "\n";
    ofs << "    tvec = " << config.cam0.tvec[0] << " " 
        << config.cam0.tvec[1] << " " 
        << config.cam0.tvec[2] << "\n";
    ofs << "    dxPx = " << config.cam0.dxPx << "\n";
    ofs << "    dyPx = " << config.cam0.dyPx << "\n";
    ofs << "}\n\n";

    // 写入 cam1（副摄）
    ofs << "cam1 {\n";
    ofs << "    rvec = " << config.cam1.rvec[0] << " " 
        << config.cam1.rvec[1] << " " 
        << config.cam1.rvec[2] << "\n";
    ofs << "    tvec = " << config.cam1.tvec[0] << " " 
        << config.cam1.tvec[1] << " " 
        << config.cam1.tvec[2] << "\n";
    ofs << "    dxPx = " << config.cam1.dxPx << "\n";
    ofs << "    dyPx = " << config.cam1.dyPx << "\n";

    for (size_t i = 0; i < config.cam1.calib.size(); ++i) {
        const CameraBoardCalibEntry& calib = config.cam1.calib[i];
        ofs << "    calib" << (i + 1) << " {\n";
        if (calib.has_x_min) {
            ofs << "        x_min = " << calib.x_min << "\n";
        }
        if (calib.has_x_max) {
            ofs << "        x_max = " << calib.x_max << "\n";
        }
        if (calib.has_y_min) {
            ofs << "        y_min = " << calib.y_min << "\n";
        }
        if (calib.has_y_max) {
            ofs << "        y_max = " << calib.y_max << "\n";
        }
        if (calib.has_z_drop_mm) {
            ofs << "        z_drop_mm = " << calib.z_drop_mm << "\n";
        }
        if (calib.has_test_height_mm) {
            ofs << "        test_height_mm = " << calib.test_height_mm << "\n";
        }
        ofs << "    }\n";
    }
    ofs << "}\n\n";

    // 写入 totalHigh（总高测量值）
    ofs << "totalHigh = " << config.totalHigh << "\n\n";
    ofs << std::fixed << std::setprecision(2);
    ofs << "focal_long = " << config.focal_long << "\n";
    ofs << "focal_short = " << config.focal_short << "\n\n";

    std::cout << "[reallinkCV] Configuration saved to: " << path << std::endl;
    return true;
}

// 从 reallinkCV.conf 读取完整配置
bool readReallinkCVConf(const std::string& path, ReallinkCVConfig& config) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "[reallinkCV] Failed to open for read: " << path
                  << " (errno=" << errno << ", " << std::strerror(errno) << ")" << std::endl;
        return false;
    }

    std::string line;
    CameraCalibData* currentCam = nullptr;
    CameraBoardCalibEntry* currentCalib = nullptr;
    bool currentCamIsCam1 = false;

    while (std::getline(ifs, line)) {
        std::string trimmed = trim(line);
        if (trimmed.empty()) {
            continue;
        }

        if (trimmed == "}") {
            if (currentCalib) {
                currentCalib = nullptr;
            } else {
                currentCam = nullptr;
                currentCamIsCam1 = false;
            }
            continue;
        }

        // 检测块开始
        if (trimmed.rfind("cam0", 0) == 0 && trimmed.find('{') != std::string::npos) {
            currentCam = &config.cam0;
            currentCamIsCam1 = false;
            currentCalib = nullptr;
            continue;
        }
        if (trimmed.rfind("cam1", 0) == 0 && trimmed.find('{') != std::string::npos) {
            currentCam = &config.cam1;
            currentCamIsCam1 = true;
            currentCalib = nullptr;
            continue;
        }

        // 解析 cam1.calib1~4 子块
        if (currentCamIsCam1 && currentCam == &config.cam1) {
            int calib_index = -1;
            if (parseCalibBlockIndex(trimmed, calib_index)) {
                currentCalib = &config.cam1.calib[static_cast<size_t>(calib_index)];
                continue;
            }
        }

        // 解析 calib 子块字段
        if (currentCalib) {
            double value = 0.0;
            if (parseDoubleLine(trimmed, "x_min", value)) {
                currentCalib->x_min = value;
                currentCalib->has_x_min = true;
                continue;
            }
            if (parseDoubleLine(trimmed, "x_max", value)) {
                currentCalib->x_max = value;
                currentCalib->has_x_max = true;
                continue;
            }
            if (parseDoubleLine(trimmed, "y_min", value)) {
                currentCalib->y_min = value;
                currentCalib->has_y_min = true;
                continue;
            }
            if (parseDoubleLine(trimmed, "y_max", value)) {
                currentCalib->y_max = value;
                currentCalib->has_y_max = true;
                continue;
            }
            if (parseDoubleLine(trimmed, "z_drop_mm", value)) {
                currentCalib->z_drop_mm = value;
                currentCalib->has_z_drop_mm = true;
                continue;
            }
            if (parseDoubleLine(trimmed, "test_height_mm", value)) {
                currentCalib->test_height_mm = value;
                currentCalib->has_test_height_mm = true;
                continue;
            }
            // backward compatibility: old single-point fields
            if (parseDoubleLine(trimmed, "x", value)) {
                if (!currentCalib->has_x_min) {
                    currentCalib->x_min = value;
                    currentCalib->has_x_min = true;
                }
                if (!currentCalib->has_x_max) {
                    currentCalib->x_max = value;
                    currentCalib->has_x_max = true;
                }
                continue;
            }
            if (parseDoubleLine(trimmed, "y", value)) {
                if (!currentCalib->has_y_min) {
                    currentCalib->y_min = value;
                    currentCalib->has_y_min = true;
                }
                if (!currentCalib->has_y_max) {
                    currentCalib->y_max = value;
                    currentCalib->has_y_max = true;
                }
                continue;
            }
        }

        // 解析相机参数
        if (currentCam) {
            if (parseVec3Line(trimmed, "rvec", currentCam->rvec)) continue;
            if (parseVec3Line(trimmed, "tvec", currentCam->tvec)) continue;
            if (parseDoubleLine(trimmed, "dxPx", currentCam->dxPx)) continue;
            if (parseDoubleLine(trimmed, "dyPx", currentCam->dyPx)) continue;
            // backward compatibility with old config keys
            if (parseDoubleLine(trimmed, "dxMm", currentCam->dxPx)) continue;
            if (parseDoubleLine(trimmed, "dyMm", currentCam->dyPx)) continue;
        }

        // 解析 totalHigh
        if (parseDoubleLine(trimmed, "totalHigh", config.totalHigh)) continue;
        if (parseDoubleLine(trimmed, "focal_long", config.focal_long)) continue;
        if (parseDoubleLine(trimmed, "focal_short", config.focal_short)) continue;

    }

    std::cout << "[reallinkCV] Configuration loaded from: " << path << std::endl;
    std::cout << "[reallinkCV] cam0: rvec=" << config.cam0.rvec[0] << " " << config.cam0.rvec[1] << " " << config.cam0.rvec[2]
              << " tvec=" << config.cam0.tvec[0] << " " << config.cam0.tvec[1] << " " << config.cam0.tvec[2] << std::endl;
    return true;
}

