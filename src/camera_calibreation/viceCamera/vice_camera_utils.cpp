#include "vice_camera_internal.h"

#include <cerrno>
#include <string>

#include <sys/stat.h>
#include <sys/types.h>

std::string shellQuote(const std::string& value) {
    // 使用单引号包裹，并对内部单引号做安全转义。
    std::string quoted = "'";
    for (char c : value) {
        if (c == '\'') {
            quoted += "'\"'\"'";
        } else {
            quoted += c;
        }
    }
    quoted += "'";
    return quoted;
}

int mkdirRecursive(const std::string& path) {
    // 逐级创建目录，兼容中间目录不存在的情况。
    if (path.empty()) {
        return 0;
    }

    std::string tmp = path;
    if (!tmp.empty() && tmp.back() == '/') {
        tmp.pop_back();
    }
    if (tmp.empty()) {
        return 0;
    }

    size_t pos = (tmp[0] == '/') ? 1 : 0;
    while ((pos = tmp.find('/', pos)) != std::string::npos) {
        const std::string dir = tmp.substr(0, pos);
        if (!dir.empty() && mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
            return -1;
        }
        ++pos;
    }

    if (mkdir(tmp.c_str(), 0755) != 0 && errno != EEXIST) {
        return -1;
    }
    return 0;
}

cv::Vec3d toVec3d(const std::array<double, 3>& values) {
    return cv::Vec3d(values[0], values[1], values[2]);
}

std::array<double, 3> toArray3(const cv::Vec3d& value) {
    return {{value[0], value[1], value[2]}};
}

bool matToVec3d(const cv::Mat& mat, cv::Vec3d& out) {
    // OpenCV 可能返回 3x1、1x3 等形式，这里统一拍平成一行读取。
    if (mat.empty() || mat.total() != 3) {
        return false;
    }

    cv::Mat mat64;
    mat.convertTo(mat64, CV_64F);
    const cv::Mat flat = mat64.reshape(1, 1);

    out = cv::Vec3d(flat.at<double>(0, 0),
                    flat.at<double>(0, 1),
                    flat.at<double>(0, 2));
    return true;
}
