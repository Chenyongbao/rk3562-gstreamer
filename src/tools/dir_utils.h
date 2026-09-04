#ifndef DIR_UTILS_H
#define DIR_UTILS_H

#include <cerrno>
#include <string>

#include <sys/stat.h>
#include <sys/types.h>

// 逐级创建目录（容忍中间目录不存在，也容忍目录已存在 EEXIST）。
// 全项目统一的递归建目录实现：原本有两份逻辑相同的拷贝——
//   CalibCommandHandler.cpp 的 mkdir_recursive（C 风格）
//   vice_camera_utils.cpp    的 mkdirRecursive（C++ string 风格）
// 现已合并到此（去重）。
inline int mkdirRecursive(const std::string& path) {
    if (path.empty()) {
        return 0;
    }

    std::string tmp = path;
    if (tmp.back() == '/') {
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

#endif // DIR_UTILS_H
