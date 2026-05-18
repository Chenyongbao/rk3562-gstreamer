#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <array>
#include <string>
#include <vector>
#include <cstdint>

// 副摄4个棋盘测高数据（cam1.calib1~calib4）
struct CameraBoardCalibEntry {
    double x_min{0.0};
    double x_max{0.0};
    double y_min{0.0};
    double y_max{0.0};
    double z_drop_mm{0.0};
    double test_height_mm{0.0};

    bool has_x_min{false};
    bool has_x_max{false};
    bool has_y_min{false};
    bool has_y_max{false};
    bool has_z_drop_mm{false};
    bool has_test_height_mm{false};
};

// 相机完整标定数据（内参 + 外参）
struct CameraCalibData {
    // 外参
    double rvec[3]{0.0, 0.0, 0.0};  // 旋转向量
    double tvec[3]{0.0, 0.0, 0.0};  // 平移向量
    // XY偏移（俯视图对齐用），单位：像素
    double dxPx{0.0};  // x方向偏移 (像素)
    double dyPx{0.0};  // y方向偏移 (像素)
    std::array<CameraBoardCalibEntry, 4> calib{};  // cam1.calib1~calib4（cam0忽略）
};


// 完整的配置文件结构
struct ReallinkCVConfig {
    CameraCalibData cam0;           // 主摄标定数据
    CameraCalibData cam1;           // 副摄标定数据
    double totalHigh{0.0};          // 总高测量值 (毫米)
    double focal_long{0.0};         // 长焦焦距 (毫米)
    double focal_short{0.0};        // 短焦焦距 (毫米)
};

bool writeReallinkCVConf(const std::string& path, const ReallinkCVConfig& config);

// 从 reallinkCV.conf 读取完整配置
bool readReallinkCVConf(const std::string& path, ReallinkCVConfig& config);

// 相机外参（用于标定结果传递）
struct CameraExtrinsic {
    double rvec[3]{0.0, 0.0, 0.0};
    double tvec[3]{0.0, 0.0, 0.0};
};

#endif // FILE_UTILS_H
