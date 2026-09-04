#ifndef VICE_CAMERA_INTERNAL_H
#define VICE_CAMERA_INTERNAL_H

#include "vice_camera_service.h"
#include "tools/dir_utils.h"

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

// 副相机标定流程里使用的固定阈值与经验参数。
inline constexpr int kLaserQueryRetries = 5;
inline constexpr double kCaptureZMm = 25.0;
inline constexpr size_t kMinValidSampleCount = 8;
inline constexpr float kCharucoSquareSizeMm = 8.0f;
inline constexpr int kMinCharucoPointCount = 8;
inline constexpr int kMinSolvePnpPointCount = 4;
inline constexpr int kSaddleFitRadius = 10;
inline constexpr double kXYoffsetMmPerPx = 0.25;

// 对命令行参数做 shell 转义，避免路径或设备名包含特殊字符。
std::string shellQuote(const std::string& value);
// 在 OpenCV 向量与业务结构之间做轻量转换。
cv::Vec3d toVec3d(const std::array<double, 3>& values);
std::array<double, 3> toArray3(const cv::Vec3d& value);
// 将 OpenCV 返回的 3 维矩阵/向量统一拉平成 Vec3d。
bool matToVec3d(const cv::Mat& mat, cv::Vec3d& out);

// 汇总副相机外参求解结果，以及残差和高度统计信息。
struct ViceExtrinsicsReport {
    cv::Matx33d board_rotation = cv::Matx33d::eye();
    cv::Vec3d board_rvec = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Vec3d board_tvec = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Vec3d laser_spot_board = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Vec3d camera_offset_gantry = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Matx33d camera_rotation = cv::Matx33d::eye();
    cv::Vec3d camera_rvec = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Vec3d camera_tvec = cv::Vec3d(0.0, 0.0, 0.0);
    double center_residual_mean_mm = 0.0;
    double center_residual_std_mm = 0.0;
    double height_diff_mean_mm = 0.0;
    double height_diff_std_mm = 0.0;
};

// 基于有效样本计算 board/gantry/camera 之间的刚体关系。
bool buildViceExtrinsicsReport(const std::vector<SampleRecord>& sample_records,
                               const cv::Vec3d& laser_spot_board,
                               ViceExtrinsicsReport& out_report,
                               std::string& error_msg);
// 将最终外参结果写出为 JSON 文件。
bool writeViceExtrinsicsJsonFile(const std::string& file_path,
                                 const ViceExtrinsicsReport& report,
                                 std::string& error_msg);
// 优先读取运行时配置中的激光点坐标，缺失时再回退到配置文件或默认值。
cv::Vec3d loadLaserSpotBoardFromConfig(const ViceCameraConfig& service_config,
                                       bool verbose = true);

std::unique_ptr<IImageCapturer> makeV4L2CtlImageCapturer();
std::unique_ptr<ICalibrationEngine> makeOpenCvCalibrationEngine();
std::unique_ptr<ISessionStore> makeLocalSessionStore();

#endif // VICE_CAMERA_INTERNAL_H
