#ifndef VICE_CAMERA_SERVICE_H
#define VICE_CAMERA_SERVICE_H

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "../../config.h"

class KlipperManager;

// 标定服务运行参数，包含相机、采集目录、运动参数与激光点先验配置。
struct ViceCameraConfig {
    std::string camera_device = CALIB_CAMERA_DEVICE;
    std::string capture_dir = CALIB_CAPTURE_DIR;
    double z_height = CALIB_DEFAULT_Z_HEIGHT;
    int moves = CALIB_DEFAULT_MOVES;
    double x_min = CALIB_X_MIN;
    double x_max = CALIB_X_MAX;
    double y_min = CALIB_Y_MIN;
    double y_max = CALIB_Y_MAX;
    double feedrate = CALIB_FEEDRATE;
    double laser_spot_board_x_mm = std::numeric_limits<double>::quiet_NaN();
    double laser_spot_board_y_mm = std::numeric_limits<double>::quiet_NaN();
    double laser_spot_board_z_mm = std::numeric_limits<double>::quiet_NaN();
};
// 标定流程的最终返回值。
struct ViceCameraResult {
    bool success = false;
    std::string error;
    std::string latest_dir;
};
// 相机内参标定摘要。
struct ViceCalibrationSummary {
    double rms = 0.0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    std::array<double, 5> dist_coeffs{{0.0, 0.0, 0.0, 0.0, 0.0}};
    int coeff_count = 0;
};

// 单次采样事务，保存移动、测高、拍照和角点检测的全过程状态。
struct SampleTxn {
    std::string sample_id;
    int step_index = 0;
    double gx = 0.0;
    double gy = 0.0;
    double gz = 25.0;
    double h = 0.0;
    std::string image_path;
    bool height_valid = false;
    bool image_ok = false;
    bool charuco_ok = false;
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> object_points;
    std::string fail_reason;
};

// 从有效样本中提炼出的几何记录，供外参与统计计算使用。
struct SampleRecord {
    std::string sample_id;
    int step_index = 0;
    double gx = 0.0;
    double gy = 0.0;
    double gz = 25.0;
    double h = 0.0;
    std::array<double, 3> rvec{{0.0, 0.0, 0.0}};
    std::array<double, 3> tvec{{0.0, 0.0, 0.0}};
    std::array<double, 3> camera_center_board{{0.0, 0.0, 0.0}};
    double camera_to_board_distance = 0.0;
};

// 采集阶段统计信息，用于汇总成功率和失败原因。
struct CaptureStats {
    int attempted = 0;
    int valid = 0;
    int height_fail = 0;
    int capture_fail = 0;
    int pose_fail = 0;
};

class IImageCapturer {
public:
    virtual ~IImageCapturer() = default;
    // 采集单张图像并保存到指定路径。
    virtual bool capture(const std::string& camera_device,
                         const std::string& filepath,
                         std::string& error) = 0;
};

class ICalibrationEngine {
public:
    virtual ~ICalibrationEngine() = default;
    // 清空上一轮标定缓存。
    virtual void reset() = 0;
    // 逐帧写入检测到的二维/三维对应点。
    virtual void ingestDetection(const std::vector<cv::Point2f>& image_points,
                                 const std::vector<cv::Point3f>& object_points,
                                 const cv::Size& image_size,
                                 int image_index) = 0;
    // 执行内参标定，并返回每张有效图像对应的位姿结果。
    virtual bool finalize(ViceCalibrationSummary& out_summary,
                          std::vector<cv::Vec3d>& out_rvecs,
                          std::vector<cv::Vec3d>& out_tvecs,
                          std::string& error) = 0;
};

class ISessionStore {
public:
    virtual ~ISessionStore() = default;
    // 创建本次采集会话目录。
    virtual std::string createSessionDir(const std::string& capture_dir) = 0;
    // 删除临时采集图片。
    virtual void removeFile(const std::string& file_path) = 0;
};

class ViceCameraService {
public:
    // 允许注入采集、标定和会话目录实现，便于替换真实硬件依赖做测试。
    explicit ViceCameraService(ViceCameraConfig config,
                               KlipperManager* klipper = nullptr,
                               std::unique_ptr<IImageCapturer> image_capturer = nullptr,
                               std::unique_ptr<ICalibrationEngine> calibration_engine = nullptr,
                               std::unique_ptr<ISessionStore> session_store = nullptr);
    ~ViceCameraService();

    // 执行完整的副相机标定流程。
    ViceCameraResult run(bool skip_homing = false);
    const std::string& getLatestCaptureDir() const { return latest_capture_dir_; }

private:
    // 以下步骤串起完整流程：建目录、回零、采样、内参整理、外参求解和清理。
    bool createSessionDir(std::string& error_msg);
    bool forceHome(std::string& error_msg);
    bool sendScript(const std::string& script, std::string& error_msg);
    bool triggerLaserRange(std::string& error_msg);
    bool queryLaserDistance(double& out_distance, bool& out_valid, std::string& error_msg);
    bool setFillLight(int brightness, std::string& error_msg);
    bool captureLoop(std::string& error_msg);
    bool finalizeCalibration(std::string& error_msg);
    bool measureHeightAtCurrentPose(double& out_height, std::string& error_msg);
    bool solveAndReportSampleRecords(std::string& error_msg);
    void cleanupCapturedImages();
    void clearSessionState();

    ViceCameraConfig config_;
    std::string latest_capture_dir_;
    KlipperManager* klipper_ = nullptr;

    std::unique_ptr<IImageCapturer> image_capturer_;
    std::unique_ptr<ICalibrationEngine> calibration_engine_;
    std::unique_ptr<ISessionStore> session_store_;

    std::vector<SampleTxn> sample_txns_;
    std::vector<SampleRecord> sample_records_;
    CaptureStats capture_stats_;

    // 预设采样点，单位为龙门坐标系下的 XY 毫米值。
    static const std::vector<std::pair<double, double>> fixed_coordinates_;
};

#endif
