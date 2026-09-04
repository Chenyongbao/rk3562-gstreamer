#include "vice_camera_internal.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

namespace {

// OpenCV 内参标定实现：累积每帧 ChArUco 对应点后统一求解。
class OpenCvCalibrationEngine final : public ICalibrationEngine {
public:
    void reset() override {
        // 每轮新标定开始前清空累计的二维/三维对应点。
        all_image_points_.clear();
        all_object_points_.clear();
        image_size_ = cv::Size();
        image_size_initialized_ = false;
    }

    void ingestDetection(const std::vector<cv::Point2f>& image_points,
                         const std::vector<cv::Point3f>& object_points,
                         const cv::Size& image_size,
                         int image_index) override {
        // 标定输入要求二维点和三维点一一对应，且至少满足最小位姿求解点数。
        if (image_points.size() != object_points.size() ||
            image_points.size() < static_cast<size_t>(kMinSolvePnpPointCount)) {
            std::cout << "[ViceCamera] Skip calibration ingest on image " << image_index
                      << " due to invalid cached charuco points" << std::endl;
            return;
        }

        if (!image_size_initialized_) {
            // 第一张有效图像用于初始化标定图像尺寸。
            image_size_ = image_size;
            image_size_initialized_ = true;
        } else if (image_size != image_size_) {
            std::cerr << "[ViceCamera] Warning: Image size changed on image " << image_index
                      << " from " << image_size_ << " to " << image_size << std::endl;
        }

        all_image_points_.push_back(image_points);
        all_object_points_.push_back(object_points);
    }

    bool finalize(ViceCalibrationSummary& out_summary,
                  std::vector<cv::Vec3d>& out_rvecs,
                  std::vector<cv::Vec3d>& out_tvecs,
                  std::string& error) override {
        // 有效图像过少时直接终止，避免得到不稳定内参。
        if (all_image_points_.size() < 3) {
            error = "Not enough valid images for calibration (need at least 3, got "
                    + std::to_string(all_image_points_.size()) + ")";
            std::cerr << "[ViceCamera] " << error << std::endl;
            return false;
        }

        const double focal_length = 1444.444;
        // 以经验焦距和主点初值作为 calibrateCamera 的初始猜测。
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            focal_length, 0, image_size_.width / 2.0,
            0, focal_length, image_size_.height / 2.0,
            0, 0, 1);
        cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
        std::vector<cv::Mat> rvecs;
        std::vector<cv::Mat> tvecs;

        // OpenCV 输出的每帧 rvec/tvec 都表示 board->camera 的位姿。
        out_summary.rms = cv::calibrateCamera(all_object_points_,
                                              all_image_points_,
                                              image_size_,
                                              camera_matrix,
                                              dist_coeffs,
                                              rvecs,
                                              tvecs,
                                              cv::CALIB_USE_INTRINSIC_GUESS);
        out_summary.fx = camera_matrix.at<double>(0, 0);
        out_summary.fy = camera_matrix.at<double>(1, 1);
        out_summary.cx = camera_matrix.at<double>(0, 2);
        out_summary.cy = camera_matrix.at<double>(1, 2);
        out_summary.coeff_count = std::min(5, dist_coeffs.rows * dist_coeffs.cols);
        for (int i = 0; i < out_summary.coeff_count; ++i) {
            const int row = (dist_coeffs.rows == 1) ? 0 : i;
            const int col = (dist_coeffs.rows == 1) ? i : 0;
            out_summary.dist_coeffs[static_cast<size_t>(i)] = dist_coeffs.at<double>(row, col);
        }

        if (rvecs.size() != tvecs.size()) {
            error = "calibrateCamera output size mismatch: rvecs="
                    + std::to_string(rvecs.size()) + ", tvecs=" + std::to_string(tvecs.size());
            return false;
        }

        out_rvecs.clear();
        out_tvecs.clear();
        out_rvecs.reserve(rvecs.size());
        out_tvecs.reserve(tvecs.size());
        for (size_t i = 0; i < rvecs.size(); ++i) {
            cv::Vec3d rvec;
            cv::Vec3d tvec;
            if (!matToVec3d(rvecs[i], rvec) || !matToVec3d(tvecs[i], tvec)) {
                error = "calibrateCamera returned invalid pose vector at index "
                        + std::to_string(i);
                return false;
            }
            out_rvecs.push_back(rvec);
            out_tvecs.push_back(tvec);
        }

        std::cout << "[ViceCamera] Calibration RMS error = " << out_summary.rms << std::endl;
        return true;
    }

private:
    std::vector<std::vector<cv::Point2f>> all_image_points_;
    std::vector<std::vector<cv::Point3f>> all_object_points_;
    cv::Size image_size_;
    bool image_size_initialized_ = false;
};

} // namespace

std::unique_ptr<ICalibrationEngine> makeOpenCvCalibrationEngine() {
    return std::make_unique<OpenCvCalibrationEngine>();
}

bool ViceCameraService::solveAndReportSampleRecords(std::string& error_msg) {
    // 用所有有效样本计算外参，并打印诊断信息便于查看结果质量。
    ViceExtrinsicsReport report;
    const cv::Vec3d laserSpotBoard = loadLaserSpotBoardFromConfig(config_);
    if (!buildViceExtrinsicsReport(sample_records_, laserSpotBoard, report, error_msg)) {
        return false;
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "===== Solve Report =====" << std::endl;
    std::cout << "Samples attempted: " << capture_stats_.attempted << std::endl;
    std::cout << "Samples valid: " << capture_stats_.valid << std::endl;
    std::cout << "Failure stats: height_fail=" << capture_stats_.height_fail
              << ", capture_fail=" << capture_stats_.capture_fail
              << ", pose_fail=" << capture_stats_.pose_fail << std::endl;
    std::cout << "[1] Board -> Gantry transform (best-fit, camera-center based)" << std::endl;
    std::cout << "Equation: P_gantry = R_gb * P_board + t_gb" << std::endl;
    std::cout << "R_gb = " << cv::Mat(report.board_rotation) << std::endl;
    std::cout << "t_gb (mm) = [" << report.board_tvec[0] << ", "
              << report.board_tvec[1] << ", " << report.board_tvec[2] << "]"
              << std::endl;
    std::cout << "Residual norm mean/std (mm) = " << report.center_residual_mean_mm
              << " / " << report.center_residual_std_mm << std::endl;
    std::cout << "[2] Camera -> Gantry transform" << std::endl;
    std::cout << "Laser spot on board comes from XYoffset config (cam0.dxPx/dyPx * "
              << kXYoffsetMmPerPx << " mm/px)." << std::endl;
    std::cout << "Laser spot on board (mm) = [" << report.laser_spot_board[0] << ", "
              << report.laser_spot_board[1] << ", " << report.laser_spot_board[2]
              << "]" << std::endl;
    std::cout << "Camera origin in gantry (t_gc, mm) = ["
              << report.camera_tvec[0] << ", "
              << report.camera_tvec[1] << ", "
              << report.camera_tvec[2] << "]" << std::endl;
    std::cout << "R_gc = " << cv::Mat(report.camera_rotation) << std::endl;
    std::cout << "rvec_gc (rad) = ["
              << report.camera_rvec[0] << ", " << report.camera_rvec[1] << ", "
              << report.camera_rvec[2] << "]"
              << std::endl;
    std::cout << "[3] Camera and rangefinder height difference" << std::endl;
    std::cout << "Per-frame diff = H_rangefinder - D_camera_to_board" << std::endl;
    std::cout << "Mean/std (mm) = " << report.height_diff_mean_mm
              << " / " << report.height_diff_std_mm << std::endl;
    std::cout << "========================" << std::endl;

    return true;
}

bool ViceCameraService::finalizeCalibration(std::string& error_msg) {
    ViceCalibrationSummary summary;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    if (!calibration_engine_->finalize(summary, rvecs, tvecs, error_msg)) {
        return false;
    }

    // 只为成功完成 ChArUco 检测的样本保留位姿，数量必须与 OpenCV 输出一致。
    const size_t expected_pose_count = static_cast<size_t>(std::count_if(
        sample_txns_.begin(), sample_txns_.end(),
        [](const SampleTxn& txn) { return txn.charuco_ok; }));
    if (rvecs.size() != tvecs.size()) {
        error_msg = "Internal pose output mismatch: rvecs=" + std::to_string(rvecs.size())
            + ", tvecs=" + std::to_string(tvecs.size());
        return false;
    }
    if (rvecs.size() != expected_pose_count) {
        error_msg = "Pose/sample mismatch: expected " + std::to_string(expected_pose_count)
            + " poses from charuco samples, got " + std::to_string(rvecs.size());
        return false;
    }

    sample_records_.clear();
    size_t pose_index = 0;
    for (auto& txn : sample_txns_) {
        if (!txn.charuco_ok) {
            if (txn.height_valid && txn.image_ok) {
                if (txn.fail_reason.empty()) {
                    txn.fail_reason = "pose_fail: charuco points insufficient";
                }
                ++capture_stats_.pose_fail;
                std::cout << "[ViceCamera] Skip sample_id=" << txn.sample_id
                          << " because " << txn.fail_reason << std::endl;
            }
            continue;
        }

        if (pose_index >= rvecs.size()) {
            error_msg = "Pose index overflow while mapping sample_id=" + txn.sample_id;
            return false;
        }

        const size_t current_pose_index = pose_index;
        const cv::Vec3d& rvec = rvecs[pose_index];
        const cv::Vec3d& tvec = tvecs[pose_index];
        ++pose_index;

        if (!txn.height_valid || !txn.image_ok) {
            continue;
        }

        cv::Matx33d rbc;
        cv::Rodrigues(rvec, rbc);
        // rbc 为 board->camera 旋转，转置后可把相机中心换回 board 坐标系。
        const cv::Vec3d cameraCenterBoard = -rbc.t() * tvec;
        // 取棋盘法向量与平移向量的点积绝对值，得到相机到棋盘平面的距离。
        const cv::Vec3d boardNormalCam(rbc(0, 2), rbc(1, 2), rbc(2, 2));
        const double camToBoard = std::abs(boardNormalCam.dot(tvec));

        std::cout << "[ViceCamera] pose_map"
                  << " sample_id=" << txn.sample_id
                  << " step=" << txn.step_index
                  << " gantry=[" << txn.gx << ", " << txn.gy << ", " << txn.gz << "]"
                  << " pose_index=" << current_pose_index
                  << " rvec=[" << rvec[0] << ", " << rvec[1] << ", " << rvec[2] << "]"
                  << " tvec=[" << tvec[0] << ", " << tvec[1] << ", " << tvec[2] << "]"
                  << std::endl;
        std::cout << "[ViceCamera] center_map"
                  << " sample_id=" << txn.sample_id
                  << " cameraCenterBoard=[" << cameraCenterBoard[0] << ", "
                  << cameraCenterBoard[1] << ", " << cameraCenterBoard[2] << "]"
                  << " camToBoard=" << camToBoard
                  << std::endl;

        SampleRecord record;
        record.sample_id = txn.sample_id;
        record.step_index = txn.step_index;
        record.gx = txn.gx;
        record.gy = txn.gy;
        record.gz = txn.gz;
        record.h = txn.h;
        record.rvec = toArray3(rvec);
        record.tvec = toArray3(tvec);
        record.camera_center_board = toArray3(cameraCenterBoard);
        record.camera_to_board_distance = camToBoard;
        sample_records_.push_back(record);
    }

    if (pose_index != rvecs.size()) {
        error_msg = "Pose/sample mapping incomplete: used " + std::to_string(pose_index)
            + ", total poses " + std::to_string(rvecs.size());
        return false;
    }

    capture_stats_.valid = static_cast<int>(sample_records_.size());
    if (sample_records_.size() < kMinValidSampleCount) {
        error_msg = "Not enough valid samples for rigid solve (need at least "
            + std::to_string(kMinValidSampleCount) + ", got "
            + std::to_string(sample_records_.size()) + ")";
        return false;
    }

    if (!solveAndReportSampleRecords(error_msg)) {
        return false;
    }

    ViceExtrinsicsReport extrinsics_report;
    const cv::Vec3d laserSpotBoard = loadLaserSpotBoardFromConfig(config_, false);
    // 再构建一次报告对象，用于落盘 JSON 文件。
    if (!buildViceExtrinsicsReport(sample_records_, laserSpotBoard, extrinsics_report, error_msg)) {
        return false;
    }

    const std::string extrinsics_path = latest_capture_dir_ + "/vice_extrinsics.json";
    if (!writeViceExtrinsicsJsonFile(extrinsics_path, extrinsics_report, error_msg)) {
        return false;
    }

    std::cout << "[ViceCamera] Calibration result:"
              << " fx=" << summary.fx
              << " fy=" << summary.fy
              << " cx=" << summary.cx
              << " cy=" << summary.cy
              << " dist=[";
    for (int i = 0; i < summary.coeff_count; ++i) {
        if (i > 0) {
            std::cout << ", ";
        }
        std::cout << summary.dist_coeffs[static_cast<size_t>(i)];
    }
    std::cout << "]" << std::endl;

    cleanupCapturedImages();
    return true;
}
