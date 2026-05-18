#include "vice_camera_internal.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "../../reallink_ogles/file_utils.h"

namespace {

const cv::Vec3d kDefaultLaserSpotBoard(1.845, -4.7, 0.0);

bool solveRigidGantryToBoard(const std::vector<SampleRecord>& records,
                             cv::Matx33d& rbg,
                             cv::Vec3d& tCombined) {
    // 用相机中心点在 board 与 gantry 两套坐标中的对应关系拟合刚体变换。
    if (records.size() < 3) {
        return false;
    }

    cv::Vec3d meanG(0, 0, 0);
    cv::Vec3d meanC(0, 0, 0);
    for (const auto& record : records) {
        meanG += cv::Vec3d(record.gx, record.gy, record.gz);
        meanC += toVec3d(record.camera_center_board);
    }
    meanG *= (1.0 / static_cast<double>(records.size()));
    meanC *= (1.0 / static_cast<double>(records.size()));

    cv::Matx33d cov = cv::Matx33d::zeros();
    for (const auto& record : records) {
        const cv::Vec3d g = cv::Vec3d(record.gx, record.gy, record.gz) - meanG;
        const cv::Vec3d c = toVec3d(record.camera_center_board) - meanC;
        cov += c * g.t();
    }

    cv::SVD svd(cv::Mat(cov), cv::SVD::FULL_UV);
    cv::Matx33d u;
    cv::Matx33d vt;
    svd.u.copyTo(u);
    svd.vt.copyTo(vt);

    cv::Matx33d rotation = u * vt;
    if (cv::determinant(rotation) < 0.0) {
        // 修正反射矩阵，保证结果仍然是合法旋转。
        cv::Matx33d d = cv::Matx33d::eye();
        d(2, 2) = -1.0;
        rotation = u * d * vt;
    }

    rbg = rotation;
    tCombined = meanC - rbg * meanG;
    return true;
}

cv::Matx33d averageRotation(const std::vector<cv::Matx33d>& rotations) {
    // 对多帧旋转做 SVD 投影平均，避免逐元素平均破坏正交性。
    cv::Matx33d sumR = cv::Matx33d::zeros();
    for (const auto& rotation : rotations) {
        sumR += rotation;
    }

    cv::SVD svd(cv::Mat(sumR), cv::SVD::FULL_UV);
    cv::Matx33d u;
    cv::Matx33d vt;
    svd.u.copyTo(u);
    svd.vt.copyTo(vt);

    cv::Matx33d rotation = u * vt;
    if (cv::determinant(rotation) < 0.0) {
        cv::Matx33d d = cv::Matx33d::eye();
        d(2, 2) = -1.0;
        rotation = u * d * vt;
    }
    return rotation;
}

double computeMean(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }

    double sum = 0.0;
    for (double value : values) {
        sum += value;
    }
    return sum / static_cast<double>(values.size());
}

double computeStd(const std::vector<double>& values, double mean) {
    if (values.empty()) {
        return 0.0;
    }

    double sum2 = 0.0;
    for (double value : values) {
        const double delta = value - mean;
        sum2 += delta * delta;
    }
    return std::sqrt(sum2 / static_cast<double>(values.size()));
}

void writeVec3dJson(std::ostream& os, const cv::Vec3d& value) {
    os << "[" << value[0] << ", " << value[1] << ", " << value[2] << "]";
}

} // namespace

bool buildViceExtrinsicsReport(const std::vector<SampleRecord>& sample_records,
                               const cv::Vec3d& laser_spot_board,
                               ViceExtrinsicsReport& out_report,
                               std::string& error_msg) {
    // 先解出 gantry->board 的刚体关系，再推导 board/camera 在 gantry 下的表达。
    cv::Matx33d rbg;
    cv::Vec3d tCombined;
    if (!solveRigidGantryToBoard(sample_records, rbg, tCombined)) {
        error_msg = "Failed to solve gantry->board rigid transform.";
        return false;
    }

    std::vector<double> centerResidualNorms;
    std::vector<cv::Vec3d> camOffsetBoardList;
    std::vector<cv::Matx33d> rcgList;
    std::vector<double> heightDiffList;
    centerResidualNorms.reserve(sample_records.size());
    camOffsetBoardList.reserve(sample_records.size());
    rcgList.reserve(sample_records.size());
    heightDiffList.reserve(sample_records.size());

    for (const auto& sample : sample_records) {
        const cv::Vec3d g(sample.gx, sample.gy, sample.gz);
        const cv::Vec3d cameraCenter = toVec3d(sample.camera_center_board);
        const cv::Vec3d predC = rbg * g + tCombined;
        // 相机中心拟合残差，用于衡量刚体解的一致性。
        centerResidualNorms.push_back(cv::norm(cameraCenter - predC));

        // 已知激光点在 board 上的位置后，可反推出相机原点的平移偏置。
        const cv::Vec3d camOffsetBoard = cameraCenter - (rbg * g + laser_spot_board);
        camOffsetBoardList.push_back(camOffsetBoard);

        cv::Matx33d rbc;
        cv::Rodrigues(toVec3d(sample.rvec), rbc);
        // board->camera 与 gantry->board 串联后，得到 gantry->camera 旋转。
        rcgList.push_back(rbc * rbg);

        // 测距值与视觉估计的棋盘距离之差，可用于评估高度一致性。
        heightDiffList.push_back(sample.h - sample.camera_to_board_distance);
    }

    cv::Vec3d camOffsetBoardMean(0.0, 0.0, 0.0);
    for (const auto& value : camOffsetBoardList) {
        camOffsetBoardMean += value;
    }
    camOffsetBoardMean *= (1.0 / static_cast<double>(camOffsetBoardList.size()));
    const cv::Vec3d camOffsetGantry = rbg.t() * camOffsetBoardMean;

    const cv::Matx33d rcgMean = averageRotation(rcgList);
    const cv::Matx33d rgb = rbg.t();
    const cv::Matx33d rgc = rcgMean.t();
    const cv::Vec3d tGb = -(rgb * tCombined);
    cv::Vec3d rvecGb;
    cv::Vec3d rvecGc;
    cv::Rodrigues(rgb, rvecGb);
    cv::Rodrigues(rgc, rvecGc);

    out_report.board_rotation = rgb;
    out_report.board_rvec = rvecGb;
    out_report.board_tvec = tGb;
    out_report.laser_spot_board = laser_spot_board;
    out_report.camera_offset_gantry = camOffsetGantry;
    out_report.camera_rotation = rgc;
    out_report.camera_rvec = rvecGc;
    out_report.camera_tvec = camOffsetGantry;
    out_report.center_residual_mean_mm = computeMean(centerResidualNorms);
    out_report.center_residual_std_mm = computeStd(centerResidualNorms,
                                                   out_report.center_residual_mean_mm);
    out_report.height_diff_mean_mm = computeMean(heightDiffList);
    out_report.height_diff_std_mm = computeStd(heightDiffList,
                                               out_report.height_diff_mean_mm);
    return true;
}

bool writeViceExtrinsicsJsonFile(const std::string& file_path,
                                 const ViceExtrinsicsReport& report,
                                 std::string& error_msg) {
    // 固定输出字段格式，方便后续程序或脚本直接消费。
    std::ofstream output(file_path.c_str(), std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        error_msg = "Failed to open vice extrinsics file for writing: " + file_path;
        return false;
    }

    output << std::fixed << std::setprecision(6);
    output << "{\n";
    output << "  \"board_to_gantry\": {\n";
    output << "    \"equation\": \"X_gantry = R_gb * X_board + t_gb\",\n";
    output << "    \"rvec_rad\": ";
    writeVec3dJson(output, report.board_rvec);
    output << ",\n";
    output << "    \"tvec_mm\": ";
    writeVec3dJson(output, report.board_tvec);
    output << "\n";
    output << "  },\n";
    output << "  \"camera_to_gantry\": {\n";
    output << "    \"equation\": \"X_gantry = R_gc * X_camera + t_gc\",\n";
    output << "    \"rvec_rad\": ";
    writeVec3dJson(output, report.camera_rvec);
    output << ",\n";
    output << "    \"tvec_mm\": ";
    writeVec3dJson(output, report.camera_tvec);
    output << "\n";
    output << "  },\n";
    output << "  \"metrics\": {\n";
    output << "    \"center_residual_mean_mm\": " << report.center_residual_mean_mm << ",\n";
    output << "    \"center_residual_std_mm\": " << report.center_residual_std_mm << ",\n";
    output << "    \"height_diff_mean_mm\": " << report.height_diff_mean_mm << ",\n";
    output << "    \"height_diff_std_mm\": " << report.height_diff_std_mm << "\n";
    output << "  }\n";
    output << "}\n";
    output.flush();

    if (!output.good()) {
        output.close();
        std::remove(file_path.c_str());
        error_msg = "Failed to write vice extrinsics file: " + file_path;
        return false;
    }

    output.close();
    if (output.fail()) {
        std::remove(file_path.c_str());
        error_msg = "Failed to finalize vice extrinsics file: " + file_path;
        return false;
    }

    return true;
}

cv::Vec3d loadLaserSpotBoardFromConfig(const ViceCameraConfig& service_config, bool verbose) {
    // 运行时显式传入时优先使用，适合调试或临时覆盖。
    if (std::isfinite(service_config.laser_spot_board_x_mm) &&
        std::isfinite(service_config.laser_spot_board_y_mm) &&
        std::isfinite(service_config.laser_spot_board_z_mm)) {
        if (verbose) {
            std::cout << "[ViceCamera] Laser spot from runtime config(mm)=["
                      << service_config.laser_spot_board_x_mm << ", "
                      << service_config.laser_spot_board_y_mm << ", "
                      << service_config.laser_spot_board_z_mm << "]" << std::endl;
        }
        return cv::Vec3d(service_config.laser_spot_board_x_mm,
                         service_config.laser_spot_board_y_mm,
                         service_config.laser_spot_board_z_mm);
    }

    ReallinkCVConfig config;
    // 运行时未指定时，回退到主配置文件中的 XYoffset 参数。
    if (!readReallinkCVConf(REALLINK_CV_CONF_PATH, config)) {
        if (verbose) {
            std::cout << "[ViceCamera] WARNING: Failed to load " << REALLINK_CV_CONF_PATH
                      << ", fallback laser spot board(mm)=[" << kDefaultLaserSpotBoard[0]
                      << ", " << kDefaultLaserSpotBoard[1] << ", " << kDefaultLaserSpotBoard[2]
                      << "]" << std::endl;
        }
        return kDefaultLaserSpotBoard;
    }

    const double laserXmm = config.cam0.dxPx * kXYoffsetMmPerPx;
    const double laserYmm = config.cam0.dyPx * kXYoffsetMmPerPx;
    if (verbose) {
        std::cout << "[ViceCamera] Laser spot from XYoffset: cam0.dxPx=" << config.cam0.dxPx
                  << ", cam0.dyPx=" << config.cam0.dyPx
                  << ", board(mm)=[" << laserXmm << ", " << laserYmm << ", 0]"
                  << std::endl;
    }
    return cv::Vec3d(laserXmm, laserYmm, 0.0);
}
