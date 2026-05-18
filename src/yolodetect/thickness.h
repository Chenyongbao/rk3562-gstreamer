/**
 * @file thickness.h
 * @brief 厚度检测模块 - 复用 YOLO 检测结果并控制 Klipper 进行测高
 * 
 * 功能流程：
 * 1. 使用 YOLO 检测结果（roi_xywh）选定目标框
 * 2. 计算中心像素坐标
 * 3. 将中心像素坐标按比例换算到 Klipper XY
 * 4. 控制 Klipper 移动并执行 Z 下探测高
 * 5. 读取 totalHigh，并将厚度补偿直接更新到 BEV 运行时状态
 */

#ifndef THICKNESS_H
#define THICKNESS_H

#include <string>
#include <cstdint>
#include "yolo_wrapper.h"
#include "../config.h"

/**
 * @brief 厚度检测配置结构体
 */
struct ThicknessConfig {
    double feedrate = 4500.0;                      // 移动速度 (mm/min)
    double z_step = 10.0;                           // Z轴下降步长 (mm)
    double z_max = 30.0;                          // Z轴最大下降距离 (mm)
    int max_attempts = 4;                         // 最大尝试次数
    std::string conf_path = REALLINK_CV_CONF_PATH;           // 配置文件路径
    double pixel_ratio = 4.0;                      // 像素到电机坐标换算比例（除数）
};

/**
 * @brief 检测结果结构体
 */
struct ThicknessResult {
    bool triggered = false;                     // 是否触发 thickness 流程
    bool selected_by_max_area = false;          // 是否使用了“多目标最大面积选择”
    int detection_count = 0;                    // 本次检测目标数量

    int class_id = -1;                          // 选中目标类别
    double confidence = 0.0;                    // 选中目标置信度
    int roi_x = 0;
    int roi_y = 0;
    int roi_w = 0;
    int roi_h = 0;

    double center_x = 0.0;                      // 选中目标中心像素 X
    double center_y = 0.0;                      // 选中目标中心像素 Y
    double move_x = 0.0;                        // 电机移动目标 X（center_x / pixel_ratio）
    double move_y = 0.0;                        // 电机移动目标 Y（center_y / pixel_ratio）

    double distance_mm = 0.0;                   // 激光 distance
    double z_drop_mm = 0.0;                     // 实际 Z 下探量 final_z_mm
    double measured_height_mm = 0.0;            // distance + z_drop（仅用于展示）
    double total_high_mm = 0.0;                 // 配置 totalHigh
    double thickness_high_mm = 0.0;             // 本次计算出的厚度补偿(mm)
};

/**
 * @brief 厚度检测服务类
 *
 * 负责基于 YOLO 检测框执行移动、测高与 BEV 运行时厚度补偿更新
 */
class ThicknessService {
public:
    /**
     * @brief 构造函数
     * @param config 配置参数
     */
    explicit ThicknessService(const ThicknessConfig& config);

    /**
     * @brief 析构函数
     */
    ~ThicknessService() = default;

    /**
     * @brief 基于 YOLO 检测结果执行完整厚度流程
     * @param yolo_result DETECT 命令本次推理结果
     * @param result 输出检测结果
     * @param errorMsg 错误信息
     * @return true 成功，false 失败
     */
    bool measureFromYolo(const YOLOFrameResult& yolo_result, ThicknessResult& result, std::string& errorMsg);

private:
    bool selectTargetDetection(const YOLOFrameResult& yolo_result,
                               YOLODetection& selected_det,
                               bool& selected_by_max_area,
                               std::string& errorMsg) const;

    bool moveToXY(double x, double y, std::string& errorMsg) const;
    bool probeDistanceAndZDrop(double& out_distance_mm, double& out_z_drop_mm, std::string& errorMsg) const;

    // 配置参数
    double feedrate_;
    double z_step_;
    double z_max_;
    int max_attempts_;
    std::string conf_path_;
    double pixel_ratio_;
};

#endif // THICKNESS_H
