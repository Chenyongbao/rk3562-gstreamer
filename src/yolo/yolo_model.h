#ifndef _REALLINK_YOLO_MODEL_H_
#define _REALLINK_YOLO_MODEL_H_

#include "rknn_api.h"
#include "yolo_common.h"
#include "yolo_preprocess.h"
#include "yolo_postprocess.h"

#include <vector>
#include <string>
#include <chrono>

// YOLO 模型封装类，负责管理 RKNN 运行环境、推理及结果后处理
class YOLOModel {
public:
    // 推理各阶段的耗时统计
    struct Timing {
        double preprocessMs = 0.0;  // 预处理耗时 (ms)
        double inferenceMs  = 0.0;  // 模型推理耗时 (ms)
        double postprocessMs = 0.0; // 后处理耗时 (ms)
        double totalMs       = 0.0; // 总耗时 (ms)
    };

    YOLOModel();
    ~YOLOModel();

    // 从内存数据初始化 RKNN 模型
    int initFromMemory(const unsigned char* modelData, unsigned int modelSize);
    // 释放模型及上下文资源
    int release();

    // 模型是否成功加载
    bool isLoaded() const { return m_loaded; }

    // 设置置信度阈值
    void setConfidence(float c) { m_confThreshold = c; }
    // 设置非极大值抑制 (NMS) 阈值
    void setNMS(float n)        { m_nmsThreshold  = n; }

    // 实例分割推理：返回包含 mask 和轮廓的完整分割结果
    std::vector<SegmentationResult> inferenceSegmentation(const cv::Mat& bgr,
                                                           Timing* timing = nullptr);

private:
    // 核心初始化逻辑
    int  initCore(void* modelData, uint32_t modelSize);
    // 核心释放逻辑
    void releaseCore();
    // 将 RKNN 输出张量统一转换为 Float 类型的 cv::Mat
    cv::Mat convertOutputToFloat(void* data, const rknn_tensor_attr& attr);

    rknn_context          m_ctx{0};                 // RKNN 上下文句柄
    rknn_input_output_num m_ioNum{};                // 模型输入输出张量数量
    rknn_tensor_attr*     m_inputAttrs{nullptr};    // 输入张量属性数组
    rknn_tensor_attr*     m_outputAttrs{nullptr};   // 输出张量属性数组
    rknn_tensor_mem**     m_inputMems{nullptr};     // 输入张量内存数组
    rknn_tensor_mem**     m_outputMems{nullptr};    // 输出张量内存数组

    bool  m_loaded{false};          // 加载状态标志
    int   m_inputW{640};            // 模型输入宽度
    int   m_inputH{640};            // 模型输入高度
    float m_confThreshold{0.05f};   // 置信度阈值
    float m_nmsThreshold{0.7f};     // NMS 阈值

    std::vector<std::string> m_classNames;          // 类别名称列表
    static std::vector<std::string> loadClassNames(); // 加载默认类别名称
};

#endif
