#ifndef _REALLINK_YOLO_MODEL_H_
#define _REALLINK_YOLO_MODEL_H_

#include "rknn_api.h"
#include "yolo_common.h"
#include "yolo_preprocess.h"
#include "yolo_postprocess.h"

#include <vector>
#include <string>
#include <chrono>

class YOLOModel {
public:
    struct Timing {
        double preprocessMs = 0.0;
        double inferenceMs  = 0.0;
        double postprocessMs = 0.0;
        double totalMs       = 0.0;
    };

    YOLOModel();
    ~YOLOModel();

    int initFromMemory(const unsigned char* modelData, unsigned int modelSize);
    int initFromFile(const std::string& modelPath);
    int release();

    bool isLoaded() const { return m_loaded; }
    int  inputWidth()  const { return m_inputW; }
    int  inputHeight() const { return m_inputH; }

    void setConfidence(float c) { m_confThreshold = c; }
    void setNMS(float n)        { m_nmsThreshold  = n; }

    // 三段一站式: 等价官方 inference_yolov5_model()
    ObjectDetectResultList inference(const cv::Mat& bgr);

    // 返回带 mask/contour 的完整结果
    std::vector<SegmentationResult> inferenceSegmentation(const cv::Mat& bgr,
                                                           Timing* timing = nullptr);

private:
    int  initCore(void* modelData, uint32_t modelSize);
    void releaseCore();
    cv::Mat convertOutputToFloat(void* data, const rknn_tensor_attr& attr);

    rknn_context          m_ctx{0};
    rknn_input_output_num m_ioNum{};
    rknn_tensor_attr*     m_inputAttrs{nullptr};
    rknn_tensor_attr*     m_outputAttrs{nullptr};
    rknn_tensor_mem**     m_inputMems{nullptr};
    rknn_tensor_mem**     m_outputMems{nullptr};

    bool  m_loaded{false};
    int   m_inputW{640};
    int   m_inputH{640};
    float m_confThreshold{0.05f};
    float m_nmsThreshold{0.7f};
    bool  m_isQuant{false};

    LetterBox m_lastLetterBox;
    cv::Size  m_lastOriginalSize;

    std::vector<std::string> m_classNames;
    static std::vector<std::string> loadClassNames();
};

#endif
