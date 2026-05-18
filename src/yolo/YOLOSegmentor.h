#pragma once

#include <opencv2/opencv.hpp>
#include "rknn_api.h"
#include <string>
#include <vector>

struct SegmentationResult {
    int classId{-1};
    std::string className;
    float confidence{0.0f};
    cv::Rect boundingBox;
    cv::Mat mask;
    std::vector<cv::Point> contour;  // unified contour for JSON / drawing / wrapper polygon
    std::vector<cv::Point> contour_full;
};

class YOLOSegmentor {
public:
    YOLOSegmentor();
    ~YOLOSegmentor();

    bool InitializeModel(const std::string& modelPath);
    bool InitializeModelFromMemory(const unsigned char* modelData, unsigned int modelSize);

    void SetConfidenceThreshold(float threshold) { m_confidenceThreshold = threshold; }
    void SetNMSThreshold(float threshold)         { m_nmsThreshold = threshold; }
    std::vector<std::string> GetClassNames() const { return m_classNames; }

    std::vector<SegmentationResult> PerformInference(const cv::Mat& frame);
    void DrawResults(cv::Mat& frame, const std::vector<SegmentationResult>& results);

    std::string ResultsToJson(const std::string& imageFile, const cv::Size& imageSize,
                              const std::vector<SegmentationResult>& results) const;
    bool SaveResultsJson(const std::string& jsonPath, const std::string& imageFile,
                         const cv::Size& imageSize,
                         const std::vector<SegmentationResult>& results) const;

private:
    bool InitRKNNCore(void* modelData, uint32_t modelSize);
    void ReleaseRKNNResources();
    cv::Mat PreprocessFrame(const cv::Mat& frame);
    std::vector<SegmentationResult> PostprocessResults(const std::vector<cv::Mat>& outputs,
                                                       const cv::Size& originalSize);
    static cv::Mat ConvertOutputToFloat(void* data, const rknn_tensor_attr& attr);
    static cv::Scalar GetClassColor(int classId);
    static std::vector<std::string> LoadClassNames();

    rknn_context          m_ctx{0};
    rknn_input_output_num m_io_num{};
    rknn_tensor_attr*     m_input_attrs{nullptr};
    rknn_tensor_attr*     m_output_attrs{nullptr};
    rknn_tensor_mem**     m_input_mems{nullptr};
    rknn_tensor_mem**     m_output_mems{nullptr};
    bool                  m_modelLoaded{false};
    float                 m_confidenceThreshold{0.05f};
    float                 m_nmsThreshold{0.7f};
    int                   m_inputWidth{640};
    int                   m_inputHeight{640};
    float                 m_lastRatio{1.0f};
    float                 m_lastDw{0.0f};
    float                 m_lastDh{0.0f};
    std::vector<std::string> m_classNames;
};
