#include "YOLOSegmentor.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <unordered_map>

// ============================================================================
// Constructor / Destructor
// ============================================================================

YOLOSegmentor::YOLOSegmentor() {
    m_classNames = LoadClassNames();
}

YOLOSegmentor::~YOLOSegmentor() {
    ReleaseRKNNResources();
}

// ============================================================================
// Public: model loading
// ============================================================================

bool YOLOSegmentor::InitializeModel(const std::string& modelPath) {
    FILE* fp = fopen(modelPath.c_str(), "rb");
    if (!fp) {
        std::cerr << "[YOLOSegmentor] Cannot open model: " << modelPath << std::endl;
        return false;
    }
    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    std::vector<unsigned char> buf((size_t)sz);
    bool ok = (fread(buf.data(), 1, (size_t)sz, fp) == (size_t)sz);
    fclose(fp);
    if (!ok) {
        std::cerr << "[YOLOSegmentor] Failed to read model file." << std::endl;
        return false;
    }
    return InitRKNNCore(buf.data(), (uint32_t)sz);
}

bool YOLOSegmentor::InitializeModelFromMemory(const unsigned char* modelData, unsigned int modelSize) {
    return InitRKNNCore((void*)modelData, (uint32_t)modelSize);
}

// ============================================================================
// Private: shared RKNN core initialization
// ============================================================================

bool YOLOSegmentor::InitRKNNCore(void* modelData, uint32_t modelSize) {
    int ret = rknn_init(&m_ctx, modelData, modelSize, 0, nullptr);
    if (ret != RKNN_SUCC) {
        std::cerr << "[YOLOSegmentor] rknn_init failed: " << ret << std::endl;
        return false;
    }

    ret = rknn_query(m_ctx, RKNN_QUERY_IN_OUT_NUM, &m_io_num, sizeof(m_io_num));
    if (ret != RKNN_SUCC || m_io_num.n_input == 0 || m_io_num.n_output == 0) {
        std::cerr << "[YOLOSegmentor] rknn_query I/O num failed: " << ret << std::endl;
        ReleaseRKNNResources();
        return false;
    }

    m_input_attrs  = new rknn_tensor_attr[m_io_num.n_input]();
    m_output_attrs = new rknn_tensor_attr[m_io_num.n_output]();

    for (uint32_t i = 0; i < m_io_num.n_input; i++) {
        m_input_attrs[i].index = i;
        ret = rknn_query(m_ctx, RKNN_QUERY_INPUT_ATTR, &m_input_attrs[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            std::cerr << "[YOLOSegmentor] rknn_query input attr[" << i << "] failed: " << ret << std::endl;
            ReleaseRKNNResources();
            return false;
        }
        if (m_input_attrs[i].n_dims == 4) {
            m_inputHeight = (int)m_input_attrs[i].dims[1];
            m_inputWidth  = (int)m_input_attrs[i].dims[2];
        }
    }

    for (uint32_t i = 0; i < m_io_num.n_output; i++) {
        m_output_attrs[i].index = i;
        ret = rknn_query(m_ctx, RKNN_QUERY_OUTPUT_ATTR, &m_output_attrs[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            std::cerr << "[YOLOSegmentor] rknn_query output attr[" << i << "] failed: " << ret << std::endl;
            ReleaseRKNNResources();
            return false;
        }
    }

    m_input_mems  = new rknn_tensor_mem*[m_io_num.n_input]();
    m_output_mems = new rknn_tensor_mem*[m_io_num.n_output]();

    for (uint32_t i = 0; i < m_io_num.n_input; i++) {
        m_input_mems[i] = rknn_create_mem(m_ctx, m_input_attrs[i].size);
        if (!m_input_mems[i]) {
            std::cerr << "[YOLOSegmentor] rknn_create_mem input[" << i << "] failed." << std::endl;
            ReleaseRKNNResources();
            return false;
        }
        ret = rknn_set_io_mem(m_ctx, m_input_mems[i], &m_input_attrs[i]);
        if (ret != RKNN_SUCC) {
            std::cerr << "[YOLOSegmentor] rknn_set_io_mem input[" << i << "] failed: " << ret << std::endl;
            ReleaseRKNNResources();
            return false;
        }
    }

    for (uint32_t i = 0; i < m_io_num.n_output; i++) {
        m_output_mems[i] = rknn_create_mem(m_ctx, m_output_attrs[i].size);
        if (!m_output_mems[i]) {
            std::cerr << "[YOLOSegmentor] rknn_create_mem output[" << i << "] failed." << std::endl;
            ReleaseRKNNResources();
            return false;
        }
        ret = rknn_set_io_mem(m_ctx, m_output_mems[i], &m_output_attrs[i]);
        if (ret != RKNN_SUCC) {
            std::cerr << "[YOLOSegmentor] rknn_set_io_mem output[" << i << "] failed: " << ret << std::endl;
            ReleaseRKNNResources();
            return false;
        }
    }

    m_modelLoaded = true;
    return true;
}

// ============================================================================
// Private: release RKNN resources (safe for partial init)
// ============================================================================

void YOLOSegmentor::ReleaseRKNNResources() {
    if (m_ctx) {
        for (uint32_t i = 0; m_input_mems && i < m_io_num.n_input; i++)
            if (m_input_mems[i]) rknn_destroy_mem(m_ctx, m_input_mems[i]);
        for (uint32_t i = 0; m_output_mems && i < m_io_num.n_output; i++)
            if (m_output_mems[i]) rknn_destroy_mem(m_ctx, m_output_mems[i]);
        rknn_destroy(m_ctx);
        m_ctx = 0;
    }
    delete[] m_input_mems;   m_input_mems   = nullptr;
    delete[] m_output_mems;  m_output_mems  = nullptr;
    delete[] m_input_attrs;  m_input_attrs  = nullptr;
    delete[] m_output_attrs; m_output_attrs = nullptr;
    m_modelLoaded = false;
}

// ============================================================================
// JSON output
// ============================================================================

std::string YOLOSegmentor::ResultsToJson(const std::string& imageFile,
                                          const cv::Size& imageSize,
                                          const std::vector<SegmentationResult>& results) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "{\n  \"image\": {\"file\": \"" << imageFile
        << "\", \"width\": "  << imageSize.width
        << ", \"height\": " << imageSize.height << "},\n";
    oss << "  \"objects\": [\n";
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        oss << "    {\"class_id\": " << r.classId
            << ", \"class_name\": \""   << r.className << "\""
            << ", \"confidence\": "     << r.confidence
            << ", \"roi_xywh\": ["      << r.boundingBox.x << ", " << r.boundingBox.y
            << ", " << r.boundingBox.width << ", " << r.boundingBox.height << "]"
            << ", \"polygon\": [";
        for (size_t j = 0; j < r.contour.size(); ++j) {
            oss << "[" << r.contour[j].x << ", " << r.contour[j].y << "]";
            if (j + 1 < r.contour.size()) oss << ", ";
        }
        oss << "]}";
        if (i + 1 < results.size()) oss << ",";
        oss << "\n";
    }
    oss << "  ]\n}\n";
    return oss.str();
}

bool YOLOSegmentor::SaveResultsJson(const std::string& jsonPath,
                                     const std::string& imageFile,
                                     const cv::Size& imageSize,
                                     const std::vector<SegmentationResult>& results) const {
    std::ofstream ofs(jsonPath);
    if (!ofs.is_open()) return false;
    ofs << ResultsToJson(imageFile, imageSize, results);
    return true;
}

// ============================================================================
// Inference
// ============================================================================

std::vector<SegmentationResult> YOLOSegmentor::PerformInference(const cv::Mat& frame) {
    if (!m_modelLoaded) {
        std::cerr << "[YOLOSegmentor] Model not loaded." << std::endl;
        return {};
    }

    cv::Mat preprocessed = PreprocessFrame(frame);
    void*  input_ptr  = m_input_mems[0]->virt_addr;
    size_t input_alloc = m_input_mems[0]->size;
    const int input_type = m_input_attrs[0].type;

    if (input_type == RKNN_TENSOR_FLOAT32) {
        cv::Mat f32;
        preprocessed.convertTo(f32, CV_32FC3, 1.0 / 255.0);
        if (f32.total() * f32.elemSize() > input_alloc) {
            std::cerr << "[YOLOSegmentor] Input exceeds allocated memory." << std::endl;
            return {};
        }
        memcpy(input_ptr, f32.data, f32.total() * f32.elemSize());
    } else if (input_type == RKNN_TENSOR_FLOAT16) {
        cv::Mat f32;
        preprocessed.convertTo(f32, CV_32FC3, 1.0 / 255.0);
        const size_t n = f32.total() * f32.channels();
        if (n * sizeof(uint16_t) > input_alloc) {
            std::cerr << "[YOLOSegmentor] Input exceeds allocated memory." << std::endl;
            return {};
        }
        const float* src = (const float*)f32.data;
        uint16_t*    dst = (uint16_t*)input_ptr;
        for (size_t k = 0; k < n; k++) {
            uint32_t f32b; memcpy(&f32b, &src[k], 4);
            uint16_t sign = (uint16_t)((f32b >> 16) & 0x8000u);
            int      exp  = (int)((f32b >> 23) & 0xFFu) - 127 + 15;
            uint16_t mant = (uint16_t)((f32b >> 13) & 0x3FFu);
            if      (exp <= 0)  dst[k] = sign;
            else if (exp >= 31) dst[k] = sign | 0x7C00u;
            else                dst[k] = sign | (uint16_t)(exp << 10) | mant;
        }
    } else {
        if (preprocessed.total() * preprocessed.elemSize() > input_alloc) {
            std::cerr << "[YOLOSegmentor] Input exceeds allocated memory." << std::endl;
            return {};
        }
        memcpy(input_ptr, preprocessed.data, preprocessed.total() * preprocessed.elemSize());
    }

    int ret = rknn_run(m_ctx, nullptr);
    if (ret != RKNN_SUCC) {
        std::cerr << "[YOLOSegmentor] rknn_run failed: " << ret << std::endl;
        return {};
    }

    std::vector<cv::Mat> outputs;
    outputs.reserve(m_io_num.n_output);
    for (uint32_t i = 0; i < m_io_num.n_output; i++) {
        cv::Mat out = ConvertOutputToFloat(m_output_mems[i]->virt_addr, m_output_attrs[i]);
        if (out.empty()) return {};
        outputs.push_back(std::move(out));
    }

    return PostprocessResults(outputs, frame.size());
}

// ============================================================================
// Private: convert output tensor to float32 Mat
// ============================================================================

cv::Mat YOLOSegmentor::ConvertOutputToFloat(void* data, const rknn_tensor_attr& attr) {
    std::vector<int> dims(attr.dims, attr.dims + attr.n_dims);
    const size_t n = attr.n_elems;

    if (attr.type == RKNN_TENSOR_FLOAT32) {
        return cv::Mat(attr.n_dims, dims.data(), CV_32F, data).clone();
    }
    if (attr.type == RKNN_TENSOR_INT8) {
        cv::Mat out(attr.n_dims, dims.data(), CV_32F);
        const int8_t* src = (const int8_t*)data;
        float*        dst = (float*)out.data;
        for (size_t j = 0; j < n; j++)
            dst[j] = (float)(src[j] - attr.zp) * attr.scale;
        return out;
    }
    if (attr.type == RKNN_TENSOR_FLOAT16) {
        cv::Mat out(attr.n_dims, dims.data(), CV_32F);
        const uint16_t* src = (const uint16_t*)data;
        float*          dst = (float*)out.data;
        for (size_t j = 0; j < n; j++) {
            uint16_t h    = src[j];
            uint32_t sign = (uint32_t)(h & 0x8000u) << 16;
            int32_t  exp  = (h >> 10) & 0x1F;
            uint32_t mant = (uint32_t)(h & 0x3FFu) << 13;
            uint32_t f32b;
            if (exp == 0) {
                if (mant == 0) {
                    f32b = sign;
                } else {
                    exp = 1;
                    while ((mant & 0x00800000u) == 0) { mant <<= 1; exp--; }
                    mant &= ~0x00800000u;
                    f32b = sign | (uint32_t)(exp + 112) << 23 | mant;
                }
            } else if (exp == 31) {
                f32b = sign | 0x7F800000u | mant;
            } else {
                f32b = sign | (uint32_t)(exp + 112) << 23 | mant;
            }
            memcpy(&dst[j], &f32b, 4);
        }
        return out;
    }
    std::cerr << "[YOLOSegmentor] Unsupported output tensor type: " << attr.type << std::endl;
    return {};
}

// ============================================================================
// Preprocessing: letterbox (BGR→RGB, pad to 640×640 with gray border)
// ============================================================================

cv::Mat YOLOSegmentor::PreprocessFrame(const cv::Mat& frame) {
    const float r = std::min((float)m_inputHeight / frame.rows,
                             (float)m_inputWidth  / frame.cols);
    const int nw = (int)std::round(frame.cols * r);
    const int nh = (int)std::round(frame.rows * r);
    m_lastRatio = r;
    m_lastDw    = (m_inputWidth  - nw) * 0.5f;
    m_lastDh    = (m_inputHeight - nh) * 0.5f;

    cv::Mat resized;
    if (frame.cols != nw || frame.rows != nh)
        cv::resize(frame, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    else
        resized = frame;

    const int top    = (int)std::round(m_lastDh - 0.1f);
    const int bottom = (int)std::round(m_lastDh + 0.1f);
    const int left   = (int)std::round(m_lastDw - 0.1f);
    const int right  = (int)std::round(m_lastDw + 0.1f);

    cv::Mat padded, rgb;
    cv::copyMakeBorder(resized, padded, top, bottom, left, right,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
    return rgb;
}

// ============================================================================
// Postprocessing: decode detections → NMS → mask extraction → polygon
// ============================================================================

std::vector<SegmentationResult> YOLOSegmentor::PostprocessResults(
    const std::vector<cv::Mat>& outputs, const cv::Size& originalSize)
{
    std::vector<SegmentationResult> results;
    if (outputs.empty() || outputs[0].empty()) return results;

    const int numClasses      = (int)m_classNames.size();
    const int numMaskCoeffs   = 32;
    const int infoDimExpected = 4 + numClasses + numMaskCoeffs;

    // ---- Parse detection tensor to (numAnchors × infoDim) ----
    cv::Mat detections;
    int numAnchors = 0;
    {
        const cv::Mat& raw = outputs[0];
        if (raw.dims == 3 && raw.size[0] == 1) {
            int d1 = raw.size[1], d2 = raw.size[2];
            if (d1 == infoDimExpected) {
                numAnchors = d2;
                cv::transpose(cv::Mat(d1, d2, CV_32F, (void*)raw.ptr<float>()), detections);
            } else if (d2 == infoDimExpected) {
                numAnchors = d1;
                detections = cv::Mat(d1, d2, CV_32F, (void*)raw.ptr<float>()).clone();
            }
        } else if (raw.dims == 2 && raw.cols == infoDimExpected) {
            numAnchors = raw.rows;
            detections = raw;
        }
        if (detections.empty() || numAnchors == 0) {
            std::cerr << "[YOLOSegmentor] Unexpected detection tensor layout." << std::endl;
            return results;
        }
    }

    // ---- Pre-NMS: filter by confidence, decode boxes ----
    std::vector<cv::Rect>              bboxes;
    std::vector<float>                 scores;
    std::vector<int>                   classIds;
    std::vector<std::vector<float>>    maskCoeffs;

    bboxes.reserve(256); scores.reserve(256); classIds.reserve(256); maskCoeffs.reserve(256);

    for (int i = 0; i < numAnchors; i++) {
        float bestScore = 0.0f;
        int   bestClass = -1;
        for (int c = 0; c < numClasses; c++) {
            float s = detections.at<float>(i, 4 + c);  // already sigmoid-activated in model output
            if (s > bestScore) { bestScore = s; bestClass = c; }
        }
        if (bestScore < m_confidenceThreshold || bestClass < 0) continue;

        float cx = detections.at<float>(i, 0);
        float cy = detections.at<float>(i, 1);
        float bw = detections.at<float>(i, 2);
        float bh = detections.at<float>(i, 3);

        float x1 = std::max(0.0f, (cx - bw * 0.5f - m_lastDw) / m_lastRatio);
        float y1 = std::max(0.0f, (cy - bh * 0.5f - m_lastDh) / m_lastRatio);
        float x2 = std::min((float)originalSize.width,  (cx + bw * 0.5f - m_lastDw) / m_lastRatio);
        float y2 = std::min((float)originalSize.height, (cy + bh * 0.5f - m_lastDh) / m_lastRatio);

        int w = (int)(x2 - x1);
        int h = (int)(y2 - y1);
        if (w <= 0 || h <= 0) continue;

        bboxes.emplace_back((int)x1, (int)y1, w, h);
        scores.emplace_back(bestScore);
        classIds.emplace_back(bestClass);

        std::vector<float> coeff(numMaskCoeffs);
        for (int m = 0; m < numMaskCoeffs; m++)
            coeff[m] = detections.at<float>(i, 4 + numClasses + m);
        maskCoeffs.emplace_back(std::move(coeff));
    }

    if (bboxes.empty()) return results;

    // ---- NMS ----
    std::vector<int> order(scores.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return scores[a] > scores[b]; });

    std::vector<bool> suppressed(scores.size(), false);
    std::vector<int>  indices;
    for (size_t i = 0; i < order.size(); i++) {
        int idx = order[i];
        if (suppressed[idx]) continue;
        indices.push_back(idx);
        const cv::Rect& b1 = bboxes[idx];
        for (size_t j = i + 1; j < order.size(); j++) {
            int idx2 = order[j];
            if (suppressed[idx2]) continue;
            const cv::Rect& b2 = bboxes[idx2];
            int ix = std::max(0, std::min(b1.x + b1.width,  b2.x + b2.width)  - std::max(b1.x, b2.x));
            int iy = std::max(0, std::min(b1.y + b1.height, b2.y + b2.height) - std::max(b1.y, b2.y));
            float iou = (float)(ix * iy) / (b1.area() + b2.area() - ix * iy);
            if (iou > m_nmsThreshold) suppressed[idx2] = true;
        }
    }

    // Per-class top-1: keep only highest-confidence detection per class
    {
        std::unordered_map<int, int> best; // class_id -> index in bboxes/scores
        for (int idx : indices) {
            auto it = best.find(classIds[idx]);
            if (it == best.end() || scores[idx] > scores[it->second])
                best[classIds[idx]] = idx;
        }
        indices.clear();
        for (auto& kv : best) indices.push_back(kv.second);
    }

    // ---- Mask proto tensor ----
    bool hasMaskProto = (outputs.size() > 1 && !outputs[1].empty() &&
                         outputs[1].dims == 4 && outputs[1].size[1] == numMaskCoeffs &&
                         outputs[1].size[2] == 160 && outputs[1].size[3] == 160);
    cv::Mat protos2D;
    if (hasMaskProto)
        protos2D = outputs[1].reshape(1, numMaskCoeffs);  // (32, 25600)

    // ---- Per-detection mask processing ----
    for (int idx : indices) {
        SegmentationResult result;
        result.classId    = classIds[idx];
        result.className  = m_classNames[classIds[idx]];
        result.confidence = scores[idx];
        result.boundingBox = bboxes[idx];

        if (!hasMaskProto) {
            results.push_back(std::move(result));
            continue;
        }

        // 1. coeffs × protos → sigmoid at prototype resolution (160×160)
        cv::Mat coeffsMat(1, numMaskCoeffs, CV_32F, const_cast<float*>(maskCoeffs[idx].data()));
        cv::Mat maskFlat;
        cv::gemm(coeffsMat, protos2D, 1.0, cv::Mat(), 0.0, maskFlat);   // 1×25600
        cv::Mat maskFloat = maskFlat.reshape(1, 160);                    // 160×160
        cv::exp(-maskFloat, maskFloat);
        maskFloat = 1.0f / (1.0f + maskFloat);                          // sigmoid

        // 2. Remove letterbox padding at prototype level
        const int protoH = maskFloat.rows, protoW = maskFloat.cols;
        const float ps = (float)protoH / m_inputHeight;                  // 0.25
        int y1p = std::max(0, std::min((int)std::round(m_lastDh * ps), protoH));
        int x1p = std::max(0, std::min((int)std::round(m_lastDw * ps), protoW));
        int y2p = std::max(0, std::min((int)std::round((m_lastDh + originalSize.height * m_lastRatio) * ps), protoH));
        int x2p = std::max(0, std::min((int)std::round((m_lastDw + originalSize.width  * m_lastRatio) * ps), protoW));

        cv::Mat maskNoPad = (y2p > y1p && x2p > x1p)
            ? maskFloat(cv::Rect(x1p, y1p, x2p - x1p, y2p - y1p))
            : maskFloat;

        // 3. One-step resize to original image size
        cv::Mat maskOrig;
        cv::resize(maskNoPad, maskOrig, originalSize, 0, 0, cv::INTER_LINEAR);

        // 4. Crop to bbox at original resolution
        cv::Rect bbox = bboxes[idx] & cv::Rect(0, 0, originalSize.width, originalSize.height);
        cv::Mat maskCropped = cv::Mat::zeros(maskOrig.size(), CV_32F);
        if (bbox.width > 0 && bbox.height > 0)
            maskOrig(bbox).copyTo(maskCropped(bbox));

        // 5. Threshold → binary mask
        cv::Mat binMask;
        cv::threshold(maskCropped, binMask, 0.5, 255.0, cv::THRESH_BINARY);
        binMask.convertTo(binMask, CV_8UC1);
        result.mask = binMask;

        // 6. Contour extraction
        std::vector<std::vector<cv::Point>> cc;
        cv::findContours(binMask.clone(), cc, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);
        if (!cc.empty()) {
            const auto& contour = *std::max_element(cc.begin(), cc.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });
            if (contour.size() >= 3) {
                result.contour_full = contour;
                std::vector<cv::Point> approx;
                cv::approxPolyDP(contour, approx,
                    0.003 * cv::arcLength(contour, true), true);
                result.contour     = approx.size() >= 3 ? approx : contour;
                result.boundingBox = cv::boundingRect(contour);
            }
        }

        results.push_back(std::move(result));
    }
    return results;
}

// ============================================================================
// Draw results
// ============================================================================

void YOLOSegmentor::DrawResults(cv::Mat& frame, const std::vector<SegmentationResult>& results) {
    for (const auto& r : results) {
        cv::Scalar color = GetClassColor(r.classId);
        if (!r.mask.empty()) {
            cv::Mat overlay = cv::Mat::zeros(frame.size(), frame.type());
            overlay.setTo(color, r.mask);
            cv::Mat blended;
            cv::addWeighted(frame, 0.45, overlay, 0.55, 0.0, blended);
            blended.copyTo(frame, r.mask);
        }
        if (r.contour.size() >= 3) {
            std::vector<std::vector<cv::Point>> poly{r.contour};
            cv::polylines(frame, poly, true, color, 1, cv::LINE_AA);
        }
    }
}

// ============================================================================
// Class names & colors
// ============================================================================

std::vector<std::string> YOLOSegmentor::LoadClassNames() {
    return {
        "304_stainless_steel", "acrylic",             "african_padauk",    "ash_wood",
        "bamboo",              "black_walnut",         "camphor_wood",      "cedar",
        "cherry_wood",         "chinaberry_tree",      "chinese_toon_wood", "cliff_cypress",
        "cobblestone",         "corrugated_paper",     "ebony",             "jade",
        "kraft_paper",         "leaf",                 "linden_plywood",    "maple_wood",
        "MDF_density_board",   "patterned_leather",    "paulownia_wood",    "pine_unpainted_board",
        "poplar_plywood",      "red_oak",              "red_sandalwood",    "rubberwood",
        "sapele",              "silicone_sheet",        "solid_color_leather","titanium_metal",
        "varnished_pine",      "velvet",               "white_marble",      "white_oak"
    };
}

cv::Scalar YOLOSegmentor::GetClassColor(int classId) {
    static const cv::Scalar palette[] = {
        {0,0,255}, {0,255,0}, {255,0,0}, {255,255,0}, {255,0,255},
        {0,255,255}, {128,0,0}, {0,128,0}, {0,0,128}, {128,128,0},
        {128,0,128}, {0,128,128}, {64,0,0}, {0,64,0}, {0,0,64},
        {192,0,0}, {0,192,0}, {0,0,192}, {192,192,0}, {192,0,192}
    };
    static const int N = (int)(sizeof(palette) / sizeof(palette[0]));
    return palette[((classId % N) + N) % N];
}
