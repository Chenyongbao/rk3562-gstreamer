#include "yolo_model.h"
#include <cstdio>
#include <cstring>
#include <numeric>
#include <unordered_map>

YOLOModel::YOLOModel() : m_classNames(loadClassNames()) {}
YOLOModel::~YOLOModel() { release(); }

int YOLOModel::initFromMemory(const unsigned char* data, unsigned int size) {
    return initCore((void*)data, (uint32_t)size);
}

int YOLOModel::initFromFile(const std::string& path) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return -1;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::vector<unsigned char> buf((size_t)sz);
    bool ok = (fread(buf.data(), 1, (size_t)sz, f) == (size_t)sz);
    fclose(f);
    if (!ok) return -1;
    return initCore(buf.data(), (uint32_t)sz);
}

int YOLOModel::release() {
    releaseCore();
    m_loaded = false;
    return 0;
}

// ============================================================================
// initCore — 对标官方 init_yolov5_model()
// ============================================================================

int YOLOModel::initCore(void* modelData, uint32_t modelSize) {
    int ret = rknn_init(&m_ctx, modelData, modelSize, 0, nullptr);
    if (ret != RKNN_SUCC) {
        fprintf(stderr, "[YOLO] rknn_init failed: %d\n", ret);
        return -1;
    }

    ret = rknn_query(m_ctx, RKNN_QUERY_IN_OUT_NUM, &m_ioNum, sizeof(m_ioNum));
    if (ret != RKNN_SUCC || m_ioNum.n_input == 0 || m_ioNum.n_output == 0) {
        fprintf(stderr, "[YOLO] query I/O num failed\n");
        releaseCore();
        return -1;
    }

    m_inputAttrs  = new rknn_tensor_attr[m_ioNum.n_input]();
    m_outputAttrs = new rknn_tensor_attr[m_ioNum.n_output]();

    for (uint32_t i = 0; i < m_ioNum.n_input; i++) {
        m_inputAttrs[i].index = i;
        ret = rknn_query(m_ctx, RKNN_QUERY_INPUT_ATTR, &m_inputAttrs[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) { releaseCore(); return -1; }
        if (m_inputAttrs[i].n_dims == 4) {
            m_inputH = (int)m_inputAttrs[i].dims[1];
            m_inputW = (int)m_inputAttrs[i].dims[2];
        }
    }

    for (uint32_t i = 0; i < m_ioNum.n_output; i++) {
        m_outputAttrs[i].index = i;
        ret = rknn_query(m_ctx, RKNN_QUERY_OUTPUT_ATTR, &m_outputAttrs[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) { releaseCore(); return -1; }
    }

    m_isQuant = (m_outputAttrs[0].type == RKNN_TENSOR_INT8);

    m_inputMems  = new rknn_tensor_mem*[m_ioNum.n_input]();
    m_outputMems = new rknn_tensor_mem*[m_ioNum.n_output]();

    for (uint32_t i = 0; i < m_ioNum.n_input; i++) {
        m_inputMems[i] = rknn_create_mem(m_ctx, m_inputAttrs[i].size);
        if (!m_inputMems[i]) { releaseCore(); return -1; }
        ret = rknn_set_io_mem(m_ctx, m_inputMems[i], &m_inputAttrs[i]);
        if (ret != RKNN_SUCC) { releaseCore(); return -1; }
    }

    for (uint32_t i = 0; i < m_ioNum.n_output; i++) {
        m_outputMems[i] = rknn_create_mem(m_ctx, m_outputAttrs[i].size);
        if (!m_outputMems[i]) { releaseCore(); return -1; }
        ret = rknn_set_io_mem(m_ctx, m_outputMems[i], &m_outputAttrs[i]);
        if (ret != RKNN_SUCC) { releaseCore(); return -1; }
    }

    m_loaded = true;
    return 0;
}

void YOLOModel::releaseCore() {
    if (m_ctx) {
        for (uint32_t i = 0; m_inputMems && i < m_ioNum.n_input; i++)
            if (m_inputMems[i]) rknn_destroy_mem(m_ctx, m_inputMems[i]);
        for (uint32_t i = 0; m_outputMems && i < m_ioNum.n_output; i++)
            if (m_outputMems[i]) rknn_destroy_mem(m_ctx, m_outputMems[i]);
        rknn_destroy(m_ctx);
        m_ctx = 0;
    }
    delete[] m_inputMems;   m_inputMems   = nullptr;
    delete[] m_outputMems;  m_outputMems  = nullptr;
    delete[] m_inputAttrs;  m_inputAttrs  = nullptr;
    delete[] m_outputAttrs; m_outputAttrs = nullptr;
}

// ============================================================================
// 推理（仅检测框，对标官方 inference_yolov5_model）
// ============================================================================

static void fillInputTensor(rknn_tensor_mem* mem, const rknn_tensor_attr& attr,
                             const cv::Mat& preprocessed) {
    void* dst = mem->virt_addr;
    size_t alloc = mem->size;

    if (attr.type == RKNN_TENSOR_FLOAT32) {
        cv::Mat f32;
        preprocessed.convertTo(f32, CV_32FC3, 1.0 / 255.0);
        if (f32.total() * f32.elemSize() <= alloc)
            memcpy(dst, f32.data, f32.total() * f32.elemSize());
    } else if (attr.type == RKNN_TENSOR_FLOAT16) {
        cv::Mat f32;
        preprocessed.convertTo(f32, CV_32FC3, 1.0 / 255.0);
        const size_t n = f32.total() * f32.channels();
        if (n * sizeof(uint16_t) > alloc) return;
        const float* src = (const float*)f32.data;
        uint16_t*    out = (uint16_t*)dst;
        for (size_t k = 0; k < n; k++) {
            uint32_t f32b; memcpy(&f32b, &src[k], 4);
            uint16_t sign = (uint16_t)((f32b >> 16) & 0x8000u);
            int      exp  = (int)((f32b >> 23) & 0xFFu) - 127 + 15;
            uint16_t mant = (uint16_t)((f32b >> 13) & 0x3FFu);
            if      (exp <= 0)  out[k] = sign;
            else if (exp >= 31) out[k] = sign | 0x7C00u;
            else                out[k] = sign | (uint16_t)(exp << 10) | mant;
        }
    } else {
        if (preprocessed.total() * preprocessed.elemSize() <= alloc)
            memcpy(dst, preprocessed.data, preprocessed.total() * preprocessed.elemSize());
    }
}

ObjectDetectResultList YOLOModel::inference(const cv::Mat& bgr) {
    ObjectDetectResultList out;
    out.clear();
    if (!m_loaded) return out;

    // Stage 1: Preprocess
    LetterBox lb;
    cv::Mat preprocessed = Preprocess::letterboxBGRtoRGB(bgr, m_inputW, m_inputH, lb);
    m_lastLetterBox  = lb;
    m_lastOriginalSize = bgr.size();

    // Stage 2: Fill input, Run, Collect outputs
    fillInputTensor(m_inputMems[0], m_inputAttrs[0], preprocessed);

    if (rknn_run(m_ctx, nullptr) != RKNN_SUCC) {
        fprintf(stderr, "[YOLO] rknn_run failed\n");
        return out;
    }

    std::vector<cv::Mat> outputs;
    outputs.reserve(m_ioNum.n_output);
    for (uint32_t i = 0; i < m_ioNum.n_output; i++) {
        cv::Mat mat = convertOutputToFloat(m_outputMems[i]->virt_addr, m_outputAttrs[i]);
        if (mat.empty()) return out;
        outputs.push_back(std::move(mat));
    }

    // Stage 3: Postprocess
    const int numClasses  = (int)m_classNames.size();
    const int infoDim     = 4 + numClasses + 32;
    const int numAnchors  = (outputs[0].dims == 2 || outputs[0].dims == 3)
                          ? outputs[0].size[1] : outputs[0].size[2];

    std::vector<ObjectDetectResult> candidates;
    Postprocess::decode((float*)outputs[0].data, numAnchors, infoDim,
                        numClasses, m_confThreshold, candidates);
    Postprocess::nms(candidates, m_nmsThreshold, out, m_inputW, m_inputH);

    return out;
}

std::vector<SegmentationResult> YOLOModel::inferenceSegmentation(
    const cv::Mat& bgr, Timing* timing)
{
    auto t0 = std::chrono::high_resolution_clock::now();
    if (!m_loaded) return {};

    // Stage 1: Preprocess
    LetterBox lb;
    cv::Mat preprocessed = Preprocess::letterboxBGRtoRGB(bgr, m_inputW, m_inputH, lb);
    m_lastLetterBox  = lb;
    m_lastOriginalSize = bgr.size();

    // Stage 2: Fill + Run + Collect
    fillInputTensor(m_inputMems[0], m_inputAttrs[0], preprocessed);

    auto t1 = std::chrono::high_resolution_clock::now();

    if (rknn_run(m_ctx, nullptr) != RKNN_SUCC) {
        fprintf(stderr, "[YOLO] rknn_run failed\n");
        return {};
    }

    std::vector<cv::Mat> outputs;
    outputs.reserve(m_ioNum.n_output);
    for (uint32_t i = 0; i < m_ioNum.n_output; i++) {
        cv::Mat mat = convertOutputToFloat(m_outputMems[i]->virt_addr, m_outputAttrs[i]);
        if (mat.empty()) return {};
        outputs.push_back(std::move(mat));
    }

    auto t2 = std::chrono::high_resolution_clock::now();

    // Stage 3: Postprocess (full segmentation)
    const int numClasses = (int)m_classNames.size();
    const int infoDim    = 4 + numClasses + 32;

    int numAnchors = 0;
    cv::Mat detMat;
    const cv::Mat& raw0 = outputs[0];
    if (raw0.dims == 3 && raw0.size[0] == 1) {
        int d1 = raw0.size[1], d2 = raw0.size[2];
        if (d1 == infoDim) { numAnchors = d2; cv::transpose(cv::Mat(d1, d2, CV_32F, (void*)raw0.ptr<float>()), detMat); }
        else if (d2 == infoDim) { numAnchors = d1; detMat = cv::Mat(d1, d2, CV_32F, (void*)raw0.ptr<float>()).clone(); }
    } else if (raw0.dims == 2 && raw0.cols == infoDim) {
        numAnchors = raw0.rows;
        detMat = raw0;
    }
    if (detMat.empty()) return {};

    std::vector<ObjectDetectResult> candidates;
    Postprocess::decode((float*)detMat.data, numAnchors, infoDim,
                        numClasses, m_confThreshold, candidates);

    ObjectDetectResultList odResults;
    odResults.clear();
    Postprocess::nms(candidates, m_nmsThreshold, odResults, m_inputW, m_inputH);

    // mask decode
    cv::Mat protoTensor;
    if (outputs.size() > 1)
        protoTensor = outputs[1];

    auto results = Postprocess::extractMasks(odResults, protoTensor,
                                              bgr.size(), lb, m_classNames);

    auto t3 = std::chrono::high_resolution_clock::now();

    if (timing) {
        timing->preprocessMs  = std::chrono::duration<double, std::milli>(t1 - t0).count();
        timing->inferenceMs   = std::chrono::duration<double, std::milli>(t2 - t1).count();
        timing->postprocessMs = std::chrono::duration<double, std::milli>(t3 - t2).count();
        timing->totalMs       = std::chrono::duration<double, std::milli>(t3 - t0).count();
    }

    return results;
}

// ============================================================================
// 数据类型转换 — 对标官方
// ============================================================================

cv::Mat YOLOModel::convertOutputToFloat(void* data, const rknn_tensor_attr& attr) {
    std::vector<int> dims(attr.dims, attr.dims + attr.n_dims);
    const size_t n = attr.n_elems;

    if (attr.type == RKNN_TENSOR_FLOAT32)
        return cv::Mat(attr.n_dims, dims.data(), CV_32F, data).clone();

    if (attr.type == RKNN_TENSOR_INT8) {
        cv::Mat out(attr.n_dims, dims.data(), CV_32F);
        const int8_t* src = (const int8_t*)data;
        float* dst = (float*)out.data;
        for (size_t j = 0; j < n; j++)
            dst[j] = (float)(src[j] - attr.zp) * attr.scale;
        return out;
    }

    if (attr.type == RKNN_TENSOR_FLOAT16) {
        cv::Mat out(attr.n_dims, dims.data(), CV_32F);
        const uint16_t* src = (const uint16_t*)data;
        float* dst = (float*)out.data;
        for (size_t j = 0; j < n; j++) {
            uint16_t h = src[j];
            uint32_t sign = (uint32_t)(h & 0x8000u) << 16;
            int32_t  exp  = (h >> 10) & 0x1F;
            uint32_t mant = (uint32_t)(h & 0x3FFu) << 13;
            uint32_t f32b;
            if (exp == 0) {
                if (mant == 0) f32b = sign;
                else {
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

    fprintf(stderr, "[YOLO] Unsupported output type: %d\n", attr.type);
    return {};
}

// ============================================================================
// Class names
// ============================================================================

std::vector<std::string> YOLOModel::loadClassNames() {
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
