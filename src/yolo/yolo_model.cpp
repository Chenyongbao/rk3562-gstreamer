#include "yolo_model.h"
#include <cstdio>
#include <cstring>

YOLOModel::YOLOModel() : m_classNames(loadClassNames()) {}
YOLOModel::~YOLOModel() { release(); }

int YOLOModel::initFromMemory(const unsigned char *data, unsigned int size) {
  return initCore((void *)data, (uint32_t)size);
}

int YOLOModel::release() {
  releaseCore();
  m_loaded = false;
  return 0;
}

// ============================================================================
// 核心初始化 — 对标官方 init_yolov5_model()，负责创建 RKNN 上下文和内存
// ============================================================================

int YOLOModel::initCore(void *modelData, uint32_t modelSize) {
  // 1. 初始化 RKNN 运行环境，传入模型内存指针和大小，获取模型句柄 (m_ctx)
  int ret = rknn_init(&m_ctx, modelData, modelSize, 0, nullptr);
  if (ret != RKNN_SUCC) {
    fprintf(stderr, "[YOLO] rknn_init failed: %d\n", ret);
    return -1;
  }

  // 2. 查询模型的输入输出张量（Tensor）数量
  ret = rknn_query(m_ctx, RKNN_QUERY_IN_OUT_NUM, &m_ioNum, sizeof(m_ioNum));
  if (ret != RKNN_SUCC || m_ioNum.n_input == 0 || m_ioNum.n_output == 0) {
    fprintf(stderr, "[YOLO] query I/O num failed\n");
    releaseCore();
    return -1;
  }

  // 为输入和输出张量属性数组分配内存
  m_inputAttrs = new rknn_tensor_attr[m_ioNum.n_input]();
  m_outputAttrs = new rknn_tensor_attr[m_ioNum.n_output]();

  // 3. 循环查询所有输入张量的属性（如形状、数据类型、量化参数等）
  for (uint32_t i = 0; i < m_ioNum.n_input; i++) {
    m_inputAttrs[i].index = i;
    ret = rknn_query(m_ctx, RKNN_QUERY_INPUT_ATTR, &m_inputAttrs[i],
                     sizeof(rknn_tensor_attr));
    if (ret != RKNN_SUCC) {
      releaseCore();
      return -1;
    }
    // 提取模型的输入宽高（通常是 NCHW 格式，这里取 dims[1] 为高，dims[2] 为宽）
    if (m_inputAttrs[i].n_dims == 4) {
      m_inputH = (int)m_inputAttrs[i].dims[1];
      m_inputW = (int)m_inputAttrs[i].dims[2];
    }
  }

  // 4. 循环查询所有输出张量的属性
  for (uint32_t i = 0; i < m_ioNum.n_output; i++) {
    m_outputAttrs[i].index = i;
    ret = rknn_query(m_ctx, RKNN_QUERY_OUTPUT_ATTR, &m_outputAttrs[i],
                     sizeof(rknn_tensor_attr));
    if (ret != RKNN_SUCC) {
      releaseCore();
      return -1;
    }
  }

  // 判断模型输出是否被量化为了 INT8（用于后续决定是否需要反量化计算）
  m_isQuant = (m_outputAttrs[0].type == RKNN_TENSOR_INT8);

  // 5. 使用“零拷贝 (Zero-Copy)” API 思想：直接为输入输出在 NPU 上申请物理内存
  m_inputMems = new rknn_tensor_mem *[m_ioNum.n_input]();
  m_outputMems = new rknn_tensor_mem *[m_ioNum.n_output]();

  // 为每个输入张量申请 NPU 内存，并将其绑定到模型上下文中
  for (uint32_t i = 0; i < m_ioNum.n_input; i++) {
    m_inputMems[i] = rknn_create_mem(m_ctx, m_inputAttrs[i].size);
    if (!m_inputMems[i]) {
      releaseCore();
      return -1;
    }
    // rknn_set_io_mem 是实现零拷贝的关键，直接把申请的内存和模型的输入端口绑定
    ret = rknn_set_io_mem(m_ctx, m_inputMems[i], &m_inputAttrs[i]);
    if (ret != RKNN_SUCC) {
      releaseCore();
      return -1;
    }
  }

  // 为每个输出张量申请 NPU 内存，并将其绑定到模型上下文中
  for (uint32_t i = 0; i < m_ioNum.n_output; i++) {
    m_outputMems[i] = rknn_create_mem(m_ctx, m_outputAttrs[i].size);
    if (!m_outputMems[i]) {
      releaseCore();
      return -1;
    }
    // 绑定输出端口，NPU 计算完的结果会直接写入这块物理内存中，CPU 可以直接读取，无需额外拷贝
    ret = rknn_set_io_mem(m_ctx, m_outputMems[i], &m_outputAttrs[i]);
    if (ret != RKNN_SUCC) {
      releaseCore();
      return -1;
    }
  }

  m_loaded = true;
  return 0;
}

void YOLOModel::releaseCore() {
  if (m_ctx) {
    for (uint32_t i = 0; m_inputMems && i < m_ioNum.n_input; i++)
      if (m_inputMems[i])
        rknn_destroy_mem(m_ctx, m_inputMems[i]);
    for (uint32_t i = 0; m_outputMems && i < m_ioNum.n_output; i++)
      if (m_outputMems[i])
        rknn_destroy_mem(m_ctx, m_outputMems[i]);
    rknn_destroy(m_ctx);
    m_ctx = 0;
  }
  delete[] m_inputMems;
  m_inputMems = nullptr;
  delete[] m_outputMems;
  m_outputMems = nullptr;
  delete[] m_inputAttrs;
  m_inputAttrs = nullptr;
  delete[] m_outputAttrs;
  m_outputAttrs = nullptr;
}

// ============================================================================
// 输入张量填充 — 将预处理后的图像写入 RKNN 输入内存
// ============================================================================

static void fillInputTensor(rknn_tensor_mem *mem, const rknn_tensor_attr &attr,
                            const cv::Mat &preprocessed) {
  void *dst = mem->virt_addr;
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
    if (n * sizeof(uint16_t) > alloc)
      return;
    const float *src = (const float *)f32.data;
    uint16_t *out = (uint16_t *)dst;
    for (size_t k = 0; k < n; k++) {
      uint32_t f32b;
      memcpy(&f32b, &src[k], 4);
      uint16_t sign = (uint16_t)((f32b >> 16) & 0x8000u);
      int exp = (int)((f32b >> 23) & 0xFFu) - 127 + 15;
      uint16_t mant = (uint16_t)((f32b >> 13) & 0x3FFu);
      if (exp <= 0)
        out[k] = sign;
      else if (exp >= 31)
        out[k] = sign | 0x7C00u;
      else
        out[k] = sign | (uint16_t)(exp << 10) | mant;
    }
  } else {
    if (preprocessed.total() * preprocessed.elemSize() <= alloc)
      memcpy(dst, preprocessed.data,
             preprocessed.total() * preprocessed.elemSize());
  }
}

std::vector<SegmentationResult>
YOLOModel::inferenceSegmentation(const cv::Mat &bgr, Timing *timing) {
  auto t0 = std::chrono::high_resolution_clock::now();
  if (!m_loaded)
    return {};

  // 阶段 1: 预处理 (Preprocess) -> LetterBox 缩放与颜色转换
  LetterBox lb;
  cv::Mat preprocessed =
      Preprocess::letterboxBGRtoRGB(bgr, m_inputW, m_inputH, lb);

  // 阶段 2: 填充输入张量，运行 NPU，收集输出
  fillInputTensor(m_inputMems[0], m_inputAttrs[0], preprocessed);

  auto t1 = std::chrono::high_resolution_clock::now();

  if (rknn_run(m_ctx, nullptr) != RKNN_SUCC) {
    fprintf(stderr, "[YOLO] rknn_run failed\n");
    return {};
  }

  std::vector<cv::Mat> outputs;
  outputs.reserve(m_ioNum.n_output);
  // 把 NPU 每个输出张量经 convertOutputToFloat() 转成 cv::Mat：
  // outputs0 = 检测主输出，numAnchors × infoDim 的
  // float 矩阵（yolo_model.cpp:173 写死 infoDim = 4 + 36 + 32）。

  // outputs1 = proto 张量（分割原型，[1, 32, 160, 160]），只有分割模型才有。
  for (uint32_t i = 0; i < m_ioNum.n_output; i++) {
    cv::Mat mat =
        convertOutputToFloat(m_outputMems[i]->virt_addr, m_outputAttrs[i]);
    if (mat.empty())
      return {};
    outputs.push_back(std::move(mat));
  }

  auto t2 = std::chrono::high_resolution_clock::now();

  // 阶段 3: 后处理 (Postprocess) -> 完整的检测与实例分割处理
  const int numClasses = (int)m_classNames.size();
  const int infoDim = 4 + numClasses + 32;

  int numAnchors = 0;
  cv::Mat detMat;
  const cv::Mat &raw0 = outputs[0];
  if (raw0.dims == 3 && raw0.size[0] == 1) {
    int d1 = raw0.size[1], d2 = raw0.size[2];
    if (d1 == infoDim) {
      numAnchors = d2;
      cv::transpose(cv::Mat(d1, d2, CV_32F, (void *)raw0.ptr<float>()), detMat);
    } else if (d2 == infoDim) {
      numAnchors = d1;
      detMat = cv::Mat(d1, d2, CV_32F, (void *)raw0.ptr<float>()).clone();
    }
  } else if (raw0.dims == 2 && raw0.cols == infoDim) {
    numAnchors = raw0.rows;
    detMat = raw0;
  }
  if (detMat.empty())
    return {};

  std::vector<ObjectDetectResult> candidates;
  Postprocess::decode((float *)detMat.data, numAnchors, infoDim, numClasses,
                      m_confThreshold, candidates);

  ObjectDetectResultList odResults;
  odResults.clear();
  Postprocess::nms(candidates, m_nmsThreshold, odResults, m_inputW, m_inputH);
  Postprocess::keepTop1PerClass(odResults);

  // mask decode
  cv::Mat protoTensor;
  if (outputs.size() > 1)
    protoTensor = outputs[1];

  auto results = Postprocess::extractMasks(odResults, protoTensor, bgr.size(),
                                           lb, m_inputW, m_inputH, m_classNames);

  auto t3 = std::chrono::high_resolution_clock::now();

  if (timing) {
    timing->preprocessMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    timing->inferenceMs =
        std::chrono::duration<double, std::milli>(t2 - t1).count();
    timing->postprocessMs =
        std::chrono::duration<double, std::milli>(t3 - t2).count();
    timing->totalMs =
        std::chrono::duration<double, std::milli>(t3 - t0).count();
  }

  return results;
}

// ============================================================================
// 数据类型转换 — 对标官方，将 RKNN 底层的 INT8/FLOAT16/FLOAT32 统一转为 OpenCV
// 的 CV_32F
// ============================================================================

cv::Mat YOLOModel::convertOutputToFloat(void *data,
                                        const rknn_tensor_attr &attr) {
  std::vector<int> dims(attr.dims, attr.dims + attr.n_dims);
  const size_t n = attr.n_elems;

  if (attr.type == RKNN_TENSOR_FLOAT32)
    return cv::Mat(attr.n_dims, dims.data(), CV_32F, data).clone();

  if (attr.type == RKNN_TENSOR_INT8) {
    cv::Mat out(attr.n_dims, dims.data(), CV_32F);
    const int8_t *src = (const int8_t *)data;
    float *dst = (float *)out.data;
    for (size_t j = 0; j < n; j++)
      dst[j] = (float)(src[j] - attr.zp) * attr.scale;
    return out;
  }

  if (attr.type == RKNN_TENSOR_FLOAT16) {
    cv::Mat out(attr.n_dims, dims.data(), CV_32F);
    const uint16_t *src = (const uint16_t *)data;
    float *dst = (float *)out.data;
    for (size_t j = 0; j < n; j++) {
      uint16_t h = src[j];
      uint32_t sign = (uint32_t)(h & 0x8000u) << 16;
      int32_t exp = (h >> 10) & 0x1F;
      uint32_t mant = (uint32_t)(h & 0x3FFu) << 13;
      uint32_t f32b;
      if (exp == 0) {
        if (mant == 0)
          f32b = sign;
        else {
          exp = 1;
          while ((mant & 0x00800000u) == 0) {
            mant <<= 1;
            exp--;
          }
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
  return {"304_stainless_steel",
          "acrylic",
          "african_padauk",
          "ash_wood",
          "bamboo",
          "black_walnut",
          "camphor_wood",
          "cedar",
          "cherry_wood",
          "chinaberry_tree",
          "chinese_toon_wood",
          "cliff_cypress",
          "cobblestone",
          "corrugated_paper",
          "ebony",
          "jade",
          "kraft_paper",
          "leaf",
          "linden_plywood",
          "maple_wood",
          "MDF_density_board",
          "patterned_leather",
          "paulownia_wood",
          "pine_unpainted_board",
          "poplar_plywood",
          "red_oak",
          "red_sandalwood",
          "rubberwood",
          "sapele",
          "silicone_sheet",
          "solid_color_leather",
          "titanium_metal",
          "varnished_pine",
          "velvet",
          "white_marble",
          "white_oak"};
}
