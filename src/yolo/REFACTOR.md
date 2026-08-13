# YOLO 推理模块重构文档

> 对标 [RKNN Model Zoo](https://github.com/airockchip/rknn_model_zoo) 官方架构，将 YOLO 推理代码从单体大函数重构为标准三段式流水线。

---

## 目录

1. [动机](#动机)
2. [文件清单](#文件清单)
3. [架构对比](#架构对比)
4. [核心概念](#核心概念)
5. [各模块说明](#各模块说明)
6. [数据流图](#数据流图)
7. [性能分析](#性能分析)
8. [安全检查清单](#安全检查清单)
9. [参考标准](#参考标准)

---

## 动机

原 `YOLOSegmentor::PerformInference()` 将预处理、推理、后处理（解码/NMS/掩码/轮廓）全部塞在一个 550 行的函数中。这导致：

- 修改任一环节需要通读整个函数
- 后处理逻辑无法被其他模型复用
- 无法对预处理/推理/后处理分别进行性能分析
- 和官方 RKNN Model Zoo 的参考代码结构完全不兼容，不利于后续基于官方示例开发新模型

本次重构遵循官方 Model Zoo 的三段式模式，将代码拆分为**积木式模块**，每个模块独立可测、可换、可复用。

---

## 文件清单

### 新增文件

| 文件 | 作用 | 对标官方 |
|------|------|----------|
| `yolo/yolo_common.h` | 共享类型定义 (`LetterBox`, `ObjectDetectResult`, `SegmentationResult` 等) | `utils/common.h` |
| `yolo/yolo_preprocess.h` | `namespace Preprocess`：letterbox、归一化 | `utils/image_utils.h` |
| `yolo/yolo_preprocess.cpp` | 实现 | `utils/image_utils.c` |
| `yolo/yolo_postprocess.h` | `namespace Postprocess`：decode、NMS、掩码提取、轮廓提取 | `examples/yolov5/cpp/postprocess.h` |
| `yolo/yolo_postprocess.cpp` | 实现 | `examples/yolov5/cpp/postprocess.cc` |
| `yolo/yolo_model.h` | `class YOLOModel`：模型加载/推理/释放 C++ API | `examples/yolov5/cpp/yolov5.h` |
| `yolo/yolo_model.cpp` | 实现（三段式流水线 + 计时） | - |

### 修改文件

| 文件 | 变化 |
|------|------|
| `yolodetect/yolo_wrapper.h` | 接口不变 |
| `yolodetect/yolo_wrapper.cpp` | 从 358 行精简到 120 行，移除调试噪音，所有推理逻辑委托给 `YOLOModel` |

### 未修改文件（安全红线）

| 文件 | 原因 |
|------|------|
| `yolo/embedded_model.h` | 硬编码模型，安全要求不可动 |
| `yolo/embedded_model.cpp` | 硬编码模型，安全要求不可动 |
| `yolodetect/thickness.h/cpp` | 业务逻辑，接口未变 |
| `handlers/DetectCommandHandler.h/cpp` | 业务逻辑，接口未变 |
| `config.h` | 全局配置 |

---

## 架构对比

### 旧架构（V1）

```
yolo_detect_nv12()  (yolo_wrapper.cpp:170行)
  └── PerformInference()  (YOLOSegmentor.cpp:550行)
        ├── PreprocessFrame()              letterbox + BGR→RGB
        ├── 手动 fp32→fp16 转换           散落在函数体中间
        ├── rknn_run()
        ├── ConvertOutputToFloat()         INT8/FP16→FP32
        └── PostprocessResults()           200行: decode + NMS + mask + contour
              ├── 解码检测框
              ├── NMS + 每类 Top-1
              ├── sigmoid(coefficients × protos)
              ├── 去除 letterbox 填充
              ├── resize 到原图
              ├── 裁切到 bbox
              ├── 阈值二值化
              └── findContours + approxPolyDP
```

问题：**所有逻辑耦合在一个函数内，改一行就要通读整段。**

### 新架构（V2）

```
yolo_detect_nv12()  (yolo_wrapper.cpp:65行)
  └── model.inferenceSegmentation()  (yolo_model.cpp)
        ├── Preprocess::letterboxBGRtoRGB()    独立 namespace
        ├── fillInputTensor() + rknn_run()      纯推理
        └── Postprocess::decode()               独立 namespace
            └── Postprocess::nms()              独立 namespace
            └── Postprocess::extractMasks()     独立 namespace
                  └── Postprocess::extractContour()  独立 namespace
```

优势：**每个环节独立封装，可以单独测试、单独换方案、单独加日志。**

---

## 核心概念

### 三段式流水线

```
输入图像 (BGR)
       │
       ▼
┌─────────────────┐
│  Stage 1: Preprocess          │  Preprocess::letterboxBGRtoRGB()
│  等比缩放 + 填充 + BGR→RGB   │  输出: 640×640 RGB + LetterBox 信息
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Stage 2: Inference           │  rknn_run()
│  NPU 推理                     │  输出: 原始输出张量 (INT8/FP16)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Stage 3: Postprocess         │  Postprocess::decode() → nms() →
│  解码 → NMS → 掩码 → 轮廓    │  extractMasks() → extractContour()
└────────┬────────┘
         │
         ▼
   检测结果列表
   (框 + 置信度 + 类别 + 掩码 + 轮廓)
```

### Builder / Engine / Runtime 类比

| TensorRT 概念 | 本项目对应 | 说明 |
|--------------|-----------|------|
| Builder | `rknn.build()` (在 Python 训练侧) | 将 ONNX 编译为 NPU 专属引擎 |
| Engine | `embedded_model.cpp` 中的二进制 | 硬编码在程序中，NPU 可执行格式 |
| Runtime | `rknn_init()` + `rknn_run()` | 加载引擎 + 执行推理 |

---

## 各模块说明

### 1. `yolo_common.h` — 共享类型

```cpp
struct LetterBox {
    float scale;      // 缩放比例
    int x_pad, y_pad; // 填充偏移（用于坐标还原）
};

struct ObjectDetectResult {
    cv::Rect box;           // 检测框
    float confidence;       // 置信度
    int classId;            // 类别 ID
};

struct ObjectDetectResultList {
    static constexpr int kMaxCount = 128;  // 对标官方 OBJ_NUMB_MAX_SIZE
    int count = 0;
    ObjectDetectResult results[kMaxCount];
};
```

### 2. `Preprocess` namespace

| 函数 | 说明 |
|------|------|
| `letterboxBGRtoRGB(bgr, targetW, targetH, lb)` | 等比缩放 + 边界填充 + BGR→RGB，返回 640×640 张量 + LetterBox 信息 |
| `normalizeFloat32(rgb, scale)` | 转为 float32 + 归一化（/255.0） |

### 3. `Postprocess` namespace

| 函数 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `decode(data, numAnchors, infoDim, numClasses, confThreshold, candidates)` | 原始输出张量 (float*) | `vector<ObjectDetectResult>` | 遍历 8400 个锚点，提取满足置信度阈值的框 |
| `iou(a, b)` | 两个 `cv::Rect` | float | 计算交并比 |
| `nms(candidates, threshold, out, modelW, modelH)` | 候选框列表 | `ObjectDetectResultList` | 按置信度排序 + NMS 去重 |
| `extractMasks(dets, protoTensor, origSize, lb, classNames)` | 检测框列表 + proto 张量 | `vector<SegmentationResult>` | 掩码系数 × proto 矩阵 → sigmoid → 后处理 → 二值掩码 |
| `extractContour(binMask, epsFactor)` | 二值掩码 | `vector<cv::Point>` | findContours + approxPolyDP |

### 4. `YOLOModel` class

```cpp
class YOLOModel {
public:
    struct Timing {
        double preprocessMs;   // Stage 1 耗时
        double inferenceMs;    // Stage 2 耗时
        double postprocessMs;  // Stage 3 耗时
        double totalMs;        // 总耗时
    };

    int    initFromMemory(modelData, modelSize);   // 硬编码模型加载 ✅ 不动
    int    initFromFile(modelPath);                // 文件加载（调试用）
    int    release();                              // 释放 NPU 资源

    // 快速检测（仅返回框，无掩码）— 对标官方 inference_yolov5_model()
    ObjectDetectResultList inference(const cv::Mat& bgr);

    // 完整分割（返回框 + 掩码 + 轮廓 + 耗时）
    std::vector<SegmentationResult> inferenceSegmentation(
        const cv::Mat& bgr, Timing* timing = nullptr);
};
```

**初始化调用（硬编码模型，不动）** ：

```cpp
#include "yolo/embedded_model.h"    // g_embedded_model_data, g_embedded_model_size

YOLOModel model;
model.initFromMemory(g_embedded_model_data, g_embedded_model_size);
```

---

## 数据流图

```
调用方 (DetectCommandHandler)
    │
    │ NV12 数据
    ▼
yolo_detect_nv12()                    yolo_wrapper.cpp (120行)
    │
    │ cv::cvtColor(NV12 → BGR)
    ▼
model.inferenceSegmentation(bgr)      yolo_model.cpp
    │
    ├─ Preprocess::letterboxBGRtoRGB  yolo_preprocess.cpp ──→ 640×640 RGB + LetterBox
    │
    ├─ fillInputTensor + rknn_run()   硬编码模型 in embedded_model.cpp
    │
    └─ Postprocess::decode            yolo_postprocess.cpp ──→ vector<ObjectDetectResult>
         └─ Postprocess::nms          ──→ ObjectDetectResultList (最多128个)
         └─ Postprocess::extractMasks ──→ vector<SegmentationResult> (含mask/contour)
              └─ Postprocess::extractContour
    │
    ▼
结果打包 → YOLOFrameResult (返回给调用方)
```

---

## 性能分析

每次推理后自动输出各阶段耗时：

```
[YOLO] frame=12345 det=2 pre=3.2ms inf=45.1ms post=8.7ms total=57.0ms
```

| 字段 | 含义 |
|------|------|
| `pre` | 预处理（letterbox + BGR→RGB）耗时 |
| `inf` | NPU 推理（`rknn_run`）耗时 |
| `post` | 后处理（decode + NMS + mask + contour）耗时 |
| `total` | 总耗时 |

日常监控只需关注 `inf` 字段；如果 `post` 突然增大，说明检测目标数暴增或轮廓提取开销异常。

---

## 安全检查清单

| 检查项 | 状态 |
|--------|------|
| 硬编码模型是否未修改？ | ✅ `embedded_model.h/cpp` 未动 |
| 外部接口是否兼容？ | ✅ `yolo_wrapper.h` API 不变 |
| `thickness` / `DetectCommandHandler` 是否受影响？ | ✅ 调用接口不变 |
| 新代码是否引入了外部文件依赖？ | ✅ 仅依赖 OpenCV 和 RKNN SDK（与原来相同） |
| 是否存在内存泄漏？ | ✅ `release()` 释放所有 NPU 资源 |

---

## 参考标准

本次重构对标以下官方文件：

| 官方文件 | 路径 |
|---------|------|
| 共享类型 | `rknn_model_zoo/utils/common.h` |
| 图像工具 | `rknn_model_zoo/utils/image_utils.h` |
| 后处理类型 | `rknn_model_zoo/examples/yolov5/cpp/postprocess.h` |
| 后处理实现 | `rknn_model_zoo/examples/yolov5/cpp/postprocess.cc` |
| 模型 API | `rknn_model_zoo/examples/yolov5/cpp/yolov5.h` |
| 推理入口 | `rknn_model_zoo/examples/yolov5/cpp/main.cc` |
| Python 对比 | `rknn_model_zoo/examples/yolov5/python/yolov5.py` |
| 运行时封装 | `rknn_model_zoo/py_utils/rknn_executor.py` |

---

*文档生成时间: 2026-06-25*
