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
7. [完整运行流程](#完整运行流程)
8. [性能分析](#性能分析)
9. [安全检查清单](#安全检查清单)
10. [参考标准](#参考标准)

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
| `handlers/DetectCommandHandler.h/cpp` | 直接持有 `YOLOModel*`，`detectNV12()` 调 `inferenceSegmentation()` |
| `yolodetect/yolo_wrapper.h/cpp` | **已删除**——推理入口改为 `DetectCommandHandler::detectNV12()` |

### 未修改文件（安全红线）

| 文件 | 原因 |
|------|------|
| `yolo/embedded_model.h` | 硬编码模型，安全要求不可动 |
| `yolo/embedded_model.cpp` | 硬编码模型，安全要求不可动 |
| `yolodetect/thickness.h/cpp` | 业务逻辑，接口未变（下游仅消费 `boundingBox`） |
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
DetectCommandHandler::detectNV12()  (DetectCommandHandler.cpp:208)
  └── model.inferenceSegmentation()  (yolo_model.cpp:159)
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
    float maskCoefs[32];    // 掩码系数（供分割掩码解码）
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

### 3. `Postprocess` namespace

| 函数 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `decode(data, numAnchors, infoDim, numClasses, confThreshold, candidates)` | 原始输出张量 (float*) | `vector<ObjectDetectResult>` | 遍历 8400 个锚点，提取满足置信度阈值的框 |
| `iou(a, b)` | 两个 `cv::Rect` | float | 计算交并比 |
| `nms(candidates, threshold, out, modelW, modelH)` | 候选框列表 | `ObjectDetectResultList` | 按置信度排序 + NMS 去重 |
| `keepTop1PerClass(dets)` | NMS 后的结果 | `ObjectDetectResultList` | 每个类别只保留分数最高的一个（材质识别每类一个主体） |
| `extractMasks(dets, protoTensor, origSize, lb, modelW, modelH, classNames)` | 检测框列表 + proto 张量 | `vector<SegmentationResult>` | 掩码系数 × proto 矩阵 → resize → 裁框二值化 → 还原原图（对标官方 yolov8_seg，阈值 >0） |
| `extractContour(binMask, epsFactor)` | 二值掩膜 | `vector<cv::Point>` | findContours + approxPolyDP（最大轮廓） |
| `drawResults(frame, results)` | BGR 图 + 结果 | 原地绘制 | mask 半透明叠加 + 轮廓 + 框 + 类别/置信度标签（人工验证推理） |

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
    int    release();                              // 释放 NPU 资源

    // 完整分割（返回框 + 掩码 + 耗时）— 唯一推理入口
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

### 完整链路：BEV 视频流截图 → YOLO 推理结果

```
[BEV 视频流] stream_workers.cpp:161
    │  ctx->app->bev_frame_provider.publish(bev_output_buf, BEV_OUTPUT_NV12_SIZE, ...)
    ▼
[FrameProvider]  frame_provider.cpp:12 publish()
    │  拷贝 NV12 到共享缓冲（只保留最新一帧）
    ▼
[DETECT 命令]  DetectCommandHandler.cpp
    │  :389  hasFrame() / :393  grab()        → 首帧（唯一抓帧，复用为回复图像）
    ▼
  Snapshot{ nv12, frame_id }  （NV12 数据，未解码）
    │
    ▼
detectNV12()  DetectCommandHandler.cpp:208
    │  :220-222  cv::Mat nv(NV12) ──cvtColor──▶ cv::Mat bgr   （NV12 → BGR）
    │  :224      yolo_model_->inferenceSegmentation(bgr)
    ▼
inferenceSegmentation(bgr)  yolo_model.cpp:159
    │
    ├─ Stage 1  Preprocess
    │   :167-168  Preprocess::letterboxBGRtoRGB(bgr, m_inputW, m_inputH, lb)
    │             └─ yolo_preprocess.cpp:8
    │                ① 等比缩放 bgr → nw×nh（lb.scale / resize_w / resize_h）
    │                ② 方形等比输入（BEV 1280×1280 → 640×640）短路：无填充，直接转色
    │                ③ cvtColor BGR → RGB
    │                → 640×640 RGB + LetterBox（lb.x_pad / y_pad 恒为 0）
    │                ※ 非方形输入才走 copyMakeBorder 补边（通用兜底）
    │
    ├─ Stage 2  Inference
    │   :171  fillInputTensor(m_inputMems[0], m_inputAttrs[0], preprocessed)
    │   :175  rknn_run(m_ctx)         （硬编码模型 in embedded_model.cpp）
    │   :188  convertOutputToFloat()  每个输出张量 INT8/FP16 → CV_32F
    │          outputs[0] = [1, 8400, 72]   (4+36 类+32 掩码系数)
    │          outputs[1] = [1, 32, 160, 160] (proto，仅分割模型)
    │
    └─ Stage 3  Postprocess
        :221  Postprocess::decode(...)           ──→ vector<ObjectDetectResult>
        :226  Postprocess::nms(...)              ──→ ObjectDetectResultList (≤128)
        :227  Postprocess::keepTop1PerClass(...) ──→ 每类只留最高分一个
        :233  Postprocess::extractMasks(odResults, protoTensor, bgr.size(),
                                        lb, m_inputW, m_inputH, classNames)
              └─ yolo_postprocess.cpp:162
                 ① 掩码系数 × proto → 160×160 分数图
                 ② resize → 640×640
                 ③ 按框裁剪二值化（阈值 >0，无 sigmoid）
                 ④ 用 lb 去 letterbox 填充 → resize 回原图尺寸
                 → SegmentationResult.mask
              └─ mapBoxToOriginal → boundingBox 还原到原图坐标
              └─ findLargestContour / extractContour → .contour / .contour_full
    │
    ▼
vector<SegmentationResult>  →  返回调用方
    │
    ├─ DetectCommandHandler.cpp:438  updateYoloJson() → buildObjectsJson()
    │     JSON: class_id / class_name / confidence / roi_xywh
    ├─ :492 sendDetectResponse(..., &first_results)
    │     Postprocess::drawResults() 把 mask/轮廓/框画到首帧定格图 → JPEG 返回
    │     （人工可直接查看模型推理是否正常）
    └─ thickness.cpp 消费 boundingBox（已还原到原图坐标）→ 机械坐标换算
```

### 关键点

| 关注点 | 说明 |
|--------|------|
| `yolo_preprocess.cpp` 与 BEV 流无直接关系 | 它只收一个 `cv::Mat`，来源由调用方决定 |
| 真正把 BEV 帧接进 YOLO 的模块 | `DetectCommandHandler::detectNV12()`（DetectCommandHandler.cpp:208） |
| NV12 → BGR 转换位置 | DetectCommandHandler.cpp:220-222（preprocess 之前完成） |
| LetterBox 参数如何传递 | `lb` 由 `letterboxBGRtoRGB` 写入（yolo_model.cpp:168），传给 `extractMasks`（:233）用于把掩码和检测框从 640×640 还原回原图坐标 |
| 掩码/轮廓下游消费者 | `drawResults`（画到返回的 JPEG 上，人工验证推理）+ `thickness`（仅消费 `boundingBox`） |
| 检测框坐标语义 | `decode`/`nms` 阶段在模型空间（640×640），`extractMasks` 内 `mapBoxToOriginal` 还原到原图（1280×1280） |

---

## 完整运行流程

### 一、启动阶段：模型加载

```
main.cpp 启动
  └─ 创建 YOLOModel 对象
       ├─ 构造函数 YOLOModel::YOLOModel()          yolo_model.cpp:5
       │    └─ m_classNames = loadClassNames()     36 类材质名
       └─ initFromMemory(g_embedded_model_data, size)  yolo_model.cpp:8
            └─ initCore()                          yolo_model.cpp:22
                 ├─ rknn_init()                    加载硬编码模型 → 上下文 m_ctx
                 ├─ rknn_query(IN_OUT_NUM)         查询输入/输出张量数量
                 ├─ rknn_query(INPUT/OUTPUT_ATTR)  逐个查询张量属性
                 │    └─ 读取模型输入尺寸 → m_inputW=m_inputH=640（运行时从模型查询）
                 ├─ rknn_create_mem() × N          为每个输入/输出分配 NPU 内存
                 ├─ rknn_set_io_mem() × N          绑定张量与内存
                 └─ m_loaded = true
```

### 二、运行阶段：DETECT 命令触发

```
客户端发 DETECT 命令
  └─ DetectCommandHandler::execute()              DetectCommandHandler.cpp:244
       ├─ 独占 Klipper → 归位 → 等待 BEV 帧
       ├─ bev_frame_provider.grab() 拿到首帧 NV12 (1280×1280)
       └─ detectNV12(nv12, w, h, first_results)    DetectCommandHandler.cpp:208
            ├─ cvtColor(NV12 → BGR)                  :220-222
            └─ yolo_model_->inferenceSegmentation(bgr)  ← 唯一推理入口
```

### 三、核心推理：inferenceSegmentation()（yolo_model.cpp:160）

#### Stage 1 — 预处理

```
Preprocess::letterboxBGRtoRGB(bgr, 640, 640, lb)   yolo_model.cpp:168
  └─ yolo_preprocess.cpp:8
       ① 计算等比缩放比 r = min(640/h, 640/w)
       ② 记录 lb{scale, resize_w/h, x_pad, y_pad}
       ③ cv::resize: 1280×1280 → 640×640
       ④ 方形等比短路：无填充（非方形才 copyMakeBorder）
       ⑤ cvtColor: BGR → RGB
       → 640×640 RGB + LetterBox
```

#### Stage 2 — 推理 + 输出转换

```
fillInputTensor(m_inputMems[0], attrs[0], rgb)     yolo_model.cpp:171
  └─ 按模型输入类型填内存：
       FLOAT32 → /255.0 后直接拷
       FLOAT16 → /255.0 + 手动 fp32→fp16
       UINT8   → 原样拷（模型已内置归一化）

rknn_run(m_ctx)                                     yolo_model.cpp:175  ← NPU 干活

for 每个输出张量: convertOutputToFloat()            yolo_model.cpp:188
  └─ INT8  → (q - zp) * scale  反量化
     FP16  → 手动转 FP32
     FP32  → clone
  → outputs[0] = [1, 8400, 72]   检测主输出
  → outputs[1] = [1, 32, 160, 160] proto 分割原型
```

#### Stage 3 — 后处理（yolo_model.cpp:197 起）

```
① 张量整形                        :201-218
   解析 outputs[0] 维度 → (8400 × 72) 矩阵 detMat
   （72 = 4 坐标 + 36 类分 + 32 掩码系数，可能需 transpose）

② decode()                        :221  → candidates
   └─ yolo_postprocess.cpp:8
       遍历 8400 个锚点，每行：
       - 找 36 类里分数最高的 → bestClass/bestScore
       - < confThreshold(0.05) 丢弃
       - cx,cy,bw,bh → cv::Rect（模型空间 640×640）
       - 拷贝末尾 32 个掩码系数 → maskCoefs[32]

③ nms()                          :226  → odResults (≤128)
   └─ yolo_postprocess.cpp:66
       - 按置信度降序排序
       - 贪心：留分数高的，IoU>0.7 的抑制
       - 框裁剪到 640×640 范围内

④ keepTop1PerClass()             :227
   └─ yolo_postprocess.cpp:106
       - 每个类别只保留分数最高的一个框
       - 材质识别语义：木头/皮革各报一个，不互相挤掉

⑤ extractMasks()                 :234  → results
   └─ yolo_postprocess.cpp:162  逐框：
       ① matmulCoeffsProto: 32 系数 × proto(32×160×160) → 160×160 分数图
       ② cv::resize → 640×640 分数图
       ③ cropMaskToBox: 按框裁剪、>0 二值化（无 sigmoid）
       ④ maskToOriginal: 去 letterbox 填充 + resize 回 1280×1280
       ⑤ findLargestContour: findContours 最大轮廓 + approxPolyDP 简化
       ⑥ mapBoxToOriginal: 检测框 (640) 还原到原图坐标 (1280)
       → SegmentationResult{classId, className, confidence,
                            boundingBox(原图), mask, contour, contour_full}

计时：preprocessMs / inferenceMs / postprocessMs / totalMs
```

### 四、结果返回与消费

```
vector<SegmentationResult> 返回给 DetectCommandHandler::detectNV12

DetectCommandHandler::execute():
  ├─ updateYoloJson() → buildObjectsJson()      :438
  │    JSON: class_id / class_name / confidence / roi_xywh
  ├─ thickness_service.measureFromYolo(results)  :451
  │    └─ thickness.cpp:304  选目标（面积最大）→
  │       roi/center ← boundingBox(原图坐标) → /pixel_ratio → 机械坐标 → 控制机械臂移动+测厚
  └─ sendDetectResponse(..., &first_results)      :492
       └─ encodeNV12ToJPEG()                     :169
            ├─ NV12 → BGR
            ├─ Postprocess::drawResults()        :200
            │    半透明 mask 叠加 + 轮廓 + 框 + "类名 置信度" 标签
            │    ← 人工直接看返回的 JPEG 就能验证推理是否正常
            └─ imencode → JPEG 二进制返回客户端
```

### 五、收尾

```
DetectCommandHandler 析构/程序退出
  └─ YOLOModel::~YOLOModel() → release() → releaseCore()  yolo_model.cpp:96
       ├─ rknn_destroy_mem() × N
       ├─ rknn_destroy(m_ctx)
       └─ delete[] 所有 attr/mem 数组
```

### 数据维度速查

| 环节 | 数据 |
|------|------|
| BEV 帧 | NV12 1280×1280 |
| 模型输入 | RGB 640×640 |
| outputs[0] | [1, 8400, 72] = 8400 锚点 × (4+36+32) |
| outputs[1] | [1, 32, 160, 160] proto |
| decode 后 | 候选框（模型空间） |
| NMS + top1 后 | ≤36 个框（每类 1 个） |
| extractMasks 后 | SegmentationResult（框/掩码/轮廓都在原图坐标） |
| 客户端 | JSON + 画好 mask 的 JPEG |

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
| `YOLOModel` API 是否稳定？ | ✅ `inferenceSegmentation(bgr)` 唯一推理入口 |
| `thickness` / `DetectCommandHandler` 是否受影响？ | ✅ `thickness` 仅消费 `boundingBox`；handler 改持 `YOLOModel*`，入口不变 |
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
