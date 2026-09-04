#include "yolo_postprocess.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace Postprocess {
// 解码 YOLO 输出张量，提取候选框。
// 本质是遍历所有锚点，把类别分数超过 confThreshold 的转成 cv::Rect。
// 分数没有再做 sigmoid —— 说明这个 RKNN 模型导出时已在图中包含 sigmoid 操作。
void decode(const float* data, int numAnchors, int infoDim,
            int numClasses, float confThreshold,
            std::vector<ObjectDetectResult>& candidates) {
    candidates.clear();
    candidates.reserve(256);

    for (int i = 0; i < numAnchors; i++) {
        float bestScore = 0.0f;
        int   bestClass = -1;

        // 寻找第 i 个锚点里分数最高的类别
        for (int c = 0; c < numClasses; c++) {
            float s = data[i * infoDim + 4 + c];    // 行首跳过 4 个坐标 (cx, cy, bw, bh)
            if (s > bestScore) { bestScore = s; bestClass = c; }
        }
        // 低于置信度阈值直接丢弃
        if (bestScore < confThreshold) continue;

        float cx = data[i * infoDim + 0];
        float cy = data[i * infoDim + 1];
        float bw = data[i * infoDim + 2];
        float bh = data[i * infoDim + 3];

        // 中心点+宽高 → 转换为左上角和右下角坐标
        int x1 = (int)(cx - bw * 0.5f);
        int y1 = (int)(cy - bh * 0.5f);
        int x2 = (int)(cx + bw * 0.5f);
        int y2 = (int)(cy + bh * 0.5f);
        if (x2 <= x1 || y2 <= y1) continue;   // 过滤退化框（宽高为负数或0）

        ObjectDetectResult det;
        det.box = cv::Rect(x1, y1, x2 - x1, y2 - y1);
        det.confidence = bestScore;
        det.classId = bestClass;
        // 掩码系数：每行末尾的 32 个分割系数（供 extractMasks 解码掩膜）
        for (int k = 0; k < kMaskCoeffCount; k++)
            det.maskCoefs[k] = data[i * infoDim + 4 + numClasses + k];

        candidates.push_back(det);
    }
}

// 计算交并比 (IoU)
float iou(const cv::Rect& a, const cv::Rect& b) {
    // 交叠宽 = max(0, min(x右a, x右b) - max(x左a, x左b))
    int ix = std::max(0, std::min(a.x + a.width, b.x + b.width)  - std::max(a.x, b.x));
    int iy = std::max(0, std::min(a.y + a.height, b.y + b.height) - std::max(a.y, b.y));
    float inter = (float)(ix * iy);
    // 并集面积 = 面积a + 面积b - 交叠面积 + 1e-6f (防止除零)
    float denom = (float)(a.area() + b.area()) - inter + 1e-6f;
    return inter / denom;
}

// 非极大值抑制 (NMS)，贪心去重：分数高的留下，跟它 IoU 超过阈值的一律抑制。
// 边界裁剪用的是模型尺寸（modelWidth/modelHeight），因为此时框还在模型输入尺寸的空间（如 640x640）。
// 注意与官方代码的差异：
// 1. 这里是全局 NMS（所有类别混在一起抑制）；官方常用按类别分开做 NMS。本项目中主要识别的是不同材质，一张图基本只有一个主体，全局 NMS 问题不大，但严格说语义不同。
// 2. 硬上限被限制为 kMaxCount = 128。
void nms(std::vector<ObjectDetectResult>& candidates,
         float nmsThreshold,
         ObjectDetectResultList& out,
         int modelWidth, int modelHeight) {
    out.clear();

    if (candidates.empty()) return;
    // 按置信度降序排序
    std::sort(candidates.begin(), candidates.end(),
        [](const ObjectDetectResult& a, const ObjectDetectResult& b) {
            return a.confidence > b.confidence;
        });

    std::vector<bool> suppressed(candidates.size(), false);

    for (size_t i = 0; i < candidates.size() && out.count < out.kMaxCount; i++) {
        if (suppressed[i]) continue;

        const auto& det = candidates[i];
        out.results[out.count] = det;       // 保留该检测框
        // 将检测框限制在模型输入尺寸范围内
        out.results[out.count].box.x      = std::max(0, std::min(det.box.x, modelWidth));
        out.results[out.count].box.y      = std::max(0, std::min(det.box.y, modelHeight));
        out.results[out.count].box.width  = std::min(det.box.width,  modelWidth  - det.box.x);
        out.results[out.count].box.height = std::min(det.box.height, modelHeight - det.box.y);
        out.count++;

        // 将与其重叠度高的框标记为抑制
        for (size_t j = i + 1; j < candidates.size(); j++) {
            if (suppressed[j]) continue;
            if (iou(det, candidates[j]) > nmsThreshold)
                suppressed[j] = true;       // 与它高度重叠的框全丢
        }
    }
}

// 每个类别只保留分数最高的一个检测。
// 适用场景：材质识别中一张图每个材质通常只认一个主体，避免同类别多框同时上报。
void keepTop1PerClass(ObjectDetectResultList& dets) {
    if (dets.count <= 1)
        return;
    std::unordered_map<int, float> bestScore;  // classId -> 最高分
    for (int i = 0; i < dets.count; i++)
        if (dets.results[i].confidence > bestScore[dets.results[i].classId])
            bestScore[dets.results[i].classId] = dets.results[i].confidence;

    int w = 0;
    for (int i = 0; i < dets.count; i++) {
        const auto& d = dets.results[i];
        if (d.confidence == bestScore[d.classId])  // 保留该类的最高分框
            dets.results[w++] = d;
    }
    dets.count = w;
}

// ============================================================================
// 掩膜解码 — 对标官方 rknn_model_zoo/examples/yolov8_seg/cpp/rknpu2/postprocess.cc
// 流程: 掩码系数 × proto → 160×160 分数图 → resize 到模型尺寸 → 按框裁剪二值化
//       → seg_reverse(去 letterbox 填充, resize 回原图)
// ============================================================================

static void findLargestContour(const cv::Mat& binMask, float epsFactor,
                               std::vector<cv::Point>& full,
                               std::vector<cv::Point>& simplified);

static void matmulCoeffsProto(const float* coeffs, const cv::Mat& protos2D,
                              int protoH, int protoW, std::vector<float>& out) {
    const int protoArea = protoH * protoW;
    const float* protoPtr = protos2D.ptr<float>();
    for (int px = 0; px < protoArea; px++) {
        float acc = 0.0f;
        for (int k = 0; k < kMaskCoeffCount; k++)
            acc += coeffs[k] * protoPtr[k * protoArea + px];
        out[px] = acc;
    }
}

// 二值化 + 只保留检测框内像素（对标官方 crop_mask，阈值 >0）
static void cropMaskToBox(const cv::Mat& maskModel, const cv::Rect& box,
                          int modelW, int modelH, cv::Mat& out) {
    out = cv::Mat::zeros(modelH, modelW, CV_8UC1);
    cv::Rect clamped = box & cv::Rect(0, 0, modelW, modelH);
    if (clamped.width <= 0 || clamped.height <= 0)
        return;
    cv::Mat boxMask = maskModel(clamped);
    for (int y = 0; y < clamped.height; y++) {
        const float* row = boxMask.ptr<float>(y);
        uchar* dst = out.ptr<uchar>(y + clamped.y) + clamped.x;
        for (int x = 0; x < clamped.width; x++)
            if (row[x] > 0.0f)
                dst[x] = 255;
    }
}

// 去 letterbox 填充并 resize 回原图尺寸（对标官方 seg_reverse）
static void maskToOriginal(const cv::Mat& modelMask, const LetterBox& lb,
                           const cv::Size& originalSize, cv::Mat& out) {
    const int modelH = modelMask.rows;
    const int modelW = modelMask.cols;
    if (lb.x_pad == 0 && lb.y_pad == 0 &&
        originalSize.width == modelW && originalSize.height == modelH) {
        out = modelMask;
        return;
    }
    const int croppedW = modelW - lb.x_pad * 2;
    const int croppedH = modelH - lb.y_pad * 2;
    if (croppedW <= 0 || croppedH <= 0) {
        out = modelMask;
        return;
    }
    cv::Mat cropped = modelMask(cv::Rect(lb.x_pad, lb.y_pad, croppedW, croppedH)).clone();
    cv::resize(cropped, out, originalSize, 0, 0, cv::INTER_LINEAR);
}

// 把模型输入空间 (640×640) 的检测框还原到原图坐标。
// 逆 letterbox：(框坐标 - 填充偏移) / 缩放比例，再裁剪到原图范围。
static cv::Rect mapBoxToOriginal(const cv::Rect& box, const LetterBox& lb,
                                 const cv::Size& originalSize) {
    if (lb.scale <= 0.0f)
        return box;
    const float invScale = 1.0f / lb.scale;
    int x1 = (int)std::round((box.x          - lb.x_pad) * invScale);
    int y1 = (int)std::round((box.y          - lb.y_pad) * invScale);
    int x2 = (int)std::round((box.x + box.width  - lb.x_pad) * invScale);
    int y2 = (int)std::round((box.y + box.height - lb.y_pad) * invScale);
    x1 = std::max(0, std::min(x1, originalSize.width));
    y1 = std::max(0, std::min(y1, originalSize.height));
    x2 = std::max(0, std::min(x2, originalSize.width));
    y2 = std::max(0, std::min(y2, originalSize.height));
    if (x2 <= x1 || y2 <= y1)
        return box;
    return cv::Rect(x1, y1, x2 - x1, y2 - y1);
}

// 提取掩膜的完整实现（原桩函数，现已对标官方补全）
std::vector<SegmentationResult> extractMasks(
    const ObjectDetectResultList& dets,
    const cv::Mat& protoTensor,
    const cv::Size& originalSize,
    const LetterBox& lb,
    int modelWidth, int modelHeight,
    const std::vector<std::string>& classNames) {

    std::vector<SegmentationResult> results;

    const int protoH = 160;
    const int protoW = 160;
    const bool hasProto = (protoTensor.dims == 4 &&
                           protoTensor.size[1] == kMaskCoeffCount &&
                           protoTensor.size[2] == protoH &&
                           protoTensor.size[3] == protoW &&
                           protoTensor.type() == CV_32F);

    cv::Mat protos2D;
    if (hasProto)
        protos2D = protoTensor.reshape(1, kMaskCoeffCount);

    for (int i = 0; i < dets.count; i++) {
        const auto& det = dets.results[i];

        SegmentationResult r;
        r.classId    = det.classId;
        r.className  = (det.classId >= 0 && det.classId < (int)classNames.size())
                      ? classNames[det.classId] : "未知类别";
        r.confidence = det.confidence;
        r.boundingBox = mapBoxToOriginal(det.box, lb, originalSize);

        if (hasProto) {
            // 1) 掩码系数 × proto → 160×160 分数图
            std::vector<float> matmulOut((size_t)protoH * protoW, 0.0f);
            matmulCoeffsProto(det.maskCoefs, protos2D, protoH, protoW, matmulOut);

            // 2) resize 到模型输入尺寸 (640×640)
            cv::Mat mask160(protoH, protoW, CV_32F, matmulOut.data());
            cv::Mat maskModel;
            cv::resize(mask160, maskModel, cv::Size(modelWidth, modelHeight),
                       0, 0, cv::INTER_LINEAR);

            // 3) 按框裁剪二值化
            cv::Mat allMask;
            cropMaskToBox(maskModel, det.box, modelWidth, modelHeight, allMask);

            // 4) 去 letterbox 填充 + resize 回原图
            cv::Mat realMask;
            maskToOriginal(allMask, lb, originalSize, realMask);
            r.mask = realMask;

            // 5) 轮廓提取
            std::vector<cv::Point> full, simplified;
            findLargestContour(realMask, 0.003f, full, simplified);
            r.contour_full = std::move(full);
            r.contour = std::move(simplified);
        } else {
            r.mask = cv::Mat::zeros(originalSize, CV_8UC1);
        }

        results.push_back(std::move(r));
    }

    return results;
}

// 提取二值掩膜的最大轮廓：full 为原始轮廓，simplified 为多边形逼近后的轮廓
static void findLargestContour(const cv::Mat& binMask, float epsFactor,
                               std::vector<cv::Point>& full,
                               std::vector<cv::Point>& simplified) {
    full.clear();
    simplified.clear();
    if (binMask.empty())
        return;

    std::vector<std::vector<cv::Point>> contours;
    // 只找外轮廓 (RETR_EXTERNAL) + TC89 快速多边形逼近 (CHAIN_APPROX_TC89_L1)
    cv::findContours(binMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);
    if (contours.empty())
        return;

    // 取面积最大的轮廓
    const auto& largest = *std::max_element(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });
    if (largest.size() < 3)
        return;

    full = largest;
    // 多边形拟合简化轮廓，epsFactor 默认 0.003
    cv::approxPolyDP(largest, simplified,
                     epsFactor * cv::arcLength(largest, true), true);
    if (simplified.size() < 3)
        simplified = largest;
}

std::vector<cv::Point> extractContour(const cv::Mat& binMask, float epsFactor) {
    std::vector<cv::Point> full, simplified;
    findLargestContour(binMask, epsFactor, full, simplified);
    return simplified;
}

// 按类别取稳定颜色（与旧版 YOLOSegmentor::GetClassColor 一致）
static cv::Scalar getClassColor(int classId) {
    static const cv::Scalar palette[] = {
        {0,0,255}, {0,255,0}, {255,0,0}, {255,255,0}, {255,0,255},
        {0,255,255}, {128,0,0}, {0,128,0}, {0,0,128}, {128,128,0},
        {128,0,128}, {0,128,128}, {64,0,0}, {0,64,0}, {0,0,64},
        {192,0,0}, {0,192,0}, {0,0,192}, {192,192,0}, {192,0,192}
    };
    static const int N = (int)(sizeof(palette) / sizeof(palette[0]));
    return palette[((classId % N) + N) % N];
}

// 把检测结果绘制到 BGR 图上（mask 半透明叠加 + 轮廓 + 框 + 类别/置信度标签）
void drawResults(cv::Mat& frame, const std::vector<SegmentationResult>& results) {
    for (const auto& r : results) {
        cv::Scalar color = getClassColor(r.classId);

        // 1) mask 半透明叠加（仅在掩码尺寸与图一致时绘制）
        if (!r.mask.empty() && r.mask.size() == frame.size()) {
            cv::Mat overlay = cv::Mat::zeros(frame.size(), frame.type());
            overlay.setTo(color, r.mask);
            cv::Mat blended;
            cv::addWeighted(frame, 0.45, overlay, 0.55, 0.0, blended);
            blended.copyTo(frame, r.mask);
        }

        // 2) 轮廓线
        if (r.contour.size() >= 3) {
            std::vector<std::vector<cv::Point>> poly{r.contour};
            cv::polylines(frame, poly, true, color, 2, cv::LINE_AA);
        }

        // 3) 检测框
        cv::rectangle(frame, r.boundingBox, color, 1, cv::LINE_AA);

        // 4) 类别 + 置信度标签
        std::string label = r.className + " " + std::to_string(r.confidence);
        cv::putText(frame, label,
                    cv::Point(r.boundingBox.x, std::max(15, r.boundingBox.y - 5)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }
}

} // namespace Postprocess
