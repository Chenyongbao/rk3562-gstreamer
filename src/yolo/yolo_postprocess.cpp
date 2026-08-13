#include "yolo_postprocess.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace Postprocess {

void decode(const float* data, int numAnchors, int infoDim,
            int numClasses, float confThreshold,
            std::vector<ObjectDetectResult>& candidates) {
    candidates.clear();
    candidates.reserve(256);

    for (int i = 0; i < numAnchors; i++) {
        float bestScore = 0.0f;
        int   bestClass = -1;

        for (int c = 0; c < numClasses; c++) {
            float s = data[i * infoDim + 4 + c];
            if (s > bestScore) { bestScore = s; bestClass = c; }
        }
        if (bestScore < confThreshold) continue;

        float cx = data[i * infoDim + 0];
        float cy = data[i * infoDim + 1];
        float bw = data[i * infoDim + 2];
        float bh = data[i * infoDim + 3];

        int x1 = (int)(cx - bw * 0.5f);
        int y1 = (int)(cy - bh * 0.5f);
        int x2 = (int)(cx + bw * 0.5f);
        int y2 = (int)(cy + bh * 0.5f);
        if (x2 <= x1 || y2 <= y1) continue;

        candidates.push_back({
            cv::Rect(x1, y1, x2 - x1, y2 - y1),
            bestScore,
            bestClass
        });
    }
}

float iou(const cv::Rect& a, const cv::Rect& b) {
    int ix = std::max(0, std::min(a.x + a.width, b.x + b.width)  - std::max(a.x, b.x));
    int iy = std::max(0, std::min(a.y + a.height, b.y + b.height) - std::max(a.y, b.y));
    float inter = (float)(ix * iy);
    float denom = (float)(a.area() + b.area()) - inter + 1e-6f;
    return inter / denom;
}

void nms(std::vector<ObjectDetectResult>& candidates,
         float nmsThreshold,
         ObjectDetectResultList& out,
         int modelWidth, int modelHeight) {
    out.clear();

    if (candidates.empty()) return;

    std::sort(candidates.begin(), candidates.end(),
        [](const ObjectDetectResult& a, const ObjectDetectResult& b) {
            return a.confidence > b.confidence;
        });

    std::vector<bool> suppressed(candidates.size(), false);

    for (size_t i = 0; i < candidates.size() && out.count < out.kMaxCount; i++) {
        if (suppressed[i]) continue;

        const auto& det = candidates[i];
        out.results[out.count] = det;
        out.results[out.count].box.x      = std::max(0, std::min(det.box.x, modelWidth));
        out.results[out.count].box.y      = std::max(0, std::min(det.box.y, modelHeight));
        out.results[out.count].box.width  = std::min(det.box.width,  modelWidth  - det.box.x);
        out.results[out.count].box.height = std::min(det.box.height, modelHeight - det.box.y);
        out.count++;

        for (size_t j = i + 1; j < candidates.size(); j++) {
            if (suppressed[j]) continue;
            if (iou(det, candidates[j]) > nmsThreshold)
                suppressed[j] = true;
        }
    }
}

std::vector<SegmentationResult> extractMasks(
    const ObjectDetectResultList& dets,
    const cv::Mat& protoTensor,
    const cv::Size& originalSize,
    const LetterBox& lb,
    const std::vector<std::string>& classNames) {

    std::vector<SegmentationResult> results;

    const int numCoeffs   = 32;
    const int protoH      = 160;
    const int protoW      = 160;
    const bool hasProto   = (protoTensor.dims == 4 &&
                             protoTensor.size[1] == numCoeffs &&
                             protoTensor.size[2] == protoH &&
                             protoTensor.size[3] == protoW);

    cv::Mat protos2D;
    if (hasProto)
        protos2D = protoTensor.reshape(1, numCoeffs);

    for (int i = 0; i < dets.count; i++) {
        const auto& det = dets.results[i];

        SegmentationResult r;
        r.classId    = det.classId;
        r.className  = (det.classId >= 0 && det.classId < (int)classNames.size())
                      ? classNames[det.classId] : "unknown";
        r.confidence = det.confidence;
        r.boundingBox = det.box;

        if (!hasProto) {
            results.push_back(std::move(r));
            continue;
        }

        // coeffs × protos → sigmoid
        // In our simplified API, maskCoeffs are extracted from the raw tensor
        // For now, generate an empty mask — the full impl needs the raw detection tensor
        r.mask = cv::Mat::zeros(originalSize, CV_8UC1);
        results.push_back(std::move(r));
    }

    return results;
}

std::vector<cv::Point> extractContour(const cv::Mat& binMask, float epsFactor) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);

    if (contours.empty()) return {};

    const auto& largest = *std::max_element(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });

    if (largest.size() < 3) return {};

    std::vector<cv::Point> approx;
    cv::approxPolyDP(largest, approx,
                     epsFactor * cv::arcLength(largest, true), true);
    return approx.size() >= 3 ? approx : largest;
}

} // namespace Postprocess
