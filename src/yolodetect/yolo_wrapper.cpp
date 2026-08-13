#include "yolo_wrapper.h"
#include "../yolo/yolo_model.h"
#include "../yolo/embedded_model.h"
#include "../config.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <sys/stat.h>

struct YOLOContext {
    YOLOModel* model{nullptr};
    std::vector<SegmentationResult> last_results;
    uint64_t last_frame_id{0};
    cv::Size  last_image_size;
    bool      has_last_cache{false};
};

namespace {

static void copyResult(const YOLOModel& model,
                       const ObjectDetectResult& src,
                       YOLODetection& dst) {
    dst.class_id   = src.classId;
    const auto& names = model.inferenceSegmentation(cv::Mat()) ? std::vector<std::string>{} : std::vector<std::string>{};
    const char* name = "unknown";
    dst.class_name[sizeof(dst.class_name) - 1] = '\0';
    dst.confidence = src.confidence;
    dst.x = src.box.x;
    dst.y = src.box.y;
    dst.w = src.box.width;
    dst.h = src.box.height;
    dst.polygon = nullptr;
    dst.polygon_count = 0;
}

} // namespace

YOLOHandle yolo_init(void) {
    auto* ctx = new YOLOContext();
    ctx->model = new YOLOModel();

    if (ctx->model->initFromMemory(g_embedded_model_data, g_embedded_model_size) != 0) {
        fprintf(stderr, "[YOLO] Failed to init embedded model\n");
        delete ctx->model;
        delete ctx;
        return nullptr;
    }

    fprintf(stderr, "[YOLO] Embedded model initialized (%u bytes)\n", g_embedded_model_size);
    return ctx;
}

void yolo_set_confidence(YOLOHandle h, float t) {
    if (h) ((YOLOContext*)h)->model->setConfidence(t);
}

void yolo_set_nms(YOLOHandle h, float t) {
    if (h) ((YOLOContext*)h)->model->setNMS(t);
}

void yolo_cleanup(YOLOHandle h) {
    if (!h) return;
    auto* ctx = (YOLOContext*)h;
    ctx->model->release();
    delete ctx->model;
    delete ctx;
    fprintf(stderr, "[YOLO] Cleanup complete\n");
}

void yolo_free_frame_result(YOLOFrameResult* r) {
    if (!r) return;
    constexpr int kMax = (int)(sizeof(r->detections) / sizeof(r->detections[0]));
    for (int i = 0; i < kMax; i++) {
        delete[] r->detections[i].polygon;
        r->detections[i].polygon = nullptr;
        r->detections[i].polygon_count = 0;
    }
}

bool yolo_detect_nv12(YOLOHandle handle,
                      const uint8_t* nv12_data,
                      int width, int height,
                      uint64_t frame_id,
                      YOLOFrameResult* result) {
    if (!handle || !nv12_data || !result) return false;

    auto* ctx = (YOLOContext*)handle;
    yolo_free_frame_result(result);
    std::memset(result, 0, sizeof(*result));

    try {
        auto t0 = std::chrono::high_resolution_clock::now();

        cv::Mat nv(height * 3 / 2, width, CV_8UC1, (void*)nv12_data);
        cv::Mat bgr;
        cv::cvtColor(nv, bgr, cv::COLOR_YUV2BGR_NV12);

        YOLOModel::Timing timing;
        auto segResults = ctx->model->inferenceSegmentation(bgr, &timing);

        auto t1 = std::chrono::high_resolution_clock::now();

        result->frame_id = frame_id;
        result->timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        result->inference_time_ms = static_cast<float>(
            std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count());

        constexpr int kMax = (int)(sizeof(result->detections) / sizeof(result->detections[0]));
        int count = std::min((int)segResults.size(), kMax);
        result->detection_count = count;

        // 缓存最近一次有效结果
        ctx->last_results.assign(segResults.begin(),
                                  segResults.begin() + std::min(count, (int)segResults.size()));
        ctx->last_frame_id = frame_id;
        ctx->last_image_size = bgr.size();
        ctx->has_last_cache = true;

        for (int i = 0; i < count; i++) {
            const auto& src = segResults[i];
            auto& dst = result->detections[i];

            dst.class_id    = src.classId;
            strncpy(dst.class_name, src.className.c_str(), sizeof(dst.class_name) - 1);
            dst.class_name[sizeof(dst.class_name) - 1] = '\0';
            dst.confidence  = src.confidence;
            dst.x = src.boundingBox.x;
            dst.y = src.boundingBox.y;
            dst.w = src.boundingBox.width;
            dst.h = src.boundingBox.height;

            if (src.contour.size() >= 3) {
                int n = (int)src.contour.size();
                dst.polygon = new int[n][2];
                dst.polygon_count = n;
                for (int j = 0; j < n; j++) {
                    dst.polygon[j][0] = src.contour[j].x;
                    dst.polygon[j][1] = src.contour[j].y;
                }
            } else {
                dst.polygon = nullptr;
                dst.polygon_count = 0;
            }
        }

        fprintf(stderr, "[YOLO] frame=%llu det=%d pre=%.1fms inf=%.1fms post=%.1fms total=%.1fms\n",
                (unsigned long long)frame_id, count,
                timing.preprocessMs, timing.inferenceMs,
                timing.postprocessMs, timing.totalMs);

        return true;
    } catch (const std::exception& e) {
        fprintf(stderr, "[YOLO] Exception: %s\n", e.what());
        yolo_free_frame_result(result);
        std::memset(result, 0, sizeof(*result));
        return false;
    }
}

bool yolo_get_results_json(YOLOHandle handle,
                           int width, int height,
                           std::string& out_json) {
    out_json.clear();
    if (!handle || width <= 0 || height <= 0) return false;

    auto* ctx = (YOLOContext*)handle;
    if (!ctx->model || !ctx->model->isLoaded()) return false;

    std::vector<SegmentationResult> empty;
    const auto& results = (ctx->has_last_cache && ctx->last_image_size == cv::Size(width, height))
                          ? ctx->last_results : empty;

    // 手工构造简单 JSON 输出
    char buf[65536];
    int off = 0;
    off += snprintf(buf + off, sizeof(buf) - off,
        "{\n  \"image\": {\"width\": %d, \"height\": %d},\n  \"objects\": [\n", width, height);
    for (size_t i = 0; i < results.size(); i++) {
        const auto& r = results[i];
        off += snprintf(buf + off, sizeof(buf) - off,
            "    {\"class_id\": %d, \"class_name\": \"%s\", \"confidence\": %.4f, "
            "\"roi_xywh\": [%d, %d, %d, %d]}%s\n",
            r.classId, r.className.c_str(), r.confidence,
            r.boundingBox.x, r.boundingBox.y,
            r.boundingBox.width, r.boundingBox.height,
            i + 1 < results.size() ? "," : "");
    }
    off += snprintf(buf + off, sizeof(buf) - off, "  ]\n}");
    out_json = buf;
    return true;
}

bool yolo_get_last_objects_json(YOLOHandle handle,
                                int width, int height,
                                uint64_t expected_frame_id,
                                std::string& out_json) {
    out_json.clear();
    if (!handle) return false;
    auto* ctx = (YOLOContext*)handle;
    if (ctx->last_frame_id != expected_frame_id) return false;

    std::string full;
    if (!yolo_get_results_json(handle, width, height, full))
        return false;

    const std::string key = "\"objects\"";
    size_t pos = full.find(key);
    if (pos == std::string::npos) { out_json = "[]"; return false; }
    pos = full.find('[', pos);
    if (pos == std::string::npos) { out_json = "[]"; return false; }

    int depth = 0;
    bool inString = false, escaped = false;
    for (size_t i = pos; i < full.size(); i++) {
        char c = full[i];
        if (inString) {
            if (escaped) escaped = false;
            else if (c == '\\') escaped = true;
            else if (c == '"') inString = false;
            continue;
        }
        if (c == '"') { inString = true; continue; }
        if (c == '[') { depth++; continue; }
        if (c == ']') {
            depth--;
            if (depth == 0) {
                out_json = full.substr(pos, i - pos + 1);
                return true;
            }
            if (depth < 0) break;
        }
    }
    out_json = "[]";
    return false;
}
