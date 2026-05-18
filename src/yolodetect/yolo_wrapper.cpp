#include "yolo_wrapper.h"
#include "../yolo/YOLOSegmentor.h"
#include "../yolo/embedded_model.h"
#include "../config.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <sys/stat.h>

struct YOLOContext {
    YOLOSegmentor* segmentor{nullptr};              // 推理核心对象
    std::vector<SegmentationResult> last_results;   // 最近一次有效推理结果
    uint64_t last_frame_id{0};                      // 缓存对应的帧号
    cv::Size last_image_size;                       // 缓存对应的图像尺寸
    bool has_last_result_cache{false};              // 是否存在可用缓存
};

namespace {

// 清空上一帧结果缓存，防止外部读取到过期数据。
void clear_last_result_cache(YOLOContext* ctx) {
    if (!ctx) {
        return;
    }
    ctx->last_results.clear();
    ctx->last_frame_id = 0;
    ctx->last_image_size = cv::Size();
    ctx->has_last_result_cache = false;
}

// 从完整结果 JSON 中提取 "objects" 数组，避免上层重复解析整段文本。
bool extract_objects_array_json(const std::string& full_json, std::string& out_objects_json) {
    out_objects_json.clear();

    const std::string key = "\"objects\"";
    const size_t key_pos = full_json.find(key);
    if (key_pos == std::string::npos) {
        return false;
    }

    const size_t array_start = full_json.find('[', key_pos + key.size());
    if (array_start == std::string::npos) {
        return false;
    }

    int depth = 0;
    bool in_string = false;
    bool escaped = false;
    for (size_t i = array_start; i < full_json.size(); ++i) {
        const char c = full_json[i];
        // 跳过字符串内部字符，防止把字符串中的 '[' 或 ']' 误判为结构符号。
        if (in_string) {
            if (escaped) {
                escaped = false;
            } else if (c == '\\') {
                escaped = true;
            } else if (c == '"') {
                in_string = false;
            }
            continue;
        }

        if (c == '"') {
            in_string = true;
            continue;
        }
        if (c == '[') {
            ++depth;
            continue;
        }
        if (c == ']') {
            --depth;
            // depth 回到 0 时，说明 objects 数组完整闭合。
            if (depth == 0) {
                out_objects_json = full_json.substr(array_start, i - array_start + 1);
                return true;
            }
            if (depth < 0) {
                return false;
            }
        }
    }

    return false;
}

}  // namespace

// 创建并初始化 YOLO 句柄，模型来自编译进程序的内存数据。
YOLOHandle yolo_init(void) {
    
    try {
        YOLOContext* ctx = new YOLOContext();
        ctx->segmentor = new YOLOSegmentor();
        
        if (!ctx->segmentor->InitializeModelFromMemory(g_embedded_model_data, g_embedded_model_size)) {
            fprintf(stderr, "[YOLO] Error: Failed to initialize embedded model\n");
            delete ctx->segmentor;
            delete ctx;
            return nullptr;
        }
        
        fprintf(stderr, "[YOLO] Embedded model initialized (size: %u bytes)\n", g_embedded_model_size);
        return ctx;
        
    } catch (const std::exception& e) {
        fprintf(stderr, "[YOLO] Exception: %s\n", e.what());
        return nullptr;
    }
}

void yolo_set_confidence(YOLOHandle handle, float threshold) {
    if (!handle) return;
    YOLOContext* ctx = static_cast<YOLOContext*>(handle);
    ctx->segmentor->SetConfidenceThreshold(threshold);
}

void yolo_set_nms(YOLOHandle handle, float threshold) {
    if (!handle) return;
    YOLOContext* ctx = static_cast<YOLOContext*>(handle);
    ctx->segmentor->SetNMSThreshold(threshold);
}

// 释放 YOLOFrameResult 中按检测框动态分配的轮廓点内存。
void yolo_free_frame_result(YOLOFrameResult* result) {
    if (!result) {
        return;
    }

    constexpr int kMaxDetections =
        static_cast<int>(sizeof(result->detections) / sizeof(result->detections[0]));
    for (int i = 0; i < kMaxDetections; ++i) {
        if (result->detections[i].polygon) {
            delete[] result->detections[i].polygon;
            result->detections[i].polygon = nullptr;
        }
        result->detections[i].polygon_count = 0;
    }
}

bool yolo_detect_nv12(YOLOHandle handle, 
                      const uint8_t* nv12_data, 
                      int width, 
                      int height,
                      uint64_t frame_id,
                      YOLOFrameResult* result) {
    // 统一校验入口参数，避免空指针导致崩溃。
    if (!handle) {
        return false;
    }

    YOLOContext* ctx = static_cast<YOLOContext*>(handle);
    if (!nv12_data || !result) {
        clear_last_result_cache(ctx);
        return false;
    }

    // 允许调用方复用同一 result：先释放旧的动态点集，再清空结构体
    yolo_free_frame_result(result);
    std::memset(result, 0, sizeof(*result));
    clear_last_result_cache(ctx);

    try {
        auto start_time = std::chrono::high_resolution_clock::now();

        // NV12 -> BGR，供 OpenCV 与模型推理使用。
        cv::Mat nv(height * 3 / 2, width, CV_8UC1, (void*)nv12_data);
        cv::Mat bgr;
        cv::cvtColor(nv, bgr, cv::COLOR_YUV2BGR_NV12);

        static int debug_count = 0;
        if (debug_count < 5) {
            fprintf(stderr, "[YOLO Debug] Frame #%d NV12 data check:\n", debug_count);
            fprintf(stderr, "  Y plane first 10 pixels: ");
            for (int i = 0; i < 10; i++) {
                fprintf(stderr, "%d ", nv12_data[i]);
            }
            fprintf(stderr, "\n");

            size_t y_size = static_cast<size_t>(width) * static_cast<size_t>(height);
            double y_sum = 0;
            for (size_t i = 0; i < y_size; i++) {
                y_sum += nv12_data[i];
            }
            double y_mean = y_sum / y_size;
            fprintf(stderr, "  Y plane mean: %.2f (should be 30-230 for normal image)\n", y_mean);
            
            // 图像数据验证：检测异常图
            bool image_valid = true;
            if (y_mean < 1.0) {
                fprintf(stderr, "    WARNING: Y plane mean is %.2f (接近0)，图像可能是全黑的！\n", y_mean);
                image_valid = false;
            } else if (y_mean > 250.0) {
                fprintf(stderr, "    WARNING: Y plane mean is %.2f (接近255)，图像可能是全白的！\n", y_mean);
                image_valid = false;
            }

            cv::Scalar mean = cv::mean(bgr);
            cv::Scalar stddev;
            cv::meanStdDev(bgr, mean, stddev);
            fprintf(stderr, "  BGR mean = (%.2f, %.2f, %.2f)\n", mean[0], mean[1], mean[2]);
            fprintf(stderr, "  BGR stddev = (%.2f, %.2f, %.2f)\n", stddev[0], stddev[1], stddev[2]);
            
            // 检查BGR标准差是否为0（单色图像）
            if (stddev[0] < 0.1 && stddev[1] < 0.1 && stddev[2] < 0.1) {
                fprintf(stderr, "    WARNING: BGR stddev接近0，图像可能是单色的（无纹理）！\n");
                image_valid = false;
            }
            
            if (!image_valid) {
                fprintf(stderr, "    检测结果可能不准确，请检查相机或BEV处理是否正常\n");
            }

            debug_count++;
        }
        // === debug end ===

        std::vector<SegmentationResult> detections = ctx->segmentor->PerformInference(bgr);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        result->frame_id = frame_id;
        result->timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        result->inference_time_ms = static_cast<float>(duration.count());
        constexpr int kMaxDetections =
            static_cast<int>(sizeof(result->detections) / sizeof(result->detections[0]));
        // 防止输出数组越界：仅写入 result 可容纳的目标数量。
        const int detection_limit = std::min(static_cast<int>(detections.size()), kMaxDetections);
        result->detection_count = detection_limit;

        // 缓存当前帧结果，供后续 JSON 接口按帧读取。
        ctx->last_results.assign(detections.begin(), detections.begin() + detection_limit);
        ctx->last_frame_id = frame_id;
        ctx->last_image_size = bgr.size();
        ctx->has_last_result_cache = true;

        for (int i = 0; i < detection_limit; i++) {
            const SegmentationResult& det = detections[i];
            YOLODetection& out_det = result->detections[i];

            out_det.class_id = det.classId;
            strncpy(out_det.class_name,
                    det.className.c_str(),
                    sizeof(out_det.class_name) - 1);
            out_det.class_name[sizeof(out_det.class_name) - 1] = '\0';
            out_det.confidence = det.confidence;
            out_det.x = det.boundingBox.x;
            out_det.y = det.boundingBox.y;
            out_det.w = det.boundingBox.width;
            out_det.h = det.boundingBox.height;

            // 轮廓点至少 3 个才构成有效多边形。
            const int contour_count = static_cast<int>(det.contour.size());
            if (contour_count >= 3) {
                out_det.polygon = new int[static_cast<size_t>(contour_count)][2];
                out_det.polygon_count = contour_count;
                for (int j = 0; j < contour_count; ++j) {
                    out_det.polygon[j][0] = det.contour[static_cast<size_t>(j)].x;
                    out_det.polygon[j][1] = det.contour[static_cast<size_t>(j)].y;
                }
            } else {
                // 小于3点时返回空点集，不丢弃目标
                out_det.polygon = nullptr;
                out_det.polygon_count = 0;
            }
        }

#if YOLO_SAVE_IMAGES
        if (result->detection_count > 0) {
            struct stat st;
            // 目录不存在时按需创建，仅在开启图像保存时执行。
            if (stat(YOLO_IMAGE_SAVE_DIR, &st) != 0) {
                char cmd[512];
                snprintf(cmd, sizeof(cmd), "mkdir -p %s", YOLO_IMAGE_SAVE_DIR);
                int ret = system(cmd);
                if (ret != 0) {
                    fprintf(stderr, "[YOLO] Failed to create directory: %s\n", YOLO_IMAGE_SAVE_DIR);
                }
            }

            cv::Mat result_img = bgr.clone();
            ctx->segmentor->DrawResults(result_img, detections);

            char filename[512];
            snprintf(filename, sizeof(filename),
                    "%s/detect_%llu_%llu.jpg",
                    YOLO_IMAGE_SAVE_DIR,
                    (unsigned long long)frame_id,
                    (unsigned long long)result->timestamp);

            try {
                if (cv::imwrite(filename, result_img)) {
                    fprintf(stderr, "[YOLO] Saved detection image: %s\n", filename);
                } else {
                    fprintf(stderr, "[YOLO] Failed to save: %s\n", filename);
                }
            } catch (const std::exception& e) {
                fprintf(stderr, "[YOLO] Save error: %s\n", e.what());
            }
        }
#endif

        return true;

    } catch (const std::exception& e) {
        fprintf(stderr, "[YOLO] Detection exception: %s\n", e.what());
        // 异常场景必须回滚输出与缓存，避免外部误用脏结果。
        clear_last_result_cache(ctx);
        yolo_free_frame_result(result);
        std::memset(result, 0, sizeof(*result));
        return false;
    }
}

bool yolo_get_results_json(YOLOHandle handle,
                           int width,
                           int height,
                           std::string& out_results_json) {
    // 统一返回 JSON 格式结果；无缓存时返回空对象列表。
    out_results_json.clear();
    if (!handle || width <= 0 || height <= 0) {
        return false;
    }

    YOLOContext* ctx = static_cast<YOLOContext*>(handle);
    if (!ctx->segmentor) {
        return false;
    }

    try {
        static const std::vector<SegmentationResult> kEmptyResults;
        // 仅在尺寸匹配时使用缓存，避免把旧分辨率结果拼入当前画面。
        const bool use_cache =
            ctx->has_last_result_cache && ctx->last_image_size == cv::Size(width, height);
        const std::vector<SegmentationResult>& results = use_cache ? ctx->last_results : kEmptyResults;
        out_results_json = ctx->segmentor->ResultsToJson("", cv::Size(width, height), results);
        return true;
    } catch (const std::exception& e) {
        fprintf(stderr, "[YOLO] ResultsToJson exception: %s\n", e.what());
        out_results_json.clear();
        return false;
    }
}

void yolo_cleanup(YOLOHandle handle) {
    if (!handle) return;
    
    YOLOContext* ctx = static_cast<YOLOContext*>(handle);
    // 先清缓存，再释放模型与上下文。
    clear_last_result_cache(ctx);
    delete ctx->segmentor;
    delete ctx;
    
    fprintf(stderr, "[YOLO] Cleanup complete\n");
}
