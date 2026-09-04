#include "DetectCommandHandler.h"

#include "app/app_context.h"
#include "calib/camToolKit/calibData.h"
#include "camera_calibration/totalHigh.h"
#include "config.h"
#include "handlers/klipper_flow.h"
#include "klipper/klipper_manager.h"
#include "pipeline/3_sink/rtsp/rtsp_streamer.h"
#include "pipeline/common/frame_provider.h"
#include "reallink_ogles/file_utils.h"
#include "tools/WRbin.h"
#include "tools/json_utils.h"
#include "yolo/yolo_model.h"
#include "yolo/yolo_postprocess.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include <sstream>
#include <sys/select.h>
#include <unistd.h>

DetectCommandHandler::DetectCommandHandler(YOLOModel* model)
    : yolo_model_(model) {}

// JSON 字符串转义（委托给共享工具）
std::string DetectCommandHandler::escapeJsonString(const std::string& input) {
    return JsonUtils::escape(input);
}
// 从 YOLO 检测结果生成检测 JSON
void DetectCommandHandler::updateYoloJson(int width,
                                          int height,
                                          const std::vector<SegmentationResult>& results,
                                          DetectResponsePayload& payload) const {
    payload.count = static_cast<int>(results.size());
    payload.yolo_json = buildObjectsJson(results, width, height);
}
// 构建 YOLO 检测结果的 image+objects JSON（与旧 yolo_get_results_json 输出格式一致）
std::string DetectCommandHandler::buildObjectsJson(
    const std::vector<SegmentationResult>& results, int width, int height) {
    std::ostringstream json_stream;
    json_stream << std::fixed << std::setprecision(4);
    json_stream << "{\n  \"image\": {\"width\": " << width << ", \"height\": " << height
                << "},\n  \"objects\": [\n";
    for (size_t i = 0; i < results.size(); i++) {
        const SegmentationResult& r = results[i];
        json_stream << "    {\"class_id\": " << r.classId
                    << ", \"class_name\": \"" << r.className << "\""
                    << ", \"confidence\": " << r.confidence
                    << ", \"roi_xywh\": [" << r.boundingBox.x << ", " << r.boundingBox.y
                    << ", " << r.boundingBox.width << ", " << r.boundingBox.height << "]}"
                    << (i + 1 < results.size() ? "," : "") << "\n";
    }
    json_stream << "  ]\n}";
    return json_stream.str();
}
// 提取 JSON 对象花括号内的内容
std::string DetectCommandHandler::extractJsonObjectMembers(const std::string& json_object) {
    const size_t begin = json_object.find('{');
    const size_t end = json_object.rfind('}');
    if (begin == std::string::npos || end == std::string::npos || begin >= end) {
        return std::string();
    }
    return json_object.substr(begin + 1, end - begin - 1);
}
// 构建完整的 DETECT 响应 JSON
std::string DetectCommandHandler::buildDetectJson(const DetectResponsePayload& payload) const {
    std::ostringstream json_stream;
    json_stream << std::fixed << std::setprecision(2);
    json_stream << "{"
                << "\"status\":\"" << (payload.success ? "SUCCESS" : "FAILED") << "\",";

    json_stream << "\"width\":" << payload.width << ","
                << "\"height\":" << payload.height << ","
                << "\"count\":" << payload.count << ",";

    if (payload.focal_ok) {
        json_stream << "\"focal_long\":" << payload.focal_long << ","
                    << "\"focal_short\":" << payload.focal_short << ",";
    } else {
        json_stream << "\"focal_long\":" << DEFAULT_FOCAL_LONG << ","
                    << "\"focal_short\":" << DEFAULT_FOCAL_SHORT << ",";
    }

    json_stream << std::setprecision(3);
    if (payload.total_high_ok) {
        json_stream << "\"TotalHigh\":" << payload.total_high << ",";
    } else {
        json_stream << "\"TotalHigh\":0,";
    }

    json_stream << "\"error_code\":";
    if (payload.error_code.empty()) {
        json_stream << "null,";
    } else {
        json_stream << "\"" << escapeJsonString(payload.error_code) << "\",";
    }

    json_stream << "\"error_message\":";
    if (payload.error_message.empty()) {
        json_stream << "null,";
    } else {
        json_stream << "\"" << escapeJsonString(payload.error_message) << "\",";
    }

    json_stream << "\"bev_stream\":{"
                << "\"connected\":" << (payload.bev_stream_connected ? "true" : "false") << ","
                << "\"ready\":" << (payload.bev_stream_ready ? "true" : "false") << ","
                << "\"refresh_requested\":" << (payload.bev_refresh_requested ? "true" : "false") << ","
                << "\"refresh_forced\":" << (payload.bev_refresh_forced ? "true" : "false") << ","
                << "\"pending_refresh_requests\":" << payload.bev_pending_refresh_requests << ","
                << "\"first_frame_id\":" << payload.first_frame_id << ","
                << "\"latest_frame_id\":" << payload.latest_bev_frame_id
                << "},";

    json_stream << "\"thickness\":{"
                << "\"roi_xywh(informal)\":[" << payload.thickness_result.roi_x << ","
                                              << payload.thickness_result.roi_y << ","
                                              << payload.thickness_result.roi_w << ","
                                              << payload.thickness_result.roi_h << "]"
                << ",\"center_xy\":[" << payload.thickness_result.center_x << ","
                                      << payload.thickness_result.center_y << "]"
                << ",\"move_xy\":[" << payload.thickness_result.move_x << ","
                                    << payload.thickness_result.move_y << "]"
                << ",\"distance_mm\":" << payload.thickness_result.distance_mm
                << ",\"z_drop_mm\":" << payload.thickness_result.z_drop_mm
                << ",\"measured_height_mm\":" << payload.thickness_result.measured_height_mm
                << ",\"thicknesshigh\":" << payload.thickness_result.thickness_high_mm
                << ",\"error\":";

    if (payload.thickness_error.empty()) {
        json_stream << "null";
    } else {
        json_stream << "\"" << escapeJsonString(payload.thickness_error) << "\"";
    }

    json_stream << "},";

    const std::string yolo_members = extractJsonObjectMembers(payload.yolo_json);
    if (!yolo_members.empty()) {
        json_stream << yolo_members;
    } else {
        json_stream << "\"image\":null,\"objects\":[]";
    }
    json_stream << "}\n";
    return json_stream.str();
}
// 发送 JSON + JPEG 二进制响应
bool DetectCommandHandler::sendDetectResponse(CommandContext& ctx,
                                              const DetectResponsePayload& payload,
                                              const uint8_t* nv12_data,
                                              const std::vector<SegmentationResult>* draw_results) {
    const std::string json_str = buildDetectJson(payload);

    std::vector<uint8_t> jpeg_data;
    if (nv12_data != nullptr && payload.width > 0 && payload.height > 0) {
        if (!encodeNV12ToJPEG(nv12_data, payload.width, payload.height, jpeg_data, draw_results)) {
            spdlog::warn("[Unified Server] WARNING: JPEG encoding failed, sending JSON only");
        }
    }

    return ctx.sendBinaryResponse(json_str, jpeg_data);
}
// NV12 转 JPEG 编码
bool DetectCommandHandler::encodeNV12ToJPEG(const uint8_t* nv12_data,
                                            int width,
                                            int height,
                                            std::vector<uint8_t>& jpeg_out,
                                            const std::vector<SegmentationResult>* draw_results,
                                            int quality) {
    jpeg_out.clear();

    if (!nv12_data || width <= 0 || height <= 0) {
        return false;
    }

    if ((width & 1) || (height & 1)) {
        return false;
    }

    if (quality < 1) {
        quality = 1;
    } else if (quality > 100) {
        quality = 100;
    }

    try {
        cv::Mat nv12_mat(height + height / 2, width, CV_8UC1, const_cast<uint8_t*>(nv12_data));
        cv::Mat bgr_mat;
        cv::cvtColor(nv12_mat, bgr_mat, cv::COLOR_YUV2BGR_NV12);

        // 有检测结果时，把 mask/轮廓/框画到图上，便于人工验证推理是否正常
        if (draw_results != nullptr && !draw_results->empty())
            Postprocess::drawResults(bgr_mat, *draw_results);

        const std::vector<int> encode_params = {
            cv::IMWRITE_JPEG_QUALITY, quality
        };
        if (!cv::imencode(".jpg", bgr_mat, jpeg_out, encode_params)) {
            return false;
        }
    } catch (const cv::Exception&) {
        return false;
    }

    return !jpeg_out.empty();
}
// NV12 → BGR → YOLO 推理（纯 C++，直接返回检测结果，无缓存）
bool DetectCommandHandler::detectNV12(const uint8_t* nv12_data,
                                      int width,
                                      int height,
                                      std::vector<SegmentationResult>& out_results,
                                      double* out_total_ms) const {
    if (!yolo_model_ || !yolo_model_->isLoaded() || !nv12_data || width <= 0 || height <= 0) {
        return false;
    }

    try {
        auto t0 = std::chrono::high_resolution_clock::now();

        cv::Mat nv(height * 3 / 2, width, CV_8UC1, const_cast<uint8_t*>(nv12_data));
        cv::Mat bgr;
        cv::cvtColor(nv, bgr, cv::COLOR_YUV2BGR_NV12);

        out_results = yolo_model_->inferenceSegmentation(bgr);

        if (out_total_ms) {
            auto t1 = std::chrono::high_resolution_clock::now();
            *out_total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        }
        return true;
    } catch (const std::exception& e) {
        spdlog::error("[Unified Server] YOLO exception: {}", e.what());
        return false;
    }
}
std::string DetectCommandHandler::getName() const {
    return "DETECT";
}

std::string DetectCommandHandler::getDescription() const {
    return "YOLO object detection with image capture";
}
// 命令入口：单帧 YOLO 检测 + 厚度补偿流程
//
// 【本函数是"同步流水线"】
// 当前实现直接在 server 线程同步执行：命令层单线程串行，阻塞无伤大雅。
// 若将来 server 改多线程，需另行引入并发仲裁机制（见 DESIGN_KlipperService.md）。
//
// 【阅读导航：5 个阶段，任何一步失败走统一出口】
//   S0 准备：取 klipper 指针；定义 4 个 lambda 助手（见下），主流程保持线性可读
//   S1 唤醒+归位：sendG4Wait(唤醒) → forceHome(归位) → FinalHomeGuard(失败保险)
//   S2 装载标定值：loadSharedMetrics 从磁盘读焦距/总高（共享配置防数据漂移）
//   S3 抓帧+检测：请求 BEV 刷新 → grab 等一帧 → detectNV12 YOLO 定位
//   S4 测高+复位：ThicknessService.measureFromYolo（内部移动+激光测距）→ sendGcode 回安全位
//   S5 回复：sendDetectResponse（JSON + 检测框 JPEG，用首帧图像）
//
// 【lambda 助手（捕获 payload/ctx，作用域内共享）】
//   updateBevStreamState：读取 RTSP 流/帧提供器的实时状态，填进 payload
//   sendFailure：统一失败出口——填错误码/信息 → 组失败响应 → 返回 ERROR_CONTINUE
//   loadSharedMetrics：读焦距与总高到 payload（多次调用保证最新）
//   triggerBevRefresh：请求 BEV 强制刷新并打日志（首帧共用）
//
// 【payload 是"累计结果集"】每个阶段往里填字段，S5 一次性序列化回给客户端；
// response_nv12 指向首帧 Snapshot 的 buffer，作用域内有效（Snapshots 生命周期覆盖全程）。
CommandResult DetectCommandHandler::execute(CommandContext& ctx) {
    spdlog::info("[Unified Server] Processing DETECT command from {}...", ctx.client_ip);

    // 组合根持有的 Klipper 服务（依赖注入，取代 KlipperManager::instance()）。
    KlipperManager* klipper = ctx.app ? ctx.app->klipper : nullptr;

    DetectResponsePayload payload;
    const uint8_t* response_nv12 = nullptr;

    // 更新 BEV (鸟瞰图) 视频流状态的 lambda 表达式
    // refresh_requested: 是否请求了刷新
    // refresh_forced: 是否强制刷新
    auto updateBevStreamState = [&](bool refresh_requested = false,
                                    bool refresh_forced = false) {
        // 初始化载荷(payload)中的 BEV 状态字段
        payload.bev_stream_connected = false;       // 视频流是否已连接
        payload.bev_stream_ready = false;           // 视频流是否已就绪
        payload.bev_refresh_requested = refresh_requested; // 记录刷新请求状态
        payload.bev_refresh_forced = refresh_forced;       // 记录强制刷新状态
        payload.bev_pending_refresh_requests = 0;   // 待处理的刷新请求数归零
        payload.latest_bev_frame_id = 0;            // 最新的一帧 ID 归零

        // 如果上下文中的 app 对象为空，则直接返回
        if (!ctx.app) {
            return;
        }

        // 获取最新的 BEV 帧 ID
        payload.latest_bev_frame_id = ctx.app->bev_frame_provider.latestFrameId();
        // 获取待处理的 BEV 刷新请求数量
        payload.bev_pending_refresh_requests =
            getPendingBevRefreshRequestCount(&ctx.app->capture_state);

        // 获取服务器中的 BEV RTSP 视频流对象
        RTSPStreamer* bev_stream = &ctx.app->server.bev_stream;
        // 检查 BEV 视频流的 appsrc 插件是否已创建，以判断是否已连接（线程安全读取）
        payload.bev_stream_connected = rtsp_streamer_has_appsrc(bev_stream);
        // 调用工具函数判断 BEV 视频流是否已完全就绪
        payload.bev_stream_ready = rtsp_streamer_is_ready(bev_stream);
    };

    //定义一个统一失败失败的接口
    auto sendFailure = [&](const char* error_code, const std::string& error_message) -> CommandResult {
        payload.success = false;
        payload.error_code = error_code ? error_code : "";
        payload.error_message = error_message;
        updateBevStreamState(payload.bev_refresh_requested, payload.bev_refresh_forced);
        spdlog::error("[Unified Server] DETECT failure ({}): {}",
                      error_code ? error_code : "UNKNOWN", error_message);

        if (!sendDetectResponse(ctx, payload, response_nv12)) {
            spdlog::error("[Unified Server] Failed to send DETECT failure response");
            return CommandResult::ERROR_DISCONNECT;
        }
        return CommandResult::ERROR_CONTINUE;
    };

    // 命令层单线程串行，无并发抢占 Klipper 的风险，直接进入业务流程。
    sendG4Wait(klipper);

    std::string homing_error;
    if (!klipper->forceHome(&homing_error)) {
        spdlog::error("[Unified Server] Forced homing failed: {}",
                      homing_error.empty() ? "(no error detail)" : homing_error);
        return sendFailure("HOMING_FAILED", homing_error);
    }

    FinalHomeGuard final_home_guard(klipper);
    final_home_guard.armed = true;

    // 从共享配置文件中读取焦距和总高，避免不同来源的数据不一致。
    auto loadSharedMetrics = [&]() {
        const std::string conf_path = std::string(REALLINK_CV_CONF_PATH);
        const std::string bin_path =
            std::string(CALIB_RESULT_DIR) + "/" + std::string(CALIB_BIN_NAME);
        ReallinkCVConfig config;
        if (!readReallinkCVConf(conf_path, config)) {
            spdlog::warn("[Unified Server] WARNING: Failed to read {}", conf_path);
        } else {
            payload.focal_long = config.focal_long;
            payload.focal_short = config.focal_short;
            payload.focal_ok = true;
        }

        std::string total_high_error;
        if (readPersistedTotalHigh(conf_path, bin_path, payload.total_high, &total_high_error)) {
            payload.total_high_ok = true;
        } else {
            payload.total_high_ok = false;
            spdlog::warn("[Unified Server] WARNING: Failed to resolve totalHigh: {}", total_high_error);
        }
    };

    loadSharedMetrics();

    // 请求 BEV 刷新并更新流状态（首帧/次帧共用的逻辑）。
    auto triggerBevRefresh = [&](const char* reason) {
        if (!ctx.app) {
            return;
        }
        updateBevStreamState();
        const bool should_force_bev_refresh =
            (!payload.bev_stream_connected || !payload.bev_stream_ready);
        requestBevRefresh(&ctx.app->capture_state);
        updateBevStreamState(true, should_force_bev_refresh);
        spdlog::info(
            "[Unified Server] DETECT stage: {} (connected={}, ready={}, force={}, pending={}, latest_frame_id={})",
            reason,
            payload.bev_stream_connected ? "true" : "false",
            payload.bev_stream_ready ? "true" : "false",
            should_force_bev_refresh ? "true" : "false",
            payload.bev_pending_refresh_requests,
            payload.latest_bev_frame_id);
    };

    constexpr int kFirstFrameTimeoutMs = 1500;

    buzzDeep(klipper); // 蜂鸣器响一声，提示开始检测
    // 首帧采集：机器刚完成 forceHome 移动，必须拿到移动后的新鲜 BEV 帧——
    // 无条件请求刷新，并阻塞等待比"开始时缓存帧"更新的帧（条件变量，非轮询）。
    // 旧逻辑 hasFrame()==true 时直接信任缓存，可能拿到数分钟前的旧图做检测。
    const uint64_t min_bev_frame_id =
        ctx.app->bev_frame_provider.latestFrameId();
    triggerBevRefresh("request fresh BEV frame for detection");
    Snapshot first_snap;
    if (!ctx.app->bev_frame_provider.grabNewerThan(min_bev_frame_id, first_snap,
                                                   nullptr, kFirstFrameTimeoutMs)) {
        updateBevStreamState(payload.bev_refresh_requested, payload.bev_refresh_forced);
        spdlog::error(
            "[Unified Server] ERROR: No BEV frame available after refresh wait (bev_connected={}, bev_ready={}, pending_refresh={}, latest_frame_id={})",
            payload.bev_stream_connected ? "true" : "false",
            payload.bev_stream_ready ? "true" : "false",
            payload.bev_pending_refresh_requests,
            payload.latest_bev_frame_id);
        return sendFailure("NO_BEV_FRAME", "No BEV frame available");
    }

    const int width = static_cast<int>(first_snap.width);
    const int height = static_cast<int>(first_snap.height);
    const size_t nv12_size = first_snap.nv12.size();
    payload.width = width;
    payload.height = height;

    if (width <= 0 || height <= 0 || nv12_size == 0) {
        spdlog::error("[Unified Server] ERROR: Invalid BEV buffer meta (w={} h={} size={})",
                      width, height, nv12_size);
        return sendFailure("INVALID_BEV_BUFFER", "Invalid BEV buffer metadata");
    }

    payload.yolo_json = buildObjectsJson(std::vector<SegmentationResult>{}, width, height);

    // 首帧数据：由 Snapshot 拥有，本函数作用域内有效（用完自动释放）
    const uint8_t* nv12_first_frame = first_snap.nv12.data();
    const uint64_t first_frame_id = first_snap.frame_id;
    response_nv12 = nv12_first_frame;
    payload.first_frame_id = first_frame_id;
    updateBevStreamState();

    // 首帧 YOLO 推理
    spdlog::info("[Unified Server] DETECT stage: first YOLO inference...");
    std::vector<SegmentationResult> first_results;
    double first_total_ms = 0.0;

    if (!detectNV12(nv12_first_frame, width, height, first_results, &first_total_ms)) {
        spdlog::error("[Unified Server] YOLO detection failed");
        return sendFailure("DETECTION_FAILED", "YOLO detection failed");
    }
    spdlog::info("[Unified Server] DETECT stage: first YOLO done (count={}, total_ms={:.2f})",
                 first_results.size(), first_total_ms);

    updateYoloJson(width, height, first_results, payload);

    if (first_results.empty()) {
        spdlog::warn("[Unified Server] DETECT stage: no object in first pass, continue thickness fallback flow");
    }

    //执行测高
    payload.thickness_triggered = true;
    spdlog::info("[Unified Server] DETECT stage: thickness flow...");
    try {
        ThicknessConfig thickness_cfg;
        ThicknessService thickness_service(klipper, thickness_cfg);
        payload.thickness_ok = thickness_service.measureFromYolo(first_results,
                                                                payload.thickness_result,
                                                                payload.thickness_error);
    } catch (const std::exception& e) {
        payload.thickness_error = e.what();
        payload.thickness_ok = false;
    }

    if (!payload.thickness_ok) {
        spdlog::error("[Unified Server] Thickness flow failed: {}",
                      payload.thickness_error.empty() ? "(no error detail)" : payload.thickness_error);
        return sendFailure("THICKNESS_FAILED", payload.thickness_error);
    }

    loadSharedMetrics();

    //测高结束后移动
    std::ostringstream move_script;
    move_script << std::fixed << std::setprecision(3);
    move_script << "G90\n";
    move_script << "G1 Z0.000" << "\n";
    move_script << "M400\n";
    move_script << "G1 X10.000 Y10.000" << "\n";
    move_script << "M400\n";

    std::string move_error;
    if (!klipper->sendGcode(move_script.str(), nullptr, 20L, &move_error)) {
        spdlog::error("[Unified Server] move to X10 Y10 Z0 failed: {}",
                      move_error.empty() ? "(no error detail)" : move_error);
        return sendFailure("MOVE_FAILED",
                           move_error.empty() ? "move to X10 Y10 Z0 failed" : move_error);
    }

    loadSharedMetrics();
    updateBevStreamState(payload.bev_refresh_requested, payload.bev_refresh_forced);

    payload.success = true;

    // 直接复用首帧的检测结果与图像，不再二次抓帧（首帧 YOLO 定位已满足补偿算法输入）
    if (!sendDetectResponse(ctx, payload, response_nv12, &first_results)) {
        spdlog::error("[Unified Server] Failed to send response");
        return CommandResult::ERROR_DISCONNECT;
    }

    spdlog::info("[Unified Server] DETECT completed (found {} objects), connection remains open",
                 first_results.size());
    return CommandResult::SUCCESS;
}
