#include "DetectCommandHandler.h"

#include "../calib/camToolKit/calibData.h"
#include "../camera_calibreation/klipper/klipper_manager.h"
#include "../config.h"
#include "../reallink_ogles/file_utils.h"
#include "../tools/WRbin.h"
#include "../app/app_context.h"
#include "../rtsp/rtsp_streamer.h"
#include "../video/LatestNv12FrameBuffer.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <sys/select.h>
#include <thread>
#include <unistd.h>

DetectCommandHandler::DetectCommandHandler(YOLOHandle handle)
    : yolo_handle_(handle) {}

    //json检测校验
std::string DetectCommandHandler::escapeJsonString(const std::string& input) {
    std::ostringstream oss;
    for (char c : input) {
        switch (c) {
        case '"':
            oss << "\\\"";
            break;
        case '\\':
            oss << "\\\\";
            break;
        case '\n':
            oss << "\\n";
            break;
        case '\r':
            oss << "\\r";
            break;
        case '\t':
            oss << "\\t";
            break;
        default:
            oss << c;
            break;
        }
    }
    return oss.str();
}
//获取yolo的检测点击
bool DetectCommandHandler::updateYoloJson(int width,
                                          int height,
                                          const YOLOFrameResult& frame_result,
                                          DetectResponsePayload& payload) const {
    payload.count = frame_result.detection_count;

    std::string yolo_json;
    if (!yolo_get_results_json(yolo_handle_, width, height, yolo_json)) {
        return false;
    }

    payload.yolo_json = yolo_json;
    return true;
}
//寻找json文本中的成员
std::string DetectCommandHandler::extractJsonObjectMembers(const std::string& json_object) {
    const size_t begin = json_object.find('{');
    const size_t end = json_object.rfind('}');
    if (begin == std::string::npos || end == std::string::npos || begin >= end) {
        return std::string();
    }
    return json_object.substr(begin + 1, end - begin - 1);
}
//构建完整的发送的文本
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
        json_stream << "\"focal_long\":12.052,"
                    << "\"focal_short\":8.154,";
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
//发送构建的文本和图片
bool DetectCommandHandler::sendDetectResponse(CommandContext& ctx,
                                              const DetectResponsePayload& payload,
                                              const uint8_t* nv12_data) {
    const std::string json_str = buildDetectJson(payload);

    std::vector<uint8_t> jpeg_data;
    if (nv12_data != nullptr && payload.width > 0 && payload.height > 0) {
        if (!encodeNV12ToJPEG(nv12_data, payload.width, payload.height, jpeg_data)) {
            fprintf(stderr, "[Unified Server] WARNING: JPEG encoding failed, sending JSON only\n");
        }
    }

    return ctx.sendBinaryResponse(json_str, jpeg_data);
}
//将nv12转成jpg
bool DetectCommandHandler::encodeNV12ToJPEG(const uint8_t* nv12_data,
                                            int width,
                                            int height,
                                            std::vector<uint8_t>& jpeg_out,
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
//路由分发
std::string DetectCommandHandler::getName() const {
    return "DETECT";
}

std::string DetectCommandHandler::getDescription() const {
    return "YOLO object detection with image capture";
}

bool DetectCommandHandler::isLongRunning() const {
    return true;
}
//入口函数
CommandResult DetectCommandHandler::execute(CommandContext& ctx) {
    fprintf(stderr, "[Unified Server] Processing DETECT command from %s...\n", ctx.client_ip.c_str());

    //结构体的函数或变量离开作用域自动触发
    struct YoloFrameResultGuard {
        YOLOFrameResult* frame{nullptr};

        ~YoloFrameResultGuard() {
            if (frame) {
                yolo_free_frame_result(frame);
            }
        }
    };

    struct FinalHomeGuard {
        bool armed{false};

        ~FinalHomeGuard() {
            if (!armed) {
                return;
            }

            std::string final_home_error;
            const std::string final_home_script = "G90\nG28\nM400\n";
            if (!KlipperManager::instance().sendGcode(final_home_script, nullptr, 60L, &final_home_error)) {
                fprintf(stderr,
                        "[Unified Server] WARNING: G28 failed during DETECT cleanup: %s\n",
                        final_home_error.c_str());
            } else {
                fprintf(stderr, "[Unified Server] DETECT cleanup homed by explicit G28\n");
            }
        }
    };

    DetectResponsePayload payload;
    const uint8_t* response_nv12 = nullptr;

    auto updateBevStreamState = [&](bool refresh_requested = false,
                                    bool refresh_forced = false) {
        payload.bev_stream_connected = false;
        payload.bev_stream_ready = false;
        payload.bev_refresh_requested = refresh_requested;
        payload.bev_refresh_forced = refresh_forced;
        payload.bev_pending_refresh_requests = 0;
        payload.latest_bev_frame_id = 0;

        if (!ctx.app) {
            return;
        }

        payload.latest_bev_frame_id = getLatestBevFrameId(&ctx.app->capture_state);
        payload.bev_pending_refresh_requests =
            getPendingBevRefreshRequestCount(&ctx.app->capture_state);

        RTSPStreamer* bev_stream = &ctx.app->server.bev_stream;
        payload.bev_stream_connected = (bev_stream->appsrc != NULL);
        payload.bev_stream_ready = rtsp_streamer_is_ready(bev_stream);
    };

    //定义一个统一失败失败的接口
    auto sendFailure = [&](const char* error_code, const std::string& error_message) -> CommandResult {
        payload.success = false;
        payload.error_code = error_code ? error_code : "";
        payload.error_message = error_message;
        updateBevStreamState(payload.bev_refresh_requested, payload.bev_refresh_forced);
        fprintf(stderr, "[Unified Server] DETECT failure (%s): %s\n",
                error_code ? error_code : "UNKNOWN",
                error_message.c_str());

        if (!sendDetectResponse(ctx, payload, response_nv12)) {
            fprintf(stderr, "[Unified Server] Failed to send DETECT failure response\n");
            return CommandResult::ERROR_DISCONNECT;
        }
        return CommandResult::ERROR_CONTINUE;
    };

    std::string wait_work;
    if(KlipperManager::instance().sendGcode("g4 p20\n",nullptr,5L,&wait_work))
    {
        fprintf(stderr, "[Unified Server] G4 wait command response: %s\n", wait_work.c_str());
    } else {
        fprintf(stderr, "[Unified Server] WARNING: Failed to send G4 wait command\n");
    }

    std::string homing_error;
    if (!KlipperManager::instance().forceHome(&homing_error)) {
        fprintf(stderr, "[Unified Server] Forced homing failed: %s\n", homing_error.c_str());
        return sendFailure("HOMING_FAILED", homing_error);
    }

    FinalHomeGuard final_home_guard;
    final_home_guard.armed = true;

    // 从共享配置文件中读取焦距和总高，避免不同来源的数据不一致。
    auto loadSharedMetrics = [&]() {
        const std::string conf_path = std::string(REALLINK_CV_CONF_PATH);
        ReallinkCVConfig config;
        if (!readReallinkCVConf(conf_path, config)) {
            fprintf(stderr, "[Unified Server] WARNING: Failed to read %s\n", conf_path.c_str());
        } else {
            payload.focal_long = config.focal_long;
            payload.focal_short = config.focal_short;
            payload.focal_ok = true;
            payload.total_high = config.totalHigh;
            payload.total_high_ok = true;
        }
    };

    loadSharedMetrics();

    constexpr int kFirstFrameTimeoutMs = 1500;
    constexpr int kFirstFramePollIntervalMs = 20;

    //开始第一帧的采集
    if (!bev_frame_buffer_has_frame()) {
        if (ctx.app) {
            updateBevStreamState();
            const bool should_force_bev_refresh =
                (!payload.bev_stream_connected || !payload.bev_stream_ready);
            requestBevRefresh(&ctx.app->capture_state);
            updateBevStreamState(true, should_force_bev_refresh);
            fprintf(stderr,
                    "[Unified Server] DETECT stage: no cached BEV frame, requesting refresh (connected=%s, ready=%s, force=%s, pending=%u, latest_frame_id=%llu)\n",
                    payload.bev_stream_connected ? "true" : "false",
                    payload.bev_stream_ready ? "true" : "false",
                    should_force_bev_refresh ? "true" : "false",
                    payload.bev_pending_refresh_requests,
                    (unsigned long long)payload.latest_bev_frame_id);
        }

        const auto first_frame_deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(kFirstFrameTimeoutMs);
        while (!bev_frame_buffer_has_frame() &&
               std::chrono::steady_clock::now() < first_frame_deadline) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(kFirstFramePollIntervalMs));
        }

        if (!bev_frame_buffer_has_frame()) {
            updateBevStreamState(payload.bev_refresh_requested, payload.bev_refresh_forced);
            fprintf(stderr,
                    "[Unified Server] ERROR: No BEV frame available after refresh wait (bev_connected=%s, bev_ready=%s, pending_refresh=%u, latest_frame_id=%llu)\n",
                    payload.bev_stream_connected ? "true" : "false",
                    payload.bev_stream_ready ? "true" : "false",
                    payload.bev_pending_refresh_requests,
                    (unsigned long long)payload.latest_bev_frame_id);
            return sendFailure("NO_BEV_FRAME", "No BEV frame available");
        }
    }

    const int width = bev_frame_buffer_get_width();
    const int height = bev_frame_buffer_get_height();
    const size_t nv12_size = bev_frame_buffer_get_frame_size();
    payload.width = width;
    payload.height = height;

    if (width <= 0 || height <= 0 || nv12_size == 0) {
        fprintf(stderr, "[Unified Server] ERROR: Invalid BEV buffer meta (w=%d h=%d size=%zu)\n",
                width, height, nv12_size);
        return sendFailure("INVALID_BEV_BUFFER", "Invalid BEV buffer metadata");
    }

    if (!yolo_get_results_json(yolo_handle_, width, height, payload.yolo_json)) {
        fprintf(stderr, "[Unified Server] WARNING: Failed to build empty YOLO JSON, fallback to null image/[]\n");
        payload.yolo_json.clear();
    }

    //大小，帧编号
    std::vector<uint8_t> nv12_first_frame(nv12_size);
    size_t nv12_filled = 0;
    uint64_t first_frame_id = 0;
    if (!bev_frame_buffer_copy(nv12_first_frame.data(), nv12_size, &nv12_filled, &first_frame_id)) {
        fprintf(stderr, "[Unified Server] ERROR: Failed to copy BEV frame\n");
        return sendFailure("COPY_FAILED", "Failed to copy BEV frame");
    }
    response_nv12 = nv12_first_frame.data();
    payload.first_frame_id = first_frame_id;
    updateBevStreamState();

    //一致性校验
    if (nv12_filled != nv12_size) {
        fprintf(stderr, "[Unified Server] WARNING: BEV frame size mismatch (filled=%zu, expected=%zu)\n",
                nv12_filled, nv12_size);
    }

    //拿到第一帧做yolo检测
    fprintf(stderr, "[Unified Server] DETECT stage: first YOLO inference...\n");
    YOLOFrameResult first_result{};
    YoloFrameResultGuard first_guard{&first_result};

    if (!yolo_detect_nv12(yolo_handle_,
                          nv12_first_frame.data(),
                          width,
                          height,
                          first_frame_id,
                          &first_result)) {
        fprintf(stderr, "[Unified Server] YOLO detection failed\n");
        return sendFailure("DETECTION_FAILED", "YOLO detection failed");
    }
    fprintf(stderr,
            "[Unified Server] DETECT stage: first YOLO done (count=%d, infer_ms=%.2f)\n",
            first_result.detection_count,
            static_cast<double>(first_result.inference_time_ms));

    if (!updateYoloJson(width, height, first_result, payload)) {
        fprintf(stderr,
                "[Unified Server] WARNING: Failed to build first-pass YOLO JSON (frame_id=%llu)\n",
                (unsigned long long)first_result.frame_id);
    }

    if (first_result.detection_count <= 0) {
        fprintf(stderr,
                "[Unified Server] DETECT stage: no object in first pass, continue thickness fallback flow\n");
    }

    //执行测高
    payload.thickness_triggered = true;
    fprintf(stderr, "[Unified Server] DETECT stage: thickness flow...\n");
    try {
        ThicknessConfig thickness_cfg;
        ThicknessService thickness_service(thickness_cfg);
        payload.thickness_ok = thickness_service.measureFromYolo(first_result,
                                                                payload.thickness_result,
                                                                payload.thickness_error);
    } catch (const std::exception& e) {
        payload.thickness_error = e.what();
        payload.thickness_ok = false;
    }

    if (!payload.thickness_ok) {
        fprintf(stderr, "[Unified Server] Thickness flow failed: %s\n", payload.thickness_error.c_str());
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
    if (!KlipperManager::instance().sendGcode(move_script.str(), nullptr, 20L, &move_error)) {
        fprintf(stderr, "[Unified Server] move to X10 Y10 Z0 failed before second frame capture: %s\n",
                move_error.c_str());
        return sendFailure("MOVE_FAILED",
                           move_error.empty() ? "move to X10 Y10 Z0 failed" : move_error);
    }

    constexpr int kSecondFrameTimeoutMs = 5000;
    constexpr int kSecondFramePollIntervalMs = 20;

    if (ctx.app) {
        updateBevStreamState();
        const bool should_force_bev_refresh =
            (!payload.bev_stream_connected || !payload.bev_stream_ready);
        requestBevRefresh(&ctx.app->capture_state);
        updateBevStreamState(true, should_force_bev_refresh);
        fprintf(stderr,
                "[Unified Server] DETECT stage: BEV refresh requested (connected=%s, ready=%s, force=%s, pending=%u, latest_frame_id=%llu)\n",
                payload.bev_stream_connected ? "true" : "false",
                payload.bev_stream_ready ? "true" : "false",
                should_force_bev_refresh ? "true" : "false",
                payload.bev_pending_refresh_requests,
                (unsigned long long)payload.latest_bev_frame_id);
    }

    //开始采集第二针的流程
    std::vector<uint8_t> nv12_second_frame(nv12_size);
    size_t nv12_second_filled = 0;
    uint64_t second_frame_id = 0;
    if (!bev_frame_buffer_copy_newer_than(nv12_second_frame.data(),
                                          nv12_size,
                                          &nv12_second_filled,
                                          &second_frame_id,
                                          first_frame_id,
                                          kSecondFrameTimeoutMs,
                                          kSecondFramePollIntervalMs)) {
        updateBevStreamState(payload.bev_refresh_requested, payload.bev_refresh_forced);
        fprintf(stderr,
                "[Unified Server] ERROR: Timed out waiting second BEV frame after compensation (first_frame_id=%llu, latest_frame_id=%llu, bev_connected=%s, bev_ready=%s, pending_refresh=%u)\n",
                (unsigned long long)first_frame_id,
                (unsigned long long)payload.latest_bev_frame_id,
                payload.bev_stream_connected ? "true" : "false",
                payload.bev_stream_ready ? "true" : "false",
                payload.bev_pending_refresh_requests);
        return sendFailure("REFRESH_FRAME_TIMEOUT",
                           "Timed out waiting second BEV frame after compensation");
    }

    response_nv12 = nv12_second_frame.data();

    if (nv12_second_filled != nv12_size) {
        fprintf(stderr, "[Unified Server] WARNING: Second BEV frame size mismatch (filled=%zu, expected=%zu)\n",
                nv12_second_filled, nv12_size);
    }

    YOLOFrameResult second_result{};
    YoloFrameResultGuard second_guard{&second_result};
    if (!yolo_detect_nv12(yolo_handle_,
                          nv12_second_frame.data(),
                          width,
                          height,
                          second_frame_id,
                          &second_result)) {
        fprintf(stderr, "[Unified Server] Second-pass YOLO detection failed\n");
        return sendFailure("REFRESH_DETECT_FAILED", "Second-pass YOLO detection failed");
    }

    if (!updateYoloJson(width, height, second_result, payload)) {
        fprintf(stderr,
                "[Unified Server] ERROR: Failed to build YOLO JSON from ResultsToJson (frame_id=%llu)\n",
                (unsigned long long)second_result.frame_id);
        return sendFailure("JSON_BUILD_FAILED", "Failed to build detection objects JSON");
    }

    loadSharedMetrics();
    updateBevStreamState(payload.bev_refresh_requested, payload.bev_refresh_forced);

    payload.success = true;

    if (!sendDetectResponse(ctx, payload, response_nv12)) {
        fprintf(stderr, "[Unified Server] Failed to send response\n");
        return CommandResult::ERROR_DISCONNECT;
    }

    fprintf(stderr, "[Unified Server]  DETECT completed (found %d objects), connection remains open\n",
            second_result.detection_count);
    return CommandResult::SUCCESS;
}
