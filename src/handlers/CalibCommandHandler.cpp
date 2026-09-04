#include "CalibCommandHandler.h"

#include "app/app_context.h"
#include "camera_calibration/focalabfocal.h"
#include "camera_calibration/mainCamera/main_camera_fisheye_calib.h"
#include "camera_calibration/viceCamera/vice_camera_service.h"
#include "camera_calibration/XYoffset.h"
#include "camera_calibration/totalHigh.h"
#include "config.h"
#include "handlers/klipper_flow.h"
#include "klipper/klipper_manager.h"
#include "pipeline/common/frame_provider.h"
#include "reallink_ogles/camera.h"
#include "tools/WRbin.h"
#include "tools/dir_utils.h"

#include <cstdio>
#include <cstring>
#include <ctime>
#include <exception>
#include <cstdint>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <thread>
#include <future>
#include <memory>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <unistd.h>
#include <limits.h>
#include <cstdlib>

namespace {

// ============================================================================
// Constants And Small Types
// ============================================================================
//空内角点或棋
constexpr const char* kMainCalibEmptyDetection = "EMPTY_DETECTION";
//内角点或棋盘过少
constexpr const char* kMainCalibDetectionCountMismatch = "DETECTION_COUNT_MISMATCH";
//主摄标定重试次数
constexpr int kMainCalibMaxAttempts = 3;
constexpr int kMainCameraRefreshTimeoutMs = 5000;
//主摄结果结构
struct MainCameraCalibResult {
    bool success = false;
    std::string error;
};
//副摄结果结构
struct ViceCameraCalibResult {
    bool success = false;
    char error[512];
    char latest_dir[512];
};
//流程控制
enum class MainCalibRetryDecision {
    ContinueRetry,                  //重试
    AbortEmptyDetection,            //检测为空终
    AbortDetectionCountMismatch,    //主摄检测数量不够终
    AbortAttemptsExhausted,         //主摄标定重试次数用完
};

struct MainCalibAttemptContext {
    int main_width = 0;
    int main_height = 0;
    size_t nv12_buf_size = 0;
    const char* session_result_dir = nullptr;
};

// ============================================================================
// Main Camera Calibration
// ============================================================================

MainCameraCalibResult runMainCameraCalib(const unsigned char* nv12_buffer,
    size_t nv12_filled,
    int width,
    int height,
    const char* output_dir,
    uint64_t frame_id) 
{
    MainCameraCalibResult result;

    const int old_cv_threads = cv::getNumThreads();
    constexpr int kCalibCvThreads = 2;
    cv::setNumThreads(kCalibCvThreads);
    spdlog::info("[Main Calib Thread] Starting main camera calibration (frame_id={}, cv_threads={}->{})...",
                 frame_id, old_cv_threads, kCalibCvThreads);

    // 超时保护：最多等
30 
    constexpr int kCalibTimeoutSeconds = 30;
    //异步执行主摄标定
    auto calib_future = std::async(std::launch::async, [&]()
                        {
                            MainCameraFisheyeCalibrator calibrator;
                            return calibrator.calibrateFromNv12(
                                nv12_buffer, nv12_filled, width, height, output_dir ? output_dir : "", frame_id);
                        });

    //超时
    if (calib_future.wait_for(std::chrono::seconds(kCalibTimeoutSeconds)) == std::future_status::timeout) {
        spdlog::warn("[Main Calib Thread] Calibration TIMEOUT after {}s (frame_id={})",
                     kCalibTimeoutSeconds, frame_id);
        result.success = false;
        result.error = "Calibration timeout after " + std::to_string(kCalibTimeoutSeconds) + "s";
    } else {
        //在规定的时间内完
        const MainCameraFisheyeCalibResult calib_result = calib_future.get();
        result.success = calib_result.success;
        result.error = calib_result.error;
    }

    //回复线程
    if (old_cv_threads > 0) {
        cv::setNumThreads(old_cv_threads);
    } else {
        cv::setNumThreads(1);
    }

    if (!result.success) {
        if (result.error.empty()) {
            result.error = "Main camera calibration failed";
        }
        spdlog::error("[Main Calib Thread] Main camera calibration failed (frame_id={})", frame_id);
    } else {
        spdlog::info("[Main Calib Thread] Main camera calibration completed (frame_id={})", frame_id);
    }

    return result;
}

// ============================================================================
// Filesystem And Session Helpers
// ============================================================================

bool createCalibrationSession(char* session_id,
                              size_t session_id_size,
                              char* session_result_dir,
                              size_t session_result_dir_size) {
    const time_t now = time(nullptr);
    struct tm* tm_info = localtime(&now);
    if (!tm_info) {
        return false;
    }

    snprintf(session_id, session_id_size, "%04d%02d%02d_%02d%02d%02d",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);

    snprintf(session_result_dir, session_result_dir_size,
             "%s/session_%s", CALIB_RESULT_DIR, session_id);

    mkdirRecursive(std::string(CALIB_CAPTURE_DIR));
    mkdirRecursive(std::string(CALIB_RESULT_DIR));
    mkdirRecursive(std::string(session_result_dir));
    return true;
}

// ============================================================================
// klipper Device Control Helpers
// ============================================================================
//---TILT_MONITOR（标定流程特有，不共享）
bool sendTiltMonitorCommand(KlipperManager* klipper, bool enable) {
    std::string response;
    std::string error;
    const int enable_flag = (enable ? 1 : 0);
    const std::string script =
        "TILT_MONITOR ENABLE=" + std::to_string(enable_flag) + "\n";
    if (klipper->sendGcode(script, &response, 10L, &error)) {
        spdlog::info("[Unified Server] TILT_MONITOR ENABLE={} command response: {}",
                     enable_flag,
                     response.empty() ? "(empty)" : response);
        return true;
    }

    spdlog::warn("[Unified Server] WARNING: Failed to send TILT_MONITOR ENABLE={}: {}",
                 enable_flag,
                 error.empty() ? "sendGcode failed" : error);
    return false;
}

// RAII：作用域结束时重新开启倾斜监控，确保标定无论成功/失败/提前返回都会恢复。
struct TiltMonitorGuard {
    KlipperManager* klipper_;
    explicit TiltMonitorGuard(KlipperManager* klipper) : klipper_(klipper) {}
    ~TiltMonitorGuard() { sendTiltMonitorCommand(klipper_, true); }
};

// ============================================================================
// Main Camera Retry Flow
// ============================================================================
//截图
bool captureMainCalibrationFrame(const MainCalibAttemptContext& attempt_ctx,
                                 FrameProvider* frame_provider,
                                 int attempt,
                                 std::vector<unsigned char>& nv12_buffer,
                                 size_t& nv12_filled,
                                 uint64_t& frame_id) {
    spdlog::info("[Unified Server] Main camera calibration attempt {}/{}: capturing frame...",
                 attempt, kMainCalibMaxAttempts);

    //开缓冲。assign：清空并赋
    nv12_buffer.assign(attempt_ctx.nv12_buf_size, 0);
    nv12_filled = 0;
    frame_id = 0;
    if (!frame_provider) {
        return false;
    }

    // 取一帧比"当前最新"更新的帧（刚采集的新帧），而非缓存旧帧。
    Snapshot snap;
    if (!frame_provider->grabNewerThan(frame_provider->latestFrameId(), snap,
                                       nullptr, kMainCameraRefreshTimeoutMs)) {
        return false;
    }
    // 拷进调用方提供的缓冲（保留原始 nv12_buffer/filled/frame_id 语义）。
    const size_t copy_n = snap.nv12.size() < nv12_buffer.size()
                              ? snap.nv12.size()
                              : nv12_buffer.size();
    std::memcpy(nv12_buffer.data(), snap.nv12.data(), copy_n);
    nv12_filled = copy_n;
    frame_id = snap.frame_id;

    //检查数据是否有
    return nv12_filled >=
        static_cast<size_t>(attempt_ctx.main_width) * static_cast<size_t>(attempt_ctx.main_height);
}
//根据错误进行下一
MainCalibRetryDecision decideMainCalibRetry(int attempt,
                                            const MainCameraCalibResult& result,
                                            int& detection_count_mismatch_failures) 
{
    //为空的情
    if (result.error == kMainCalibEmptyDetection)
     {
        return MainCalibRetryDecision::AbortEmptyDetection;
    }
    //数量偏少
    if (result.error == kMainCalibDetectionCountMismatch) {
        ++detection_count_mismatch_failures;
        if (detection_count_mismatch_failures >= 2) {
            return MainCalibRetryDecision::AbortDetectionCountMismatch;
        }
    }
    //重试次数
    if (attempt >= kMainCalibMaxAttempts) {
        return MainCalibRetryDecision::AbortAttemptsExhausted;
    }

    return MainCalibRetryDecision::ContinueRetry;
}
//根据结果做的响应
CommandResult sendMainCalibFailureResponse(CommandContext& ctx,
                            MainCalibRetryDecision decision,
                            const MainCameraCalibResult& result) 
{
    switch (decision) {
    case MainCalibRetryDecision::AbortEmptyDetection:
        spdlog::error("[Unified Server] Main camera calibration detected no boards/corners, aborting retry loop");
        ctx.sendErrorResponse("CALIBRATION_FAILED",
                              "Main camera calibration failed: empty detection");
        return CommandResult::ERROR_CONTINUE;

    case MainCalibRetryDecision::AbortDetectionCountMismatch:
        spdlog::error("[Unified Server] Main camera calibration detected board/corner count mismatch twice, aborting retry loop");
        ctx.sendErrorResponse("CALIBRATION_FAILED",
                              "Main camera calibration failed: expected 6 boards and 294 corners");
        return CommandResult::ERROR_CONTINUE;

    case MainCalibRetryDecision::AbortAttemptsExhausted: {
        spdlog::error("[Unified Server] Main camera calibration failed 3 times, aborting calibration");
        std::string error_message = "Main camera calibration failed 3 times";
        if (!result.error.empty()) {
            error_message += ": ";
            error_message += result.error;
        }
        ctx.sendErrorResponse("CALIBRATION_FAILED", error_message);
        return CommandResult::ERROR_CONTINUE;
    }

    case MainCalibRetryDecision::ContinueRetry:
        break;
    }

    return CommandResult::SUCCESS;
}

CommandResult tryCalibrateMainCamera(CommandContext& ctx,
                                     const MainCalibAttemptContext& attempt_ctx,
                                     MainCameraCalibResult& main_result) {
    int detection_count_mismatch_failures = 0;

    for (int attempt = 1; attempt <= kMainCalibMaxAttempts; ++attempt) {
        std::vector<unsigned char> nv12_buffer;
        size_t nv12_filled = 0;
        uint64_t frame_id = 0;
        FrameProvider* frame_provider = ctx.app ? &ctx.app->main_frame_provider : nullptr;
        if (!captureMainCalibrationFrame(
                attempt_ctx, frame_provider, attempt, nv12_buffer, nv12_filled, frame_id)) {
            spdlog::error("[Unified Server] Failed to capture valid latest main camera frame");
            ctx.sendErrorResponse("CAPTURE_FAILED", "Failed to copy latest main camera frame");
            return CommandResult::ERROR_CONTINUE;
        }

        spdlog::info("[Unified Server] Main camera frame captured (frame_id={}), running calibration...",
                     frame_id);
        main_result = runMainCameraCalib(nv12_buffer.data(),
                                         nv12_filled,
                                         attempt_ctx.main_width,
                                         attempt_ctx.main_height,
                                         attempt_ctx.session_result_dir,
                                         frame_id);

        if (main_result.success) {
            spdlog::info("[Unified Server] Main camera calibration succeeded on attempt {}/{}",
                         attempt, kMainCalibMaxAttempts);
            return CommandResult::SUCCESS;
        }

        spdlog::warn("[Unified Server] Main camera calibration failed on attempt {}/{}: {}",
                     attempt, kMainCalibMaxAttempts,
                     main_result.error.empty() ? "(no error detail)" : main_result.error);

        const MainCalibRetryDecision decision =
            decideMainCalibRetry(attempt, main_result, detection_count_mismatch_failures);
        if (decision != MainCalibRetryDecision::ContinueRetry) {
            return sendMainCalibFailureResponse(ctx, decision, main_result);
        }
    }

    ctx.sendErrorResponse("CALIBRATION_FAILED", "Main camera calibration failed");
    return CommandResult::ERROR_CONTINUE;
}

// ============================================================================
// Special Command Paths
// ============================================================================

CommandResult handleFocalOnly(CommandContext& ctx) {
    buzzDeep(ctx.app ? ctx.app->klipper : nullptr); // 蜂鸣器响一声，提示开始校
    spdlog::info("[Unified Server] Running focal autofocus only (CALIB-FOCALS)");
    Focalabfocal focal_service(ctx.app ? ctx.app->klipper : nullptr,
                               ctx.app ? &ctx.app->main_frame_provider : nullptr);
    double focal_long = 0.0;
    double focal_short = 0.0;
    std::string focal_error;
    if (!focal_service.runAutoFocusAndSave(focal_long, focal_short, focal_error)) {
        spdlog::error("[Unified Server] CALIB-FOCALS failed: {}", focal_error);
        ctx.sendErrorResponse("FOCAL_FAILED", focal_error);
        return CommandResult::ERROR_CONTINUE;
    }

    std::ostringstream success_json;
    success_json << "{\"status\":\"SUCCESS\",\"focal_long\":"
                 << focal_long
                 << ",\"focal_short\":"
                 << focal_short
                 << "}\n";
    if (!ctx.sendBinaryResponse(success_json.str())) {
        spdlog::error("[Unified Server] Failed to send CALIB-FOCALS success response");
        return CommandResult::ERROR_DISCONNECT;
    }

    spdlog::info("[Unified Server] CALIB-FOCALS completed: focal_long={:.2f} focal_short={:.2f}",
                 focal_long, focal_short);
    return CommandResult::SUCCESS;
}

CommandResult handleTotalHighOnly(CommandContext& ctx) {
    buzzDeep(ctx.app ? ctx.app->klipper : nullptr); // 蜂鸣器响一声，提示开始校
    spdlog::info("[Unified Server] Running totalHigh measurement only (CALIB-1)");
    std::string total_high_error;
    TotalHighService total_high_service(ctx.app ? ctx.app->klipper : nullptr);
    double total_high = 0.0;
    if (!total_high_service.measure(total_high, total_high_error)) {
        spdlog::error("[Unified Server] totalHigh measurement failed: {}", total_high_error);
        ctx.sendErrorResponse("TOTALHIGH_FAILED", total_high_error);
        runFinalHoming(ctx.app->klipper);
        return CommandResult::ERROR_CONTINUE;
    }

    if (!total_high_service.readTotalHigh(total_high, total_high_error)) {
        spdlog::error("[Unified Server] Failed to read totalHigh: {}", total_high_error);
        ctx.sendErrorResponse("TOTALHIGH_READ_FAILED", total_high_error);
        return CommandResult::ERROR_CONTINUE;
    }

    char json_buf[128];
    snprintf(json_buf, sizeof(json_buf), "{\"TotalHigh\":%.3f}\n", total_high);
    if (!ctx.sendBinaryResponse(json_buf)) {
        spdlog::error("[Unified Server] Failed to send totalHigh response");
        return CommandResult::ERROR_DISCONNECT;
    }

    spdlog::info("[Unified Server] CALIB-1 completed: TotalHigh={:.3f}mm, connection remains open",
                 total_high);
    return CommandResult::SUCCESS;
}

// ============================================================================
// Vice Camera Calibration
// ============================================================================

ViceCameraCalibResult runViceCameraCalib(const ViceCameraConfig &config, KlipperManager* klipper) {
    ViceCameraCalibResult result;
    result.error[0] = '\0';
    result.latest_dir[0] = '\0';

    spdlog::info("[Vice Calib Thread] Starting vice camera calibration...");
    try {
        ViceCameraService service(config, klipper);
        const ViceCameraResult vice_result = service.run(true);
        result.success = vice_result.success;
        if (!result.success) {
            snprintf(result.error, sizeof(result.error), "%s", vice_result.error.c_str());
            spdlog::error("[Vice Calib Thread] Vice camera calibration failed: {}", result.error);
        } else {
            snprintf(result.latest_dir, sizeof(result.latest_dir), "%s", vice_result.latest_dir.c_str());
            spdlog::info("[Vice Calib Thread] Vice camera calibration completed (dir: {})",
                         result.latest_dir);
        }
    } catch (const std::exception &e) {
        snprintf(result.error, sizeof(result.error), "Failed to create vice camera service: %s", e.what());
        spdlog::error("[Vice Calib Thread] {}", result.error);
    }
    return result;
}

// 主摄 + 副摄完整标定流程（阶段化线性推进，失败即返回，不再使用 do/while+break）。
CommandResult runFullCalibration(CommandContext& ctx, const char* session_result_dir) {
    MainCalibAttemptContext main_attempt_ctx;
    main_attempt_ctx.main_width = INPUT_WIDTH;
    main_attempt_ctx.main_height = INPUT_HEIGHT;
    main_attempt_ctx.nv12_buf_size =
        static_cast<size_t>(INPUT_WIDTH) * static_cast<size_t>(INPUT_HEIGHT) * 3 / 2;
    main_attempt_ctx.session_result_dir = session_result_dir;

    buzzDeep(ctx.app ? ctx.app->klipper : nullptr); // 蜂鸣器响一声，提示开始校
    MainCameraCalibResult main_result;
    const CommandResult main_calib_cmd_result =
        tryCalibrateMainCamera(ctx, main_attempt_ctx, main_result);
    if (main_calib_cmd_result != CommandResult::SUCCESS) {
        return main_calib_cmd_result;
    }

    spdlog::info("[Unified Server] Starting XYoffset calibration...");
    double xy_dxPx = 0.0, xy_dyPx = 0.0;
    std::string xy_error;
    XYoffsetService xyoffset_service(ctx.app ? ctx.app->klipper : nullptr,
                                     ctx.app ? &ctx.app->main_frame_provider : nullptr);
    if (!xyoffset_service.calibrate(xy_dxPx, xy_dyPx, xy_error)) {
        spdlog::error("[Unified Server] XYoffset calibration failed: {}", xy_error);
        ctx.sendErrorResponse("CALIBRATION_FAILED", xy_error);
        runFinalHoming(ctx.app->klipper);
        return CommandResult::ERROR_CONTINUE;
    }
    spdlog::info("[Unified Server] XYoffset calibration completed: dxPx={:.2f}, dyPx={:.2f}",
                 xy_dxPx, xy_dyPx);

    // 副摄标定与主摄并行执行，再同步等待结果
    spdlog::info("[Unified Server] Starting vice camera calibration...");
    ViceCameraConfig camera_config{};
    auto vice_future = std::async(std::launch::async, runViceCameraCalib, camera_config, ctx.app->klipper);
    ViceCameraCalibResult vice_result = vice_future.get();

    if (!vice_result.success) {
        spdlog::error("[Unified Server] Vice camera calibration failed: {}", vice_result.error);
        ctx.sendErrorResponse("CALIBRATION_FAILED", vice_result.error);
        runFinalHoming(ctx.app->klipper);
        return CommandResult::ERROR_CONTINUE;
    }
    spdlog::info("[Unified Server]   - Vice camera:  (dir: {})", vice_result.latest_dir);

    cameraInit();
    spdlog::info("[Unified Server] Camera parameters reloaded with thickness compensation");

    const char *success_json = "{\"status\":\"SUCCESS\"}\n";
    if (!ctx.sendBinaryResponse(success_json)) {
        spdlog::error("[Unified Server] Failed to send success response");
        return CommandResult::ERROR_DISCONNECT;
    }

    runFinalHoming(ctx.app->klipper);
    spdlog::info("[Unified Server] CALIB completed, connection remains open");
    return CommandResult::SUCCESS;
}

} // namespace

std::string CalibCommandHandler::getName() const {
    return "CALIB";
}

std::string CalibCommandHandler::getDescription() const {
    return "Camera calibration (main + vice cameras in parallel)";
}

CommandResult CalibCommandHandler::execute(CommandContext& ctx) {
    spdlog::info("[Unified Server] Processing CALIB command from {}...", ctx.client_ip);

    KlipperManager* klipper = ctx.app ? ctx.app->klipper : nullptr;

    sendTiltMonitorCommand(klipper, false);
    TiltMonitorGuard tilt_guard(klipper);   // 作用域结束自动重新开启倾斜监控

    sendG4Wait(klipper);
    runFinalHoming(klipper);

    if (ctx.command == "CALIB-FOCALS") {
        return handleFocalOnly(ctx);
    }
    if (ctx.command == "CALIB-1") {
        return handleTotalHighOnly(ctx);
    }

    char session_id[64];
    char session_result_dir[512];
    if (!createCalibrationSession(session_id, sizeof(session_id),
                                  session_result_dir, sizeof(session_result_dir))) {
        ctx.sendErrorResponse("CALIBRATION_FAILED", "Failed to create calibration session");
        return CommandResult::ERROR_CONTINUE;
    }
    spdlog::info("[Unified Server] Created session: {}", session_id);

    return runFullCalibration(ctx, session_result_dir);
}
