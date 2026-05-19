#include "CalibCommandHandler.h"

#include "../camera_calibreation/klipper/klipper_manager.h"
#include "../camera_calibreation/focalabfocal.h"
#include "../camera_calibreation/mainCamera/main_camera_fisheye_calib.h"
#include "../camera_calibreation/viceCamera/vice_camera_service.h"
#include "../camera_calibreation/XYoffset.h"
#include "../camera_calibreation/totalHigh.h"
#include "../video/LatestNv12FrameBuffer.h"
#include "../app/app_context.h"
#include "../config.h"
#include "../tools/WRbin.h"

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
#include <unistd.h>
#include <limits.h>
#include <cstdlib>

namespace {

// ============================================================================
// Constants And Small Types
// ============================================================================
//空内角点或棋盘
constexpr const char* kMainCalibEmptyDetection = "EMPTY_DETECTION";
//内角点或棋盘过少
constexpr const char* kMainCalibDetectionCountMismatch = "DETECTION_COUNT_MISMATCH";
//主摄标定重试次数
constexpr int kMainCalibMaxAttempts = 3;
constexpr int kMainCameraRefreshTimeoutMs = 5000;
constexpr int kMainCameraRefreshPollIntervalMs = 20;
//主摄结果结构体
struct MainCameraCalibResult {
    bool success = false;
    std::string error;
};
//副摄结果结构体
struct ViceCameraCalibResult {
    bool success = false;
    char error[512];
    char latest_dir[512];
};
//流程控制
enum class MainCalibRetryDecision {
    ContinueRetry,                  //重试
    AbortEmptyDetection,            //检测为空终止
    AbortDetectionCountMismatch,    //主摄检测数量不够终止
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
    fprintf(stderr,
            "[Main Calib Thread]  Starting main camera calibration (frame_id=%llu, cv_threads=%d->%d)...\n",
            (unsigned long long)frame_id, old_cv_threads, kCalibCvThreads);

    // 超时保护：最多等待15秒
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
        fprintf(stderr,
                "[Main Calib Thread]  Calibration TIMEOUT after %ds (frame_id=%llu)\n",
                kCalibTimeoutSeconds, (unsigned long long)frame_id);
        result.success = false;
        result.error = "Calibration timeout after " + std::to_string(kCalibTimeoutSeconds) + "s";
    } else {
        //在规定的时间内完成
        const MainCameraFisheyeCalibResult calib_result = calib_future.get();
        result.success = calib_result.success;
        result.error = calib_result.error;
    }

    //回复线程数
    if (old_cv_threads > 0) {
        cv::setNumThreads(old_cv_threads);
    } else {
        cv::setNumThreads(1);
    }

    if (!result.success) {
        if (result.error.empty()) {
            result.error = "Main camera calibration failed";
        }
        fprintf(stderr, "[Main Calib Thread]  Main camera calibration failed (frame_id=%llu)\n",
                (unsigned long long)frame_id);
    } else {
        fprintf(stderr, "[Main Calib Thread]  Main camera calibration completed (frame_id=%llu)\n",
                (unsigned long long)frame_id);
    }

    return result;
}

// ============================================================================
// Filesystem And Session Helpers
// ============================================================================

int mkdir_recursive(const char *path) {
    char tmp[1024];
    char *p = nullptr;
    size_t len;

    snprintf(tmp, sizeof(tmp), "%s", path);
    len = strlen(tmp);
    if (len == 0) {
        return 0;
    }
    if (tmp[len - 1] == '/') {
        tmp[len - 1] = 0;
    }

    for (p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = 0;
            if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
                return -1;
            }
            *p = '/';
        }
    }
    if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
        return -1;
    }
    return 0;
}

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

    mkdir_recursive(CALIB_CAPTURE_DIR);
    mkdir_recursive(CALIB_RESULT_DIR);
    mkdir_recursive(session_result_dir);
    return true;
}

// ============================================================================
// klipper Device Control Helpers
// ============================================================================
//---唤醒
void sendG4Wait() {
    std::string wait_work;
    if (KlipperManager::instance().sendGcode("g4 p20\n", nullptr, 5L, &wait_work)) {
        fprintf(stderr, "[Unified Server] G4 wait command response: %s\n", wait_work.c_str());
    } else {
        fprintf(stderr, "[Unified Server] WARNING: Failed to send G4 wait command\n");
    }
}
//---归位
void runFinalHoming() {
    std::string homing_error;
    fprintf(stderr, "[Unified Server] Final homing (G28) before exit...\n");
    if (!KlipperManager::instance().forceHome(&homing_error)) {
        fprintf(stderr, "[Unified Server] Final homing failed: %s\n", homing_error.c_str());
    } else {
        fprintf(stderr, "[Unified Server] Final homing completed\n");
    }
}

bool sendTiltMonitorCommand(bool enable) {
    std::string response;
    std::string error;
    const int enable_flag = (enable ? 1 : 0);
    const std::string script =
        "TILT_MONITOR ENABLE=" + std::to_string(enable_flag) + "\n";
    if (KlipperManager::instance().sendGcode(script, &response, 10L, &error)) {
        fprintf(stderr,
                "[Unified Server] TILT_MONITOR ENABLE=%d command response: %s\n",
                enable_flag,
                response.c_str());
        return true;
    }

    fprintf(stderr,
            "[Unified Server] WARNING: Failed to send TILT_MONITOR ENABLE=%d: %s\n",
            enable_flag,
            error.empty() ? "sendGcode failed" : error.c_str());
    return false;
}

// ============================================================================
// Main Camera Retry Flow
// ============================================================================
//截图
bool captureMainCalibrationFrame(const MainCalibAttemptContext& attempt_ctx,
                                 CaptureLoopState* capture_state,
                                 int attempt,
                                 std::vector<unsigned char>& nv12_buffer,
                                 size_t& nv12_filled,
                                 uint64_t& frame_id) {
    fprintf(stderr, "[Unified Server] Main camera calibration attempt %d/%d: capturing frame...\n",
            attempt,
            kMainCalibMaxAttempts);

    //开缓冲。assign：清空并赋值
    nv12_buffer.assign(attempt_ctx.nv12_buf_size, 0);
    nv12_filled = 0;
    frame_id = 0;
    const bool copied = capture_state
        ? main_camera_frame_buffer_request_fresh_copy(capture_state,
                                                      nv12_buffer.data(),
                                                      attempt_ctx.nv12_buf_size,
                                                      &nv12_filled,
                                                      &frame_id,
                                                      kMainCameraRefreshTimeoutMs,
                                                      kMainCameraRefreshPollIntervalMs)
        : main_camera_frame_buffer_copy(
              nv12_buffer.data(), attempt_ctx.nv12_buf_size, &nv12_filled, &frame_id);
    if (!copied) {
        return false;
    }

    //检查数据是否有效
    return nv12_filled >=
        static_cast<size_t>(attempt_ctx.main_width) * static_cast<size_t>(attempt_ctx.main_height);
}
//根据错误进行下一步
MainCalibRetryDecision decideMainCalibRetry(int attempt,
                                            const MainCameraCalibResult& result,
                                            int& detection_count_mismatch_failures) 
{
    //为空的情况
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
        fprintf(stderr,
                "[Unified Server] Main camera calibration detected no boards/corners, aborting retry loop\n");
        ctx.sendErrorResponse("CALIBRATION_FAILED",
                              "Main camera calibration failed: empty detection");
        return CommandResult::ERROR_CONTINUE;

    case MainCalibRetryDecision::AbortDetectionCountMismatch:
        fprintf(stderr,
                "[Unified Server] Main camera calibration detected board/corner count mismatch twice, aborting retry loop\n");
        ctx.sendErrorResponse("CALIBRATION_FAILED",
                              "Main camera calibration failed: expected 6 boards and 294 corners");
        return CommandResult::ERROR_CONTINUE;

    case MainCalibRetryDecision::AbortAttemptsExhausted: {
        fprintf(stderr, "[Unified Server] Main camera calibration failed 3 times, aborting calibration\n");
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
        std::string fill_light_error;
        // if (!KlipperManager::instance().setFillLight(255, &fill_light_error)) {
        //     fprintf(stderr,
        //             "[Unified Server] Failed to set fill light to 60 before main camera capture: %s\n",
        //             fill_light_error.c_str());
        //     ctx.sendErrorResponse("CALIBRATION_FAILED",
        //                           fill_light_error.empty()
        //                               ? "Failed to set fill light before main camera capture"
        //                               : fill_light_error);
        //     return CommandResult::ERROR_CONTINUE;
        // }

        std::vector<unsigned char> nv12_buffer;
        size_t nv12_filled = 0;
        uint64_t frame_id = 0;
        CaptureLoopState* capture_state = ctx.app ? &ctx.app->capture_state : nullptr;
        if (!captureMainCalibrationFrame(
                attempt_ctx, capture_state, attempt, nv12_buffer, nv12_filled, frame_id)) {
            fprintf(stderr, "[Unified Server] Failed to capture valid latest main camera frame\n");
            ctx.sendErrorResponse("CAPTURE_FAILED", "Failed to copy latest main camera frame");
            return CommandResult::ERROR_CONTINUE;
        }

        fprintf(stderr, "[Unified Server] Main camera frame captured (frame_id=%llu), running calibration...\n",
                (unsigned long long)frame_id);
        main_result = runMainCameraCalib(nv12_buffer.data(),
                                         nv12_filled,
                                         attempt_ctx.main_width,
                                         attempt_ctx.main_height,
                                         attempt_ctx.session_result_dir,
                                         frame_id);

        if (main_result.success) {
            fprintf(stderr, "[Unified Server] Main camera calibration succeeded on attempt %d/%d\n",
                    attempt,
                    kMainCalibMaxAttempts);
            return CommandResult::SUCCESS;
        }

        fprintf(stderr, "[Unified Server] Main camera calibration failed on attempt %d/%d: %s\n",
                attempt,
                kMainCalibMaxAttempts,
                main_result.error.c_str());

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
    fprintf(stderr, "[Unified Server] Running focal autofocus only (CALIB-FOCALS)\n");
    Focalabfocal focal_service(ctx.app ? &ctx.app->capture_state : nullptr);
    double focal_long = 0.0;
    double focal_short = 0.0;
    std::string focal_error;
    if (!focal_service.runAutoFocusAndSave(focal_long, focal_short, focal_error)) {
        fprintf(stderr, "[Unified Server] CALIB-FOCALS failed: %s\n", focal_error.c_str());
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
        fprintf(stderr, "[Unified Server] Failed to send CALIB-FOCALS success response\n");
        return CommandResult::ERROR_DISCONNECT;
    }

    fprintf(stderr,
            "[Unified Server] CALIB-FOCALS completed: focal_long=%.2f focal_short=%.2f\n",
            focal_long,
            focal_short);
    return CommandResult::SUCCESS;
}

CommandResult handleTotalHighOnly(CommandContext& ctx) {
    fprintf(stderr, "[Unified Server] Running totalHigh measurement only (CALIB-1)\n");
    std::string total_high_error;
    TotalHighService total_high_service;
    double total_high = 0.0;
    if (!total_high_service.measure(total_high, total_high_error)) {
        fprintf(stderr, "[Unified Server] totalHigh measurement failed: %s\n", total_high_error.c_str());
        ctx.sendErrorResponse("TOTALHIGH_FAILED", total_high_error);
        runFinalHoming();
        return CommandResult::ERROR_CONTINUE;
    }

    if (!total_high_service.readTotalHigh(total_high, total_high_error)) {
        fprintf(stderr, "[Unified Server] Failed to read totalHigh: %s\n", total_high_error.c_str());
        ctx.sendErrorResponse("TOTALHIGH_READ_FAILED", total_high_error);
        return CommandResult::ERROR_CONTINUE;
    }

    char json_buf[128];
    snprintf(json_buf, sizeof(json_buf), "{\"TotalHigh\":%.3f}\n", total_high);
    if (!ctx.sendBinaryResponse(json_buf)) {
        fprintf(stderr, "[Unified Server] Failed to send totalHigh response\n");
        return CommandResult::ERROR_DISCONNECT;
    }

    fprintf(stderr, "[Unified Server] CALIB-1 completed: TotalHigh=%.3fmm, connection remains open\n",
            total_high);
    return CommandResult::SUCCESS;
}

// ============================================================================
// Vice Camera Calibration
// ============================================================================

ViceCameraCalibResult runViceCameraCalib(const ViceCameraConfig &config) {
    ViceCameraCalibResult result;
    result.error[0] = '\0';
    result.latest_dir[0] = '\0';

    fprintf(stderr, "[Vice Calib Thread]  Starting vice camera calibration...\n");
    try {
        ViceCameraService service(config);
        const ViceCameraResult vice_result = service.run(true);
        result.success = vice_result.success;
        if (!result.success) {
            snprintf(result.error, sizeof(result.error), "%s", vice_result.error.c_str());
            fprintf(stderr, "[Vice Calib Thread]  Vice camera calibration failed: %s\n", result.error);
        } else {
            snprintf(result.latest_dir, sizeof(result.latest_dir), "%s", vice_result.latest_dir.c_str());
            fprintf(stderr, "[Vice Calib Thread]  Vice camera calibration completed (dir: %s)\n",
                    result.latest_dir);
        }
    } catch (const std::exception &e) {
        snprintf(result.error, sizeof(result.error), "Failed to create vice camera service: %s", e.what());
        fprintf(stderr, "[Vice Calib Thread]  %s\n", result.error);
    }
    return result;
}

} // namespace

std::string CalibCommandHandler::getName() const {
    return "CALIB";
}

std::string CalibCommandHandler::getDescription() const {
    return "Camera calibration (main + vice cameras in parallel)";
}

bool CalibCommandHandler::isLongRunning() const {
    return true;
}

CommandResult CalibCommandHandler::execute(CommandContext& ctx) {
    fprintf(stderr, "[Unified Server] Processing CALIB command from %s...\n", ctx.client_ip.c_str());

    CommandResult result = CommandResult::ERROR_CONTINUE;
    sendTiltMonitorCommand(false);

    do {
        sendG4Wait();
        runFinalHoming();

        if (ctx.command == "CALIB-FOCALS") {
            result = handleFocalOnly(ctx);
            break;
        }

        if (ctx.command == "CALIB-1") {
            result = handleTotalHighOnly(ctx);
            break;
        }

        char session_id[64];
        char session_result_dir[512];
        if (!createCalibrationSession(
                session_id, sizeof(session_id), session_result_dir, sizeof(session_result_dir))) {
            ctx.sendErrorResponse("CALIBRATION_FAILED", "Failed to create calibration session");
            result = CommandResult::ERROR_CONTINUE;
            break;
        }

        fprintf(stderr, "[Unified Server] Created session: %s\n", session_id);

        ViceCameraConfig camera_config{};

        MainCameraCalibResult main_result;
        MainCalibAttemptContext main_attempt_ctx;
        main_attempt_ctx.main_width = INPUT_WIDTH;
        main_attempt_ctx.main_height = INPUT_HEIGHT;
        main_attempt_ctx.nv12_buf_size =
            static_cast<size_t>(main_attempt_ctx.main_width) *
            static_cast<size_t>(main_attempt_ctx.main_height) * 3 / 2;
        main_attempt_ctx.session_result_dir = session_result_dir;

        const CommandResult main_calib_cmd_result =
            tryCalibrateMainCamera(ctx, main_attempt_ctx, main_result);
        if (main_calib_cmd_result != CommandResult::SUCCESS) {
            result = main_calib_cmd_result;
            break;
        }

        fprintf(stderr, "[Unified Server] Starting XYoffset calibration...\n");
        double xy_dxPx = 0.0, xy_dyPx = 0.0;
        std::string xy_error;
        XYoffsetService xyoffset_service(ctx.app ? &ctx.app->capture_state : nullptr);
        if (!xyoffset_service.calibrate(xy_dxPx, xy_dyPx, xy_error)) {
            fprintf(stderr, "[Unified Server] XYoffset calibration failed: %s\n", xy_error.c_str());
            ctx.sendErrorResponse("CALIBRATION_FAILED", xy_error);
            runFinalHoming();
            result = CommandResult::ERROR_CONTINUE;
            break;
        }
        fprintf(stderr, "[Unified Server] XYoffset calibration completed: dxPx=%.2f, dyPx=%.2f\n", xy_dxPx, xy_dyPx);

        fprintf(stderr, "[Unified Server]  Starting vice camera calibration...\n");
        auto vice_future = std::async(std::launch::async,
                                      runViceCameraCalib,
                                      camera_config);

        fprintf(stderr, "[Unified Server]  Waiting for vice camera calibration to complete...\n");
        ViceCameraCalibResult vice_result = vice_future.get();

        if (!main_result.success) {
            fprintf(stderr, "[Unified Server]  Calibration failed:\n");
            fprintf(stderr, "  - Main camera: %s\n", main_result.error.c_str());
            if (!vice_result.success) {
                fprintf(stderr, "  - Vice camera: %s\n", vice_result.error);
            }

            char error_msg[1024];
            snprintf(error_msg, sizeof(error_msg), "Main: %s, Vice: %s",
                     main_result.success ? "OK" : main_result.error.c_str(),
                     vice_result.success ? "OK" : vice_result.error);
            ctx.sendErrorResponse("CALIBRATION_FAILED", error_msg);
            result = CommandResult::ERROR_CONTINUE;
            break;
        }

        fprintf(stderr, "[Unified Server]  Main camera calibration completed successfully\n");
        fprintf(stderr, "[Unified Server]   - Main camera: \n");
        if (!vice_result.success) {
            fprintf(stderr, "[Unified Server] Vice camera calibration failed: %s\n", vice_result.error);
            ctx.sendErrorResponse("CALIBRATION_FAILED", vice_result.error);
            runFinalHoming();
            result = CommandResult::ERROR_CONTINUE;
            break;
        }
        fprintf(stderr, "[Unified Server]   - Vice camera:  (dir: %s)\n", vice_result.latest_dir);

        {
            extern void cameraInit();
            cameraInit();
            std::fprintf(stderr, "[Unified Server]  Camera parameters reloaded with thickness compensation\n");
        }

        const char *success_json = "{\"status\":\"SUCCESS\"}\n";
        if (!ctx.sendBinaryResponse(success_json)) {
            fprintf(stderr, "[Unified Server] Failed to send success response\n");
            result = CommandResult::ERROR_DISCONNECT;
            break;
        }

        runFinalHoming();

        fprintf(stderr, "[Unified Server]  CALIB completed, connection remains open\n");
        result = CommandResult::SUCCESS;
    } while (false);

    sendTiltMonitorCommand(true);
    return result;
}
