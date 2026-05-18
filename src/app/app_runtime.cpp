#include "app_runtime.h"

#include <cstdio>
#include <memory>
#include <string>

#include <curl/curl.h>

#include "config.h"
#include "rtsp/rga_processor.h"
#include "video/LatestNv12FrameBuffer.h"
#include "camera_calibreation/klipper/klipper_manager.h"
#include "handlers/PingCommandHandler.h"
#include "handlers/DetectCommandHandler.h"
#include "handlers/CalibCommandHandler.h"
#include "handlers/QuitCommandHandler.h"
#include "stream_workers.h"
#include "capture_loop.h"

AppRuntime::AppRuntime(volatile sig_atomic_t* running_flag)
    : running_flag_(running_flag)
{
    // 统一初始化上下文，避免未初始化字段带来的清理风险。
    initAppContext(&app_);
}

bool AppRuntime::initCoreServices()
{
    // 初始化 curl（失败仅告警，不阻塞主流程）。
    if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {
        fprintf(stderr, "[MAIN] WARNING: curl_global_init failed\n");
    } else {
        app_.curl_initialized = true;
    }

    // Klipper/Moonraker 连接参数使用固定配置。
    const char* moonraker_host = KLIPPER_MOONRAKER_DEFAULT_HOST;
    fprintf(stderr, "[MAIN] Using fixed Klipper/Moonraker host: %s\n", moonraker_host);

    KlipperManagerConfig cfg;
    cfg.host = moonraker_host;
    cfg.port = KLIPPER_MOONRAKER_PORT;
    cfg.homing_timeout_ms = KLIPPER_HOMING_TIMEOUT_MS;
    std::string klipper_error;
    if (!KlipperManager::instance().initialize(cfg, &klipper_error)) {
        fprintf(stderr, "[MAIN] WARNING: KlipperManager init failed: %s\n", klipper_error.c_str());
    }

    // RGA 是后续图像缩放/转换的关键能力，失败则直接退出。
    if (!rga_processor_init()) {
        fprintf(stderr, "[MAIN] ERROR: RGA init failed\n");
        return false;
    }
    app_.rga_initialized = true;

    return true;
}

bool AppRuntime::initMediaPipeline()
{
    // 先启动 RTSP 服务端，再逐步初始化缓冲与采集链路。
    const char* bind_ip = RTSP_BIND_ADDRESS;
    if (!dual_rtsp_server_init(&app_.server, bind_ip)) {
        fprintf(stderr, "[MAIN] ERROR: RTSP server init failed\n");
        return false;
    }
    app_.rtsp_initialized = true;

    if (!bev_frame_buffer_init(BEV_OUTPUT_WIDTH, BEV_OUTPUT_HEIGHT)) {
        fprintf(stderr, "[MAIN] ERROR: 初始化俯视图帧缓冲区失败\n");
        return false;
    }
    app_.bev_buffer_initialized = true;

    if (!main_camera_frame_buffer_init(INPUT_WIDTH, INPUT_HEIGHT)) {
        fprintf(stderr, "[MAIN] ERROR: failed to initialize main camera frame buffer\n");
        return false;
    }
    app_.main_buffer_initialized = true;

    if (!v4l2_camera_open(&app_.cam, V4L2_DEVICE, INPUT_WIDTH, INPUT_HEIGHT, V4L2_BUFFER_COUNT)) {
        fprintf(stderr, "[MAIN] ERROR: V4L2 open failed\n");
        return false;
    }
    app_.v4l2_opened = true;

    if (!v4l2_camera_start(&app_.cam)) {
        fprintf(stderr, "[MAIN] ERROR: V4L2 start failed\n");
        return false;
    }

    if (!frame_queue_init(&app_.original_queue, INPUT_WIDTH * INPUT_HEIGHT * 3 / 2)) {
        fprintf(stderr, "[MAIN] ERROR: Original queue init failed\n");
        return false;
    }
    app_.original_queue_initialized = true;

    if (!frame_queue_init(&app_.bev_queue, INPUT_WIDTH * INPUT_HEIGHT * 3 / 2)) {
        fprintf(stderr, "[MAIN] ERROR: BEV queue init failed\n");
        return false;
    }
    app_.bev_queue_initialized = true;

    // 绑定原图线程上下文。视频流结构体
    app_.original_ctx.input_queue = &app_.original_queue;
    app_.original_ctx.stream = &app_.server.original_stream;
    app_.original_ctx.bev_processor = NULL;
    app_.original_ctx.v4l2_cam = &app_.cam;
    app_.original_ctx.running = running_flag_;
    app_.original_ctx.thread_name = "Original-Worker";
    app_.original_ctx.app = &app_;

    // 绑定 BEV 线程上下文。
    app_.bev_ctx.input_queue = &app_.bev_queue;
    app_.bev_ctx.stream = &app_.server.bev_stream;
    app_.bev_ctx.bev_processor = app_.server.bev_processor;
    app_.bev_ctx.v4l2_cam = &app_.cam;
    app_.bev_ctx.running = running_flag_;
    app_.bev_ctx.thread_name = "BEV-Worker";
    app_.bev_ctx.app = &app_;

    return true;
}

bool AppRuntime::initCommandServer()
{
    // YOLO 为可选能力：初始化失败时保留服务主体可运行。
    app_.yolo_handle = yolo_init();
    if (!app_.yolo_handle) {
        fprintf(stderr, "[MAIN] WARNING: YOLO init failed\n");
        fprintf(stderr, "[MAIN] Continuing without YOLO detection service\n");
    } else {
        yolo_set_confidence(app_.yolo_handle, YOLO_CONF_THRESHOLD);
        yolo_set_nms(app_.yolo_handle, YOLO_NMS_THRESHOLD);
        fprintf(stderr, "[MAIN]  YOLO initialized\n");
    }

    // 统一命令服务：PING/DETECT/CALIB/QUIT 共用同一端口。
    app_.unified_server = std::make_unique<UnifiedSocketServer>(YOLO_SOCKET_PORT, &app_);
    app_.unified_server->getRouter().registerHandler(std::make_shared<PingCommandHandler>());
    app_.unified_server->getRouter().registerHandler(std::make_shared<QuitCommandHandler>());
    if (app_.yolo_handle) {
        app_.unified_server->getRouter().registerHandler(std::make_shared<DetectCommandHandler>(app_.yolo_handle));
    } else {
        fprintf(stderr, "[MAIN] WARNING: YOLO handle is NULL, DETECT command disabled\n");
    }
    app_.unified_server->getRouter().registerHandler(std::make_shared<CalibCommandHandler>());

    if (!app_.unified_server->start()) {
        fprintf(stderr, "[MAIN] ERROR: Failed to start unified server\n");
        return false;
    }

    fprintf(stderr, "[MAIN] Unified server started on port %d (DETECT + CALIB + PING + QUIT)\n", YOLO_SOCKET_PORT);
    return true;
}

void AppRuntime::shutdownApp()
{
    fprintf(stderr, "\n[MAIN] Cleaning up...\n");

    // 先停工作线程，避免后续资源在被使用时被释放。
    stopStreamWorkers(app_);

    // 队列与采集资源清理。
    if (app_.original_queue_initialized) {
        frame_queue_cleanup(&app_.original_queue);
        app_.original_queue_initialized = false;
    }
    if (app_.bev_queue_initialized) {
        frame_queue_cleanup(&app_.bev_queue);
        app_.bev_queue_initialized = false;
    }

    if (app_.v4l2_opened) {
        v4l2_camera_close(&app_.cam);
        app_.v4l2_opened = false;
    }

    if (app_.main_buffer_initialized) {
        main_camera_frame_buffer_cleanup();
        app_.main_buffer_initialized = false;
    }
    if (app_.bev_buffer_initialized) {
        bev_frame_buffer_cleanup();
        app_.bev_buffer_initialized = false;
    }

    // 命令服务与推理句柄清理。
    if (app_.unified_server) {
        if (app_.unified_server->isRunning()) {
            app_.unified_server->stop();
        }
        app_.unified_server.reset();
    }

    if (app_.yolo_handle) {
        yolo_cleanup(app_.yolo_handle);
        app_.yolo_handle = nullptr;
    }

    // 媒体、图像处理与外部服务清理。
    if (app_.rtsp_initialized) {
        dual_rtsp_server_cleanup(&app_.server);
        app_.rtsp_initialized = false;
    }

    if (app_.rga_initialized) {
        rga_processor_cleanup();
        app_.rga_initialized = false;
    }

    KlipperManager::instance().shutdown();

    if (app_.curl_initialized) {
        curl_global_cleanup();
        app_.curl_initialized = false;
    }

    fprintf(stderr, "[MAIN]  Clean shutdown complete\n");
}

int AppRuntime::run()
{
    // 串行初始化；任一步失败都进入统一清理分支。
    int exit_code = 0;
    if (!initCoreServices() ||
        !initMediaPipeline() ||
        !initCommandServer() ||
        !startStreamWorkers(app_)) {
        exit_code = -1;
    } else {
        runCaptureLoop(app_, running_flag_);
    }

    shutdownApp();
    return exit_code;
}
