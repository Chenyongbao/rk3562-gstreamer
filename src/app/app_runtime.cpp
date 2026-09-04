#include "app/app_runtime.h"

#include <memory>
#include <string>

#include <curl/curl.h>

#include "config.h"
#include "core/logging.h"
#include "handlers/CalibCommandHandler.h"
#include "handlers/DetectCommandHandler.h"
#include "handlers/MetricsCommandHandler.h"
#include "handlers/PingCommandHandler.h"
#include "klipper/klipper_manager.h"
#include "pipeline/2_workers/stream_workers.h"
#include "yolo/embedded_model.h"
#include "yolo/yolo_model.h"

AppRuntime::AppRuntime(volatile sig_atomic_t *running_flag)
    : running_flag_(running_flag) {
  initAppContext(&app_);
}

bool AppRuntime::initCoreServices() {
  if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {
    spdlog::warn("[MAIN] WARNING: curl_global_init failed");
  } else {
    app_.curl_initialized = true;
  }

  KlipperManagerConfig cfg;
  cfg.host = KLIPPER_MOONRAKER_DEFAULT_HOST;
  cfg.port = KLIPPER_MOONRAKER_PORT;
  cfg.homing_timeout_ms = KLIPPER_HOMING_TIMEOUT_MS;
  std::string klipper_error;
  if (!KlipperManager::instance().initialize(cfg, &klipper_error)) {
    spdlog::warn("[MAIN] WARNING: KlipperManager init failed: {}", klipper_error);
  }
  // 组合根持有 Klipper 服务，供命令处理器通过 ctx.app->klipper 访问（依赖注入第一步）。
  app_.klipper = &KlipperManager::instance();

  return true;
}

bool AppRuntime::initMediaPipeline() {
  const char *bind_ip = RTSP_BIND_ADDRESS;
  if (!dual_rtsp_server_init(&app_.server, bind_ip, app_.klipper)) {
    spdlog::error("[MAIN] ERROR: RTSP server init failed");
    return false;
  }
  app_.rtsp_initialized = true;

  // 主相机/BEV 取帧都由 FrameProvider 按需分配（publish 时动态 resize），无需显式 init。

  if (!app_.capture.open(V4L2_DEVICE, INPUT_WIDTH, INPUT_HEIGHT)) {
    spdlog::error("[MAIN] ERROR: capture source open failed");
    return false;
  }
  app_.capture_opened = true;

  // 绑定原画线程上下文。
  app_.original_ctx.source = &app_.capture;
  app_.original_ctx.branch = GstSourceBranch::kOriginal;
  app_.original_ctx.stream = &app_.server.original_stream;
  app_.original_ctx.bev_processor = NULL;
  app_.original_ctx.running = running_flag_;
  app_.original_ctx.thread_name = "Original-Worker";
  app_.original_ctx.app = &app_;

  // 绑定 BEV 线程上下文。
  app_.bev_ctx.source = &app_.capture;
  app_.bev_ctx.branch = GstSourceBranch::kBev;
  app_.bev_ctx.stream = &app_.server.bev_stream;
  app_.bev_ctx.bev_processor = app_.server.bev_processor;
  app_.bev_ctx.running = running_flag_;
  app_.bev_ctx.thread_name = "BEV-Worker";
  app_.bev_ctx.app = &app_;

  return true;
}

bool AppRuntime::initCommandServer() {
  //yolo检测初始化
  app_.yolo_model = std::make_unique<YOLOModel>();
  if (app_.yolo_model->initFromMemory(g_embedded_model_data, g_embedded_model_size) != 0) {
    spdlog::warn("[MAIN] WARNING: YOLO init failed");
    app_.yolo_model.reset();
  } else {
    app_.yolo_model->setConfidence(YOLO_CONF_THRESHOLD);
    app_.yolo_model->setNMS(YOLO_NMS_THRESHOLD);
    spdlog::info("[MAIN]  YOLO initialized");
  }

  app_.unified_server =
      std::make_unique<UnifiedSocketServer>(YOLO_SOCKET_PORT, &app_);
  app_.unified_server->getRouter().registerHandler(
      std::make_unique<PingCommandHandler>());
  if (app_.yolo_model) {
    app_.unified_server->getRouter().registerHandler(
        std::make_unique<DetectCommandHandler>(app_.yolo_model.get()));
  }
  app_.unified_server->getRouter().registerHandler(
      std::make_unique<CalibCommandHandler>());
  app_.unified_server->getRouter().registerHandler(
      std::make_unique<MetricsCommandHandler>(
          std::string(REALLINK_CV_CONF_PATH),
          std::string(CALIB_RESULT_DIR) + "/" + std::string(CALIB_BIN_NAME)));

  if (!app_.unified_server->start()) {
    spdlog::error("[MAIN] ERROR: Failed to start unified server");
    return false;
  }

  spdlog::info("[MAIN] Unified server started on port {}", YOLO_SOCKET_PORT);
  return true;
}

void AppRuntime::shutdownApp() {
  spdlog::info("[MAIN] Cleaning up...");

  stopStreamWorkers(app_);

  if (app_.capture_opened) {
    app_.capture.close();
    app_.capture_opened = false;
  }

  // 主相机/BEV FrameProvider 释放（唤醒可能阻塞的取帧等待者）。
  app_.main_frame_provider.clear();
  app_.bev_frame_provider.clear();

  if (app_.unified_server) {
    if (app_.unified_server->isRunning()) {
      app_.unified_server->stop();
    }
    app_.unified_server.reset();
  }

  app_.yolo_model.reset();

  if (app_.rtsp_initialized) {
    dual_rtsp_server_cleanup(&app_.server);
    app_.rtsp_initialized = false;
  }

  KlipperManager::instance().shutdown();

  if (app_.curl_initialized) {
    curl_global_cleanup();
    app_.curl_initialized = false;
  }

  spdlog::info("[MAIN]  Clean shutdown complete");
}

int AppRuntime::run() {
  int exit_code = 0;
  if (!initCoreServices() || !initMediaPipeline() || !initCommandServer() ||
      !startStreamWorkers(app_)) {
    exit_code = -1;
  } else {
    runSnapshotLoop(app_, running_flag_);
  }

  shutdownApp();
  return exit_code;
}
