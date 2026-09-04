#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include <cstring>
#include <memory>
#include <signal.h>
#include <thread>

#include "app/capture_state.h"
#include "pipeline/1_source/gst_v4l2_source.h"
#include "pipeline/3_sink/rtsp/dual_rtsp_server.h"
#include "pipeline/common/frame_provider.h"
#include "protocol/UnifiedSocketServer.h"
#include "yolo/yolo_model.h"

struct AppContext;

class KlipperManager;

// 视频流处理线程上下文：从采集源的指定 tee 分支拉帧并加工。
typedef struct {
  GstV4L2Source *source;  // 采集源
  GstSourceBranch branch; // 拉取哪个 tee 分支
  RTSPStreamer *stream;   // 目标 RTSP 流（snapshot 分支为 NULL）
  void *bev_processor;    // BEV 处理器（仅 bev 分支使用）
  volatile sig_atomic_t *running;
  const char *thread_name;
  struct AppContext *app;
} WorkerContext;

// 依赖注入，把KlipperManager打包在一起，任何模块只需要拿到AppContext就可以，而不是调用一个全新的实例
// 应用运行期全局状态与资源句柄（组合根）。
typedef struct AppContext {
  GstV4L2Source capture; // GStreamer v4l2src + tee 采集源
  DualRTSPServer server; // gst-rtsp-server（原画 + BEV 两路）
  CaptureLoopState capture_state;
  FrameProvider main_frame_provider; // 主相机"按需取帧"服务
  FrameProvider bev_frame_provider; // BEV 俯视图帧服务（取代旧的 bev_frame_buffer）

  WorkerContext original_ctx;
  WorkerContext bev_ctx;

  std::thread original_thread;
  std::thread bev_thread;
  bool original_thread_started;
  bool bev_thread_started;

  std::unique_ptr<YOLOModel> yolo_model;
  std::unique_ptr<UnifiedSocketServer> unified_server;
  KlipperManager* klipper = nullptr; // 组合根持有的 Klipper 服务（依赖注入，取代各处
                                     // KlipperManager::instance()）

  bool capture_opened;
  bool curl_initialized;
  bool rtsp_initialized;
} AppContext;

static inline void initAppContext(AppContext *app) {
  if (!app) {
    return;
  }
  // 注意：不能 memset app->server——DualRTSPServer 内含 GMutex（RTSPStreamer），
  // 清零互斥锁属未定义行为；其初始化由 dual_rtsp_server_init 负责。
  std::memset(&app->original_ctx, 0, sizeof(app->original_ctx));
  std::memset(&app->bev_ctx, 0, sizeof(app->bev_ctx));

  app->yolo_model.reset();
  app->unified_server.reset();
  app->original_thread_started = false;
  app->bev_thread_started = false;
  app->curl_initialized = false;
  app->rtsp_initialized = false;
  app->capture_opened = false;

  initCaptureLoopState(&app->capture_state);
}

#endif // APP_CONTEXT_H
