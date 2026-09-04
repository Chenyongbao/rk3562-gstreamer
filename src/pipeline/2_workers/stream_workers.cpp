#define _POSIX_C_SOURCE 199309L

#include "stream_workers.h"

#include <gst/gst.h>

#include <stdlib.h>
#include <unistd.h>

#include "config.h"
#include "core/logging.h"
#include "reallink_ogles/bev_api.h"

namespace {

//======================辅助函数：等待就绪=========================

// 阻塞等待 RTSP 管道就绪，避免推流器未初始化时盲目塞帧。
bool waitRtspReady(RTSPStreamer *stream, volatile sig_atomic_t *running) {
  if (!stream) {
    return false;
  }
  if (rtsp_streamer_is_ready(stream)) {
    return true;
  }
  for (int retry = 0; retry < 10 && (!running || *running); ++retry) {
    usleep(50000);
    if (rtsp_streamer_is_ready(stream)) {
      return true;
    }
  }
  return false;
}

//==========原画工作线程：拉原画分支（已缩放）→ 直推
// RTSP=========================

// 原画工作线程：从原画分支 appsink 拉已缩放好的 1280x960 帧，直推 RTSP。
// 缩放已在采集管线内由 RGA 元素完成，这里只做转发（真零拷贝，无私有池）。
void original_worker_thread(WorkerContext *ctx) {
  spdlog::info("[{}] Worker started", ctx->thread_name);

  uint64_t frame_idx = 0;
  while (ctx->running && *ctx->running) {
    GstFrame frame;
    // 从原画分支拉取已缩放的帧
    if (!ctx->source->dequeue(ctx->branch, &frame)) {
      continue;
    }
    const uint64_t current_idx = frame_idx++;

    // 线程安全地判断是否有人观看（appsrc 由 RTSP 回调线程写，见
    // rtsp_streamer.h）。
    if (!rtsp_streamer_has_appsrc(ctx->stream) ||
        !waitRtspReady(ctx->stream, ctx->running)) {
      GstV4L2Source::release(&frame);
      continue;
    }

    // 直推：push_buffer 接管并释放引用，下游（mpph264enc）用完自动 unref，
    // 届时 v4l2 源缓冲才被复用。之后无需再 release。
    rtsp_streamer_push_buffer(ctx->stream, frame.buffer, current_idx);
  }

  spdlog::info("[{}] Worker stopped", ctx->thread_name);
}

//=================bev_worker_thread BEV 线程=========

// BEV 工作线程：拉 BEV 分支 → OpenGL 生成鸟瞰图（鱼眼去畸变+透视重映射）→
// 发布到 BEV 帧服务 + 推 RTSP。
// 零拷贝设计：OpenGL 直接渲染进输出池的 dmabuf，RTSP 推流只传 fd、不拷数据；
// 快照/检测路径则把 dmabuf 读回 CPU（一次性读回，替代旧的三次拷贝）。
void bev_worker_thread(WorkerContext *ctx) {
  uint8_t *bev_output_buf = (uint8_t *)malloc(BEV_OUTPUT_NV12_SIZE);
  if (!bev_output_buf) {
    spdlog::error("[{}] ERROR: Buffer allocation failed", ctx->thread_name);
    return;
  }

  spdlog::info("[{}] Worker started", ctx->thread_name);

  // ====================================EGL
  // 上下文是线程独占的==========================================
  // 必须在当前线程绑定成功后，后续才能调用 GPU
  // 着色器执行 BEV 渲染。 若绑定失败，释放已分配的内存并退出线程。
  if (ctx->bev_processor && !bev_bind_context_to_thread(ctx->bev_processor)) {
    spdlog::error("[{}] ERROR: Failed to bind OpenGL context",
                  ctx->thread_name);
    free(bev_output_buf);
    return;
  }

  uint64_t frame_idx = 0;
  while (ctx->running && *ctx->running) {
    GstFrame frame;
    if (!ctx->source->dequeue(ctx->branch, &frame)) {
      continue;
    }
    const uint64_t current_idx = frame_idx++;

    // 线程安全地判断是否有人观看（appsrc 由 RTSP 回调线程写，见
    // rtsp_streamer.h）。
    const bool has_rtsp =
        (ctx->stream && rtsp_streamer_has_appsrc(ctx->stream));
    const bool refresh_pending = ctx->app && getPendingBevRefreshRequestCount(
                                                 &ctx->app->capture_state) > 0;
    // 是否有消费者阻塞在 grab/grabNewerThan 等新鲜帧（决定是否值得做 GPU→CPU
    // 读回+发布）。
    const bool provider_wants =
        ctx->app && ctx->app->bev_frame_provider.hasWaiter();

    // 无人消费（无客户端、无刷新请求、无等待者）时，跳过（省算力）。
    if (!ctx->bev_processor ||
        (!has_rtsp && !refresh_pending && !provider_wants)) {
      GstV4L2Source::release(&frame);
      continue;
    }
    // 仅在推流时才要求管道就绪；纯刷新只需产出 BEV 帧。
    if (has_rtsp && (!ctx->stream->use_dmabuf ||
                     !waitRtspReady(ctx->stream, ctx->running))) {
      GstV4L2Source::release(&frame);
      continue;
    }

    // 渲染进输出池当前槽位，拿到单块 NV12 dmabuf fd（零 CPU 拷贝）。
    int fd = -1, stride_y = 0, stride_uv = 0;
    // 将 V4L2 摄像头采集出的原始 NV12 frame.dmabuf_fd 传入，GPU
    // 处理完成后输出单块 NV12 格式的 fd 以及跨距 stride_y / stride_uv
    const bool process_ok = bev_process_frame_dmabuf_pooled(
        ctx->bev_processor, frame.dmabuf_fd, frame.stride, frame.size, &fd,
        &stride_y, &stride_uv);

    // 源帧处理完毕即可归还（unref GstBuffer）。
    GstV4L2Source::release(&frame);
    if (!process_ok) {
      continue;
    }

    // 只有当有模块真正需要一张新鲜图片时，才触发截帧动作：
    // 1. refresh_pending：外部发来了拍照/检测的刷新请求
    // 2. provider_wants：有外部线程正在阻塞等待新帧（hasWaiter）
    if (ctx->app && (refresh_pending || provider_wants)) {
      // 步骤 A：把当前这一帧从 GPU 显存读回到 CPU 内存
      if (bev_read_last_output(ctx->bev_processor, bev_output_buf,
                               BEV_OUTPUT_NV12_SIZE)) {
        // 步骤 B：发布给 FrameProvider，供检测/拍照/其他服务使用
        ctx->app->bev_frame_provider.publish(
            bev_output_buf, BEV_OUTPUT_NV12_SIZE, current_idx, BEV_OUTPUT_WIDTH,
            BEV_OUTPUT_HEIGHT, BEV_OUTPUT_WIDTH);
      }
      consumeBevRefreshRequest(&ctx->app->capture_state);
    }

    // 推 RTSP：直接把单块 NV12 dmabuf fd 交给编码器（真零拷贝，无私有池、无
    // memcpy）。
    if (has_rtsp) {
      rtsp_streamer_push_dmabuf_nv12_single(ctx->stream, fd, stride_y,
                                            stride_uv, BEV_OUTPUT_WIDTH,
                                            BEV_OUTPUT_HEIGHT, current_idx);
    }

    // 归还输出池槽位（结束本次读同步区间）。
    bev_release_output_pool_slot(ctx->bev_processor);
  }

  free(bev_output_buf);
  spdlog::info("[{}] Worker stopped", ctx->thread_name);
}

} // namespace

//=================外部接口：启动/停止线程、运行快照循环=========================

// 启动所有的工作线程（包括原画处理线程和 BEV 处理线程）
bool startStreamWorkers(AppContext &app) {
  if (!app.original_ctx.running) {
    spdlog::error("[MAIN] ERROR: running flag is null before worker start");
    return false;
  }

  app.original_ctx.app = &app;
  app.bev_ctx.app = &app;

  spdlog::info("[MAIN] Creating worker threads...");

  try {
    app.original_thread =
        std::thread(original_worker_thread, &app.original_ctx);
    app.original_thread_started = true;
  } catch (...) {
    spdlog::error("[MAIN] ERROR: Failed to create original thread");
    return false;
  }

  try {
    app.bev_thread = std::thread(bev_worker_thread, &app.bev_ctx);
    app.bev_thread_started = true;
  } catch (...) {
    spdlog::error("[MAIN] ERROR: Failed to create BEV thread");
    *(app.original_ctx.running) = 0;
    if (app.original_thread.joinable()) {
      app.original_thread.join();
    }
    app.original_thread_started = false;
    return false;
  }

  return true;
}

// 停止所有的工作线程，释放关联的资源
void stopStreamWorkers(AppContext &app) {
  if (!(app.original_thread_started || app.bev_thread_started)) {
    return;
  }

  if (app.original_ctx.running) {
    *(app.original_ctx.running) = 0;
  }

  if (app.original_thread_started && app.original_thread.joinable()) {
    app.original_thread.join();
    app.original_thread_started = false;
  }
  if (app.bev_thread_started && app.bev_thread.joinable()) {
    app.bev_thread.join();
    app.bev_thread_started = false;
  }
  spdlog::info("[MAIN] All worker threads stopped");
}

//============快照循环（跑主线程)================

// 快照循环：从 snapshot 分支拉帧，按需把最新原始帧喂给 FrameProvider。
// 按需优化：只有 FrameProvider 有消费者在等（hasWaiter）时才做昂贵的
// CPU 映射 + memcpy，避免每帧无谓拷贝 ~19MB 的原始帧。
void runSnapshotLoop(AppContext &app, volatile sig_atomic_t *running) {
  while (!running || *running) {
    GstFrame frame;
    // 从采集队列中取出一帧（阻塞式，取不到时返回 false）。
    if (!app.capture.dequeue(GstSourceBranch::kSnapshot, &frame)) {
      continue;
    }

    // 有消费者等待时才值得做 CPU 映射 + 拷贝。
    if (app.main_frame_provider.hasWaiter()) {
      GstMapInfo map;
      if (gst_buffer_map(frame.buffer, &map, GST_MAP_READ)) {
        // 只发布"紧凑 NV12"所需的字节数（width*height*3/2），与旧实现一致：
        // 若驱动有 stride 补齐，多出的行填充字节不拷贝给消费者。
        const size_t packed =
            static_cast<size_t>(frame.width) * frame.height * 3 / 2;
        if (frame.size >= packed) {
          // 帧序号自增：消费者用 frame_id 判断"新旧"。
          const uint64_t next_id = app.main_frame_provider.latestFrameId() + 1;
          app.main_frame_provider.publish(map.data, packed, next_id,
                                          frame.width, frame.height,
                                          frame.stride);
        }
        gst_buffer_unmap(frame.buffer, &map);
      }
    }

    GstV4L2Source::release(&frame);
  }
}
