#define _POSIX_C_SOURCE 199309L

#include "pipeline/2_dispatcher/capture_loop.h"

#include <stdio.h>

#include "config.h"
#include "core/video_frame_types.h"

namespace {

constexpr int kV4L2RecoverAfterFailures = 3;

// 性能统计
void logPeriodicStats(AppContext &app, CaptureLoopState &state) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  const long elapsed_sec = now.tv_sec - state.last_stats_time.tv_sec;
  if (elapsed_sec < 10) {
    return;
  }

  const double fps = (double)state.frame_count / elapsed_sec;

  fprintf(stderr, "\n");
  fprintf(stderr, "[STATS] Frames: %llu, FPS: %.2f, Skipped: %llu\n",
          (unsigned long long)state.frame_count, fps,
          (unsigned long long)state.skip_no_client_count);
  fprintf(
      stderr, "[STATS] Original: sent=%llu, skipped=%llu\n",
      (unsigned long long)app.server.original_stream.total_frames_sent,
      (unsigned long long)app.server.original_stream.backpressure_skip_count);
  fprintf(stderr, "[STATS] BEV: sent=%llu, skipped=%llu\n",
          (unsigned long long)app.server.bev_stream.total_frames_sent,
          (unsigned long long)app.server.bev_stream.backpressure_skip_count);

  fprintf(stderr, "\n");

  state.frame_count = 0;
  state.skip_no_client_count = 0;
  state.last_stats_time = now;
}

} // namespace

void runCaptureLoop(AppContext &app, volatile sig_atomic_t *running) {
  initCaptureLoopState(&app.capture_state);

  int consecutive_dequeue_failures = 0;

  // 死循环，持续取帧
  while (!running || *running) {

    // 临时托盘
    int index = -1;
    void *data = NULL;
    size_t size = 0;
    int dmabuf_fd = -1;

    // 死循环持续取帧
    //  “出参”（Out Parameter）模式，通过传指针来让子函数修改外面的变量。
    if (!v4l2_camera_dequeue(&app.cam, &index, &data, &size, &dmabuf_fd)) {
      fprintf(stderr, "[MAIN] V4L2 dequeue failed\n");
      ++consecutive_dequeue_failures;
      if (consecutive_dequeue_failures >= kV4L2RecoverAfterFailures) {
        fprintf(stderr,
                "[MAIN] Attempting V4L2 recovery after %d consecutive dequeue "
                "failures\n",
                consecutive_dequeue_failures);
        // 触发第一层恢复
        if (v4l2_camera_recover(&app.cam)) {
          fprintf(stderr, "[MAIN] V4L2 recovery completed\n");
        } else {
          fprintf(
              stderr,
              "[MAIN] V4L2 recovery failed; will retry after more timeouts\n");
        }
        consecutive_dequeue_failures = 0;
      }
      continue;
    }
    // 取帧成功，连续失败计数清零（说明摄像头已恢复正常）
    consecutive_dequeue_failures = 0;

    // 检查两路 RTSP 流是否各自有客户端在连接
    // appsrc != NULL 说明 GStreamer 推流管道已建立，有人在看
    const bool has_original_client =
        (app.server.original_stream.appsrc != NULL);
    const bool has_bev_client = (app.server.bev_stream.appsrc != NULL);
    // 只要有任意一路流有客户端，就认为"当前有人在看"
    const bool has_any_client = (has_original_client || has_bev_client);

    // ── 无客户端 → 省电跳过这一帧
    // ───────────────────────────────────────────── 没有任何 RTSP
    // 客户端时直接跳过，不分发。
    if (!has_any_client) {
      // 累计跳过的帧数，用于日志统计
      app.capture_state.skip_no_client_count++;

      if (app.capture_state.last_has_any_client) {
        // 上一帧还有客户端，这一帧突然没了 → 刚刚断开，打印一次"进入空闲"日志
        fprintf(stderr, "[MAIN] No clients. Idling...\n");
        // 记录进入空闲的时间点，作为下一次定时日志的起始基准
        clock_gettime(CLOCK_MONOTONIC, &app.capture_state.last_no_client_log);
      } else {
        // 已经持续无客户端，做日志节流：每隔 60 秒才打印一次"还在等待"
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        const long sec =
            now.tv_sec - app.capture_state.last_no_client_log.tv_sec;
        if (sec >= 60) {
          // 60 秒到了，打印一次累计跳帧数，然后更新基准时间
          fprintf(stderr,
                  "[MAIN] Waiting for clients... (skipped %llu frames)\n",
                  (unsigned long long)app.capture_state.skip_no_client_count);
          app.capture_state.last_no_client_log = now;
        }
      }

      // 这帧不需要处理，立刻还给驱动，让摄像头继续往里写下一帧
      v4l2_camera_queue(&app.cam, index);
      // 更新状态：当前无客户端
      app.capture_state.last_has_any_client = false;
      // 跳过后续的所有分发逻辑，回到循环顶部取下一帧
      continue;

    } else if (has_any_client && !app.capture_state.last_has_any_client) {
      // 上一帧无客户端，这一帧有了 →
      // 刚有人连上，打印"恢复推流"日志，重置跳帧计数
      fprintf(stderr, "[MAIN] Client connected. Resuming...\n");
      app.capture_state.skip_no_client_count = 0;
      app.capture_state.last_has_any_client = true;
    }

    // 从 RTSP Server 获取全局统一递增的帧序号
    const uint64_t current_frame_idx =
        dual_rtsp_server_get_next_frame_idx(&app.server);

    // 动态控制后端的工作线程（Consumer）是否需要干活：
    if (app.original_consumer) {
      // 原图消费者：只要有客户端就开启
      app.original_consumer->set_enabled(has_original_client);
    }
    if (app.bev_consumer) {
      // BEV 消费者：只要有客户端就开启
      app.bev_consumer->set_enabled(has_bev_client);
    }

    // 分发派单给后端的“消费者
    VideoFrameDesc frame{};
    frame.src_dmabuf_fd = dmabuf_fd;
    frame.src_buffer_index = index;
    frame.src_camera = &app.cam;
    frame.width = INPUT_WIDTH;
    frame.height = INPUT_HEIGHT;
    frame.stride = app.cam.y_stride;
    frame.size = size;
    frame.data = static_cast<const uint8_t *>(data);
    frame.frame_idx = current_frame_idx;

    // 分发路由
    app.video_router.dispatch_frame(frame);

    if (has_original_client && app.original_consumer && // 有客户端（门开着）
        app.original_consumer->enabled() &&             // 且消费者开启了
        app.original_consumer->last_dispatch_result() ==
            VideoConsumerDispatchResult::kDropped) { // 但帧还是被丢了！
      static uint64_t original_dispatch_drop_count = 0;
      ++original_dispatch_drop_count;
      if ((original_dispatch_drop_count % 300) == 1) {
        fprintf(stderr,
                "[MAIN] WARNING: Original consumer dropped frame "
                "(dmabuf_fd=%d, count=%llu)\n",
                frame.src_dmabuf_fd,
                (unsigned long long)original_dispatch_drop_count);
      }
    }

    // 归还的判断方式
    // 是否被bev消费了并且没有还，那么就不用还了，
    const bool bev_retains_source_buffer =
        app.bev_consumer && app.bev_consumer->enabled() &&
        app.bev_consumer->last_dispatch_result() ==
            VideoConsumerDispatchResult::kQueuedBorrowedDmabufFrame;
    if (!bev_retains_source_buffer) {
      v4l2_camera_queue(&app.cam, index);
    }

    app.capture_state.frame_count++;
    logPeriodicStats(app, app.capture_state);
  }
}
