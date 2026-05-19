#define _POSIX_C_SOURCE 199309L

#include "capture_loop.h"

#include <stdio.h>

#include "config.h"
#include "video/video_frame_types.h"

namespace {

constexpr int kV4L2RecoverAfterFailures = 3;

}  // namespace
// 每 10 秒输出一次运行统计，便于在线观测吞吐与丢帧情况。
static void logPeriodicStats(AppContext& app, CaptureLoopState& state)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    const long elapsed_sec = now.tv_sec - state.last_stats_time.tv_sec;
    if (elapsed_sec < 10) {
        return;
    }

    const double fps = (double)state.frame_count / elapsed_sec;

#ifdef ENABLE_PERFORMANCE_MONITORING
    fprintf(stderr, "\n");
    fprintf(stderr, "════════════════════════════════════════════════════════════════════\n");
#else
    fprintf(stderr, "\n");
#endif
    fprintf(stderr, "[STATS] Frames: %llu, FPS: %.2f, Skipped: %llu\n",
            (unsigned long long)state.frame_count, fps,
            (unsigned long long)state.skip_no_client_count);
    fprintf(stderr, "[STATS] Original: sent=%llu, skipped=%llu\n",
            (unsigned long long)app.server.original_stream.total_frames_sent,
            (unsigned long long)app.server.original_stream.backpressure_skip_count);
    fprintf(stderr, "[STATS] BEV: sent=%llu, skipped=%llu\n",
            (unsigned long long)app.server.bev_stream.total_frames_sent,
            (unsigned long long)app.server.bev_stream.backpressure_skip_count);

#ifdef ENABLE_PERFORMANCE_MONITORING
    fprintf(stderr, "────────────────────────────────────────────────────────────────────\n");
    fprintf(stderr, "  Worker threads: Parallel processing (Original + BEV)\n");
    fprintf(stderr, "────────────────────────────────────────────────────────────────────\n");
    fprintf(stderr, "[QUEUE] Original: count=%d, dropped=%llu\n",
            frame_queue_count(&app.original_queue),
            (unsigned long long)app.original_queue.dropped_frames);
    fprintf(stderr, "[QUEUE] BEV: count=%d, dropped=%llu\n",
            frame_queue_count(&app.bev_queue),
            (unsigned long long)app.bev_queue.dropped_frames);
    perf_stats_print("Main->BEV queue push", &app.perf_main_bev_queue_push);
    perf_stats_print("BEV pop (wait+pop)", &app.perf_bev_pop);
    perf_stats_print("BEV bev_process_frame", &app.perf_bev_process);
    perf_stats_print("BEV acquire_output_dmabuf", &app.perf_bev_acquire);
    perf_stats_print("BEV rtsp_push_dmabuf_nv12", &app.perf_bev_push);
    fprintf(stderr, "════════════════════════════════════════════════════════════════════\n");
#endif
    fprintf(stderr, "\n");

    state.frame_count = 0;
    state.skip_no_client_count = 0;
    state.last_stats_time = now;
}

void runCaptureLoop(AppContext& app, volatile sig_atomic_t* running)
{
    // 启动前重置统计状态，防止继承旧计数。
    initCaptureLoopState(&app.capture_state);

#ifdef ENABLE_PERFORMANCE_MONITORING
    fprintf(stderr, " Performance Monitoring: ENABLED\n");
    perf_stats_init(&app.perf_main_bev_queue_push);
    perf_stats_init(&app.perf_bev_pop);
    perf_stats_init(&app.perf_bev_process);
    perf_stats_init(&app.perf_bev_acquire);
    perf_stats_init(&app.perf_bev_push);
#endif
    fprintf(stderr, "========================================\n");
    fprintf(stderr, "Press Ctrl+C to stop\n\n");

    int consecutive_dequeue_failures = 0;

    // 主循环：采集 -> 客户端判断 -> 入队 -> 回收 -> 统计。
    while (!running || *running) {
        int index = -1;
        void* data = NULL;
        size_t size = 0;
        int dmabuf_fd = -1;

        if (!v4l2_camera_dequeue(&app.cam, &index, &data, &size, &dmabuf_fd)) {
            fprintf(stderr, "[MAIN] V4L2 dequeue failed\n");
            ++consecutive_dequeue_failures;
            if (consecutive_dequeue_failures >= kV4L2RecoverAfterFailures) {
                fprintf(stderr,
                        "[MAIN] Attempting V4L2 recovery after %d consecutive dequeue failures\n",
                        consecutive_dequeue_failures);
                if (v4l2_camera_recover(&app.cam)) {
                    fprintf(stderr, "[MAIN] V4L2 recovery completed\n");
                } else {
                    fprintf(stderr, "[MAIN] V4L2 recovery failed; will retry after more timeouts\n");
                }
                consecutive_dequeue_failures = 0;
            }
            continue;
        }
        consecutive_dequeue_failures = 0;

        const bool has_pending_main_camera_refresh =
            hasPendingMainCameraRefreshRequest(&app.capture_state);

        const bool has_original_client = (app.server.original_stream.appsrc != NULL);
        const bool has_bev_client = (app.server.bev_stream.appsrc != NULL);
        const bool has_forced_bev_refresh = hasPendingBevRefreshRequest(&app.capture_state);
        const bool has_any_client = (has_original_client || has_bev_client);

        // 即使当前没有 RTSP 客户端，也继续更新主摄 latest 帧缓存。
        if (!has_any_client && !has_forced_bev_refresh && !has_pending_main_camera_refresh) {
            app.capture_state.skip_no_client_count++;
            if (app.capture_state.last_has_any_client) {
                fprintf(stderr, "[MAIN] No clients. Idling...\n");
                clock_gettime(CLOCK_MONOTONIC, &app.capture_state.last_no_client_log);
            } else {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                const long sec = now.tv_sec - app.capture_state.last_no_client_log.tv_sec;
                if (sec >= 60) {
                    fprintf(stderr, "[MAIN] Waiting for clients... (skipped %llu frames)\n",
                            (unsigned long long)app.capture_state.skip_no_client_count);
                    app.capture_state.last_no_client_log = now;
                }
            }
            v4l2_camera_queue(&app.cam, index);
            app.capture_state.last_has_any_client = false;
            continue;
        } else if (has_any_client && !app.capture_state.last_has_any_client) {
            fprintf(stderr, "[MAIN] Client connected. Resuming...\n");
            app.capture_state.skip_no_client_count = 0;
            app.capture_state.last_has_any_client = true;
        } else if (!has_any_client) {
            app.capture_state.last_has_any_client = false;
        }

        const bool consumed_bev_refresh = consumeBevRefreshRequest(&app.capture_state);
        const bool force_process = consumed_bev_refresh && !has_bev_client;
        const uint64_t current_frame_idx = dual_rtsp_server_get_next_frame_idx(&app.server);

        if (app.original_consumer) {
            app.original_consumer->set_enabled(has_original_client);
        }
        if (app.bev_consumer) {
            app.bev_consumer->set_enabled(has_bev_client || consumed_bev_refresh);
        }

        VideoFrameDesc frame{};
        frame.src_dmabuf_fd = dmabuf_fd;
        frame.width = INPUT_WIDTH;
        frame.height = INPUT_HEIGHT;
        frame.stride = app.cam.y_stride;
        frame.size = size;
        frame.data = static_cast<const uint8_t*>(data);
        frame.frame_idx = current_frame_idx;
        frame.force_process = force_process;

        const VideoRouterDispatchSummary summary = app.video_router.dispatch_frame(frame);

        if (has_original_client &&
            app.original_consumer &&
            app.original_consumer->enabled() &&
            app.original_consumer->last_dispatch_result() == VideoConsumerDispatchResult::kDropped) {
            static uint64_t original_dispatch_drop_count = 0;
            ++original_dispatch_drop_count;
            if ((original_dispatch_drop_count % 300) == 1) {
                fprintf(stderr,
                        "[MAIN] WARNING: Original consumer dropped frame (dmabuf_fd=%d, count=%llu)\n",
                        frame.src_dmabuf_fd,
                        (unsigned long long)original_dispatch_drop_count);
            }
        }

#ifdef ENABLE_PERFORMANCE_MONITORING
        struct timespec t_bev_push_start, t_bev_push_end;
        if (has_bev_client || consumed_bev_refresh) {
            clock_gettime(CLOCK_MONOTONIC, &t_bev_push_start);
        }
#endif
        if ((has_bev_client || consumed_bev_refresh) && summary.delivered_count > 0 && force_process) {
            fprintf(stderr,
                    "[MAIN] Forced BEV refresh queued without RTSP client (frame_idx=%llu)\n",
                    (unsigned long long)current_frame_idx);
        }
#ifdef ENABLE_PERFORMANCE_MONITORING
        if (has_bev_client || consumed_bev_refresh) {
            clock_gettime(CLOCK_MONOTONIC, &t_bev_push_end);
            int64_t bev_push_us = (int64_t)(t_bev_push_end.tv_sec - t_bev_push_start.tv_sec) * 1000000LL +
                                  (int64_t)(t_bev_push_end.tv_nsec - t_bev_push_start.tv_nsec) / 1000LL;
            if (bev_push_us < 0) {
                bev_push_us = 0;
            }
            perf_stats_add(&app.perf_main_bev_queue_push, (uint64_t)bev_push_us);
        }
#endif
        v4l2_camera_queue(&app.cam, index);

        app.capture_state.frame_count++;
        logPeriodicStats(app, app.capture_state);
    }
}
