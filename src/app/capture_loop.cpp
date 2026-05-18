#define _POSIX_C_SOURCE 199309L

#include "capture_loop.h"

#include <stdio.h>

#include "config.h"
#include "video/LatestNv12FrameBuffer.h"

namespace {

constexpr int kV4L2RecoverAfterFailures = 3;

}  // namespace

// 更新“主摄最新帧”共享缓存，供截图/检测等异步模块读取。
static void updateMainCameraLatestFrame(const void* data,
                                        size_t size,
                                        CaptureLoopState* state)
{
    if (!data || !state) {
        return;
    }

    if (!main_camera_frame_buffer_update((const uint8_t*)data, size, ++state->main_camera_frame_id)) {
        static uint64_t main_buffer_update_fail_count = 0;
        ++main_buffer_update_fail_count;
        if ((main_buffer_update_fail_count % 300) == 1) {
            fprintf(stderr, "[MAIN] WARNING: main camera frame buffer update failed (size=%zu, count=%llu)\n",
                    size, (unsigned long long)main_buffer_update_fail_count);
        }
    }
}

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

        // 无论是否有客户端，都维护一份最新主摄帧。
        updateMainCameraLatestFrame(data, size, &app.capture_state);

        const bool has_original_client = (app.server.original_stream.appsrc != NULL);
        const bool has_bev_client = (app.server.bev_stream.appsrc != NULL);
        const bool has_forced_bev_refresh = hasPendingBevRefreshRequest(&app.capture_state);
        const bool has_any_client = (has_original_client || has_bev_client);

        // 即使当前没有 RTSP 客户端，也继续更新主摄 latest 帧缓存。
        if (!has_any_client && !has_forced_bev_refresh) {
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

        const uint64_t current_frame_idx = dual_rtsp_server_get_next_frame_idx(&app.server);
        const bool use_dmabuf = (dmabuf_fd >= 0);
        bool original_holds_v4l2_buffer = false;

        // 原图流只走 DMA-BUF 零拷贝路径，由 worker 处理完后归还 V4L2 buffer。
        if (has_original_client) {
            if (use_dmabuf &&
                frame_queue_push_dmabuf(&app.original_queue, dmabuf_fd, index,
                                        INPUT_WIDTH, INPUT_HEIGHT, app.cam.y_stride,
                                        current_frame_idx, &app.cam)) {
                original_holds_v4l2_buffer = true;
            } else {
                if (use_dmabuf) {
                    static uint64_t original_enqueue_fail_count = 0;
                    ++original_enqueue_fail_count;
                    if ((original_enqueue_fail_count % 300) == 1) {
                        fprintf(stderr, "[MAIN] WARNING: original dmabuf queue push failed (count=%llu)\n",
                                (unsigned long long)original_enqueue_fail_count);
                    }
                } else {
                    static uint64_t original_non_dmabuf_drop_count = 0;
                    ++original_non_dmabuf_drop_count;
                    if ((original_non_dmabuf_drop_count % 300) == 1) {
                        fprintf(stderr, "[MAIN] WARNING: drop non-dmabuf frame for Original stream (count=%llu)\n",
                                (unsigned long long)original_non_dmabuf_drop_count);
                    }
                }
            }
        }

#ifdef ENABLE_PERFORMANCE_MONITORING
        struct timespec t_bev_push_start, t_bev_push_end;
        if (has_bev_client) {
            clock_gettime(CLOCK_MONOTONIC, &t_bev_push_start);
        }
#endif
        const bool consumed_bev_refresh = consumeBevRefreshRequest(&app.capture_state);
        if (has_bev_client || consumed_bev_refresh) {
            const bool force_process = consumed_bev_refresh && !has_bev_client;
            if (frame_queue_push(&app.bev_queue,
                                 (const uint8_t*)data,
                                 size,
                                 current_frame_idx,
                                 force_process) && force_process) {
                fprintf(stderr,
                        "[MAIN] Forced BEV refresh queued without RTSP client (frame_idx=%llu)\n",
                        (unsigned long long)current_frame_idx);
            }
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

        if (!original_holds_v4l2_buffer) {
            v4l2_camera_queue(&app.cam, index);
        }

        app.capture_state.frame_count++;
        logPeriodicStats(app, app.capture_state);
    }
}
