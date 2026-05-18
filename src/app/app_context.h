#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include <stdint.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <memory>
#include <atomic>
#include <cstring>
#include <limits>
#include <cstdio>

#include "video/v4l2_capture.h"
#include "core/frame_queue.h"
#include "rtsp/dual_rtsp_server.h"
#include "yolodetect/yolo_wrapper.h"
#include "protocol/UnifiedSocketServer.h"

// 全局应用上下文前置声明，用于线程上下文中的回指。
struct AppContext;

// 简单耗时统计结构（单位：微秒）。
typedef struct {
    uint64_t count;
    uint64_t total_us;
    uint64_t min_us;
    uint64_t max_us;
} PerfStats;

// 初始化性能统计数据。
static inline void perf_stats_init(PerfStats* stats)
{
    if (!stats) {
        return;
    }
    stats->count = 0;
    stats->total_us = 0;
    stats->min_us = std::numeric_limits<uint64_t>::max();
    stats->max_us = 0;
}

// 追加一次耗时样本。
static inline void perf_stats_add(PerfStats* stats, uint64_t time_us)
{
    if (!stats) {
        return;
    }
    stats->count++;
    stats->total_us += time_us;
    if (time_us < stats->min_us) stats->min_us = time_us;
    if (time_us > stats->max_us) stats->max_us = time_us;
}

// 打印统计结果（均值/最小/最大）。
static inline void perf_stats_print(const char* name, const PerfStats* stats)
{
    if (!stats) {
        return;
    }
    if (stats->count == 0) {
        fprintf(stderr, "[PERF] %s: No data\n", name ? name : "unknown");
        return;
    }

    const double avg_ms = (stats->total_us / (double)stats->count) / 1000.0;
    const double min_ms = stats->min_us / 1000.0;
    const double max_ms = stats->max_us / 1000.0;

    fprintf(stderr, "[PERF] %-22s: avg=%6.2fms, min=%6.2fms, max=%6.2fms, samples=%llu\n",
            name ? name : "unknown",
            avg_ms, min_ms, max_ms,
            (unsigned long long)stats->count);
}

// 视频流处理线程上下文，包含必要的资源句柄和状态指针。
typedef struct {
    FrameQueue* input_queue;
    RTSPStreamer* stream;
    void* bev_processor;
    V4L2Camera* v4l2_cam;
    volatile sig_atomic_t* running;
    const char* thread_name;
    struct AppContext* app;
} WorkerContext;

// 主采集循环状态：用于统计、日志节流及帧序号维护。
typedef struct {
    uint64_t frame_count;
    uint64_t skip_no_client_count;
    uint64_t main_camera_frame_id;
    std::atomic<uint32_t> pending_bev_refresh_requests;
    std::atomic<uint64_t> latest_bev_frame_id;
    struct timespec last_stats_time;
    bool last_has_any_client;
    struct timespec last_no_client_log;
} CaptureLoopState;

// 应用运行期全局状态与资源句柄。
typedef struct AppContext {
    V4L2Camera cam;
    FrameQueue original_queue;
    FrameQueue bev_queue;
    DualRTSPServer server;
    YOLOHandle yolo_handle;
    std::unique_ptr<UnifiedSocketServer> unified_server;

    WorkerContext original_ctx;
    WorkerContext bev_ctx;
    pthread_t original_thread;
    pthread_t bev_thread;

    bool original_thread_started;
    bool bev_thread_started;
    bool curl_initialized;
    bool rga_initialized;
    bool rtsp_initialized;
    bool bev_buffer_initialized;
    bool main_buffer_initialized;
    bool v4l2_opened;
    bool original_queue_initialized;
    bool bev_queue_initialized;

    CaptureLoopState capture_state;

#ifdef ENABLE_PERFORMANCE_MONITORING
    PerfStats perf_main_bev_queue_push;
    PerfStats perf_bev_pop;
    PerfStats perf_bev_process;
    PerfStats perf_bev_acquire;
    PerfStats perf_bev_push;
#endif
} AppContext;

// 重置采集循环状态。
static inline void initCaptureLoopState(CaptureLoopState* state)
{
    if (!state) {
        return;
    }
    state->frame_count = 0;
    state->skip_no_client_count = 0;
    state->main_camera_frame_id = 0;
    state->pending_bev_refresh_requests.store(0);
    state->latest_bev_frame_id.store(0);
    state->last_has_any_client = false;
    state->last_no_client_log.tv_sec = 0;
    state->last_no_client_log.tv_nsec = 0;
    // 初始化为当前时间，确保统计节奏正确。
    clock_gettime(CLOCK_MONOTONIC, &state->last_stats_time);
}

static inline void requestBevRefresh(CaptureLoopState* state, uint32_t count = 1)
{
    if (!state || count == 0) {
        return;
    }
    state->pending_bev_refresh_requests.fetch_add(count, std::memory_order_relaxed);
}

static inline bool hasPendingBevRefreshRequest(const CaptureLoopState* state)
{
    if (!state) {
        return false;
    }
    return state->pending_bev_refresh_requests.load(std::memory_order_relaxed) > 0;
}

static inline bool consumeBevRefreshRequest(CaptureLoopState* state)
{
    if (!state) {
        return false;
    }

    uint32_t pending = state->pending_bev_refresh_requests.load(std::memory_order_relaxed);
    while (pending > 0) {
        if (state->pending_bev_refresh_requests.compare_exchange_weak(
                pending,
                pending - 1,
                std::memory_order_relaxed,
                std::memory_order_relaxed)) {
            return true;
        }
    }
    return false;
}

static inline uint32_t getPendingBevRefreshRequestCount(const CaptureLoopState* state)
{
    if (!state) {
        return 0;
    }
    return state->pending_bev_refresh_requests.load(std::memory_order_relaxed);
}

static inline void recordLatestBevFrameId(CaptureLoopState* state, uint64_t frame_id)
{
    if (!state) {
        return;
    }
    state->latest_bev_frame_id.store(frame_id, std::memory_order_relaxed);
}

static inline uint64_t getLatestBevFrameId(const CaptureLoopState* state)
{
    if (!state) {
        return 0;
    }
    return state->latest_bev_frame_id.load(std::memory_order_relaxed);
}

// 初始化 AppContext，确保资源标志和句柄处于可预测初始状态。
static inline void initAppContext(AppContext* app)
{
    if (!app) {
        return;
    }

    std::memset(&app->cam, 0, sizeof(app->cam));
    app->cam.fd = -1;
    std::memset(&app->original_queue, 0, sizeof(app->original_queue));
    std::memset(&app->bev_queue, 0, sizeof(app->bev_queue));
    std::memset(&app->server, 0, sizeof(app->server));
    std::memset(&app->original_ctx, 0, sizeof(app->original_ctx));
    std::memset(&app->bev_ctx, 0, sizeof(app->bev_ctx));

    app->yolo_handle = nullptr;
    app->unified_server.reset();
    app->original_thread_started = false;
    app->bev_thread_started = false;
    app->curl_initialized = false;
    app->rga_initialized = false;
    app->rtsp_initialized = false;
    app->bev_buffer_initialized = false;
    app->main_buffer_initialized = false;
    app->v4l2_opened = false;
    app->original_queue_initialized = false;
    app->bev_queue_initialized = false;

    // 初始化采集循环状态。
    initCaptureLoopState(&app->capture_state);

}

#endif // APP_CONTEXT_H
