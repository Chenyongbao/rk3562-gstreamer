#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include <stdint.h>
#include <time.h>
#include <signal.h>
#include <thread>
#include <memory>
#include <atomic>
#include <cstring>
#include <limits>
#include <cstdio>

#include "pipeline/1_source/v4l2_capture.h"
#include "pipeline/2_dispatcher/video_router.h"
#include "pipeline/3_consumers/original/original_rtsp_consumer.h"
#include "pipeline/3_consumers/bev/bev_rtsp_consumer.h"
#include "pipeline/3_consumers/latest/latest_frame_consumer.h"
#include "core/frame_queue.h"
#include "pipeline/5_sink/rtsp/dual_rtsp_server.h"
#include "yolodetect/yolo_wrapper.h"
#include "protocol/UnifiedSocketServer.h"
#include "app/capture_state.h"

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
    volatile sig_atomic_t* running;
    const char* thread_name;
    struct AppContext* app;
} WorkerContext;

// 应用运行期全局状态与资源句柄。
typedef struct AppContext {
    V4L2Camera cam;
    FrameQueue original_queue;
    FrameQueue bev_queue;
    VideoRouter video_router;                                   //分发路由
    //==================== 消费者 ====================
    std::unique_ptr<OriginalRtspConsumer> original_consumer;
    std::unique_ptr<BevRtspConsumer> bev_consumer;
    std::unique_ptr<MainCameraLatestFrameConsumer> main_camera_latest_frame_consumer;
    
    BevLatestFrameUpdater bev_latest_frame_updater;
    DualRTSPServer server;
    bool bev_buffer_initialized;
    bool main_buffer_initialized;
    bool v4l2_opened;
    bool original_queue_initialized;
    bool bev_queue_initialized;

    CaptureLoopState capture_state;

    WorkerContext  original_ctx;
    WorkerContext  bev_ctx;
    YOLOHandle     yolo_handle;
    std::unique_ptr<UnifiedSocketServer> unified_server;
    bool           original_thread_started;
    bool           bev_thread_started;
    bool           curl_initialized;
    bool           rga_initialized;
    bool           rtsp_initialized;

#ifdef ENABLE_PERFORMANCE_MONITORING
    PerfStats perf_main_bev_queue_push;
    PerfStats perf_bev_pop;
    PerfStats perf_bev_process;
    PerfStats perf_bev_acquire;
    PerfStats perf_bev_push;
#endif
} AppContext;

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
    app->original_consumer.reset();
    app->bev_consumer.reset();
    app->main_camera_latest_frame_consumer.reset();
    app->bev_latest_frame_updater.bind(nullptr);
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
