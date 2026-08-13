#ifndef CAPTURE_STATE_H
#define CAPTURE_STATE_H

#include <atomic>
#include <cstdint>
#include <ctime>

// 主采集循环状态：用于统计、日志节流及帧序号维护。
struct CaptureLoopState {
    uint64_t frame_count;
    uint64_t skip_no_client_count;
    uint64_t main_camera_frame_id;
    struct timespec last_stats_time;
    bool last_has_any_client;
    struct timespec last_no_client_log;
};

// 重置采集循环状态。
static inline void initCaptureLoopState(CaptureLoopState* state)
{
    if (!state) {
        return;
    }
    state->frame_count = 0;
    state->skip_no_client_count = 0;
    state->main_camera_frame_id = 0;
    state->last_has_any_client = false;
    state->last_no_client_log.tv_sec = 0;
    state->last_no_client_log.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &state->last_stats_time);
}

#endif // CAPTURE_STATE_H
