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
    std::atomic<uint32_t> pending_bev_refresh_requests;
    std::atomic<uint64_t> latest_bev_frame_id;
    std::atomic<uint32_t> pending_main_camera_refresh_requests;
    std::atomic<uint64_t> latest_main_camera_frame_id;
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
    state->pending_bev_refresh_requests.store(0);
    state->latest_bev_frame_id.store(0);
    state->pending_main_camera_refresh_requests.store(0);
    state->latest_main_camera_frame_id.store(0);
    state->last_has_any_client = false;
    state->last_no_client_log.tv_sec = 0;
    state->last_no_client_log.tv_nsec = 0;
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

static inline void requestMainCameraRefresh(CaptureLoopState* state, uint32_t count = 1)
{
    if (!state || count == 0) {
        return;
    }
    state->pending_main_camera_refresh_requests.fetch_add(count, std::memory_order_relaxed);
}

static inline bool hasPendingMainCameraRefreshRequest(const CaptureLoopState* state)
{
    if (!state) {
        return false;
    }
    return state->pending_main_camera_refresh_requests.load(std::memory_order_relaxed) > 0;
}

static inline bool consumeMainCameraRefreshRequest(CaptureLoopState* state)
{
    if (!state) {
        return false;
    }

    uint32_t pending = state->pending_main_camera_refresh_requests.load(std::memory_order_relaxed);
    while (pending > 0) {
        if (state->pending_main_camera_refresh_requests.compare_exchange_weak(
                pending,
                pending - 1,
                std::memory_order_relaxed,
                std::memory_order_relaxed)) {
            return true;
        }
    }
    return false;
}

static inline uint32_t getPendingMainCameraRefreshRequestCount(const CaptureLoopState* state)
{
    if (!state) {
        return 0;
    }
    return state->pending_main_camera_refresh_requests.load(std::memory_order_relaxed);
}

static inline void recordLatestMainCameraFrameId(CaptureLoopState* state, uint64_t frame_id)
{
    if (!state) {
        return;
    }
    state->latest_main_camera_frame_id.store(frame_id, std::memory_order_relaxed);
}

static inline uint64_t getLatestMainCameraFrameId(const CaptureLoopState* state)
{
    if (!state) {
        return 0;
    }
    return state->latest_main_camera_frame_id.load(std::memory_order_relaxed);
}

#endif // CAPTURE_STATE_H
