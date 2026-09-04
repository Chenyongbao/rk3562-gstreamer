#ifndef CAPTURE_STATE_H
#define CAPTURE_STATE_H

#include <atomic>
#include <cstdint>

// 跨线程共享的 BEV 刷新请求标志。
//
// 说明：主相机的"按需取帧"已重构到 FrameProvider（pipeline/common/frame_provider.h），
// BEV 的"最新帧序号"也随 BEV 帧缓冲一并迁入 FrameProvider（bev_frame_provider）。
// 这里只保留 BEV 的"刷新请求计数"——它属于生产者侧的调度信号：
// DETECT 命令在无缓存帧时 +1，BEV 工作线程处理完一帧后消费（决定"是否值得跑 OpenGL"）。
struct CaptureLoopState {
    std::atomic<int> bev_refresh_request_count{0};
};

static inline void initCaptureLoopState(CaptureLoopState* state)
{
    if (!state) {
        return;
    }
    state->bev_refresh_request_count.store(0);
}

// BEV 刷新请求：DETECT 命令在无缓存帧时置位，BEV 工作线程处理完一帧后消费。
static inline void requestBevRefresh(CaptureLoopState* s)
{
    if (s) s->bev_refresh_request_count.fetch_add(1);
}

static inline int getPendingBevRefreshRequestCount(const CaptureLoopState* s)
{
    return s ? s->bev_refresh_request_count.load() : 0;
}

static inline void consumeBevRefreshRequest(CaptureLoopState* s)
{
    if (!s) {
        return;
    }
    int expected = s->bev_refresh_request_count.load();
    while (expected > 0 &&
           !s->bev_refresh_request_count.compare_exchange_weak(expected, expected - 1)) {
        // 自旋重试
    }
}

#endif // CAPTURE_STATE_H
