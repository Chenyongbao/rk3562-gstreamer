#ifndef LATEST_FRAME_CONSUMER_H
#define LATEST_FRAME_CONSUMER_H

#include "pipeline/3_consumers/common/video_consumer.h"

struct CaptureLoopState;

// 仅在主线程请求“刷新主相机最新帧”时，把当前帧写入全局缓存。
// 
// 【工作原理详解】：
// 1. 职责：这是一个极度轻量级的组件，专门为 HTTP Web 端等偶尔需要请求一张最新截图的需求设计。
// 2. 按需执行：平常哪怕有一万帧闪过，它都不干活。只有当外部设置了刷新请求标志时，它才出马。
// 3. 同步处理：它不需要复杂的队列机制，只要被激活，就立马把当前的画面（用 memcpy）拷贝到内存里供前端读取，然后立刻结束。
class MainCameraLatestFrameConsumer : public IVideoConsumer {
public:
    MainCameraLatestFrameConsumer();

    // ------------------- IVideoConsumer 多态接口 -------------------
    const char* name() const override;
    
    // 接收派发的视频帧。如果不缺截图，直接返回 kDropped；如果需要截图，则拷贝当前画面并返回 kSideEffectOnly。
    VideoConsumerDispatchResult dispatch_frame(const VideoFrameDesc& frame) override;
    // ---------------------------------------------------------------
    void bind(CaptureLoopState* state);
    VideoConsumerDispatchResult last_dispatch_result() const;

private:
    CaptureLoopState* state_;
    VideoConsumerDispatchResult last_dispatch_result_;
};

// 将 BEV 处理后的 NV12 帧发布到全局最新帧缓存。
class BevLatestFrameUpdater {
public:
    BevLatestFrameUpdater();

    void bind(CaptureLoopState* state);
    bool publish(const uint8_t* nv12_data, size_t size, uint64_t frame_id) const;

private:
    CaptureLoopState* state_;
};

#endif // LATEST_FRAME_CONSUMER_H
