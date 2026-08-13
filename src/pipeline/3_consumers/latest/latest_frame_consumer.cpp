#include "pipeline/3_consumers/latest/latest_frame_consumer.h"

#include "../../app/capture_state.h"
#include "pipeline/3_consumers/common/LatestNv12FrameBuffer.h"

MainCameraLatestFrameConsumer::MainCameraLatestFrameConsumer()
    : state_(nullptr),
      last_dispatch_result_(VideoConsumerDispatchResult::kDropped)
{
}

// 返回组件名称，多态机制下供路由器(VideoRouter)等组件打印调试或报错日志使用。
const char* MainCameraLatestFrameConsumer::name() const
{
    return "main-camera-latest-frame-consumer";
}

// 核心路由派发接口实现（多态调用入口）。
// 负责处理前端页面等请求的单张“快照/封面图”，只拷贝当前这一帧，不占用主队列通道。
VideoConsumerDispatchResult MainCameraLatestFrameConsumer::dispatch_frame(const VideoFrameDesc& frame)
{
    last_dispatch_result_ = VideoConsumerDispatchResult::kDropped;
    // 只有存在待处理刷新请求时，才把这一帧写入全局 latest buffer。
    if (!state_ || !hasPendingMainCameraRefreshRequest(state_)) {
        return last_dispatch_result_;
    }
    if (!frame.data || frame.size == 0) {
        return last_dispatch_result_;
    }

    const uint64_t next_frame_id = getLatestMainCameraFrameId(state_) + 1;
    if (!main_camera_frame_buffer_update(frame.data, frame.size, next_frame_id)) {
        return last_dispatch_result_;
    }

    recordLatestMainCameraFrameId(state_, next_frame_id);
    consumeMainCameraRefreshRequest(state_);
    last_dispatch_result_ = VideoConsumerDispatchResult::kSideEffectOnly;
    return last_dispatch_result_;
}

void MainCameraLatestFrameConsumer::bind(CaptureLoopState* state)
{
    state_ = state;
}

VideoConsumerDispatchResult MainCameraLatestFrameConsumer::last_dispatch_result() const
{
    return last_dispatch_result_;
}

BevLatestFrameUpdater::BevLatestFrameUpdater()
    : state_(nullptr)
{
}

void BevLatestFrameUpdater::bind(CaptureLoopState* state)
{
    state_ = state;
}

bool BevLatestFrameUpdater::publish(const uint8_t* nv12_data, size_t size, uint64_t frame_id) const
{
    // 发布成功后顺手记录最新 BEV 帧号，供其它线程做 freshness 判断。
    if (!bev_frame_buffer_update(nv12_data, size, frame_id)) {
        return false;
    }
    if (state_) {
        recordLatestBevFrameId(state_, frame_id);
    }
    return true;
}
