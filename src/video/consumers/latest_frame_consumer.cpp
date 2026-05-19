#include "latest_frame_consumer.h"

#include "../../app/capture_state.h"
#include "../LatestNv12FrameBuffer.h"

MainCameraLatestFrameConsumer::MainCameraLatestFrameConsumer()
    : state_(nullptr),
      last_dispatch_result_(VideoConsumerDispatchResult::kDropped)
{
}

const char* MainCameraLatestFrameConsumer::name() const
{
    return "main-camera-latest-frame-consumer";
}

VideoConsumerDispatchResult MainCameraLatestFrameConsumer::dispatch_frame(const VideoFrameDesc& frame)
{
    last_dispatch_result_ = VideoConsumerDispatchResult::kDropped;
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
    if (!bev_frame_buffer_update(nv12_data, size, frame_id)) {
        return false;
    }
    if (state_) {
        recordLatestBevFrameId(state_, frame_id);
    }
    return true;
}
