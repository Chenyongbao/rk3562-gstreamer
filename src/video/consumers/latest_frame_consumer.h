#ifndef LATEST_FRAME_CONSUMER_H
#define LATEST_FRAME_CONSUMER_H

#include "../video_consumer.h"

struct CaptureLoopState;

class MainCameraLatestFrameConsumer : public IVideoConsumer {
public:
    MainCameraLatestFrameConsumer();

    const char* name() const override;
    VideoConsumerDispatchResult dispatch_frame(const VideoFrameDesc& frame) override;
    void bind(CaptureLoopState* state);
    VideoConsumerDispatchResult last_dispatch_result() const;

private:
    CaptureLoopState* state_;
    VideoConsumerDispatchResult last_dispatch_result_;
};

class BevLatestFrameUpdater {
public:
    BevLatestFrameUpdater();

    void bind(CaptureLoopState* state);
    bool publish(const uint8_t* nv12_data, size_t size, uint64_t frame_id) const;

private:
    CaptureLoopState* state_;
};

#endif // LATEST_FRAME_CONSUMER_H
