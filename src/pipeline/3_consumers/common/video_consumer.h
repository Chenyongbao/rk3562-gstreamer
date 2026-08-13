#ifndef VIDEO_CONSUMER_H
#define VIDEO_CONSUMER_H

#include "core/video_frame_types.h"

// 路由层根据消费者返回值统计“丢帧 / 复制入队 / 借用 DMABUF 入队”等结果。
enum class VideoConsumerDispatchResult {
    kDropped = 0,                   //拒绝了
    kQueuedCopiedFrame,             //复制了
    kQueuedBorrowedDmabufFrame,     //借用了
    kSideEffectOnly,
};

// 所有视频消费者都通过统一接口接收路由分发的帧描述。
class IVideoConsumer {
public:
    virtual ~IVideoConsumer() = default;

    virtual const char* name() const = 0;
    virtual VideoConsumerDispatchResult dispatch_frame(const VideoFrameDesc& frame) = 0;
};

#endif // VIDEO_CONSUMER_H
