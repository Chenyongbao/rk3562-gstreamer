#ifndef VIDEO_CONSUMER_H
#define VIDEO_CONSUMER_H

#include "video_frame_types.h"

enum class VideoConsumerDispatchResult {
    kDropped = 0,
    kQueuedCopiedFrame,
    kSideEffectOnly,
};

class IVideoConsumer {
public:
    virtual ~IVideoConsumer() = default;

    virtual const char* name() const = 0;
    virtual VideoConsumerDispatchResult dispatch_frame(const VideoFrameDesc& frame) = 0;
};

#endif // VIDEO_CONSUMER_H
