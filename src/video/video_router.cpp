#include "video_router.h"

void VideoRouter::register_consumer(IVideoConsumer* consumer)
{
    if (!consumer) {
        return;
    }
    consumers_.push_back(consumer);
}

VideoRouterDispatchSummary VideoRouter::dispatch_frame(const VideoFrameDesc& frame) const
{
    VideoRouterDispatchSummary summary{};
    summary.consumer_count = consumers_.size();

    for (IVideoConsumer* consumer : consumers_) {
        if (!consumer) {
            continue;
        }

        const VideoConsumerDispatchResult result = consumer->dispatch_frame(frame);
        if (result == VideoConsumerDispatchResult::kDropped) {
            continue;
        }

        if (result == VideoConsumerDispatchResult::kSideEffectOnly) {
            continue;
        }

        ++summary.delivered_count;
        if (result == VideoConsumerDispatchResult::kQueuedCopiedFrame) {
            ++summary.copied_count;
        }
    }

    return summary;
}
