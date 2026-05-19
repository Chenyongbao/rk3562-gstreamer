#ifndef VIDEO_ROUTER_H
#define VIDEO_ROUTER_H

#include <stddef.h>
#include <vector>

#include "video_consumer.h"

struct VideoRouterDispatchSummary {
    size_t consumer_count = 0;
    size_t delivered_count = 0;
    size_t copied_count = 0;
};

class VideoRouter {
public:
    void register_consumer(IVideoConsumer* consumer);
    VideoRouterDispatchSummary dispatch_frame(const VideoFrameDesc& frame) const;

private:
    std::vector<IVideoConsumer*> consumers_;
};

#endif // VIDEO_ROUTER_H
