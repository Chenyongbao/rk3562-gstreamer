#ifndef VIDEO_ROUTER_H
#define VIDEO_ROUTER_H

#include <stddef.h>
#include <vector>

#include "pipeline/3_consumers/common/video_consumer.h"



// 将同一帧分发给多个消费者，并统计分发结果。
class VideoRouter {
public:
    void register_consumer(IVideoConsumer* consumer);
    //分发帧
    void dispatch_frame(const VideoFrameDesc& frame) const;

private:
    std::vector<IVideoConsumer*> consumers_;
};

#endif // VIDEO_ROUTER_H
