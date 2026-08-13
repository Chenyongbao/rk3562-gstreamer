#include "pipeline/2_dispatcher/video_router.h"

// 消费者注册->放入容器，可以添加多个消费者
void VideoRouter::register_consumer(IVideoConsumer *consumer) {
  // 空消费者直接忽略，避免在路由表中留下无效项。
  if (!consumer) {
    return;
  }
  consumers_.push_back(consumer);
}

void VideoRouter::dispatch_frame(const VideoFrameDesc &frame) const {
  // 每个消费者独立决定如何处理该帧，路由层只做汇总不做重试。
  // IVideoConsumer* consumer 基类，
  for (IVideoConsumer *consumer : consumers_) {
    if (!consumer) {
      continue;
    }

    // 消费者分发帧
    const VideoConsumerDispatchResult result = consumer->dispatch_frame(frame);
    // 拒绝了
    if (result == VideoConsumerDispatchResult::kDropped) {
      continue;
    }

    // 没有拷贝帧，只是消费了
    if (result == VideoConsumerDispatchResult::kSideEffectOnly) {
      continue;
    }
  }
}
