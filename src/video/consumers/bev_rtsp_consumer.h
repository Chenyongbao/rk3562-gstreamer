#ifndef BEV_RTSP_CONSUMER_H
#define BEV_RTSP_CONSUMER_H

#include <signal.h>
#include <stddef.h>

#include "../../core/frame_queue.h"
#include "../consumer_buffer_pool.h"
#include "../video_consumer.h"

class BevRtspConsumer : public IVideoConsumer {
public:
    using DmabufHeapInitFn = bool (*)();
    using DmabufAllocFn = int (*)(size_t);
    using DmabufFreeFn = void (*)(int);

    BevRtspConsumer();
    ~BevRtspConsumer() override;

    const char* name() const override;
    VideoConsumerDispatchResult dispatch_frame(const VideoFrameDesc& frame) override;
    void set_enabled(bool enabled);
    bool enabled() const;
    VideoConsumerDispatchResult last_dispatch_result() const;

    bool init_pool(int pool_size, int width, int height, int stride);
    void shutdown_pool();

    bool acquire_private_buffer(ConsumerBuffer* out);
    void release_private_buffer(int dmabuf_fd);
    size_t pool_total_count() const;
    size_t pool_free_count() const;

    void bind(FrameQueue* queue, volatile sig_atomic_t* running);
    void set_dmabuf_allocator(DmabufHeapInitFn heap_init_fn,
                              DmabufAllocFn alloc_fn,
                              DmabufFreeFn free_fn);

private:
    static void releaseQueuedBuffer(void* user_data, int dmabuf_fd);

    FrameQueue* queue_;
    volatile sig_atomic_t* running_;
    ConsumerBufferPool pool_;
    int width_;
    int height_;
    int stride_;
    size_t buffer_size_;
    bool enabled_;
    VideoConsumerDispatchResult last_dispatch_result_;
    DmabufHeapInitFn heap_init_fn_;
    DmabufAllocFn alloc_fn_;
    DmabufFreeFn free_fn_;
};

#endif // BEV_RTSP_CONSUMER_H
