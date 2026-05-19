#include "original_rtsp_consumer.h"

#include "../../core/dmabuf_utils.h"
#include "../../core/frame_queue.h"
#include "../../rtsp/rga_processor.h"

OriginalRtspConsumer::OriginalRtspConsumer()
    : queue_(nullptr),
      stream_(nullptr),
      running_(nullptr),
      width_(0),
      height_(0),
      stride_(0),
      buffer_size_(0),
      enabled_(true),
      last_dispatch_result_(VideoConsumerDispatchResult::kDropped),
      heap_init_fn_(&dmabuf_heap_init),
      alloc_fn_(&dmabuf_alloc),
      free_fn_(&dmabuf_free)
{
}

OriginalRtspConsumer::~OriginalRtspConsumer()
{
    shutdown_pool();
}

const char* OriginalRtspConsumer::name() const
{
    return "original-rtsp-consumer";
}

VideoConsumerDispatchResult OriginalRtspConsumer::dispatch_frame(const VideoFrameDesc& frame)
{
    last_dispatch_result_ = VideoConsumerDispatchResult::kDropped;
    if (!enabled_ || !queue_ || !stream_ || !running_ || !(*running_) || frame.src_dmabuf_fd < 0) {
        return last_dispatch_result_;
    }

    ConsumerBuffer buffer{};
    if (!acquire_private_buffer(&buffer)) {
        return last_dispatch_result_;
    }

    const bool copied = rga_processor_resize_nv12_dmabuf_to_dmabuf(
        frame.src_dmabuf_fd,
        frame.width,
        frame.height,
        frame.stride,
        buffer.dmabuf_fd,
        width_,
        height_,
        stride_);
    if (!copied) {
        release_private_buffer(buffer.dmabuf_fd);
        return last_dispatch_result_;
    }

    const bool pushed = frame_queue_push_dmabuf_ex(queue_,
                                                   buffer.dmabuf_fd,
                                                   width_,
                                                   height_,
                                                   stride_,
                                                   buffer.size,
                                                   frame.frame_idx,
                                                   false,
                                                   &OriginalRtspConsumer::releaseQueuedBuffer,
                                                   this);
    if (!pushed) {
        release_private_buffer(buffer.dmabuf_fd);
        return last_dispatch_result_;
    }

    last_dispatch_result_ = VideoConsumerDispatchResult::kQueuedCopiedFrame;
    return last_dispatch_result_;
}

void OriginalRtspConsumer::set_enabled(bool enabled)
{
    enabled_ = enabled;
}

bool OriginalRtspConsumer::enabled() const
{
    return enabled_;
}

VideoConsumerDispatchResult OriginalRtspConsumer::last_dispatch_result() const
{
    return last_dispatch_result_;
}

bool OriginalRtspConsumer::init_pool(int pool_size, int width, int height, int stride)
{
    shutdown_pool();

    if (pool_size <= 0 || width <= 0 || height <= 0 || stride <= 0) {
        return false;
    }

    width_ = width;
    height_ = height;
    stride_ = stride;
    buffer_size_ = static_cast<size_t>(stride_) * static_cast<size_t>(height_) +
                   static_cast<size_t>(stride_) * static_cast<size_t>(height_ / 2);

    if (!heap_init_fn_ || !alloc_fn_ || !free_fn_) {
        shutdown_pool();
        return false;
    }

    if (!heap_init_fn_()) {
        return false;
    }

    for (int i = 0; i < pool_size; ++i) {
        ConsumerBuffer buffer{};
        buffer.dmabuf_fd = alloc_fn_(buffer_size_);
        if (buffer.dmabuf_fd < 0) {
            shutdown_pool();
            return false;
        }
        buffer.width = width_;
        buffer.height = height_;
        buffer.stride = stride_;
        buffer.size = buffer_size_;
        if (!pool_.add_buffer(buffer)) {
            free_fn_(buffer.dmabuf_fd);
            shutdown_pool();
            return false;
        }
    }

    return true;
}

void OriginalRtspConsumer::shutdown_pool()
{
    for (const ConsumerBuffer& buffer : pool_.buffers()) {
        if (free_fn_) {
            free_fn_(buffer.dmabuf_fd);
        }
    }
    pool_.clear();
    width_ = 0;
    height_ = 0;
    stride_ = 0;
    buffer_size_ = 0;
}

bool OriginalRtspConsumer::acquire_private_buffer(ConsumerBuffer* out)
{
    return pool_.acquire_buffer(out);
}

void OriginalRtspConsumer::release_private_buffer(int dmabuf_fd)
{
    pool_.release_buffer(dmabuf_fd);
}

size_t OriginalRtspConsumer::pool_total_count() const
{
    return pool_.total_count();
}

size_t OriginalRtspConsumer::pool_free_count() const
{
    return pool_.free_count();
}

void OriginalRtspConsumer::bind(FrameQueue* queue, RTSPStreamer* stream, volatile sig_atomic_t* running)
{
    queue_ = queue;
    stream_ = stream;
    running_ = running;
}

void OriginalRtspConsumer::set_dmabuf_allocator(DmabufHeapInitFn heap_init_fn,
                                                DmabufAllocFn alloc_fn,
                                                DmabufFreeFn free_fn)
{
    heap_init_fn_ = heap_init_fn;
    alloc_fn_ = alloc_fn;
    free_fn_ = free_fn;
}

void OriginalRtspConsumer::releaseQueuedBuffer(void* user_data, int dmabuf_fd)
{
    if (!user_data) {
        return;
    }
    static_cast<OriginalRtspConsumer*>(user_data)->release_private_buffer(dmabuf_fd);
}
