#include "pipeline/3_consumers/bev/bev_rtsp_consumer.h"

#include <cstdio>
#include <new>

#include "../../core/dmabuf_utils.h"
#include "pipeline/1_source/v4l2_capture.h"

namespace {

struct PendingV4L2BufferRelease {
    V4L2Camera* camera = nullptr;
    int buffer_index = -1;
};

// 当 RTSP 队列释放借用帧时，把原始 V4L2 buffer 重新 QBUF 回驱动。
void releaseV4L2SourceBuffer(void* user_data, int)
{
    PendingV4L2BufferRelease* pending =
        static_cast<PendingV4L2BufferRelease*>(user_data);
    if (!pending) {
        return;
    }

    if (pending->camera && pending->buffer_index >= 0) {
        if (!v4l2_camera_queue(pending->camera, pending->buffer_index)) {
            fprintf(stderr,
                    "[BevRtspConsumer] WARNING: failed to QBUF source buffer %d\n",
                    pending->buffer_index);
        }
    }

    delete pending;
}

}  // namespace

BevRtspConsumer::BevRtspConsumer()
    : queue_(nullptr),running_(nullptr),width_(0),
      height_(0),stride_(0),buffer_size_(0),
      enabled_(true),
      last_dispatch_result_(VideoConsumerDispatchResult::kDropped),
      heap_init_fn_(&dmabuf_heap_init),
      alloc_fn_(&dmabuf_alloc),
      free_fn_(&dmabuf_free)
{
}

BevRtspConsumer::~BevRtspConsumer()
{
    shutdown_pool();
}

// 返回组件名称，多态机制下供路由器(VideoRouter)等组件打印调试或报错日志使用。
const char* BevRtspConsumer::name() const
{
    return "bev-rtsp-consumer";
}

// 核心路由派发接口实现（多态调用入口）。
// 接收来自主循环抓取的视频帧，并将其压入到 BEV(鸟瞰图/AI推理) 处理队列中，唤醒后台 Worker 线程。
VideoConsumerDispatchResult BevRtspConsumer::dispatch_frame(const VideoFrameDesc& frame)
{
    last_dispatch_result_ = VideoConsumerDispatchResult::kDropped;
    if (!enabled_ || !queue_ || !running_ || !(*running_)) {
        return last_dispatch_result_;
    }

    // 1. 最优路径：直接借用摄像头导出的 DMABUF，并延迟到队列消费后再 QBUF。
    if (frame.src_dmabuf_fd >= 0 &&
        frame.src_buffer_index >= 0 &&
        frame.src_camera != nullptr) {
        PendingV4L2BufferRelease* pending =
            new (std::nothrow) PendingV4L2BufferRelease();
        if (!pending) {
            return last_dispatch_result_;
        }
        pending->camera = frame.src_camera;
        pending->buffer_index = frame.src_buffer_index;

        const bool pushed = frame_queue_push_dmabuf_ex(queue_,
                                                       frame.src_dmabuf_fd,
                                                       frame.width,
                                                       frame.height,
                                                       frame.stride,
                                                       frame.size,
                                                       frame.frame_idx,
                                                       &releaseV4L2SourceBuffer,
                                                       pending);
        if (!pushed) {
            delete pending;
            return last_dispatch_result_;
        }

        last_dispatch_result_ = VideoConsumerDispatchResult::kQueuedBorrowedDmabufFrame;
        return last_dispatch_result_;
    }

    // 仅支持零拷贝路径，无法借用 DMABUF 时直接丢弃。
    return last_dispatch_result_;
}

void BevRtspConsumer::set_enabled(bool enabled)
{
    enabled_ = enabled;
}

bool BevRtspConsumer::enabled() const
{
    return enabled_;
}

VideoConsumerDispatchResult BevRtspConsumer::last_dispatch_result() const
{
    return last_dispatch_result_;
}

bool BevRtspConsumer::init_pool(int pool_size, int width, int height, int stride)
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

    // 为复制路径预先分配一组可循环复用的 DMABUF。
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

void BevRtspConsumer::shutdown_pool()
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

bool BevRtspConsumer::acquire_private_buffer(ConsumerBuffer* out)
{
    return pool_.acquire_buffer(out);
}

void BevRtspConsumer::release_private_buffer(int dmabuf_fd)
{
    pool_.release_buffer(dmabuf_fd);
}

size_t BevRtspConsumer::pool_total_count() const
{
    return pool_.total_count();
}

size_t BevRtspConsumer::pool_free_count() const
{
    return pool_.free_count();
}

void BevRtspConsumer::bind(FrameQueue* queue, volatile sig_atomic_t* running)
{
    queue_ = queue;
    running_ = running;
}

void BevRtspConsumer::set_dmabuf_allocator(DmabufHeapInitFn heap_init_fn,
                                           DmabufAllocFn alloc_fn,
                                           DmabufFreeFn free_fn)
{
    heap_init_fn_ = heap_init_fn;
    alloc_fn_ = alloc_fn;
    free_fn_ = free_fn;
}


