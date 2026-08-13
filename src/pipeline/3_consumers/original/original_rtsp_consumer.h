#ifndef ORIGINAL_RTSP_CONSUMER_H
#define ORIGINAL_RTSP_CONSUMER_H

#include <signal.h>
#include <stddef.h>

#include "../../core/frame_queue.h"
#include "pipeline/3_consumers/common/consumer_buffer_pool.h"
#include "pipeline/3_consumers/common/video_consumer.h"

typedef struct RTSPStreamer RTSPStreamer;

// 将主相机原始帧缩放后送入 RTSP 队列，始终使用消费者私有 DMABUF 池。
// 
// 【工作原理详解】：
// 1. 职责：负责将抓取到的原画流（通常分辨率较高）处理并压制成 H264 发送到网络。
// 2. 隔离性：为了不长时间霸占底层的摄像头内存池（防止相机卡死），它在内部维护了一个私有的 DMABUF 内存池 (pool_)。
// 3. 硬件加速：当主循环发来一帧画面时，它会调用 RGA 硬件加速模块，瞬间把原图缩放并拷贝到自己的私有内存池中。
// 4. 异步处理：拷贝完成后，它立刻将私有帧推入缓冲队列 (queue_)，交由后台 Worker 线程慢条斯理地进行视频编码推流。
class OriginalRtspConsumer : public IVideoConsumer {
public:
    using DmabufHeapInitFn = bool (*)();
    using DmabufAllocFn = int (*)(size_t);
    using DmabufFreeFn = void (*)(int);

    OriginalRtspConsumer();
    ~OriginalRtspConsumer() override;

    // ------------------- IVideoConsumer 多态接口 -------------------
    // 返回消费者的固定名称，方便路由层打印日志。
    const char* name() const override;
    
    // 接收主循环派发来的当前视频帧。内部执行：获取私有缓冲 -> RGA硬件缩放拷贝 -> 压入推送队列。
    VideoConsumerDispatchResult dispatch_frame(const VideoFrameDesc& frame) override;
    // ---------------------------------------------------------------
    void set_enabled(bool enabled);
    bool enabled() const;
    VideoConsumerDispatchResult last_dispatch_result() const;

    bool init_pool(int pool_size, int width, int height, int stride);
    void shutdown_pool();

    bool acquire_private_buffer(ConsumerBuffer* out);
    void release_private_buffer(int dmabuf_fd);
    size_t pool_total_count() const;
    size_t pool_free_count() const;

    void bind(FrameQueue* queue, RTSPStreamer* stream, volatile sig_atomic_t* running);
    void set_dmabuf_allocator(DmabufHeapInitFn heap_init_fn,
                              DmabufAllocFn alloc_fn,
                              DmabufFreeFn free_fn);

private:
    static void releaseQueuedBuffer(void* user_data, int dmabuf_fd);

    // ------------------- 核心成员变量 -------------------
    FrameQueue* queue_;                  // 指向后台推流任务的缓冲队列，用于桥接主循环和后台编码线程
    RTSPStreamer* stream_;               // 底层 RTSP 推流服务器上下文句柄
    volatile sig_atomic_t* running_;     // 整个系统的运行标志位，用于安全退出机制
    ConsumerBufferPool pool_;            // 消费者私有的内存池，专门用来存放 RGA 缩放复制后的目标图像
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

#endif // ORIGINAL_RTSP_CONSUMER_H
