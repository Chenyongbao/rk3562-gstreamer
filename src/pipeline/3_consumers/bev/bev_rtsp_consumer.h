#ifndef BEV_RTSP_CONSUMER_H
#define BEV_RTSP_CONSUMER_H

#include <signal.h>
#include <stddef.h>

#include "../../core/frame_queue.h"
#include "pipeline/3_consumers/common/consumer_buffer_pool.h"
#include "pipeline/3_consumers/common/video_consumer.h"

// 将 BEV 结果帧送入 RTSP 队列，优先借用源 DMABUF，必要时再复制到私有池。
// 
// 【工作原理详解】：
// 1. 职责：负责接收图像供后台 AI（YOLO目标检测）处理，并将画框后的结果推流。
// 2. 极致性能 (Zero-Copy)：与原画推流不同，为了让 AI 处理达到最高帧率，它的“最优路径”是直接“借用”底层相机拍出来的物理内存 (DMABUF)。
// 3. 异步归还：它把相机的原生物理内存句柄塞进队列，交由后台独立线程进行 NPU 推理。主循环不用等它算完。
// 4. 责任转移：等后台线程跑完 YOLO 画完框并发送流后，由后台线程负责调用 releaseV4L2SourceBuffer 把物理内存还给相机驱动。
class BevRtspConsumer : public IVideoConsumer {
public:
    using DmabufHeapInitFn = bool (*)();
    using DmabufAllocFn = int (*)(size_t);
    using DmabufFreeFn = void (*)(int);

    BevRtspConsumer();
    ~BevRtspConsumer() override;

    // ------------------- IVideoConsumer 多态接口 -------------------
    // 返回消费者的固定名称，方便路由层打印日志。
    const char* name() const override;
    
    // 接收主循环派发来的当前视频帧。内部执行：尝试借用 DMABUF -> 压入后台处理队列 -> 通知主循环暂不归还相机内存。
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

    void bind(FrameQueue* queue, volatile sig_atomic_t* running);
    void set_dmabuf_allocator(DmabufHeapInitFn heap_init_fn,
                              DmabufAllocFn alloc_fn,
                              DmabufFreeFn free_fn);

private:
    // 队列消费完成后归还借用的源 V4L2 buffer 或私有 DMABUF。
    static void releaseQueuedBuffer(void* user_data, int dmabuf_fd);

    // ------------------- 核心成员变量 -------------------
    FrameQueue* queue_;                  // 指向后台 AI 处理任务的缓冲队列
    volatile sig_atomic_t* running_;     // 整个系统的运行标志位
    ConsumerBufferPool pool_;            // 私有内存池（仅在极端情况下，无法直接借用相机的零拷贝内存时，作为次优备选方案使用）
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
