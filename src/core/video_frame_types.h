#ifndef VIDEO_FRAME_TYPES_H
#define VIDEO_FRAME_TYPES_H

#include <stddef.h>
#include <stdint.h>

typedef struct V4L2Camera V4L2Camera;

// 描述一次路由分发中的输入帧，可同时携带 CPU 指针和 DMABUF 信息。
struct VideoFrameDesc {
    int src_dmabuf_fd = -1;
    int src_buffer_index = -1;
    V4L2Camera* src_camera = nullptr;
    int width = 0;
    int height = 0;
    int stride = 0;
    size_t size = 0;
    const uint8_t* data = nullptr;
    uint64_t frame_idx = 0;
    uint64_t timestamp_ns = 0;
};

// 消费者私有缓冲的元数据，主要用于零拷贝/半零拷贝分发路径。
struct ConsumerBuffer {
    int dmabuf_fd = -1;
    int width = 0;
    int height = 0;
    int stride = 0;
    size_t size = 0;
    uint64_t frame_idx = 0;
};

#endif // VIDEO_FRAME_TYPES_H
