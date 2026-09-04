#ifndef VIDEO_FRAME_TYPES_H
#define VIDEO_FRAME_TYPES_H

#include <stddef.h>
#include <stdint.h>

// 私有 DMABUF 缓冲元数据，用于 RGA 缩放复制输出池。
struct ConsumerBuffer {
    int dmabuf_fd = -1;
    int width = 0;
    int height = 0;
    int stride = 0;
    size_t size = 0;
    uint64_t frame_idx = 0;
};

#endif // VIDEO_FRAME_TYPES_H
