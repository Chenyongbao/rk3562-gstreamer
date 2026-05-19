#ifndef VIDEO_FRAME_TYPES_H
#define VIDEO_FRAME_TYPES_H

#include <stddef.h>
#include <stdint.h>

struct VideoFrameDesc {
    int src_dmabuf_fd = -1;
    int width = 0;
    int height = 0;
    int stride = 0;
    size_t size = 0;
    const uint8_t* data = nullptr;
    uint64_t frame_idx = 0;
    uint64_t timestamp_ns = 0;
    bool force_process = false;
};

struct ConsumerBuffer {
    int dmabuf_fd = -1;
    int width = 0;
    int height = 0;
    int stride = 0;
    size_t size = 0;
    uint64_t frame_idx = 0;
};

#endif // VIDEO_FRAME_TYPES_H
