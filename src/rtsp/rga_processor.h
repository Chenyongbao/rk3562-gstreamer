#ifndef RGA_PROCESSOR_H
#define RGA_PROCESSOR_H

#include <stdint.h>
#include <stdbool.h>

bool rga_processor_init(void);

bool rga_processor_resize_nv12(
    const uint8_t* src_data, int src_width, int src_height, int src_stride,
    uint8_t* dst_data, int dst_width, int dst_height);

bool rga_processor_resize_nv12_dmabuf_to_dmabuf(
    int src_dmabuf_fd, int src_width, int src_height, int src_stride,
    int dst_dmabuf_fd, int dst_width, int dst_height, int dst_stride);

void rga_processor_cleanup(void);

#endif // RGA_PROCESSOR_H

