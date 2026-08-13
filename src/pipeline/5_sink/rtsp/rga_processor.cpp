#include "rga_processor.h"
#include <stdio.h>
#include <string.h>
#include "im2d.h"
#include "im2d_type.h"

bool rga_processor_init(void) {
    // librga/im2d 当前按需调用即可，这里仅保留统一的启动日志入口。
    fprintf(stderr, "[RGA] RGA processor initialized\n");
    return true;
}

bool rga_processor_resize_nv12_dmabuf_to_dmabuf(
    int src_dmabuf_fd, int src_width, int src_height, int src_stride,
    int dst_dmabuf_fd, int dst_width, int dst_height, int dst_stride) {
    if (src_dmabuf_fd < 0 || dst_dmabuf_fd < 0) {
        fprintf(stderr, "[RGA] Invalid dmabuf fds: src=%d, dst=%d\n", src_dmabuf_fd, dst_dmabuf_fd);
        return false;
    }
    if (src_width <= 0 || src_height <= 0 || dst_width <= 0 || dst_height <= 0) {
        fprintf(stderr, "[RGA] Invalid dimensions: src=%dx%d, dst=%dx%d\n",
                src_width, src_height, dst_width, dst_height);
        return false;
    }

    rga_buffer_t src_buf;
    memset(&src_buf, 0, sizeof(src_buf));
    src_buf = wrapbuffer_fd(src_dmabuf_fd, src_width, src_height, RK_FORMAT_YCbCr_420_SP);
    src_buf.wstride = src_stride;
    src_buf.hstride = src_height;

    rga_buffer_t dst_buf;
    memset(&dst_buf, 0, sizeof(dst_buf));
    dst_buf = wrapbuffer_fd(dst_dmabuf_fd, dst_width, dst_height, RK_FORMAT_YCbCr_420_SP);
    dst_buf.wstride = dst_stride;
    dst_buf.hstride = dst_height;

    im_rect src_rect;
    src_rect.x = 0;
    src_rect.y = 0;
    src_rect.width = src_width;
    src_rect.height = src_height;

    im_rect dst_rect;
    dst_rect.x = 0;
    dst_rect.y = 0;
    dst_rect.width = dst_width;
    dst_rect.height = dst_height;

    IM_STATUS status = improcess(src_buf, dst_buf,
                                 rga_buffer_t{},
                                 src_rect, dst_rect,
                                 im_rect{},
                                 IM_SYNC);

    if (status != IM_STATUS_SUCCESS) {
        fprintf(stderr, "[RGA] improcess (dmabuf->dmabuf) failed: %s\n", imStrError(status));
        return false;
    }

    return true;
}

bool rga_processor_resize_nv12(
    const uint8_t* src_data, int src_width, int src_height, int src_stride,
    uint8_t* dst_data, int dst_width, int dst_height) {
    
    if (!src_data || !dst_data) {
        fprintf(stderr, "[RGA] Invalid parameters: NULL pointers\n");
        return false;
    }
    
    if (src_width <= 0 || src_height <= 0 || dst_width <= 0 || dst_height <= 0) {
        fprintf(stderr, "[RGA] Invalid dimensions: src=%dx%d, dst=%dx%d\n",
                src_width, src_height, dst_width, dst_height);
        return false;
    }
    
    // 
    rga_buffer_t src_buf;
    memset(&src_buf, 0, sizeof(src_buf));
    src_buf = wrapbuffer_virtualaddr((void*)src_data, src_width, src_height, 
                                     RK_FORMAT_YCbCr_420_SP);
    src_buf.wstride = src_stride;  // 
    src_buf.hstride = src_height;
    
    // 
    rga_buffer_t dst_buf;
    memset(&dst_buf, 0, sizeof(dst_buf));
    dst_buf = wrapbuffer_virtualaddr((void*)dst_data, dst_width, dst_height, 
                                     RK_FORMAT_YCbCr_420_SP);
    
    // 
    im_rect src_rect;
    src_rect.x = 0;
    src_rect.y = 0;
    src_rect.width = src_width;
    src_rect.height = src_height;
    
    // 
    im_rect dst_rect;
    dst_rect.x = 0;
    dst_rect.y = 0;
    dst_rect.width = dst_width;
    dst_rect.height = dst_height;
    
    // 
    IM_STATUS status = improcess(src_buf, dst_buf, 
                                 rga_buffer_t{},  // 
                                 src_rect, dst_rect, 
                                 im_rect{},       // 
                                 IM_SYNC);          // 
    
    if (status != IM_STATUS_SUCCESS) {
        fprintf(stderr, "[RGA] improcess failed: %s\n", imStrError(status));
        return false;
    }
    
    return true;
}

void rga_processor_cleanup(void) {
    // 
    fprintf(stderr, "[RGA] RGA processor cleaned up\n");
}

