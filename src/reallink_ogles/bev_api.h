

#ifndef BEV_API_H
#define BEV_API_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>



typedef void* BevHandle;


BevHandle bev_init(int input_width, int input_height, 
                   int output_width, int output_height);


bool bev_process_frame(BevHandle handle,
                       const uint8_t* nv12_input, size_t input_size,
                       uint8_t* nv12_output, size_t output_size);

bool bev_process_frame_dmabuf(BevHandle handle,
                              int nv12_input_fd,
                              int input_stride,
                              size_t input_size,
                              uint8_t* nv12_output,
                              size_t output_size);


bool bev_bind_context_to_thread(BevHandle handle);


bool bev_get_output_dmabuf_info(BevHandle handle,
                                int* fd_y, int* stride_y,
                                int* width_y, int* height_y,
                                int* fd_uv, int* stride_uv,
                                int* width_uv, int* height_uv,
                                int* uv_is_rg88);
bool bev_acquire_output_dmabuf(BevHandle handle);
void bev_release_output_dmabuf(BevHandle handle);

// 渲染一帧到输出池当前槽位，返回该槽位单块 NV12 dmabuf 的 fd（零 CPU 拷贝）。
// Y 在 fd offset 0，UV 在 fd offset (stride_y * height)。渲染完成后已 glFinish +
// DMA_BUF_SYNC_START|READ，调用方用完须调 bev_release_output_pool_slot。
bool bev_process_frame_dmabuf_pooled(BevHandle handle,
                                     int nv12_input_fd,
                                     int input_stride,
                                     size_t input_size,
                                     int* out_fd, int* out_stride_y, int* out_stride_uv);

// 归还上一帧输出池槽位（DMA_BUF_SYNC_END|READ），与 bev_process_frame_dmabuf_pooled 配对。
void bev_release_output_pool_slot(BevHandle handle);

// 把上一帧输出池槽位读回紧凑 NV12（供快照/检测等需要 CPU 数据的场景，按需调用）。
bool bev_read_last_output(BevHandle handle, uint8_t* nv12_output, size_t output_size);


long bev_get_avg_process_time_us(BevHandle handle);


void bev_cleanup(BevHandle handle);


#endif // BEV_API_H
