

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


bool bev_bind_context_to_thread(BevHandle handle);


bool bev_get_output_dmabuf_info(BevHandle handle,
                                int* fd_y, int* stride_y,
                                int* width_y, int* height_y,
                                int* fd_uv, int* stride_uv,
                                int* width_uv, int* height_uv,
                                int* uv_is_rg88);
bool bev_acquire_output_dmabuf(BevHandle handle);
void bev_release_output_dmabuf(BevHandle handle);


long bev_get_avg_process_time_us(BevHandle handle);


void bev_cleanup(BevHandle handle);


#endif // BEV_API_H
