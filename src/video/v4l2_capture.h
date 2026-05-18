#ifndef V4L2_CAPTURE_H
#define V4L2_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef struct {
    int fd;                    
    int width;                 
    int height;               
    int y_stride;              
    bool use_mplane;
    char device_path[256];
    int requested_width;
    int requested_height;
    int requested_buffer_count;
    
    void** buffers;           
    size_t* buffer_lengths;   
    int buffer_count;         
    
    int* dmabuf_fds;          
} V4L2Camera;

bool v4l2_camera_open(V4L2Camera* cam, const char* device, 
                     int width, int height, int buffer_count);

bool v4l2_camera_start(V4L2Camera* cam);

bool v4l2_camera_dequeue(V4L2Camera* cam, int* out_index, 
                        void** out_data, size_t* out_size, int* out_dmabuf_fd);

bool v4l2_camera_queue(V4L2Camera* cam, int index);

uint32_t v4l2_camera_buffer_type(const V4L2Camera* cam);

bool v4l2_camera_fill_dequeue_result(const V4L2Camera* cam, uint32_t buf_index,
                                    int* out_index, void** out_data,
                                    size_t* out_size, int* out_dmabuf_fd);

bool v4l2_camera_stop(V4L2Camera* cam);

bool v4l2_camera_resume(V4L2Camera* cam);

bool v4l2_camera_recover(V4L2Camera* cam);

void v4l2_camera_close(V4L2Camera* cam);

#endif // V4L2_CAPTURE_H

