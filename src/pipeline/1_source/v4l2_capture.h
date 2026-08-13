#ifndef V4L2_CAPTURE_H
#define V4L2_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// 【第一步：定义上下文结构体】
// V4L2 相机句柄，保存设备参数、mmap 缓冲和导出的 DMABUF fd。
typedef struct V4L2Camera{
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

// 【第二步：实现设备初始化】
// 打开设备并完成 NV12 格式、mmap 缓冲和 DMABUF 导出初始化。
bool v4l2_camera_open(V4L2Camera* cam, const char* device, 
                     int width, int height, int buffer_count);

// 【第三步：实现启动与停止 (Start部分)】
// 启停采集流，并提供 DQBUF/QBUF 及故障恢复等基础操作。
bool v4l2_camera_start(V4L2Camera* cam);

// 【第四步：实现数据流转 (取帧与还帧)】
bool v4l2_camera_dequeue(V4L2Camera* cam, int* out_index, 
                        void** out_data, size_t* out_size, int* out_dmabuf_fd);

bool v4l2_camera_queue(V4L2Camera* cam, int index);

uint32_t v4l2_camera_buffer_type(const V4L2Camera* cam);

bool v4l2_camera_fill_dequeue_result(const V4L2Camera* cam, uint32_t buf_index,
                                    int* out_index, void** out_data,
                                    size_t* out_size, int* out_dmabuf_fd);

// 【第三步：实现启动与停止 (Stop部分)】
bool v4l2_camera_stop(V4L2Camera* cam);

// 【第六步 (进阶)：容错与恢复】
bool v4l2_camera_resume(V4L2Camera* cam);

bool v4l2_camera_recover(V4L2Camera* cam);

// 【第五步：实现资源清理】
void v4l2_camera_close(V4L2Camera* cam);


