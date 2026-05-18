#ifndef RTSP_STREAMER_H
#define RTSP_STREAMER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <glib.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

typedef void (*RTSPStreamerReleaseCallback)(void* user_data);

typedef struct {
    GstElement* appsrc;
    
    int width;
    int height;
    int fps;
    size_t appsrc_max_bytes;
    int appsrc_max_time_ms;
    gboolean use_dmabuf;
    GstAllocator* dmabuf_alloc;
    
    gboolean need_data;
    GMutex need_data_mutex;
    
    uint64_t total_frames_sent;
    uint64_t backpressure_skip_count;
    
    uint64_t pts_base_frame_idx;
} RTSPStreamer;

bool rtsp_streamer_init(RTSPStreamer* streamer, 
                       int width, int height, int fps,
                       size_t max_bytes, int max_time_ms);

bool rtsp_streamer_push_frame(RTSPStreamer* streamer, 
                             const void* nv12_data, size_t size,
                             uint64_t frame_idx);
 
bool rtsp_streamer_push_dmabuf_nv12(RTSPStreamer* streamer,
                                    int fd_y, int stride_y,
                                    int fd_uv, int stride_uv,
                                    int width, int height,
                                    uint64_t frame_idx);

bool rtsp_streamer_push_dmabuf_nv12_single(RTSPStreamer* streamer,
                                           int fd, int stride_y, int stride_uv,
                                           int width, int height,
                                           uint64_t frame_idx);

bool rtsp_streamer_push_dmabuf_nv12_single_with_release(
    RTSPStreamer* streamer,
    int fd, int stride_y, int stride_uv,
    int width, int height,
    uint64_t frame_idx,
    RTSPStreamerReleaseCallback release_cb,
    void* release_user_data);

bool rtsp_streamer_is_ready(RTSPStreamer* streamer);

void rtsp_streamer_cleanup(RTSPStreamer* streamer);

#endif // RTSP_STREAMER_H

