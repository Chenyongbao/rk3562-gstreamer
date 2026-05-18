#define _POSIX_C_SOURCE 199309L

#include "rtsp_streamer.h"
#include "../config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <gst/allocators/gstdmabuf.h>
#include <gst/video/video.h>

typedef struct {
    RTSPStreamerReleaseCallback release_cb;
    void* release_user_data;
} RTSPStreamerReleaseNotify;

static void rtsp_streamer_invoke_release_cb(RTSPStreamerReleaseCallback cb, void* user_data) {
    if (cb) {
        cb(user_data);
    }
}

static void rtsp_streamer_dmabuf_release_notify(gpointer data, GstMiniObject* where_the_object_was) {
    (void)where_the_object_was;
    RTSPStreamerReleaseNotify* notify = (RTSPStreamerReleaseNotify*)data;
    if (!notify) {
        return;
    }
    rtsp_streamer_invoke_release_cb(notify->release_cb, notify->release_user_data);
    free(notify);
}

bool rtsp_streamer_init(RTSPStreamer* streamer, 
                       int width, int height, int fps,
                       size_t max_bytes, int max_time_ms) {
    if (!streamer) {
        return false;
    }
    memset(streamer, 0, sizeof(*streamer));
    
    streamer->width = width;
    streamer->height = height;
    streamer->fps = fps;
    streamer->appsrc_max_bytes = max_bytes;
    streamer->appsrc_max_time_ms = max_time_ms;
    streamer->need_data = TRUE;
    
    g_mutex_init(&streamer->need_data_mutex);
    
    streamer->dmabuf_alloc = gst_dmabuf_allocator_new();
    if (!streamer->dmabuf_alloc) {
        fprintf(stderr, "[RTSP] WARNING: Failed to create DMABUF allocator; DMABUF push may fail.\n");
    }
    
    // fprintf(stderr, "[RTSP] Streamer initialized: %dx%d @ %dfps\n", 
    //         width, height, fps);
    // fprintf(stderr, "[RTSP] Flow control: max_bytes=%zu, max_time=%dms\n",
    //         max_bytes, max_time_ms);
    
    return true;
}

bool rtsp_streamer_push_dmabuf_nv12_single(RTSPStreamer* streamer,
                                           int fd, int stride_y, int stride_uv,
                                           int width, int height,
                                           uint64_t frame_idx) {
    return rtsp_streamer_push_dmabuf_nv12_single_with_release(
        streamer, fd, stride_y, stride_uv, width, height, frame_idx, NULL, NULL);
}

bool rtsp_streamer_push_dmabuf_nv12_single_with_release(
    RTSPStreamer* streamer,
    int fd, int stride_y, int stride_uv,
    int width, int height,
    uint64_t frame_idx,
    RTSPStreamerReleaseCallback release_cb,
    void* release_user_data) {
    if (!streamer || !streamer->appsrc) {
        rtsp_streamer_invoke_release_cb(release_cb, release_user_data);
        return false;
    }
    if (!streamer->use_dmabuf || !streamer->dmabuf_alloc) {
        rtsp_streamer_invoke_release_cb(release_cb, release_user_data);
        return false;
    }

    GstState state;
    if (gst_element_get_state(streamer->appsrc, &state, NULL, 0) == GST_STATE_CHANGE_FAILURE) {
        rtsp_streamer_invoke_release_cb(release_cb, release_user_data);
        return false;
    }
    if (state != GST_STATE_PLAYING) {
        rtsp_streamer_invoke_release_cb(release_cb, release_user_data);
        return false;
    }

    gboolean should_push = FALSE;
    g_mutex_lock(&streamer->need_data_mutex);
    should_push = streamer->need_data;
    g_mutex_unlock(&streamer->need_data_mutex);
    if (!should_push) {
        streamer->backpressure_skip_count++;
        rtsp_streamer_invoke_release_cb(release_cb, release_user_data);
        return false;
    }

    gsize y_size = (gsize)stride_y * (gsize)height;
    gsize uv_size = (gsize)stride_uv * (gsize)(height / 2);
    gsize total_size = y_size + uv_size;

    int dup_fd = dup(fd);
    if (dup_fd < 0) {
        rtsp_streamer_invoke_release_cb(release_cb, release_user_data);
        return false;
    }

    GstMemory* mem = gst_dmabuf_allocator_alloc(streamer->dmabuf_alloc, dup_fd, total_size);
    if (!mem) {
        close(dup_fd);
        rtsp_streamer_invoke_release_cb(release_cb, release_user_data);
        return false;
    }

    GstBuffer* buffer = gst_buffer_new();
    if (!buffer) {
        gst_memory_unref(mem);
        rtsp_streamer_invoke_release_cb(release_cb, release_user_data);
        return false;
    }
    gst_buffer_append_memory(buffer, mem);

    if (release_cb) {
        RTSPStreamerReleaseNotify* notify =
            (RTSPStreamerReleaseNotify*)calloc(1, sizeof(RTSPStreamerReleaseNotify));
        if (!notify) {
            gst_buffer_unref(buffer);
            rtsp_streamer_invoke_release_cb(release_cb, release_user_data);
            return false;
        }
        notify->release_cb = release_cb;
        notify->release_user_data = release_user_data;
        gst_mini_object_weak_ref(GST_MINI_OBJECT(buffer),
                                 rtsp_streamer_dmabuf_release_notify,
                                 notify);
    }

    gint n_planes = 2;
    gint strides[GST_VIDEO_MAX_PLANES] = { stride_y, stride_uv };
    gsize offsets[GST_VIDEO_MAX_PLANES] = { 0, y_size };
    gst_buffer_add_video_meta_full(buffer,
                                   GST_VIDEO_FRAME_FLAG_NONE,
                                   GST_VIDEO_FORMAT_NV12,
                                   width, height,
                                   n_planes, offsets, strides);

    if (streamer->total_frames_sent == 0) {
        streamer->pts_base_frame_idx = frame_idx;
    }
    uint64_t relative_frame_idx = frame_idx - streamer->pts_base_frame_idx;
    GstClockTime pts = relative_frame_idx * (GST_SECOND / streamer->fps);
    GST_BUFFER_PTS(buffer) = pts;
    GST_BUFFER_DTS(buffer) = pts;
    GST_BUFFER_DURATION(buffer) = GST_SECOND / streamer->fps;
    GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_LIVE);

    GstFlowReturn ret;
    g_signal_emit_by_name(streamer->appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret == GST_FLOW_OK) {
        streamer->total_frames_sent++;
        return true;
    } else {
        fprintf(stderr, "[RTSP] push-buffer(dmabuf single) failed: %d\n", ret);
        return false;
    }
}

bool rtsp_streamer_push_dmabuf_nv12(RTSPStreamer* streamer,
                                    int fd_y, int stride_y,
                                    int fd_uv, int stride_uv,
                                    int width, int height,
                                    uint64_t frame_idx) {
    if (!streamer || !streamer->appsrc) {
        return false;
    }

    if (!streamer->use_dmabuf) {
        return false;
    }
    if (!streamer->dmabuf_alloc) {
        return false;
    }

    GstState state;
    if (gst_element_get_state(streamer->appsrc, &state, NULL, 0) == GST_STATE_CHANGE_FAILURE) {
        return false;
    }
    if (state != GST_STATE_PLAYING) {
        return false;
    }

    gboolean should_push = FALSE;
    g_mutex_lock(&streamer->need_data_mutex);
    should_push = streamer->need_data;
    g_mutex_unlock(&streamer->need_data_mutex);
    if (!should_push) {
        streamer->backpressure_skip_count++;
        return false;
    }

    gsize y_size = (gsize)stride_y * (gsize)height;
    gsize uv_size = (gsize)stride_uv * (gsize)(height / 2);

    int dup_y = dup(fd_y);
    int dup_uv = dup(fd_uv);
    if (dup_y < 0 || dup_uv < 0) {
        if (dup_y >= 0) close(dup_y);
        if (dup_uv >= 0) close(dup_uv);
        return false;
    }

    GstMemory* mem_y = gst_dmabuf_allocator_alloc(streamer->dmabuf_alloc, dup_y, y_size);
    GstMemory* mem_uv = gst_dmabuf_allocator_alloc(streamer->dmabuf_alloc, dup_uv, uv_size);
    if (!mem_y || !mem_uv) {
        if (mem_y) gst_memory_unref(mem_y);
        if (mem_uv) gst_memory_unref(mem_uv);
        if(!mem_y) close(dup_y);
        if(!mem_uv) close(dup_uv);
        return false;
    }

    GstBuffer* buffer = gst_buffer_new();
    gst_buffer_append_memory(buffer, mem_y);
    gst_buffer_append_memory(buffer, mem_uv);

    gint n_planes = 2;
    gint strides[GST_VIDEO_MAX_PLANES] = { stride_y, stride_uv };
    gsize offsets[GST_VIDEO_MAX_PLANES] = { 0, y_size };
    gst_buffer_add_video_meta_full(buffer,
                                   GST_VIDEO_FRAME_FLAG_NONE,
                                   GST_VIDEO_FORMAT_NV12,
                                   width, height,
                                   n_planes, offsets, strides);

    if (streamer->total_frames_sent == 0) {
        streamer->pts_base_frame_idx = frame_idx;
    }
    uint64_t relative_frame_idx = frame_idx - streamer->pts_base_frame_idx;
    GstClockTime pts = relative_frame_idx * (GST_SECOND / streamer->fps);
    GST_BUFFER_PTS(buffer) = pts;
    GST_BUFFER_DTS(buffer) = pts;
    GST_BUFFER_DURATION(buffer) = GST_SECOND / streamer->fps;
    GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_LIVE);

    GstFlowReturn ret;
    g_signal_emit_by_name(streamer->appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
    
    if (ret == GST_FLOW_OK) {
        streamer->total_frames_sent++;
        return true;
    } else {
        fprintf(stderr, "[RTSP] push-buffer(dmabuf) failed: %d\n", ret);
        return false;
    }
}

bool rtsp_streamer_push_frame(RTSPStreamer* streamer, 
                             const void* nv12_data, size_t size,
                             uint64_t frame_idx) {
    if (!streamer || !nv12_data) {
        return false;
    }
    
    if (!streamer->appsrc) {
        return false;
    }
    
    GstState state;
    if (gst_element_get_state(streamer->appsrc, &state, NULL, 0) == GST_STATE_CHANGE_FAILURE) {
        return false;
    }
    
    if (state != GST_STATE_PLAYING) {
        return false;
    }
    
    gboolean should_push = FALSE;
    g_mutex_lock(&streamer->need_data_mutex);
    should_push = streamer->need_data;
    g_mutex_unlock(&streamer->need_data_mutex);
    
    if (!should_push) {
        streamer->backpressure_skip_count++;
        return false;
    }
    
    GstBuffer* buffer = gst_buffer_new_allocate(NULL, size, NULL);
    if (!buffer) {
        fprintf(stderr, "[RTSP] Failed to allocate buffer\n");
        return false;
    }
    
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        gst_buffer_unref(buffer);
        return false;
    }
    
    memcpy(map.data, nv12_data, size);
    gst_buffer_unmap(buffer, &map);
    
    if (streamer->total_frames_sent == 0) {
        streamer->pts_base_frame_idx = frame_idx;
        
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        fprintf(stderr, "[RTSP]  First frame pushed at %ld.%03ld (%dx%d, frame_idx=%llu, pts_base=%llu)\n",
                ts.tv_sec, ts.tv_nsec / 1000000,
                streamer->width, streamer->height,
                (unsigned long long)frame_idx,
                (unsigned long long)streamer->pts_base_frame_idx);
    }
    
    uint64_t relative_frame_idx = frame_idx - streamer->pts_base_frame_idx;
    GstClockTime pts = relative_frame_idx * (GST_SECOND / streamer->fps);
    GST_BUFFER_PTS(buffer) = pts;
    GST_BUFFER_DTS(buffer) = pts;
    GST_BUFFER_DURATION(buffer) = GST_SECOND / streamer->fps;
    GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_LIVE);
    
    GstFlowReturn ret;
    g_signal_emit_by_name(streamer->appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
    
    if (ret == GST_FLOW_OK) {
        streamer->total_frames_sent++;
        return true;
    } else {
        fprintf(stderr, "[RTSP] push-buffer failed: %d\n", ret);
        return false;
    }
}

bool rtsp_streamer_is_ready(RTSPStreamer* streamer) {
    if (!streamer || !streamer->appsrc) {
        return false;
    }
    
    GstState state;
    if (gst_element_get_state(streamer->appsrc, &state, NULL, 0) == GST_STATE_CHANGE_FAILURE) {
        return false;
    }
    
    return (state == GST_STATE_PLAYING);
}

void rtsp_streamer_cleanup(RTSPStreamer* streamer) {
    if (!streamer) {
        return;
    }
    
    if (streamer->dmabuf_alloc) {
        gst_object_unref(streamer->dmabuf_alloc);
        streamer->dmabuf_alloc = NULL;
    }

    g_mutex_clear(&streamer->need_data_mutex);
    
    fprintf(stderr, "[RTSP] Streamer cleanup: sent=%llu, skipped=%llu\n",
           (unsigned long long)streamer->total_frames_sent,
           (unsigned long long)streamer->backpressure_skip_count);
}
