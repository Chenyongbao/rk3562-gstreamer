#define _POSIX_C_SOURCE 199309L

#include "rtsp_streamer.h"

#include <unistd.h>

#include <gst/allocators/gstdmabuf.h>
#include <gst/video/video.h>

#include "core/logging.h"

// ============================================================================
// appsrc 的线程安全访问。
// appsrc 由 RTSP 回调线程（media-configure / unprepared）写入，由推流工作线程与
// 业务线程（is_ready / 检测）读取——裸读写是 data race 且会 use-after-free。
// 统一走 set / take / has：take 持一份引用，使元素在跨线程操作期间保持存活。
// ============================================================================

void rtsp_streamer_set_appsrc(RTSPStreamer* streamer, GstElement* appsrc) {
    if (!streamer) {
        return;
    }
    g_mutex_lock(&streamer->appsrc_mutex);
    if (streamer->appsrc) {
        gst_object_unref(streamer->appsrc);
    }
    streamer->appsrc = appsrc;
    g_mutex_unlock(&streamer->appsrc_mutex);
}

GstElement* rtsp_streamer_take_appsrc(RTSPStreamer* streamer) {
    GstElement* appsrc = NULL;
    if (streamer) {
        g_mutex_lock(&streamer->appsrc_mutex);
        if (streamer->appsrc) {
            appsrc = gst_object_ref(streamer->appsrc);
        }
        g_mutex_unlock(&streamer->appsrc_mutex);
    }
    return appsrc;
}

bool rtsp_streamer_has_appsrc(RTSPStreamer* streamer) {
    bool has = false;
    if (streamer) {
        g_mutex_lock(&streamer->appsrc_mutex);
        has = (streamer->appsrc != NULL);
        g_mutex_unlock(&streamer->appsrc_mutex);
    }
    return has;
}

// appsrc 是否处于 PLAYING（调用方须保证 appsrc 非空且持有引用）。
static bool rtsp_appsrc_is_playing(GstElement* appsrc) {
    GstState state;
    if (gst_element_get_state(appsrc, &state, NULL, 0) == GST_STATE_CHANGE_FAILURE) {
        return false;
    }
    return (state == GST_STATE_PLAYING);
}

// 背压查询：appsrc 是否还需要数据（need-data/enough-data 维护的标志）。
static bool rtsp_should_push(RTSPStreamer* streamer) {
    gboolean should_push = FALSE;
    g_mutex_lock(&streamer->need_data_mutex);
    should_push = streamer->need_data;
    g_mutex_unlock(&streamer->need_data_mutex);
    return (should_push == TRUE);
}

// 核心推流：对已确认 PLAYING 的 appsrc 推一块已构造好的 buffer。
// 负责打时间戳 + push-buffer + 接管并释放传入 buffer 的一份引用，返回推流结果。
// 调用方不得再 unref 该 buffer；appsrc 的引用由调用方自行释放（本函数不负责）。
static bool rtsp_push_core(RTSPStreamer* streamer, GstElement* appsrc,
                           GstBuffer* buffer, uint64_t frame_idx) {
    // 以首帧为基准、按 fps 计算相对 PTS（保证时序单调）。
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
    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret == GST_FLOW_OK) {
        streamer->total_frames_sent++;
        return true;
    }
    spdlog::error("[RTSP] push-buffer failed: {}", static_cast<int>(ret));
    return false;
}

// 初始化一路推流的状态（appsrc 需等客户端连接后在 media 工厂里创建）。
// 注意：不能用 memset 整体清零——结构体内含 GMutex，必须由 g_mutex_init 初始化，
// memset 清零 GLib 互斥锁属未定义行为（换版本/平台可能崩溃）。这里逐字段显式赋值。
bool rtsp_streamer_init(RTSPStreamer* streamer,
                       int width, int height, int fps,
                       size_t max_bytes, int max_time_ms) {
    if (!streamer) {
        return false;
    }
    streamer->appsrc = NULL;
    g_mutex_init(&streamer->appsrc_mutex);
    streamer->width = width;
    streamer->height = height;
    streamer->fps = fps;
    streamer->appsrc_max_bytes = max_bytes;
    streamer->appsrc_max_time_ms = max_time_ms;
    streamer->use_dmabuf = FALSE;
    streamer->need_data = TRUE;
    streamer->total_frames_sent = 0;
    streamer->backpressure_skip_count = 0;
    streamer->pts_base_frame_idx = 0;

    g_mutex_init(&streamer->need_data_mutex);

    streamer->dmabuf_alloc = gst_dmabuf_allocator_new();
    if (!streamer->dmabuf_alloc) {
        spdlog::warn("[RTSP] WARNING: Failed to create DMABUF allocator");
    }
    return true;
}

// 把一块 NV12 DMABUF 包装成 GstBuffer 推给 appsrc。
// 所有权约定：只接管构造出的 buffer 这一份引用（由 appsrc 消费后释放）；
// fd 与底层资源不归本函数管理，由调用方自行释放。
bool rtsp_streamer_push_dmabuf_nv12_single(RTSPStreamer* streamer,
                                           int fd, int stride_y, int stride_uv,
                                           int width, int height,
                                           uint64_t frame_idx) {
    if (!streamer || !streamer->use_dmabuf || !streamer->dmabuf_alloc) {
        return false;
    }

    GstElement* appsrc = rtsp_streamer_take_appsrc(streamer);
    if (!appsrc) {
        return false;
    }
    if (!rtsp_appsrc_is_playing(appsrc)) {
        gst_object_unref(appsrc);
        return false;
    }
    if (!rtsp_should_push(streamer)) {
        // appsrc 队列已满（背压），丢帧。
        streamer->backpressure_skip_count++;
        gst_object_unref(appsrc);
        return false;
    }

    gsize y_size = (gsize)stride_y * (gsize)height;
    gsize uv_size = (gsize)stride_uv * (gsize)(height / 2);
    gsize total_size = y_size + uv_size;

    // dup 一份 fd：GstBuffer 的释放与上游 fd 生命周期解耦。
    int dup_fd = dup(fd);
    if (dup_fd < 0) {
        gst_object_unref(appsrc);
        return false;
    }

    // 把处理好的数据重新打包成 dmabuf 的 GstMemory。
    GstMemory* mem = gst_dmabuf_allocator_alloc(streamer->dmabuf_alloc, dup_fd, total_size);
    if (!mem) {
        close(dup_fd);
        gst_object_unref(appsrc);
        return false;
    }

    GstBuffer* buffer = gst_buffer_new();
    if (!buffer) {
        gst_memory_unref(mem);
        gst_object_unref(appsrc);
        return false;
    }
    gst_buffer_append_memory(buffer, mem);

    gint n_planes = 2;
    gint strides[GST_VIDEO_MAX_PLANES] = { stride_y, stride_uv };
    gsize offsets[GST_VIDEO_MAX_PLANES] = { 0, y_size };
    // 附加 NV12 视频元信息（宽高/stride/offset），供下游 mpph264enc 正确读取。
    gst_buffer_add_video_meta_full(buffer,
                                   GST_VIDEO_FRAME_FLAG_NONE,
                                   GST_VIDEO_FORMAT_NV12,
                                   width, height,
                                   n_planes, offsets, strides);

    // 核心推流（时间戳 + push + unref buffer）。
    bool ok = rtsp_push_core(streamer, appsrc, buffer, frame_idx);
    gst_object_unref(appsrc);
    return ok;
}

// 直接推送一块已带 caps/video-meta 的 GstBuffer（原画分支缩放后由 appsink 产出）。
// 与 push_dmabuf_nv12 系列不同：不 dup fd、不重建 buffer、不附加 video meta——源 buffer
// 自带的 caps 与 GstVideoMeta（v4l2src/rgaconvert 已填好）直接透传，实现真零拷贝。
// 所有权约定：本函数总是释放传入的这一份引用（成功进管线 / 失败丢弃皆如此），
// 调用方 push 之后不得再 unref 该 buffer。
bool rtsp_streamer_push_buffer(RTSPStreamer* streamer, GstBuffer* buffer,
                               uint64_t frame_idx) {
    if (!buffer) {
        return false;
    }
    if (!streamer || !streamer->use_dmabuf) {
        gst_buffer_unref(buffer);
        return false;
    }

    GstElement* appsrc = rtsp_streamer_take_appsrc(streamer);
    if (!appsrc) {
        gst_buffer_unref(buffer);
        return false;
    }
    if (!rtsp_appsrc_is_playing(appsrc)) {
        gst_object_unref(appsrc);
        gst_buffer_unref(buffer);
        return false;
    }

    // 背压：need_data=FALSE 时丢帧（与 push_dmabuf_nv12 系列一致）。
    if (!rtsp_should_push(streamer)) {
        streamer->backpressure_skip_count++;
        gst_object_unref(appsrc);
        gst_buffer_unref(buffer);
        return false;
    }

    // 核心推流（时间戳 + push + unref buffer）。
    bool ok = rtsp_push_core(streamer, appsrc, buffer, frame_idx);
    gst_object_unref(appsrc);
    return ok;
}

// 编码管线是否就绪：appsrc 存在且已进入 PLAYING（线程安全：持引用后查询再释放）。
bool rtsp_streamer_is_ready(RTSPStreamer* streamer) {
    GstElement* appsrc = rtsp_streamer_take_appsrc(streamer);
    if (!appsrc) {
        return false;
    }
    bool ready = rtsp_appsrc_is_playing(appsrc);
    gst_object_unref(appsrc);
    return ready;
}

// 释放一路推流状态（残余 appsrc 引用、DMABUF 分配器与互斥锁）。
void rtsp_streamer_cleanup(RTSPStreamer* streamer) {
    if (!streamer) {
        return;
    }

    // 先在线程安全接口下释放可能遗留的 appsrc 引用，再清理互斥锁。
    rtsp_streamer_set_appsrc(streamer, NULL);

    if (streamer->dmabuf_alloc) {
        gst_object_unref(streamer->dmabuf_alloc);
        streamer->dmabuf_alloc = NULL;
    }

    g_mutex_clear(&streamer->need_data_mutex);
    g_mutex_clear(&streamer->appsrc_mutex);

    spdlog::info("[RTSP] Streamer cleanup: sent={}, skipped={}",
                 streamer->total_frames_sent,
                 streamer->backpressure_skip_count);
}
