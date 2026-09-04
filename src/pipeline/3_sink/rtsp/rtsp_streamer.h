#ifndef RTSP_STREAMER_H
#define RTSP_STREAMER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <glib.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// 前向声明：仅在客户端连接回调里"轻触"设备（如 BEV 连接时发 G4 唤醒），
// 不在此头文件引入 klipper 依赖，避免污染纯 C 风格结构体。
class KlipperManager;

// 一路 RTSP 推流的运行时状态。
// appsrc 由 gst-rtsp-server 的 media 工厂动态创建——有客户端连接时才存在，
// 因此它是"是否有人观看"的判断依据。
typedef struct RTSPStreamer {
    GstElement* appsrc;              // 编码管线内的 appsrc（无客户端时为 NULL）
    GMutex appsrc_mutex;             // 保护 appsrc：RTSP 回调线程写、推流/业务线程读，
                                     // 跨线程访问必须走 set/take/has_appsrc
    int width;
    int height;
    int fps;
    size_t appsrc_max_bytes;         // appsrc 队列字节上限
    int appsrc_max_time_ms;          // appsrc 队列时长上限
    gboolean use_dmabuf;             // 是否走 DMABUF 零拷贝
    GstAllocator* dmabuf_alloc;      // DMABUF 分配器（把裸 fd 包装成 GstMemory）
    gboolean need_data;              // 背压标志：appsrc 是否还需要数据
    GMutex need_data_mutex;
    uint64_t total_frames_sent;      // 累计发送帧数
    uint64_t backpressure_skip_count;// 因背压丢弃的帧数
    uint64_t pts_base_frame_idx;     // 首帧序号，PTS 以此为基准（相对计时）

    KlipperManager* klipper;         // 设备客户端（依赖注入，供 BEV 连接时发 G4 唤醒；可空）
} RTSPStreamer;

// 初始化流状态（appsrc 此时尚未创建，等客户端连接后再填充）。
bool rtsp_streamer_init(RTSPStreamer* streamer,
                       int width, int height, int fps,
                       size_t max_bytes, int max_time_ms);

// 推送单 DMABUF NV12 帧：把 fd 包装成 GstBuffer 推给 appsrc。
// 本函数只接管 buffer 这一份引用，不关闭/归还 fd 及池槽位——
// 底层资源的生命周期由调用方管理（如 BEV 路径的 pool slot 归还）。
bool rtsp_streamer_push_dmabuf_nv12_single(RTSPStreamer* streamer,
                                           int fd, int stride_y, int stride_uv,
                                           int width, int height,
                                           uint64_t frame_idx);

// 直接推送一块已带 caps/video-meta 的 GstBuffer（原画分支缩放后由 appsink 产出）。
// 不 dup fd、不重建 buffer，真零拷贝；接管 buf 的一份引用，下游用完自动 unref。
bool rtsp_streamer_push_buffer(RTSPStreamer* streamer, GstBuffer* buffer,
                               uint64_t frame_idx);

// 当前 appsrc 的线程安全访问（RTSP 回调线程写、推流/业务线程读）：
void rtsp_streamer_set_appsrc(RTSPStreamer* streamer, GstElement* appsrc); // 设置（NULL=清除），仅 media 工厂回调调用
GstElement* rtsp_streamer_take_appsrc(RTSPStreamer* streamer);             // 取临时引用（调用方必须 gst_object_unref）
bool rtsp_streamer_has_appsrc(RTSPStreamer* streamer);                     // 是否有客户端连接

// 编码管线是否已就绪（appsrc 存在且处于 PLAYING，线程安全）。
bool rtsp_streamer_is_ready(RTSPStreamer* streamer);

void rtsp_streamer_cleanup(RTSPStreamer* streamer);

#endif // RTSP_STREAMER_H
