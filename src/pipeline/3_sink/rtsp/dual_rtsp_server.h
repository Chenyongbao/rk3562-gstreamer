#ifndef DUAL_RTSP_SERVER_H
#define DUAL_RTSP_SERVER_H

#include <stdint.h>
#include <stdbool.h>
#include <glib.h>
#include <gst/rtsp-server/rtsp-server.h>
#include "rtsp_streamer.h"

class KlipperManager;  // 前向声明，避免在 C 风格头文件里引入 klipper 依赖

// 双路 RTSP 服务器：原画流 + BEV 流，共用一个 gst-rtsp-server。
// 采用 shared=false 的 media 工厂——每个客户端连接都拉起独立编码管线。
typedef struct {
    GMainLoop* main_loop;          // GLib 主循环（跑在独立线程）
    GThread* main_loop_thread;

    GstRTSPServer* rtsp_server;    // gst-rtsp-server 实例

    RTSPStreamer original_stream;  // 原画流（1280x960）
    RTSPStreamer bev_stream;       // BEV 流（1280x1280）

    void* bev_processor;           // BEV 处理器（OpenGL+YOLO），失败可为空
} DualRTSPServer;

// 初始化 RTSP 服务器并创建两路挂载点。
// klipper 为设备客户端（依赖注入，供 BEV 连接回调发 G4 唤醒），可为空。
bool dual_rtsp_server_init(DualRTSPServer* server, const char* bind_address, KlipperManager* klipper = nullptr);

void dual_rtsp_server_cleanup(DualRTSPServer* server);

#endif // DUAL_RTSP_SERVER_H
