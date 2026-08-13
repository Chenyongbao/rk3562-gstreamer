#ifndef DUAL_RTSP_SERVER_H
#define DUAL_RTSP_SERVER_H

#include <stdint.h>
#include <stdbool.h>
#include <glib.h>
#include <gst/rtsp-server/rtsp-server.h>
#include "rtsp_streamer.h"

typedef struct {

    GMainLoop* main_loop;
    GThread* main_loop_thread;
    GstRTSPServer* rtsp_server;
    

    RTSPStreamer original_stream;
    RTSPStreamer bev_stream;
    

    void* bev_processor;


    uint64_t global_frame_idx;
    GMutex frame_idx_mutex;
} DualRTSPServer;


bool dual_rtsp_server_init(DualRTSPServer* server, const char* bind_address);


uint64_t dual_rtsp_server_get_next_frame_idx(DualRTSPServer* server);


void dual_rtsp_server_cleanup(DualRTSPServer* server);

#endif // DUAL_RTSP_SERVER_H

