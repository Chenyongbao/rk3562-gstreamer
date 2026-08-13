#define _POSIX_C_SOURCE 199309L

#include "dual_rtsp_server.h"
#include "../config.h"
#include "../camera_calibreation/klipper/klipper_manager.h"
#include "../reallink_ogles/bev_api.h"
#include <stdio.h>
#include <string.h>
#include <string>
#include <time.h>

static gpointer main_loop_thread_func(gpointer user_data) {
    GMainLoop* loop = (GMainLoop*)user_data;
    fprintf(stderr, "[RTSP] Main loop thread started\n");
    g_main_loop_run(loop);
    fprintf(stderr, "[RTSP] Main loop thread stopped\n");
    return NULL;
}

static void media_configured_cb(GstRTSPMediaFactory* factory, GstRTSPMedia* media, RTSPStreamer* streamer);
static void media_unprepared_cb(GstRTSPMedia* media, RTSPStreamer* streamer);

static void on_need_data(GstElement* appsrc, guint unused, RTSPStreamer* streamer) {
    (void)appsrc;
    (void)unused;
    
    g_mutex_lock(&streamer->need_data_mutex);
    streamer->need_data = TRUE;
    g_mutex_unlock(&streamer->need_data_mutex);
}

static void on_enough_data(GstElement* appsrc, RTSPStreamer* streamer) {
    (void)appsrc;
    
    g_mutex_lock(&streamer->need_data_mutex);
    streamer->need_data = FALSE;
    g_mutex_unlock(&streamer->need_data_mutex);
}

static void media_configured_cb(GstRTSPMediaFactory* factory, GstRTSPMedia* media, RTSPStreamer* streamer) {
    (void)factory;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    fprintf(stderr, "[RTSP]  Client connected at %ld.%03ld, configuring media...\n",
            ts.tv_sec, ts.tv_nsec / 1000000);

    const bool is_bev_stream =
        streamer && streamer->width == BEV_OUTPUT_WIDTH && streamer->height == BEV_OUTPUT_HEIGHT;
    if (is_bev_stream) {

        std::string wait_work;
        if(KlipperManager::instance().sendGcode("g4 p20\n",nullptr,5L,&wait_work))
        {
            fprintf(stderr, "[Unified Server] G4 wait command response: %s\n", wait_work.c_str());
        } else {
            fprintf(stderr, "[Unified Server] WARNING: Failed to send G4 wait command\n");
        }
    }
	    
    GstElement* element = gst_rtsp_media_get_element(media);
    if (!element) {
        fprintf(stderr, "[RTSP] ERROR: Failed to get media element\n");
        return;
    }

    if (streamer->appsrc) {
        gst_object_unref(streamer->appsrc);
        streamer->appsrc = NULL;
    }
    
    GstElement* appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(element), "mysrc");
    if (!appsrc) {
        fprintf(stderr, "[RTSP] ERROR: Cannot find appsrc 'mysrc'\n");
        g_object_unref(element);
        return;
    }
    
    g_signal_connect(appsrc, "need-data", G_CALLBACK(on_need_data), streamer);
    g_signal_connect(appsrc, "enough-data", G_CALLBACK(on_enough_data), streamer);
    
    gst_util_set_object_arg(G_OBJECT(appsrc), "format", "time");
    
    size_t max_bytes = streamer->appsrc_max_bytes;
    if (max_bytes == 0) {
        max_bytes = (streamer->width == ORIGINAL_WIDTH) ?
                    APPSRC_MAX_BYTES_ORIGINAL : APPSRC_MAX_BYTES_BEV;
    }
    int max_time_ms = streamer->appsrc_max_time_ms > 0 ?
                      streamer->appsrc_max_time_ms : APPSRC_MAX_TIME_MS;
    
    g_object_set(G_OBJECT(appsrc),
                 "max-bytes", (guint64)max_bytes,
                 "max-time", (guint64)(max_time_ms * GST_MSECOND),
                 "block", FALSE,
                 NULL);
    
    clock_gettime(CLOCK_MONOTONIC, &ts);
    fprintf(stderr, "[RTSP]  AppSrc configured at %ld.%03ld: %dx%d, max_bytes=%zu, max_time=%dms\n",
            ts.tv_sec, ts.tv_nsec / 1000000,
            streamer->width, streamer->height, max_bytes, max_time_ms);

    streamer->total_frames_sent = 0;
    streamer->backpressure_skip_count = 0;
    

    //  关键修改：等待管道进入 PLAYING 状态后再设置 appsrc
    GstState current_state, pending_state;
    GstStateChangeReturn ret = gst_element_get_state(appsrc, &current_state, &pending_state, 500 * GST_MSECOND);
    
    if (ret == GST_STATE_CHANGE_SUCCESS && current_state == GST_STATE_PLAYING) {
        // 管道已经是 PLAYING 状态，立即设置
        streamer->appsrc = GST_ELEMENT(gst_object_ref(appsrc));
        fprintf(stderr, "[RTSP]  Pipeline already PLAYING, appsrc ready immediately\n");
    } else if (ret == GST_STATE_CHANGE_ASYNC || pending_state == GST_STATE_PLAYING) {
        // 管道正在切换到 PLAYING，等待完成
        fprintf(stderr, "[RTSP]  Waiting for pipeline to reach PLAYING state (current=%d, pending=%d)...\n",
                current_state, pending_state);
        
        ret = gst_element_get_state(appsrc, &current_state, NULL, 2 * GST_SECOND);
        if (ret == GST_STATE_CHANGE_SUCCESS && current_state == GST_STATE_PLAYING) {
            streamer->appsrc = GST_ELEMENT(gst_object_ref(appsrc));
            fprintf(stderr, "[RTSP]  Pipeline reached PLAYING, appsrc ready\n");
        } else {
            fprintf(stderr, "[RTSP]  Pipeline state change timeout or failed (ret=%d, state=%d)\n",
                    ret, current_state);
            // 仍然设置 appsrc，但工作线程会检查状态
            streamer->appsrc = GST_ELEMENT(gst_object_ref(appsrc));
        }
    } else {
        // 其他情况，仍然设置但打印警告
        fprintf(stderr, "[RTSP]  Unexpected pipeline state (ret=%d, current=%d, pending=%d)\n",
                ret, current_state, pending_state);
        streamer->appsrc = GST_ELEMENT(gst_object_ref(appsrc));
    }

    g_signal_connect(media, "unprepared", G_CALLBACK(media_unprepared_cb), streamer);
    
    gst_object_unref(appsrc);
    g_object_unref(element);
}

static void media_unprepared_cb(GstRTSPMedia* media, RTSPStreamer* streamer) {
    (void)media;
    
    fprintf(stderr, "[RTSP] Client disconnected\n");
    
    if (streamer->appsrc) {
        gst_object_unref(streamer->appsrc);
        streamer->appsrc = NULL;
    }
}

static GstRTSPMediaFactory* create_media_factory(int width, int height, int fps, 
                                                 RTSPStreamer* streamer) {
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();

    int bps = (int)(width * height * fps * ENCODE_BPP);

    int actual_gop = (ENCODE_GOP < fps) ? fps : ENCODE_GOP;

    const char* mem_caps = streamer && streamer->use_dmabuf ? "(memory:DMABuf)" : "";
    gchar* launch = NULL;
    if (width == BEV_OUTPUT_WIDTH && height == BEV_OUTPUT_HEIGHT) {

        launch = g_strdup_printf(
            "( appsrc name=mysrc is-live=1 format=time do-timestamp=1 "
            "caps=video/x-raw%s,format=NV12,width=%d,height=%d,framerate=%d/1 ! "
            "queue leaky=2 max-size-buffers=1 max-size-time=0 max-size-bytes=0 ! "
            "videoconvert ! mpph264enc bps=%d gop=%d rc-mode=cbr profile=baseline ! "
            "h264parse ! "
            "queue leaky=2 max-size-buffers=1 max-size-time=0 max-size-bytes=0 ! "
            "rtph264pay name=pay0 pt=96 mtu=1200 config-interval=-1 aggregate-mode=none )",
            mem_caps, width, height, fps, bps, actual_gop
        );
    } else {

        launch = g_strdup_printf(
            "( appsrc name=mysrc is-live=1 format=time do-timestamp=1 "
            "caps=video/x-raw%s,format=NV12,width=%d,height=%d,framerate=%d/1 ! "
            "queue max-size-buffers=3 ! "
            "mpph264enc bps=%d gop=%d rc-mode=cbr ! "
            "h264parse ! "
            "rtph264pay name=pay0 pt=96 mtu=1200 config-interval=-1 aggregate-mode=none )",
            mem_caps, width, height, fps, bps, actual_gop
        );
    }
    
    fprintf(stderr, "[RTSP] Pipeline (shared=false): %s\n", launch);
    
    gst_rtsp_media_factory_set_launch(factory, launch);
    gst_rtsp_media_factory_set_shared(factory, FALSE);
    

    g_signal_connect(factory, "media-configure", G_CALLBACK(media_configured_cb), streamer);
    
    g_free(launch);
    return factory;
}

bool dual_rtsp_server_init(DualRTSPServer* server, const char* bind_address) {
    if (!server) {
        return false;
    }
    (void)bind_address;
    
    memset(server, 0, sizeof(*server));

    gst_init(NULL, NULL);
    
    server->main_loop = g_main_loop_new(NULL, FALSE);

    server->rtsp_server = gst_rtsp_server_new();

    server->global_frame_idx = 0;
    g_mutex_init(&server->frame_idx_mutex);
    
    // 强制绑定到配置的IP地址
    const char* address_to_bind = SERVER_BIND_ADDRESS;
    //fprintf(stderr, "[NETWORK] Binding RTSP to configured IP: %s\n", address_to_bind);
    
    gst_rtsp_server_set_address(server->rtsp_server, address_to_bind);
    gst_rtsp_server_set_service(server->rtsp_server, RTSP_SERVER_PORT);
    
    if (!rtsp_streamer_init(&server->original_stream, 
                           ORIGINAL_WIDTH, ORIGINAL_HEIGHT, TARGET_FPS,
                           APPSRC_MAX_BYTES_ORIGINAL, APPSRC_MAX_TIME_MS)) {
        fprintf(stderr, "[RTSP] ERROR: Failed to init original stream\n");
        return false;
    }

    server->original_stream.use_dmabuf = TRUE;
    
    if (!rtsp_streamer_init(&server->bev_stream, 
                           BEV_OUTPUT_WIDTH, BEV_OUTPUT_HEIGHT, TARGET_FPS,
                           APPSRC_MAX_BYTES_BEV, APPSRC_MAX_TIME_MS)) {
        fprintf(stderr, "[RTSP] ERROR: Failed to init BEV stream\n");
        rtsp_streamer_cleanup(&server->original_stream);
        return false;
    }
    server->bev_stream.use_dmabuf = TRUE;
    
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server->rtsp_server);
    
    GstRTSPMediaFactory* original_factory = create_media_factory(
        ORIGINAL_WIDTH, ORIGINAL_HEIGHT, TARGET_FPS, &server->original_stream);
    gst_rtsp_mount_points_add_factory(mounts, RTSP_PATH_ORIGINAL, original_factory);
    
    GstRTSPMediaFactory* bev_factory = create_media_factory(
        BEV_OUTPUT_WIDTH, BEV_OUTPUT_HEIGHT, TARGET_FPS, &server->bev_stream);
    gst_rtsp_mount_points_add_factory(mounts, RTSP_PATH_BEV, bev_factory);
    
    g_object_unref(mounts);
    
    gst_rtsp_server_attach(server->rtsp_server, NULL);
    
    server->main_loop_thread = g_thread_new("rtsp-main-loop", 
                                            main_loop_thread_func, 
                                            server->main_loop);
    if (!server->main_loop_thread) {
        fprintf(stderr, "[RTSP] ERROR: Failed to create main loop thread\n");
        return false;
    }
    
    g_usleep(100000); // 100ms
    
    // fprintf(stderr, "[RTSP] Initializing BEV processor with params: %dx%d -> %dx%d\n",
    //         BEV_INPUT_WIDTH, BEV_INPUT_HEIGHT, BEV_OUTPUT_WIDTH, BEV_OUTPUT_HEIGHT);
    server->bev_processor = bev_init(BEV_INPUT_WIDTH, BEV_INPUT_HEIGHT,
                                     BEV_OUTPUT_WIDTH, BEV_OUTPUT_HEIGHT);
    if (!server->bev_processor) {
        fprintf(stderr, "[BEV] WARNING: BEV processor init failed\n");
    } else {
        fprintf(stderr, "[BEV]  BEV processor initialized successfully\n");
    }
    
    fprintf(stderr, "\n");
    fprintf(stderr, "========================================\n");
    fprintf(stderr, " RTSP Server Started\n");
    fprintf(stderr, "========================================\n");
    fprintf(stderr, "Address:  %s:%s\n", address_to_bind, RTSP_SERVER_PORT);
    fprintf(stderr, "Original: rtsp://%s:%s%s\n", address_to_bind, RTSP_SERVER_PORT, RTSP_PATH_ORIGINAL);
    fprintf(stderr, "BEV:      rtsp://%s:%s%s\n", address_to_bind, RTSP_SERVER_PORT, RTSP_PATH_BEV);
    fprintf(stderr, "========================================\n\n");
    
    return true;
}

uint64_t dual_rtsp_server_get_next_frame_idx(DualRTSPServer* server) {
    if (!server) {
        return 0;
    }
    
    g_mutex_lock(&server->frame_idx_mutex);
    uint64_t current_idx = server->global_frame_idx;
    server->global_frame_idx++;
    g_mutex_unlock(&server->frame_idx_mutex);
    
    return current_idx;
}

void dual_rtsp_server_cleanup(DualRTSPServer* server) {
    if (!server) {
        return;
    }
    

    if (server->bev_processor) {
        bev_cleanup(server->bev_processor);
        server->bev_processor = NULL;
    }

    rtsp_streamer_cleanup(&server->original_stream);
    rtsp_streamer_cleanup(&server->bev_stream);
    
    g_mutex_clear(&server->frame_idx_mutex);
    

    if (server->main_loop) {
        if (g_main_loop_is_running(server->main_loop)) {
            fprintf(stderr, "[RTSP] Stopping main loop...\n");
            g_main_loop_quit(server->main_loop);
        }
        

        if (server->main_loop_thread) {
            g_thread_join(server->main_loop_thread);
            server->main_loop_thread = NULL;
        }
        
        g_main_loop_unref(server->main_loop);
        server->main_loop = NULL;
    }
    

    if (server->rtsp_server) {
        g_object_unref(server->rtsp_server);
        server->rtsp_server = NULL;
    }
    
    fprintf(stderr, "[RTSP] Server cleaned up\n");
}

