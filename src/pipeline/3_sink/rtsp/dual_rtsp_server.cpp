#define _POSIX_C_SOURCE 199309L

#include "dual_rtsp_server.h"
#include "config.h"
#include "core/logging.h"
#include "klipper/klipper_manager.h"
#include "reallink_ogles/bev_api.h"
#include <stdio.h>
#include <string.h>
#include <string>
#include <time.h>

// ============================================================================
// 双路 RTSP 服务器：原画流 + BEV 流，共用一个 gst-rtsp-server 实例。
//
// 架构要点：
//   - 两路流各对应一个挂载点（/stream_original、/stream_bev）。
//   - 每个挂载点由一个 GstRTSPMediaFactory 描述，工厂里定义了
//     appsrc -> mpph264enc -> rtph264pay 的编码管线字符串。
//   - shared=false：每个客户端连接都拉起一条独立编码管线（互不影响）。
//   - appsrc 是"是否有观众"的唯一信号：客户端连接后 media 工厂动态创建
//     appsrc 并填进 RTSPStreamer；断开后置 NULL，推流工作线程据此停止工作。
// ============================================================================

// 宏说明：_POSIX_C_SOURCE 必须在所有系统头之前定义，值 199309L 对应 POSIX.1b
// 实时扩展，用于启用 clock_gettime()（本文件记录连接/配置时间戳用）。
// 见文件第 1 行。

// RTSP 主循环线程：跑 GLib 事件循环，处理 RTSP 信令（DESCRIBE/SETUP/PLAY/TEARDOWN）。
// 为什么必须跑在独立线程？
//   g_main_loop_run() 是阻塞调用，会一直循环分发事件直到 g_main_loop_quit()。
//   若跑在主线程，主线程就无法继续做快照循环等其他工作。所以丢到独立线程，
//   主线程初始化完 RTSP 后继续往下走。
static gpointer main_loop_thread_func(gpointer user_data) {
    GMainLoop* loop = (GMainLoop*)user_data;
    spdlog::info("[RTSP] Main loop thread started");
    g_main_loop_run(loop);
    spdlog::info("[RTSP] Main loop thread stopped");
    return NULL;
}

// 前置声明：media 工厂的两个生命周期回调（定义见下方）。
static void media_configured_cb(GstRTSPMediaFactory* factory, GstRTSPMedia* media, RTSPStreamer* streamer);
static void media_unprepared_cb(GstRTSPMedia* media, RTSPStreamer* streamer);

// appsrc 的 need-data 信号回调：管线说"我缺数据，快来喂"，置背压标志允许推流。
// 信号由 GStreamer 的流控线程触发，因此必须加锁保护跨线程访问的 need_data。
static void on_need_data(GstElement* appsrc, guint unused, RTSPStreamer* streamer) {
    (void)appsrc;
    (void)unused;

    g_mutex_lock(&streamer->need_data_mutex);
    streamer->need_data = TRUE;
    g_mutex_unlock(&streamer->need_data_mutex);
}

// appsrc 的 enough-data 信号回调：管线队列已满，清背压标志（推流侧据此丢帧）。
// 与 on_need_data 成对，构成"背压闭环"：
//   队列空 -> need-data -> need_data=TRUE  -> 工作线程推帧
//   队列满 -> enough-data -> need_data=FALSE -> 工作线程丢帧（见 rtsp_streamer.cpp）
static void on_enough_data(GstElement* appsrc, RTSPStreamer* streamer) {
    (void)appsrc;

    g_mutex_lock(&streamer->need_data_mutex);
    streamer->need_data = FALSE;
    g_mutex_unlock(&streamer->need_data_mutex);
}

// 客户端连接后，media 工厂回调：从管线里取出 appsrc，配置好参数交给推流线程使用。
// 这是本文件的核心——它把"GStreamer 内部创建的 appsrc"和"我们维护的 RTSPStreamer"对接起来。
static void media_configured_cb(GstRTSPMediaFactory* factory, GstRTSPMedia* media, RTSPStreamer* streamer) {
    (void)factory;

    // 打点记录客户端连接时刻（相对单调时钟，用于日志排障）。
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    spdlog::info("[RTSP]  Client connected at {}.{:03}, configuring media...",
                 ts.tv_sec, ts.tv_nsec / 1000000);

    // 通过宽高识别当前连的是不是 BEV 流。
    const bool is_bev_stream =
        streamer && streamer->width == BEV_OUTPUT_WIDTH && streamer->height == BEV_OUTPUT_HEIGHT;
    if (is_bev_stream) {
        // 客户端要看 BEV 流时，先唤醒 Klipper（发送最轻量的一条 gcode：g4 p20 延时）。
        // 机器有功耗休眠：休眠停掉 Klipper 并丢失运行状态，固件无专用唤醒指令，
        // 故用 G4 占位——它不动电机、不改状态，只把机器从休眠拉回就绪（见 klipper_flow.h）。
        // 这是"唤醒平面"操作（免仲裁），与 DETECT/CALIB 开头的 sendG4Wait 语义一致。
        std::string wait_work;
        if (streamer->klipper && streamer->klipper->sendGcode("g4 p20\n", nullptr, 5L, &wait_work)) {
            spdlog::info("[Unified Server] G4 wake command response: {}", wait_work);
        } else {
            spdlog::warn("[Unified Server] WARNING: Failed to send G4 wake command");
        }
    }

    // 从 media 里取出整条编码管线的顶层 bin（就是 gst-launch 字符串描述的那条管线）。
    GstElement* element = gst_rtsp_media_get_element(media);
    if (!element) {
        spdlog::error("[RTSP] ERROR: Failed to get media element");
        return;
    }

    // 同一路流可能被反复连接/重连：set_appsrc(NULL) 先释放上次遗留的引用，防止泄漏。
    rtsp_streamer_set_appsrc(streamer, NULL);

    // 从管线里按名字递归找到我们声明的 appsrc（名字 "mysrc" 定义在 gst-launch 字符串里）。
    GstElement* appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(element), "mysrc");
    if (!appsrc) {
        spdlog::error("[RTSP] ERROR: Cannot find appsrc 'mysrc'");
        g_object_unref(element);
        return;
    }

    // 挂载背压信号：appsrc 的 need-data / enough-data 驱动 streamer 的 need_data 标志。
    g_signal_connect(appsrc, "need-data", G_CALLBACK(on_need_data), streamer);
    g_signal_connect(appsrc, "enough-data", G_CALLBACK(on_enough_data), streamer);

    // format=time：让 appsrc 按时间（PTS）而不是字节数来管理缓冲队列，配合时间戳推流。
    gst_util_set_object_arg(G_OBJECT(appsrc), "format", "time");

    // 计算队列上限：优先用 streamer 里配置的值，为 0 时按原画/BEV 用默认值。
    size_t max_bytes = streamer->appsrc_max_bytes;
    if (max_bytes == 0) {
        max_bytes = (streamer->width == ORIGINAL_WIDTH) ?
                    APPSRC_MAX_BYTES_ORIGINAL : APPSRC_MAX_BYTES_BEV;
    }
    int max_time_ms = streamer->appsrc_max_time_ms > 0 ?
                      streamer->appsrc_max_time_ms : APPSRC_MAX_TIME_MS;

    // 设置 appsrc 的背压参数：
    //   max-bytes / max-time : 内部队列的字节/时长上限，超过即触发 enough-data
    //   block = FALSE         : 关键——队列满时不阻塞 push，而是丢帧（配合 need_data 标志）
    //                           若非 FALSE，push 会阻塞推流线程，失去直播"实时丢帧"能力。
    g_object_set(G_OBJECT(appsrc),
                 "max-bytes", (guint64)max_bytes,
                 "max-time", (guint64)(max_time_ms * GST_MSECOND),
                 "block", FALSE,
                 NULL);

    clock_gettime(CLOCK_MONOTONIC, &ts);
    spdlog::info("[RTSP]  AppSrc configured at {}.{:03}: {}x{}, max_bytes={}, max_time={}ms",
                 ts.tv_sec, ts.tv_nsec / 1000000,
                 streamer->width, streamer->height, max_bytes, max_time_ms);

    // 新连接开始，清零统计计数器（不累积上次连接的旧账）。
    streamer->total_frames_sent = 0;
    streamer->backpressure_skip_count = 0;


    // 关键：等待管道进入 PLAYING 状态后再设置 appsrc。
    // 原因：appsrc 只有在管线处于 PLAYING 时 push 才有效；过早设置并推帧会失败或被丢。
    // 状态切换是异步的，所以这里用 gst_element_get_state 带超时地等待。
    GstState current_state, pending_state;
    GstStateChangeReturn ret = gst_element_get_state(appsrc, &current_state, &pending_state, 500 * GST_MSECOND);

    // 线程安全地接管 appsrc：内部先 unref 旧值、再加锁发布（接管传入引用，见 rtsp_streamer.h）。
    auto publish_appsrc = [&]() {
        rtsp_streamer_set_appsrc(streamer, GST_ELEMENT(gst_object_ref(appsrc)));
    };

    if (ret == GST_STATE_CHANGE_SUCCESS && current_state == GST_STATE_PLAYING) {
        // 管线已经 PLAYING，立即接管 appsrc（多持一份引用交给 streamer）。
        publish_appsrc();
        spdlog::info("[RTSP]  Pipeline already PLAYING, appsrc ready immediately");
    } else if (ret == GST_STATE_CHANGE_ASYNC || pending_state == GST_STATE_PLAYING) {
        // 管线正在异步切换到 PLAYING，再等最多 2 秒。
        spdlog::info("[RTSP]  Waiting for pipeline to reach PLAYING state (current={}, pending={})...",
                     static_cast<int>(current_state), static_cast<int>(pending_state));

        ret = gst_element_get_state(appsrc, &current_state, NULL, 2 * GST_SECOND);
        if (ret == GST_STATE_CHANGE_SUCCESS && current_state == GST_STATE_PLAYING) {
            publish_appsrc();
            spdlog::info("[RTSP]  Pipeline reached PLAYING, appsrc ready");
        } else {
            spdlog::warn("[RTSP]  Pipeline state change timeout or failed (ret={}, state={})",
                         static_cast<int>(ret), static_cast<int>(current_state));
            // 超时也仍然设置 appsrc：推流工作线程每次 push 前会再检查状态（rtsp_streamer.cpp），
            // 即便这里没等到 PLAYING，也不至于丢 appsrc，等状态就绪后可自然恢复。
            publish_appsrc();
        }
    } else {
        // 其它意外状态，同样先设置 appsrc 并打印警告，由工作线程兜底判断。
        spdlog::warn("[RTSP]  Unexpected pipeline state (ret={}, current={}, pending={})",
                     static_cast<int>(ret), static_cast<int>(current_state),
                     static_cast<int>(pending_state));
        publish_appsrc();
    }

    // 注册 media 的 unprepared 回调：客户端断开时释放 appsrc 引用。
    g_signal_connect(media, "unprepared", G_CALLBACK(media_unprepared_cb), streamer);

    // 释放本函数内的临时引用（streamer->appsrc 已另外 ref 了一份）。
    gst_object_unref(appsrc);
    g_object_unref(element);
}

// 客户端断开回调：释放 appsrc 引用并置 NULL。
// appsrc==NULL 是"当前无人观看"的唯一信号，推流工作线程据此停止推帧。
static void media_unprepared_cb(GstRTSPMedia* media, RTSPStreamer* streamer) {
    (void)media;

    spdlog::info("[RTSP] Client disconnected");

    // set_appsrc(NULL)：线程安全地清除引用（内部加锁 unref），推流侧据此停止推帧。
    rtsp_streamer_set_appsrc(streamer, NULL);
}

// 创建 media 工厂：定义 appsrc → mpph264enc → rtph264pay 的编码管线。
// 每个客户端连接会据此拉起一条独立管线（shared=false，见下方 set_shared）。
//
// 注意 BEV 与原画走两条不同的管线字符串（下方 if/else），区别在：
//   BEV(1280x1280)：queue leaky + videoconvert + 编码前再一个 leaky queue，强丢帧抗积压；
//   原画(1280x960)：单 queue(max-size-buffers=3)，相对宽松。
static GstRTSPMediaFactory* create_media_factory(int width, int height, int fps,
                                                 RTSPStreamer* streamer) {
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();

    // 码率估算：像素数 × 帧率 × 每像素比特数（ENCODE_BPP=0.20），即经验压缩比。
    int bps = (int)(width * height * fps * ENCODE_BPP);

    // GOP 下限设为 fps：保证每秒至少一个关键帧（I 帧），加快首屏出图和断线重连恢复。
    int actual_gop = (ENCODE_GOP < fps) ? fps : ENCODE_GOP;

    // use_dmabuf 时在 caps 里声明 (memory:DMABuf)，告诉下游走零拷贝 dmabuf 内存。
    const char* mem_caps = streamer && streamer->use_dmabuf ? "(memory:DMABuf)" : "";
    gchar* launch = NULL;
    if (width == BEV_OUTPUT_WIDTH && height == BEV_OUTPUT_HEIGHT) {
        // BEV 管线：appsrc -> [leaky queue] -> videoconvert -> mpph264enc -> h264parse -> [leaky queue] -> rtph264pay
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
        // 原画管线：appsrc -> queue(3) -> mpph264enc -> h264parse -> rtph264pay
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

    spdlog::info("[RTSP] Pipeline (shared=false): {}", launch);

    gst_rtsp_media_factory_set_launch(factory, launch);
    // shared=FALSE：每个客户端各起一条独立编码管线，互不共享（本机算力足够时更稳）。
    gst_rtsp_media_factory_set_shared(factory, FALSE);


    // 注册 media-configure 回调：客户端真正连上、管线建好后触发（对接 appsrc 用）。
    g_signal_connect(factory, "media-configure", G_CALLBACK(media_configured_cb), streamer);

    g_free(launch);
    return factory;
}

// 初始化双路 RTSP 服务器：创建原画 + BEV 两个挂载点，并启动主循环线程。
bool dual_rtsp_server_init(DualRTSPServer* server, const char* bind_address, KlipperManager* klipper) {
    if (!server) {
        return false;
    }
    // 注意：bind_address 参数当前未被使用，实际绑定地址固定为 config.h 的
    // SERVER_BIND_ADDRESS（见下方 address_to_bind）。传入值（如 "auto"）被忽略。
    (void)bind_address;

    // 逐字段初始化指针，避免 memset 清零内嵌的 GMutex（必须用 g_mutex_init，
    // 见 rtsp_streamer_init）。两路 RTSPStreamer 由 rtsp_streamer_init 负责初始化。
    server->main_loop = NULL;
    server->main_loop_thread = NULL;
    server->rtsp_server = NULL;
    server->bev_processor = NULL;

    gst_init(NULL, NULL);  // 初始化 GStreamer，任何 gst_* 调用之前必须执行

    server->main_loop = g_main_loop_new(NULL, FALSE);  // 创建 GLib 主循环（跑在独立线程）

    server->rtsp_server = gst_rtsp_server_new();  // 创建 RTSP 服务器实例

    // 强制绑定到 config.h 里配置的 IP 地址和端口。
    const char* address_to_bind = SERVER_BIND_ADDRESS;

    gst_rtsp_server_set_address(server->rtsp_server, address_to_bind);
    gst_rtsp_server_set_service(server->rtsp_server, RTSP_SERVER_PORT);

    // 初始化原画流状态（appsrc 尚未创建，等客户端连接后填充）。
    if (!rtsp_streamer_init(&server->original_stream,
                           ORIGINAL_WIDTH, ORIGINAL_HEIGHT, TARGET_FPS,
                           APPSRC_MAX_BYTES_ORIGINAL, APPSRC_MAX_TIME_MS)) {
        spdlog::error("[RTSP] ERROR: Failed to init original stream");
        return false;
    }

    // 原画流走 DMABUF 零拷贝路径。
    server->original_stream.use_dmabuf = TRUE;

    // 初始化 BEV 流状态。
    if (!rtsp_streamer_init(&server->bev_stream,
                           BEV_OUTPUT_WIDTH, BEV_OUTPUT_HEIGHT, TARGET_FPS,
                           APPSRC_MAX_BYTES_BEV, APPSRC_MAX_TIME_MS)) {
        spdlog::error("[RTSP] ERROR: Failed to init BEV stream");
        rtsp_streamer_cleanup(&server->original_stream);  // 回滚已初始化的原画流
        return false;
    }
    server->bev_stream.use_dmabuf = TRUE;

    // 注入设备客户端：BEV 客户端连接回调里用 streamer->klipper 发 G4 唤醒，
    // 直接使用 KlipperManager，不再直调 KlipperManager::instance()。
    server->original_stream.klipper = klipper;
    server->bev_stream.klipper = klipper;

    // 拿到 RTSP 服务器的挂载点表，往里面注册两个路径 -> 工厂的映射。
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server->rtsp_server);

    GstRTSPMediaFactory* original_factory = create_media_factory(
        ORIGINAL_WIDTH, ORIGINAL_HEIGHT, TARGET_FPS, &server->original_stream);
    gst_rtsp_mount_points_add_factory(mounts, RTSP_PATH_ORIGINAL, original_factory);

    GstRTSPMediaFactory* bev_factory = create_media_factory(
        BEV_OUTPUT_WIDTH, BEV_OUTPUT_HEIGHT, TARGET_FPS, &server->bev_stream);
    gst_rtsp_mount_points_add_factory(mounts, RTSP_PATH_BEV, bev_factory);

    g_object_unref(mounts);

    // attach：让服务器接管默认主上下文，开始监听端口接受连接。
    gst_rtsp_server_attach(server->rtsp_server, NULL);

    // 启动主循环线程，跑 GLib 事件循环处理 RTSP 信令。
    server->main_loop_thread = g_thread_new("rtsp-main-loop",
                                            main_loop_thread_func,
                                            server->main_loop);
    if (!server->main_loop_thread) {
        spdlog::error("[RTSP] ERROR: Failed to create main loop thread");
        return false;
    }

    g_usleep(100000); // 等 100ms，让主循环线程真正跑起来，避免后续操作竞态

    // 初始化 BEV 处理器（OpenGL + YOLO 画框）。失败只警告不致命——RTSP 仍可用，
    // 只是 BEV 流拿不到处理结果。它独立于 RTSP 服务器生命周期。
    server->bev_processor = bev_init(BEV_INPUT_WIDTH, BEV_INPUT_HEIGHT,
                                     BEV_OUTPUT_WIDTH, BEV_OUTPUT_HEIGHT);
    if (!server->bev_processor) {
        spdlog::warn("[BEV] WARNING: BEV processor init failed");
    } else {
        spdlog::info("[BEV]  BEV processor initialized successfully");
    }

    spdlog::info(
        "\n========================================\n"
        " RTSP Server Started\n"
        "========================================\n"
        "Address:  {}:{}\n"
        "Original: rtsp://{}:{}{}\n"
        "BEV:      rtsp://{}:{}{}\n"
        "========================================\n",
        address_to_bind, RTSP_SERVER_PORT,
        address_to_bind, RTSP_SERVER_PORT, RTSP_PATH_ORIGINAL,
        address_to_bind, RTSP_SERVER_PORT, RTSP_PATH_BEV);

    return true;
}

// 清理：按"先停线程、再释放对象"的顺序，避免 use-after-free。
void dual_rtsp_server_cleanup(DualRTSPServer* server) {
    if (!server) {
        return;
    }


    // 1) 释放 BEV 处理器（OpenGL/YOLO 资源）。
    if (server->bev_processor) {
        bev_cleanup(server->bev_processor);
        server->bev_processor = NULL;
    }

    // 2) 清理两路推流状态（DMABUF 分配器、互斥锁等）。
    rtsp_streamer_cleanup(&server->original_stream);
    rtsp_streamer_cleanup(&server->bev_stream);


    // 3) 停止主循环线程：先 quit 事件循环，再 join 等线程退出，最后 unref。
    if (server->main_loop) {
        if (g_main_loop_is_running(server->main_loop)) {
            spdlog::info("[RTSP] Stopping main loop...");
            g_main_loop_quit(server->main_loop);
        }


        if (server->main_loop_thread) {
            g_thread_join(server->main_loop_thread);  // 等待主循环线程真正退出
            server->main_loop_thread = NULL;
        }

        g_main_loop_unref(server->main_loop);
        server->main_loop = NULL;
    }


    // 4) 最后释放 RTSP 服务器对象（此时已无线程在用）。
    if (server->rtsp_server) {
        g_object_unref(server->rtsp_server);
        server->rtsp_server = NULL;
    }

    spdlog::info("[RTSP] Server cleaned up");
}
