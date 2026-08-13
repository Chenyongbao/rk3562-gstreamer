#define _POSIX_C_SOURCE 199309L

#include "stream_workers.h"

#include <linux/dma-buf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "config.h"
#include "core/dmabuf_utils.h"
#include "reallink_ogles/bev_api.h"
#include "../3_consumers/common/LatestNv12FrameBuffer.h"

struct RtspReleaseBridge {
    FrameSlotReleaseCallback release_cb = NULL;
    void* release_user_data = NULL;
    int dmabuf_fd = -1;
};

/**
 * @brief 强制同步 DMABUF 的 CPU 缓存。
 * 当 CPU 需要直接读写 DMABUF 物理内存时，必须调用此函数刷新 Cache，
 * 防止 CPU 读到旧数据或硬件读到未写入物理内存的 Cache 脏数据。
 */
static bool syncDmabufCpuWrite(int fd, unsigned long flags)
{
    if (fd < 0) {
        return false;
    }

    struct dma_buf_sync sync;
    memset(&sync, 0, sizeof(sync));
    sync.flags = flags;
    return ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) == 0;
}

/**
 * @brief 释放一个从队列中取出的帧槽（FrameSlot）。
 * 它会调用帧槽内部绑定的回调函数，将这块物理内存（DMABUF）或者指针还给上游（例如内存池或 V4L2 驱动）。
 */
static void releaseCopiedSlot(FrameSlot* slot)
{
    if (!slot || !slot->is_dmabuf_mode || !slot->release_cb) {
        return;
    }
    slot->release_cb(slot->release_user_data, slot->dmabuf_fd);
    slot->release_cb = NULL;
    slot->release_user_data = NULL;
}

/**
 * @brief 供 RTSP 底层回调使用的释放函数。
 * 当 RTSP 硬件编码器（MPP）压缩并发送完一帧后，会通过此回调通知应用层。
 * 它内部再调用原生的 release_cb 将内存还给上游池。
 */
static void releaseCopiedSlotViaRtsp(void* user_data)
{
    RtspReleaseBridge* bridge = static_cast<RtspReleaseBridge*>(user_data);
    if (!bridge) {
        return;
    }

    if (bridge->release_cb) {
        bridge->release_cb(bridge->release_user_data, bridge->dmabuf_fd);
    }
    free(bridge);
}

/**
 * @brief 阻塞等待 RTSP 流媒体服务器就绪。
 * 防止在网络推流器还没初始化好时，后台线程就盲目地疯狂塞入视频帧。
 */
static bool waitRtspReady(RTSPStreamer* stream,
                          volatile sig_atomic_t* running,
                          const char* thread_name)
{
    if (!stream) {
        return false;
    }
    if (rtsp_streamer_is_ready(stream)) {
        return true;
    }

    for (int retry = 0; retry < 10 && (!running || *running); ++retry) {
        usleep(50000);
        if (rtsp_streamer_is_ready(stream)) {
            if (thread_name) {
                fprintf(stderr, "[%s] Pipeline became ready after %dms\n",
                        thread_name, (retry + 1) * 50);
            }
            return true;
        }
    }

    return false;
}

/**
 * @brief 原画推流后台工作线程（核心流水线工人）。
 * 不断从 original_queue 中取出硬件拷贝好的私有帧，并直接将其推入 RTSP 硬件编码流水线。
 * 发送完毕后归还私有内存池。
 */
static void original_worker_thread(WorkerContext* ctx)
{
    fprintf(stderr, "[%s] Worker started\n", ctx->thread_name);

    uint64_t processed_frames = 0;
    while (*ctx->running) {
        uint64_t frame_idx = 0;
        FrameSlot slot{};
        if (!frame_queue_pop_copy(ctx->input_queue, &slot, &frame_idx)) {
            break;
        }

        if (!ctx->stream->appsrc) {
            releaseCopiedSlot(&slot);
            continue;
        }

        if (!waitRtspReady(ctx->stream, ctx->running, ctx->thread_name)) {
            releaseCopiedSlot(&slot);
            continue;
        }

        if (!slot.is_dmabuf_mode) {
            static uint64_t non_dmabuf_slot_drop_count = 0;
            ++non_dmabuf_slot_drop_count;
            if ((non_dmabuf_slot_drop_count % 300) == 1) {
                fprintf(stderr,
                        "[%s] WARNING: drop non-dmabuf frame in Original worker (count=%llu)\n",
                        ctx->thread_name,
                        (unsigned long long)non_dmabuf_slot_drop_count);
            }
            continue;
        }

        RtspReleaseBridge* release_bridge =
            static_cast<RtspReleaseBridge*>(calloc(1, sizeof(RtspReleaseBridge)));
        if (!release_bridge) {
            releaseCopiedSlot(&slot);
            continue;
        }
        release_bridge->release_cb = slot.release_cb;
        release_bridge->release_user_data = slot.release_user_data;
        release_bridge->dmabuf_fd = slot.dmabuf_fd;

        const bool push_ok = rtsp_streamer_push_dmabuf_nv12_single_with_release(
            ctx->stream,
            slot.dmabuf_fd,
            slot.stride,
            slot.stride,
            slot.width,
            slot.height,
            frame_idx,
            &releaseCopiedSlotViaRtsp,
            release_bridge);
        if (push_ok) {
            processed_frames++;
            slot.release_cb = NULL;
            slot.release_user_data = NULL;
        } else {
            free(release_bridge);
            releaseCopiedSlot(&slot);
        }
    }

    fprintf(stderr, "[%s] Worker stopped (processed %llu frames)\n",
            ctx->thread_name, (unsigned long long)processed_frames);
    return;
}

/**
 * @brief YOLO (BEV) 俯视图推流后台工作线程（核心算力工人）。
 * 它从 bev_queue 拿到原生的 V4L2 缓冲区后，先调用 NPU/OpenGL 做算法识别和画框。
 * 算完后立即将 V4L2 缓冲区归还给底层摄像头驱动。
 * 随后，它把画好框的画面拷贝到自己的局域私有池中，最后推送给 RTSP 发送。
 */
static void bev_worker_thread(WorkerContext* ctx)
{
    const int bev_pool_size = 3;
    const int bev_stride = BEV_OUTPUT_WIDTH;
    const size_t bev_dmabuf_size = BEV_OUTPUT_NV12_SIZE;

    uint8_t* bev_output_buf = (uint8_t*)malloc(BEV_OUTPUT_NV12_SIZE);
    if (!bev_output_buf) {
        fprintf(stderr, "[%s] ERROR: Buffer allocation failed\n", ctx->thread_name);
        return;
    }

    int bev_pool_fds[bev_pool_size];
    void* bev_pool_ptrs[bev_pool_size];
    for (int i = 0; i < bev_pool_size; ++i) {
        bev_pool_fds[i] = -1;
        bev_pool_ptrs[i] = NULL;
    }

    fprintf(stderr, "[%s] Worker started\n", ctx->thread_name);

    if (ctx->bev_processor) {
        if (!bev_bind_context_to_thread(ctx->bev_processor)) {
            fprintf(stderr, "[%s] ERROR: Failed to bind OpenGL context to thread\n", ctx->thread_name);
            free(bev_output_buf);
            return;
        }
    }

    if (!dmabuf_heap_init()) {
        fprintf(stderr, "[%s] ERROR: dmabuf heap init failed for BEV output pool\n", ctx->thread_name);
        free(bev_output_buf);
        return;
    }

    for (int i = 0; i < bev_pool_size; ++i) {
        bev_pool_fds[i] = dmabuf_alloc(bev_dmabuf_size);
        if (bev_pool_fds[i] < 0) {
            fprintf(stderr, "[%s] ERROR: BEV dmabuf alloc failed\n", ctx->thread_name);
            for (int j = 0; j < i; ++j) {
                if (bev_pool_ptrs[j]) {
                    dmabuf_munmap(bev_pool_ptrs[j], bev_dmabuf_size);
                }
                dmabuf_free(bev_pool_fds[j]);
            }
            dmabuf_heap_deinit();
            free(bev_output_buf);
            return;
        }

        bev_pool_ptrs[i] = dmabuf_mmap(bev_pool_fds[i], BEV_OUTPUT_NV12_SIZE);
        if (!bev_pool_ptrs[i]) {
            fprintf(stderr, "[%s] ERROR: BEV dmabuf mmap failed\n", ctx->thread_name);
            dmabuf_free(bev_pool_fds[i]);
            bev_pool_fds[i] = -1;
            for (int j = 0; j < i; ++j) {
                if (bev_pool_ptrs[j]) {
                    dmabuf_munmap(bev_pool_ptrs[j], bev_dmabuf_size);
                }
                dmabuf_free(bev_pool_fds[j]);
            }
            dmabuf_heap_deinit();
            free(bev_output_buf);
            return;
        }
    }

    uint64_t processed_frames = 0;
    int bev_pool_idx = 0;
    while (*ctx->running) {
        uint64_t frame_idx = 0;
        FrameSlot slot{};
        const bool has_slot = frame_queue_pop_copy(ctx->input_queue, &slot, &frame_idx);
        if (!has_slot) {
            break;
        }

        const bool has_rtsp_appsrc = (ctx->stream && ctx->stream->appsrc);
        if (!ctx->bev_processor || !has_rtsp_appsrc) {
            releaseCopiedSlot(&slot);
            continue;
        }

        bool rtsp_ready = false;
        if (has_rtsp_appsrc) {
            rtsp_ready = waitRtspReady(ctx->stream, ctx->running, ctx->thread_name);
        }
        if (!rtsp_ready) {
            releaseCopiedSlot(&slot);
            continue;
        }

        const gboolean use_dmabuf_out = (ctx->stream && ctx->stream->use_dmabuf);
        if (!use_dmabuf_out) {
            static uint64_t bev_non_dmabuf_drop_count = 0;
            ++bev_non_dmabuf_drop_count;
            if ((bev_non_dmabuf_drop_count % 300) == 1) {
                fprintf(stderr,
                        "[%s] WARNING: BEV stream requires dmabuf output, drop frame (count=%llu)\n",
                        ctx->thread_name,
                        (unsigned long long)bev_non_dmabuf_drop_count);
            }
            releaseCopiedSlot(&slot);
            continue;
        }

        bool process_ok = false;
        if (slot.is_dmabuf_mode) {
            process_ok = bev_process_frame_dmabuf(ctx->bev_processor,
                                                  slot.dmabuf_fd,
                                                  slot.stride,
                                                  slot.size,
                                                  bev_output_buf,
                                                  BEV_OUTPUT_NV12_SIZE);
        } else if (slot.buffer && slot.size > 0) {
            process_ok = bev_process_frame(ctx->bev_processor,
                                           slot.buffer,
                                           slot.size,
                                           bev_output_buf,
                                           BEV_OUTPUT_NV12_SIZE);
        }

        // The source V4L2 buffer can go back to the driver as soon as BEV processing ends.
        releaseCopiedSlot(&slot);

        if (!process_ok) {
            continue;
        }

        if (ctx->app) {
            ctx->app->bev_latest_frame_updater.publish(bev_output_buf,
                                                       BEV_OUTPUT_NV12_SIZE,
                                                       frame_idx);
        }

        int dst_fd = bev_pool_fds[bev_pool_idx];
        uint8_t* dst_ptr = static_cast<uint8_t*>(bev_pool_ptrs[bev_pool_idx]);
        bool push_ok = false;
        if (rtsp_ready) {
            bool copy_ok = (dst_fd >= 0 && dst_ptr != NULL);
            if (copy_ok) {
                copy_ok = syncDmabufCpuWrite(dst_fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);
                if (copy_ok) {
                    memcpy(dst_ptr, bev_output_buf, bev_dmabuf_size);
                    copy_ok = syncDmabufCpuWrite(dst_fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
                }
            }

            if (!copy_ok) {
                static uint64_t bev_dmabuf_copy_fail_count = 0;
                ++bev_dmabuf_copy_fail_count;
                if ((bev_dmabuf_copy_fail_count % 60) == 1) {
                    fprintf(stderr,
                            "[%s] WARNING: BEV dmabuf copy/sync failed (count=%llu)\n",
                            ctx->thread_name,
                            (unsigned long long)bev_dmabuf_copy_fail_count);
                }
            } else {
                push_ok = rtsp_streamer_push_dmabuf_nv12_single(ctx->stream,
                                                                dst_fd,
                                                                bev_stride,
                                                                bev_stride,
                                                                BEV_OUTPUT_WIDTH,
                                                                BEV_OUTPUT_HEIGHT,
                                                                frame_idx);
            }
        }

        if (push_ok) {
            processed_frames++;
            bev_pool_idx = (bev_pool_idx + 1) % bev_pool_size;
        }
    }

    for (int i = 0; i < bev_pool_size; ++i) {
        if (bev_pool_ptrs[i]) {
            dmabuf_munmap(bev_pool_ptrs[i], bev_dmabuf_size);
        }
        dmabuf_free(bev_pool_fds[i]);
    }
    dmabuf_heap_deinit();
    free(bev_output_buf);
    fprintf(stderr, "[%s] Worker stopped (processed %llu frames)\n",
            ctx->thread_name, (unsigned long long)processed_frames);
    return;
}

/**
 * @brief 启动所有后台视频处理线程。
 * 负责拉起原画推流 Worker 和 YOLO (BEV) 推流 Worker，正式启动流水线运转。
 */
bool startStreamWorkers(AppContext& app)
{
    if (!app.original_ctx.running) {
        fprintf(stderr, "[MAIN] ERROR: running flag is null before worker start\n");
        return false;
    }

    app.original_ctx.app = &app;
    app.bev_ctx.app = &app;

    fprintf(stderr, "[MAIN] Creating worker threads...\n");

    try {
        app.original_thread = std::thread(original_worker_thread, &app.original_ctx);
        app.original_thread_started = true;
    } catch (...) {
        fprintf(stderr, "[MAIN] ERROR: Failed to create original thread\n");
        return false;
    }

    try {
        app.bev_thread = std::thread(bev_worker_thread, &app.bev_ctx);
        app.bev_thread_started = true;
    } catch (...) {
        fprintf(stderr, "[MAIN] ERROR: Failed to create BEV thread\n");
        *(app.original_ctx.running) = 0;
        frame_queue_shutdown(&app.original_queue);
        if (app.original_thread.joinable()) {
            app.original_thread.join();
        }
        app.original_thread_started = false;
        return false;
    }

    return true;
}

/**
 * @brief 停止所有后台视频处理线程。
 * 向队列发送退出信号（shutdown），并阻塞等待所有 Worker 线程安全收尾并退出。
 */
void stopStreamWorkers(AppContext& app)
{
    if (!(app.original_thread_started || app.bev_thread_started)) {
        return;
    }

    fprintf(stderr, "[MAIN] Shutting down frame queues...\n");
    if (app.original_queue_initialized) {
        frame_queue_shutdown(&app.original_queue);
    }
    if (app.bev_queue_initialized) {
        frame_queue_shutdown(&app.bev_queue);
    }

    fprintf(stderr, "[MAIN] Waiting for worker threads to finish...\n");
    if (app.original_thread_started && app.original_thread.joinable()) {
        app.original_thread.join();
        app.original_thread_started = false;
    }
    if (app.bev_thread_started && app.bev_thread.joinable()) {
        app.bev_thread.join();
        app.bev_thread_started = false;
    }
    fprintf(stderr, "[MAIN] All worker threads stopped\n");
}
