#define _POSIX_C_SOURCE 199309L

#include "stream_workers.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>

#include "config.h"
#include "rtsp/rga_processor.h"
#include "core/dmabuf_utils.h"
#include "reallink_ogles/bev_api.h"
#include "video/LatestNv12FrameBuffer.h"

struct RtspReleaseBridge {
    FrameSlotReleaseCallback release_cb = NULL;
    void* release_user_data = NULL;
    int dmabuf_fd = -1;
};

static uint64_t elapsedUs(const struct timespec& start, const struct timespec& end)
{
    int64_t us = (int64_t)(end.tv_sec - start.tv_sec) * 1000000LL +
                 (int64_t)(end.tv_nsec - start.tv_nsec) / 1000LL;
    return us > 0 ? (uint64_t)us : 0;
}

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

static bool syncDmabufCpuRead(int fd, unsigned long flags)
{
    if (fd < 0) {
        return false;
    }

    struct dma_buf_sync sync;
    memset(&sync, 0, sizeof(sync));
    sync.flags = flags;
    return ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) == 0;
}

static void releaseCopiedSlot(FrameSlot* slot)
{
    if (!slot || !slot->is_dmabuf_mode || !slot->release_cb) {
        return;
    }
    slot->release_cb(slot->release_user_data, slot->dmabuf_fd);
    slot->release_cb = NULL;
    slot->release_user_data = NULL;
}

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

// 等待 RTSP 管道进入 ready 状态，最长约 500ms
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
        usleep(50000); // 50ms
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

// 原图推流工作线程：只处理 dmabuf 路径
static void* original_worker_thread(void* arg)
{
    WorkerContext* ctx = (WorkerContext*)arg;

    const int dst_w = ORIGINAL_WIDTH;
    const int dst_h = ORIGINAL_HEIGHT;
    const int dst_stride = ORIGINAL_WIDTH;
    const size_t dst_size = (size_t)dst_stride * dst_h + (size_t)dst_stride * (dst_h / 2);
    const int pool_size = 3;

    int pool_fds[pool_size];
    for (int i = 0; i < pool_size; ++i) {
        pool_fds[i] = -1;
    }

    if (!dmabuf_heap_init()) {
        fprintf(stderr, "[%s] ERROR: dmabuf heap init failed\n", ctx->thread_name);
        return NULL;
    }

    for (int i = 0; i < pool_size; ++i) {
        pool_fds[i] = dmabuf_alloc(dst_size);
        if (pool_fds[i] < 0) {
            fprintf(stderr, "[%s] ERROR: dmabuf alloc failed\n", ctx->thread_name);
            for (int j = 0; j < i; ++j) {
                dmabuf_free(pool_fds[j]);
            }
            dmabuf_heap_deinit();
            return NULL;
        }
    }
    int pool_idx = 0;

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
                fprintf(stderr, "[%s] WARNING: drop non-dmabuf frame in Original worker (count=%llu)\n",
                        ctx->thread_name, (unsigned long long)non_dmabuf_slot_drop_count);
            }
            continue;
        }

        bool push_ok = false;
        RtspReleaseBridge* release_bridge =
            static_cast<RtspReleaseBridge*>(calloc(1, sizeof(RtspReleaseBridge)));
        if (!release_bridge) {
            releaseCopiedSlot(&slot);
            continue;
        }
        release_bridge->release_cb = slot.release_cb;
        release_bridge->release_user_data = slot.release_user_data;
        release_bridge->dmabuf_fd = slot.dmabuf_fd;

        push_ok = rtsp_streamer_push_dmabuf_nv12_single_with_release(
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

    for (int i = 0; i < pool_size; ++i) {
        dmabuf_free(pool_fds[i]);
    }
    dmabuf_heap_deinit();

    fprintf(stderr, "[%s] Worker stopped (processed %llu frames)\n",
            ctx->thread_name, (unsigned long long)processed_frames);
    return NULL;
}

// BEV 推流工作线程：BEV 变换、双FD dmabuf 推流、性能统计和资源回
static void* bev_worker_thread(void* arg)
{
    // 线程上下文：包含输入队列、RTSP流、BEV处理器、V4L2句柄和共享统计对
    WorkerContext* ctx = (WorkerContext*)arg;
    const int bev_pool_size = 3;
    const int bev_stride = BEV_OUTPUT_WIDTH;
    const size_t bev_dmabuf_size = BEV_OUTPUT_NV12_SIZE;

    // 本地输出缓冲：承?bev_process_frame?NV12 结果，循环复用以降低分配开销?
    uint8_t* bev_output_buf = (uint8_t*)malloc(BEV_OUTPUT_NV12_SIZE);
    if (!bev_output_buf) {
        fprintf(stderr, "[%s] ERROR: Buffer allocation failed\n", ctx->thread_name);
        return NULL;
    }

    int bev_pool_fds[bev_pool_size];
    void* bev_pool_ptrs[bev_pool_size];
    for (int i = 0; i < bev_pool_size; ++i) {
        bev_pool_fds[i] = -1;
        bev_pool_ptrs[i] = NULL;
    }

    fprintf(stderr, "[%s] Worker started\n", ctx->thread_name);

    //BEV/OpenGL 上下文绑定到当前线程
    // 这是线程亲和操作，必须在实际执行 BEVworker 线程中完成
    if (ctx->bev_processor) {
        if (!bev_bind_context_to_thread(ctx->bev_processor)) {
            fprintf(stderr, "[%s] ERROR: Failed to bind OpenGL context to thread\n", ctx->thread_name);
            free(bev_output_buf);
            return NULL;
        }
    }

    if (!dmabuf_heap_init()) {
        fprintf(stderr, "[%s] ERROR: dmabuf heap init failed for BEV output pool\n", ctx->thread_name);
        free(bev_output_buf);
        return NULL;
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
            return NULL;
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
            return NULL;
        }
    }

    uint64_t processed_frames = 0;
    int bev_pool_idx = 0;
    // 主循环：取帧 -> 就绪判断 -> BEV处理 -> 推流 -> 回队
    while (*ctx->running) {
        uint64_t frame_idx = 0;
#ifdef ENABLE_PERFORMANCE_MONITORING
        struct timespec t_bev_pop_start, t_bev_pop_end;
        clock_gettime(CLOCK_MONOTONIC, &t_bev_pop_start);
#endif
        FrameSlot slot{};
        const bool has_slot = frame_queue_pop_copy(ctx->input_queue, &slot, &frame_idx);
#ifdef ENABLE_PERFORMANCE_MONITORING
        clock_gettime(CLOCK_MONOTONIC, &t_bev_pop_end);
        if (ctx->app) {
            perf_stats_add(&ctx->app->perf_bev_pop, elapsedUs(t_bev_pop_start, t_bev_pop_end));
        }
#endif
        if (!has_slot) {
            // 队列关闭或线程退出时，pop 返回空，线程结束
            break;
        }

        const bool force_process = slot.force_process;
        const bool has_rtsp_appsrc = (ctx->stream && ctx->stream->appsrc);
        if (!ctx->bev_processor || (!has_rtsp_appsrc && !force_process)) {
            releaseCopiedSlot(&slot);
            continue;
        }

        bool rtsp_ready = false;
        if (has_rtsp_appsrc) {
            rtsp_ready = waitRtspReady(ctx->stream, ctx->running, ctx->thread_name);
        }

        // 等待管道 ready，避免初连阶段频繁无效处理（最长约500ms
        if (!rtsp_ready && !force_process) {
            releaseCopiedSlot(&slot);
            continue;
        }

        // 标记当前帧是否完?BEV 处理
        bool process_ok = false;
        bool mapped_dmabuf = false;
        void* mapped_input = NULL;
        // RTSP 输出模式：true dmabuf 输出，false 为内存帧输
        gboolean use_dmabuf_out = (ctx->stream && ctx->stream->use_dmabuf);

        if (use_dmabuf_out) {
            // 路径A：BEV处理后写入单FD连续NV12 dmabuf并推流
            const uint8_t* bev_input = NULL;
            size_t bev_input_size = 0;
            if (slot.is_dmabuf_mode) {
                const size_t mapped_size = slot.size;
                mapped_input = dmabuf_mmap(slot.dmabuf_fd, mapped_size);
                if (mapped_input &&
                    syncDmabufCpuRead(slot.dmabuf_fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ)) {
                    bev_input = static_cast<const uint8_t*>(mapped_input);
                    bev_input_size = mapped_size;
                    mapped_dmabuf = true;
                } else {
                    static uint64_t bev_input_map_fail_count = 0;
                    ++bev_input_map_fail_count;
                    if ((bev_input_map_fail_count % 60) == 1) {
                        fprintf(stderr,
                                "[%s] WARNING: BEV input dmabuf map/sync failed (count=%llu)\n",
                                ctx->thread_name,
                                (unsigned long long)bev_input_map_fail_count);
                    }
                }
            } else if (slot.buffer && slot.size > 0) {
                bev_input = slot.buffer;
                bev_input_size = slot.size;
            }

            if (bev_input && bev_input_size > 0) {
#ifdef ENABLE_PERFORMANCE_MONITORING
                struct timespec t_bev_process_start, t_bev_process_end;
                clock_gettime(CLOCK_MONOTONIC, &t_bev_process_start);
#endif
                process_ok = bev_process_frame(ctx->bev_processor,
                                               bev_input, bev_input_size,
                                               bev_output_buf, BEV_OUTPUT_NV12_SIZE);
#ifdef ENABLE_PERFORMANCE_MONITORING
                clock_gettime(CLOCK_MONOTONIC, &t_bev_process_end);
                if (ctx->app) {
                    perf_stats_add(&ctx->app->perf_bev_process,
                                   elapsedUs(t_bev_process_start, t_bev_process_end));
                }
#endif
            }

            if (mapped_dmabuf) {
                syncDmabufCpuRead(slot.dmabuf_fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
            }
            if (mapped_input) {
                dmabuf_munmap(mapped_input, slot.size);
                mapped_input = NULL;
            }

            if (process_ok) {
                if (ctx->app) {
                    ctx->app->bev_latest_frame_updater.publish(bev_output_buf,
                                                               BEV_OUTPUT_NV12_SIZE,
                                                               frame_idx);
                }

                int dst_fd = bev_pool_fds[bev_pool_idx];
                uint8_t* dst_ptr = static_cast<uint8_t*>(bev_pool_ptrs[bev_pool_idx]);
#ifdef ENABLE_PERFORMANCE_MONITORING
                struct timespec t_bev_push_start, t_bev_push_end;
                clock_gettime(CLOCK_MONOTONIC, &t_bev_push_start);
#endif

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
                            fprintf(stderr, "[%s] WARNING: BEV dmabuf copy/sync failed (count=%llu)\n",
                                    ctx->thread_name,
                                    (unsigned long long)bev_dmabuf_copy_fail_count);
                        }
                    } else {
                        push_ok = rtsp_streamer_push_dmabuf_nv12_single(ctx->stream,
                                                                        dst_fd, bev_stride, bev_stride,
                                                                        BEV_OUTPUT_WIDTH, BEV_OUTPUT_HEIGHT,
                                                                        frame_idx);
                    }
                }
#ifdef ENABLE_PERFORMANCE_MONITORING
                clock_gettime(CLOCK_MONOTONIC, &t_bev_push_end);
                if (ctx->app) {
                    perf_stats_add(&ctx->app->perf_bev_push,
                                   elapsedUs(t_bev_push_start, t_bev_push_end));
                }
#endif

                if (!rtsp_ready && force_process) {
                    fprintf(stderr,
                            "[%s] Forced BEV refresh processed without RTSP push (frame_idx=%llu)\n",
                            ctx->thread_name,
                            (unsigned long long)frame_idx);
                    processed_frames++;
                } else if (push_ok) {
                    processed_frames++;
                    bev_pool_idx = (bev_pool_idx + 1) % bev_pool_size;
                }
            }
        } else {
            // 强制策略：BEV 仅dmabuf 推流；若配置为非 dmabuf，则丢帧并限频告警
            static uint64_t bev_non_dmabuf_drop_count = 0;
            ++bev_non_dmabuf_drop_count;
            if ((bev_non_dmabuf_drop_count % 300) == 1) {
                fprintf(stderr,
                        "[%s] WARNING: BEV stream requires dmabuf output, drop frame (count=%llu)\n",
                        ctx->thread_name,
                        (unsigned long long)bev_non_dmabuf_drop_count);
            }
        }

        // dmabuf 模式下，处理完成后将 V4L2 buffer 归还驱动队列
        releaseCopiedSlot(&slot);
    }

    // 线程退出前释放本地输出缓冲
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
    return NULL;
}

bool startStreamWorkers(AppContext& app)
{
    if (!app.original_ctx.running) {
        fprintf(stderr, "[MAIN] ERROR: running flag is null before worker start\n");
        return false;
    }

    app.original_ctx.app = &app;
    app.bev_ctx.app = &app;

    fprintf(stderr, "[MAIN] Creating worker threads...\n");

    if (pthread_create(&app.original_thread, NULL, original_worker_thread, &app.original_ctx) != 0) {
        fprintf(stderr, "[MAIN] ERROR: Failed to create original thread\n");
        return false;
    }
    app.original_thread_started = true;

    if (pthread_create(&app.bev_thread, NULL, bev_worker_thread, &app.bev_ctx) != 0) {
        fprintf(stderr, "[MAIN] ERROR: Failed to create BEV thread\n");
        *(app.original_ctx.running) = 0;
        frame_queue_shutdown(&app.original_queue);
        pthread_join(app.original_thread, NULL);
        app.original_thread_started = false;
        return false;
    }
    app.bev_thread_started = true;

    return true;
}

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
    if (app.original_thread_started) {
        pthread_join(app.original_thread, NULL);
        app.original_thread_started = false;
    }
    if (app.bev_thread_started) {
        pthread_join(app.bev_thread, NULL);
        app.bev_thread_started = false;
    }
    fprintf(stderr, "[MAIN] All worker threads stopped\n");
}
