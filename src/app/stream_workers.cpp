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

static uint64_t elapsedUs(const struct timespec& start, const struct timespec& end)
{
    int64_t us = (int64_t)(end.tv_sec - start.tv_sec) * 1000000LL +
                 (int64_t)(end.tv_nsec - start.tv_nsec) / 1000LL;
    return us > 0 ? (uint64_t)us : 0;
}

static bool syncDmabufCpuAccess(int fd, unsigned long flags)
{
    if (fd < 0) {
        return false;
    }

    struct dma_buf_sync sync;
    memset(&sync, 0, sizeof(sync));
    sync.flags = flags;
    return ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) == 0;
}

// Wait for RTSP pipeline to become ready.
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

static void* original_worker_thread(void* arg)
{
    WorkerContext* ctx = (WorkerContext*)arg;
    const int dst_w = ORIGINAL_WIDTH;
    const int dst_h = ORIGINAL_HEIGHT;
    const int dst_stride = ORIGINAL_WIDTH;
    const size_t dst_size = (size_t)dst_stride * (size_t)dst_h * 3 / 2;
    const int pool_size = 3;

    int pool_fds[pool_size];
    void* pool_ptrs[pool_size];
    for (int i = 0; i < pool_size; ++i) {
        pool_fds[i] = -1;
        pool_ptrs[i] = NULL;
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
                if (pool_ptrs[j]) {
                    dmabuf_munmap(pool_ptrs[j], dst_size);
                }
                dmabuf_free(pool_fds[j]);
            }
            dmabuf_heap_deinit();
            return NULL;
        }

        pool_ptrs[i] = dmabuf_mmap(pool_fds[i], dst_size);
        if (!pool_ptrs[i]) {
            fprintf(stderr, "[%s] ERROR: dmabuf mmap failed\n", ctx->thread_name);
            dmabuf_free(pool_fds[i]);
            pool_fds[i] = -1;
            for (int j = 0; j < i; ++j) {
                if (pool_ptrs[j]) {
                    dmabuf_munmap(pool_ptrs[j], dst_size);
                }
                dmabuf_free(pool_fds[j]);
            }
            dmabuf_heap_deinit();
            return NULL;
        }
    }

    fprintf(stderr, "[%s] Worker started\n", ctx->thread_name);

    uint64_t processed_frames = 0;
    int pool_idx = 0;
    while (*ctx->running) {
        uint64_t frame_idx = 0;
        FrameSlot* slot = frame_queue_pop_dmabuf(ctx->input_queue, &frame_idx);
        if (!slot) {
            break;
        }

        if (!ctx->stream->appsrc) {
            if (slot->is_dmabuf_mode && ctx->v4l2_cam) {
                v4l2_camera_queue(ctx->v4l2_cam, slot->v4l2_index);
            }
            continue;
        }

        if (!waitRtspReady(ctx->stream, ctx->running, ctx->thread_name)) {
            if (slot->is_dmabuf_mode && ctx->v4l2_cam) {
                v4l2_camera_queue(ctx->v4l2_cam, slot->v4l2_index);
            }
            continue;
        }

        if (!slot->is_dmabuf_mode) {
            static uint64_t non_dmabuf_slot_drop_count = 0;
            ++non_dmabuf_slot_drop_count;
            if ((non_dmabuf_slot_drop_count % 300) == 1) {
                fprintf(stderr, "[%s] WARNING: drop non-dmabuf frame in Original worker (count=%llu)\n",
                        ctx->thread_name, (unsigned long long)non_dmabuf_slot_drop_count);
            }
            continue;
        }

        int dst_fd = pool_fds[pool_idx];
        void* dst_ptr = pool_ptrs[pool_idx];
        bool resize_ok = rga_processor_resize_nv12_dmabuf_to_dmabuf(
            slot->dmabuf_fd, slot->width, slot->height, slot->stride,
            dst_fd, dst_w, dst_h, dst_stride);

        if (resize_ok && dst_ptr) {
            bool sync_ok = syncDmabufCpuAccess(dst_fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
            if (sync_ok) {
                if (rtsp_streamer_push_frame(ctx->stream,
                                             dst_ptr,
                                             dst_size,
                                             frame_idx)) {
                    processed_frames++;
                    pool_idx = (pool_idx + 1) % pool_size;
                }
                syncDmabufCpuAccess(dst_fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
            }
        }

        if (slot->is_dmabuf_mode && ctx->v4l2_cam) {
            v4l2_camera_queue(ctx->v4l2_cam, slot->v4l2_index);
        }
    }

    for (int i = 0; i < pool_size; ++i) {
        if (pool_ptrs[i]) {
            dmabuf_munmap(pool_ptrs[i], dst_size);
        }
        dmabuf_free(pool_fds[i]);
    }
    dmabuf_heap_deinit();

    fprintf(stderr, "[%s] Worker stopped (processed %llu frames)\n",
            ctx->thread_name, (unsigned long long)processed_frames);
    return NULL;
}

static void* bev_worker_thread(void* arg)
{
    WorkerContext* ctx = (WorkerContext*)arg;
    const size_t bev_frame_size = BEV_OUTPUT_NV12_SIZE;

    uint8_t* bev_output_buf = (uint8_t*)malloc(bev_frame_size);
    if (!bev_output_buf) {
        fprintf(stderr, "[%s] ERROR: Buffer allocation failed\n", ctx->thread_name);
        return NULL;
    }

    fprintf(stderr, "[%s] Worker started\n", ctx->thread_name);

    if (ctx->bev_processor) {
        if (!bev_bind_context_to_thread(ctx->bev_processor)) {
            fprintf(stderr, "[%s] ERROR: Failed to bind OpenGL context to thread\n", ctx->thread_name);
            free(bev_output_buf);
            return NULL;
        }
    }

    uint64_t processed_frames = 0;
    while (*ctx->running) {
        uint64_t frame_idx = 0;
#ifdef ENABLE_PERFORMANCE_MONITORING
        struct timespec t_bev_pop_start, t_bev_pop_end;
        clock_gettime(CLOCK_MONOTONIC, &t_bev_pop_start);
#endif
        FrameSlot* slot = frame_queue_pop_dmabuf(ctx->input_queue, &frame_idx);
#ifdef ENABLE_PERFORMANCE_MONITORING
        clock_gettime(CLOCK_MONOTONIC, &t_bev_pop_end);
        if (ctx->app) {
            perf_stats_add(&ctx->app->perf_bev_pop, elapsedUs(t_bev_pop_start, t_bev_pop_end));
        }
#endif
        if (!slot) {
            break;
        }

        const bool force_process = slot->force_process;
        const bool has_rtsp_appsrc = (ctx->stream && ctx->stream->appsrc);
        if (!ctx->bev_processor || (!has_rtsp_appsrc && !force_process)) {
            if (slot->is_dmabuf_mode && ctx->v4l2_cam) {
                v4l2_camera_queue(ctx->v4l2_cam, slot->v4l2_index);
            }
            continue;
        }

        bool rtsp_ready = false;
        if (has_rtsp_appsrc) {
            rtsp_ready = waitRtspReady(ctx->stream, ctx->running, ctx->thread_name);
        }

        if (!rtsp_ready && !force_process) {
            if (slot->is_dmabuf_mode && ctx->v4l2_cam) {
                v4l2_camera_queue(ctx->v4l2_cam, slot->v4l2_index);
            }
            continue;
        }

        bool process_ok = false;
        if (slot->buffer && slot->size > 0) {
#ifdef ENABLE_PERFORMANCE_MONITORING
            struct timespec t_bev_process_start, t_bev_process_end;
            clock_gettime(CLOCK_MONOTONIC, &t_bev_process_start);
#endif
            process_ok = bev_process_frame(ctx->bev_processor,
                                           slot->buffer, slot->size,
                                           bev_output_buf, bev_frame_size);
#ifdef ENABLE_PERFORMANCE_MONITORING
            clock_gettime(CLOCK_MONOTONIC, &t_bev_process_end);
            if (ctx->app) {
                perf_stats_add(&ctx->app->perf_bev_process,
                               elapsedUs(t_bev_process_start, t_bev_process_end));
            }
#endif
        }

        if (process_ok) {
            bev_frame_buffer_update(bev_output_buf, bev_frame_size, frame_idx);
            if (ctx->app) {
                recordLatestBevFrameId(&ctx->app->capture_state, frame_idx);
            }

#ifdef ENABLE_PERFORMANCE_MONITORING
            struct timespec t_bev_push_start, t_bev_push_end;
            clock_gettime(CLOCK_MONOTONIC, &t_bev_push_start);
#endif
            bool push_ok = false;
            if (rtsp_ready) {
                push_ok = rtsp_streamer_push_frame(ctx->stream,
                                                   bev_output_buf,
                                                   bev_frame_size,
                                                   frame_idx);
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
            }
        }

        if (slot->is_dmabuf_mode && ctx->v4l2_cam) {
            v4l2_camera_queue(ctx->v4l2_cam, slot->v4l2_index);
        }
    }

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
