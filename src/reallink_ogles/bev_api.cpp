#include "bev_api.h"
#include "gles_context.h"
#include "camera.h"

#include <EGL/egl.h>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include <vector>

using namespace std;
using namespace cv;

struct BevProcessor {
    GLContext gl_ctx;
    int input_width;
    int input_height;
    int output_width;
    int output_height;
    bool initialized;
    bool first_frame;
    uint64_t map_version;
    long total_process_time_us;
    int process_count;
    int last_rendered_slot;
};

namespace {

bool refreshOverheadMaps(BevProcessor* processor)
{
    if (!processor) {
        return false;
    }

    const uint64_t current_ver = getOverHeadMapVersion();
    if (current_ver == processor->map_version) {
        return true;
    }

    cv::Mat mapX, mapY;
    getOverHeadMaps(mapX, mapY);
    if (!loadMapTextures(processor->gl_ctx, mapX, mapY)) {
        cerr << "[BEV] ERROR: Failed to reload map textures!" << endl;
        return false;
    }

    processor->map_version = current_ver;
    cout << "[BEV] Map textures reloaded (version=" << processor->map_version << ")" << endl;
    return true;
}

bool copyOutputIfRequested(BevProcessor* processor,
                           uint8_t* nv12_output,
                           size_t output_size)
{
    if (!processor) {
        return false;
    }

    const size_t expected_output =
        static_cast<size_t>(processor->output_width) *
        static_cast<size_t>(processor->output_height) * 3 / 2;
    if (!nv12_output) {
        return true;
    }
    if (output_size < expected_output) {
        return false;
    }

    Mat output_mat = readOutputBuffer(processor->gl_ctx,
                                      processor->output_width,
                                      processor->output_height);
    if (output_mat.empty()) {
        return false;
    }

    memcpy(nv12_output, output_mat.data, expected_output);
    return true;
}

void recordProcessTime(BevProcessor* processor,
                       const chrono::high_resolution_clock::time_point& start)
{
    if (!processor) {
        return;
    }

    const auto end = chrono::high_resolution_clock::now();
    const long process_time =
        chrono::duration_cast<chrono::microseconds>(end - start).count();

    processor->total_process_time_us += process_time;
    processor->process_count++;

    if (processor->process_count % 60 == 0) {
        const long avg_time = processor->total_process_time_us / processor->process_count;
        cout << "[BEV] Perf: avg=" << (avg_time / 1000.0) << " ms"
             << ", upload=" << processor->gl_ctx.texture_upload_time << " ms"
             << ", remap=" << processor->gl_ctx.remap_render_time << " ms"
             << ", readback=" << processor->gl_ctx.buffer_export_time << " ms"
             << endl;
    }
}

bool processWithCpuInput(BevProcessor* processor,
                         const uint8_t* nv12_input,
                         size_t input_size,
                         uint8_t* nv12_output,
                         size_t output_size)
{
    if (!processor || !nv12_input) {
        return false;
    }

    const size_t expected_input =
        static_cast<size_t>(processor->input_width) *
        static_cast<size_t>(processor->input_height) * 3 / 2;
    if (input_size < expected_input) {
        return false;
    }

    vector<uint8_t> empty_data;
    const bool create_new_input_textures =
        processor->first_frame ||
        processor->gl_ctx.input_uses_external_dmabuf ||
        processor->gl_ctx.input_texture_y == 0 ||
        processor->gl_ctx.input_texture_uv == 0;

    if (create_new_input_textures) {
        if (!uploadNV12Textures(processor->gl_ctx,
                                empty_data,
                                processor->input_width,
                                processor->input_height,
                                true,
                                nv12_input)) {
            cerr << "[BEV] ERROR: First frame texture upload failed!" << endl;
            return false;
        }

        const bool using_dma =
            processor->gl_ctx.dma_input_y.fd >= 0 &&
            processor->gl_ctx.dma_input_y.ptr != nullptr;
        cout << "[BEV] Using " << (using_dma ? "DMA buffer" : "regular texture") << endl;
        if (using_dma) {
            const char* gl_ver = reinterpret_cast<const char*>(glGetString(GL_VERSION));
            if (gl_ver) {
                cout << "  GL_VERSION: " << gl_ver << endl;
            }
            cout << "  Input DMA Y: fd=" << processor->gl_ctx.dma_input_y.fd
                 << " egl=" << processor->gl_ctx.egl_image_input_y
                 << " tex=" << processor->gl_ctx.input_texture_y << endl;
            cout << "  Input DMA UV: fd=" << processor->gl_ctx.dma_input_uv.fd
                 << " egl=" << processor->gl_ctx.egl_image_input_uv
                 << " tex=" << processor->gl_ctx.input_texture_uv
                 << " fmt=" << (processor->gl_ctx.uv_uses_rg88_format ? "RG88" : "GR88")
                 << endl;
        }
    } else {
        if (!writeNV12ToDmaBuffersPtr(processor->gl_ctx,
                                      nv12_input,
                                      processor->input_width,
                                      processor->input_height)) {
            cerr << "[BEV] ERROR: Failed to update input DMA buffers" << endl;
            return false;
        }

        const uint8_t* data_ptr =
            static_cast<const uint8_t*>(processor->gl_ctx.dma_input_y.ptr);
        if (!uploadNV12Textures(processor->gl_ctx,
                                empty_data,
                                processor->input_width,
                                processor->input_height,
                                false,
                                data_ptr)) {
            cerr << "[BEV] ERROR: Failed to refresh input textures" << endl;
            return false;
        }
    }

    if (!performRemap(processor->gl_ctx,
                      processor->input_width,
                      processor->input_height,
                      processor->output_width,
                      processor->output_height)) {
        return false;
    }

    if (!copyOutputIfRequested(processor, nv12_output, output_size)) {
        return false;
    }

    processor->first_frame = false;
    return true;
}

bool processWithExternalDmabuf(BevProcessor* processor,
                               int nv12_input_fd,
                               int input_stride,
                               size_t input_size,
                               uint8_t* nv12_output,
                               size_t output_size)
{
    if (!processor || nv12_input_fd < 0) {
        return false;
    }
    if (input_stride < processor->input_width) {
        return false;
    }

    const size_t expected_input =
        static_cast<size_t>(input_stride) *
        static_cast<size_t>(processor->input_height) * 3 / 2;
    if (input_size < expected_input) {
        return false;
    }

    if (!uploadNV12DmabufTextures(processor->gl_ctx,
                                  nv12_input_fd,
                                  processor->input_width,
                                  processor->input_height,
                                  input_stride,
                                  input_size)) {
        cerr << "[BEV] ERROR: Failed to import external NV12 dmabuf" << endl;
        return false;
    }

    if (processor->first_frame) {
        const char* gl_ver = reinterpret_cast<const char*>(glGetString(GL_VERSION));
        cout << "[BEV] Using external dmabuf input" << endl;
        if (gl_ver) {
            cout << "  GL_VERSION: " << gl_ver << endl;
        }
        cout << "  Input DMA Y: fd=" << nv12_input_fd
             << " egl=" << processor->gl_ctx.egl_image_input_y
             << " tex=" << processor->gl_ctx.input_texture_y << endl;
        cout << "  Input DMA UV: fd=" << nv12_input_fd
             << " egl=" << processor->gl_ctx.egl_image_input_uv
             << " tex=" << processor->gl_ctx.input_texture_uv
             << " fmt=" << (processor->gl_ctx.uv_uses_rg88_format ? "RG88" : "GR88")
             << endl;
    }

    if (!performRemap(processor->gl_ctx,
                      processor->input_width,
                      processor->input_height,
                      processor->output_width,
                      processor->output_height)) {
        return false;
    }

    if (!copyOutputIfRequested(processor, nv12_output, output_size)) {
        return false;
    }

    processor->first_frame = false;
    return true;
}

}  // namespace

BevHandle bev_init(int input_width, int input_height,
                   int output_width, int output_height)
{
    try {
        BevProcessor* processor = new BevProcessor();
        processor->input_width = input_width;
        processor->input_height = input_height;
        processor->output_width = output_width;
        processor->output_height = output_height;
        processor->first_frame = true;
        processor->total_process_time_us = 0;
        processor->process_count = 0;
        processor->initialized = false;
        processor->last_rendered_slot = -1;

        cameraInit();

        cv::Mat mapX, mapY;
        getOverHeadMaps(mapX, mapY);
        cout << "[BEV] Map size: " << mapX.cols << "x" << mapX.rows << endl;
        processor->map_version = getOverHeadMapVersion();

        if (!initGLContext(processor->gl_ctx, output_width, output_height)) {
            cerr << "[BEV] ERROR: Failed to initialize OpenGL ES context!" << endl;
            delete processor;
            return nullptr;
        }
        cout << "[BEV] EGL initialized successfully" << endl;

        if (!createShaderProgram(processor->gl_ctx)) {
            cerr << "[BEV] ERROR: Failed to create shader program!" << endl;
            cleanupGLContext(processor->gl_ctx);
            delete processor;
            return nullptr;
        }
        cout << "[BEV] Shader program created" << endl;

        if (!loadMapTextures(processor->gl_ctx, mapX, mapY)) {
            cerr << "[BEV] ERROR: Failed to load map textures!" << endl;
            cleanupGLContext(processor->gl_ctx);
            delete processor;
            return nullptr;
        }
        cout << "[BEV] Map textures loaded" << endl;

        if (!initFramebuffers(processor->gl_ctx, output_width, output_height)) {
            cerr << "[BEV] ERROR: Failed to init framebuffers!" << endl;
            cleanupGLContext(processor->gl_ctx);
            delete processor;
            return nullptr;
        }

        if (!initOutputPool(processor->gl_ctx, output_width, output_height)) {
            cerr << "[BEV] ERROR: Failed to init zero-copy output pool!" << endl;
            cleanupGLContext(processor->gl_ctx);
            delete processor;
            return nullptr;
        }

        processor->initialized = true;
        cout << "[BEV] Initialization complete!" << endl;

        eglMakeCurrent(processor->gl_ctx.display,
                       EGL_NO_SURFACE,
                       EGL_NO_SURFACE,
                       EGL_NO_CONTEXT);

        return processor;
    } catch (const exception& e) {
        cerr << "[BEV] Exception during initialization: " << e.what() << endl;
        return nullptr;
    }
}

bool bev_bind_context_to_thread(BevHandle handle)
{
    if (!handle) return false;
    BevProcessor* processor = (BevProcessor*)handle;
    if (!processor->initialized) return false;
    if (!eglMakeCurrent(processor->gl_ctx.display,
                        processor->gl_ctx.egl_surface,
                        processor->gl_ctx.egl_surface,
                        processor->gl_ctx.context)) {
        fprintf(stderr, "[BEV] ERROR: eglMakeCurrent failed to bind context to this thread\n");
        return false;
    }
    return true;
}

bool bev_process_frame(BevHandle handle,
                       const uint8_t* nv12_input,
                       size_t input_size,
                       uint8_t* nv12_output,
                       size_t output_size)
{
    if (!handle || !nv12_input) {
        return false;
    }

    BevProcessor* processor = (BevProcessor*)handle;
    if (!processor->initialized) {
        return false;
    }

    const auto start = chrono::high_resolution_clock::now();

    try {
        if (!refreshOverheadMaps(processor)) {
            return false;
        }
        if (!processWithCpuInput(processor,
                                 nv12_input,
                                 input_size,
                                 nv12_output,
                                 output_size)) {
            return false;
        }

        recordProcessTime(processor, start);
        return true;
    } catch (const exception& e) {
        cerr << "[BEV] Exception: " << e.what() << endl;
        return false;
    }
}

bool bev_process_frame_dmabuf(BevHandle handle,
                              int nv12_input_fd,
                              int input_stride,
                              size_t input_size,
                              uint8_t* nv12_output,
                              size_t output_size)
{
    if (!handle || nv12_input_fd < 0) {
        return false;
    }

    BevProcessor* processor = (BevProcessor*)handle;
    if (!processor->initialized) {
        return false;
    }

    const auto start = chrono::high_resolution_clock::now();

    try {
        if (!refreshOverheadMaps(processor)) {
            return false;
        }
        if (!processWithExternalDmabuf(processor,
                                       nv12_input_fd,
                                       input_stride,
                                       input_size,
                                       nv12_output,
                                       output_size)) {
            return false;
        }

        recordProcessTime(processor, start);
        return true;
    } catch (const exception& e) {
        cerr << "[BEV] Exception: " << e.what() << endl;
        return false;
    }
}

long bev_get_avg_process_time_us(BevHandle handle)
{
    if (!handle) return -1;
    BevProcessor* processor = (BevProcessor*)handle;
    if (processor->process_count == 0) return 0;
    return processor->total_process_time_us / processor->process_count;
}

void bev_cleanup(BevHandle handle)
{
    if (!handle) return;
    BevProcessor* processor = (BevProcessor*)handle;
    cout << "[BEV] Cleaning up..." << endl;
    if (processor->initialized) {
        cleanupGLContext(processor->gl_ctx);
    }
    delete processor;
    cout << "[BEV] Cleanup complete" << endl;
}

bool bev_get_output_dmabuf_info(BevHandle handle,
                                int* fd_y, int* stride_y,
                                int* width_y, int* height_y,
                                int* fd_uv, int* stride_uv,
                                int* width_uv, int* height_uv,
                                int* uv_is_rg88)
{
    if (!handle) return false;
    BevProcessor* p = (BevProcessor*)handle;
    if (fd_y) *fd_y = p->gl_ctx.dma_output_y.fd;
    if (stride_y) *stride_y = p->gl_ctx.dma_output_y.stride;
    if (width_y) *width_y = p->output_width;
    if (height_y) *height_y = p->output_height;
    if (fd_uv) *fd_uv = p->gl_ctx.dma_output_uv.fd;
    if (stride_uv) *stride_uv = p->gl_ctx.dma_output_uv.stride;
    if (width_uv) *width_uv = p->output_width / 2;
    if (height_uv) *height_uv = p->output_height / 2;
    if (uv_is_rg88) *uv_is_rg88 = p->gl_ctx.uv_uses_rg88_format ? 1 : 0;
    bool ok = (p->gl_ctx.dma_output_y.fd >= 0 && p->gl_ctx.dma_output_uv.fd >= 0);
    return ok;
}

bool bev_acquire_output_dmabuf(BevHandle handle)
{
    if (!handle) return false;
    BevProcessor* p = (BevProcessor*)handle;
    if (!(p->gl_ctx.dma_output_y.fd >= 0 && p->gl_ctx.dma_output_uv.fd >= 0)) return false;
    glFlush();
    glFinish();
    struct dma_buf_sync sync;
    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
    ioctl(p->gl_ctx.dma_output_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
    ioctl(p->gl_ctx.dma_output_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);
    return true;
}

void bev_release_output_dmabuf(BevHandle handle)
{
    if (!handle) return;
    BevProcessor* p = (BevProcessor*)handle;
    if (!(p->gl_ctx.dma_output_y.fd >= 0 && p->gl_ctx.dma_output_uv.fd >= 0)) return;
    struct dma_buf_sync sync;
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    ioctl(p->gl_ctx.dma_output_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
    ioctl(p->gl_ctx.dma_output_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);
}

bool bev_process_frame_dmabuf_pooled(BevHandle handle,
                                     int nv12_input_fd,
                                     int input_stride,
                                     size_t input_size,
                                     int* out_fd, int* out_stride_y, int* out_stride_uv)
{
    if (!handle || nv12_input_fd < 0) {
        return false;
    }

    BevProcessor* processor = (BevProcessor*)handle;
    if (!processor->initialized) {
        return false;
    }
    if (processor->gl_ctx.output_pool_count == 0) {
        return false;
    }

    if (!refreshOverheadMaps(processor)) {
        return false;
    }

    if (!uploadNV12DmabufTextures(processor->gl_ctx,
                                  nv12_input_fd,
                                  processor->input_width,
                                  processor->input_height,
                                  input_stride,
                                  input_size)) {
        cerr << "[BEV] ERROR: Failed to import external NV12 dmabuf" << endl;
        return false;
    }

    // 渲染到输出池当前槽位，渲染后 index 推进到下一槽位。
    if (!performRemapPooled(processor->gl_ctx,
                            processor->input_width,
                            processor->input_height,
                            processor->output_width,
                            processor->output_height)) {
        return false;
    }

    // 反推出刚渲染的槽位。
    int idx = processor->gl_ctx.output_pool_index;
    idx = (idx == 0) ? (processor->gl_ctx.output_pool_count - 1) : (idx - 1);
    processor->last_rendered_slot = idx;

    GLOutputSlot& s = processor->gl_ctx.output_pool[idx];

    // 确保 GPU 渲染完成，并使数据对读者（CPU 读回 / 编码器）可见。
    glFlush();
    glFinish();
    struct dma_buf_sync sync;
    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
    ioctl(s.dma.fd, DMA_BUF_IOCTL_SYNC, &sync);

    if (out_fd) *out_fd = s.dma.fd;
    if (out_stride_y) *out_stride_y = s.stride_y;
    if (out_stride_uv) *out_stride_uv = s.stride_uv;

    processor->first_frame = false;
    return true;
}

void bev_release_output_pool_slot(BevHandle handle)
{
    if (!handle) return;
    BevProcessor* p = (BevProcessor*)handle;
    if (p->last_rendered_slot < 0 || p->last_rendered_slot >= p->gl_ctx.output_pool_count) return;
    GLOutputSlot& s = p->gl_ctx.output_pool[p->last_rendered_slot];
    struct dma_buf_sync sync;
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    ioctl(s.dma.fd, DMA_BUF_IOCTL_SYNC, &sync);
}

bool bev_read_last_output(BevHandle handle, uint8_t* nv12_output, size_t output_size)
{
    if (!handle || !nv12_output) return false;
    BevProcessor* p = (BevProcessor*)handle;
    if (p->last_rendered_slot < 0 || p->last_rendered_slot >= p->gl_ctx.output_pool_count) {
        return false;
    }

    const size_t expected =
        static_cast<size_t>(p->output_width) * static_cast<size_t>(p->output_height) * 3 / 2;
    if (output_size < expected) {
        return false;
    }

    GLOutputSlot& s = p->gl_ctx.output_pool[p->last_rendered_slot];
    if (!s.dma.ptr) {
        return false;
    }

    const int w = p->output_width;
    const int h = p->output_height;
    const uint8_t* base = static_cast<const uint8_t*>(s.dma.ptr);

    // Y 平面：按 stride_y 逐行拷贝紧凑行（stride 可能 > width）。
    for (int r = 0; r < h; ++r) {
        memcpy(nv12_output + r * w, base + r * s.stride_y, w);
    }

    // UV 平面：交错 UV，紧跟 Y 之后（offset = y_size）。
    const uint8_t* uv = base + s.y_size;
    uint8_t* out_uv = nv12_output + w * h;
    for (int r = 0; r < h / 2; ++r) {
        memcpy(out_uv + r * w, uv + r * s.stride_uv, w);
    }

    return true;
}
