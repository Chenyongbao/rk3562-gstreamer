#include "gles_context.h"
#include "gles_texture.h"
#include "dma_buffer.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <cstring>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include <cstdio>
#include <iomanip>

using namespace std;


static bool createDmaTexture(GLContext& ctx, const dma_buffer_t& dma_buf, GLuint& texture_id, EGLImageKHR& egl_image, uint32_t drm_format) {
    PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR = 
        (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES =
        (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
    
    if (!eglCreateImageKHR || !glEGLImageTargetTexture2DOES) {
        return false;
    }
    

    glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    

    EGLint attribs[] = {
        EGL_WIDTH, dma_buf.width,
        EGL_HEIGHT, dma_buf.height,
        EGL_LINUX_DRM_FOURCC_EXT, (EGLint)drm_format,
        EGL_DMA_BUF_PLANE0_FD_EXT, dma_buf.fd,
        EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
        EGL_DMA_BUF_PLANE0_PITCH_EXT, dma_buf.stride,
        EGL_NONE
    };
    
    egl_image = eglCreateImageKHR(
        ctx.display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, 
        (EGLClientBuffer)NULL, attribs);
    
    if (egl_image == EGL_NO_IMAGE_KHR) {
        EGLint egl_error = eglGetError();



        fprintf(stderr, "  Debug: eglCreateImageKHR failed for format 0x%x: 0x%x (size=%dx%d) - trying fallback\n", 
                drm_format, egl_error, dma_buf.width, dma_buf.height);
        glDeleteTextures(1, &texture_id);
        return false;
    }
    

    glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, egl_image);
    GLenum gl_error = glGetError();
    if (gl_error != GL_NO_ERROR) {

        #ifdef DEBUG
        EGLint egl_error = eglGetError();
        fprintf(stderr, "  Debug: glEGLImageTargetTexture2DOES failed: GL=0x%x, EGL=0x%x\n", gl_error, egl_error);
        #else
        eglGetError();
        #endif
        PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR = 
            (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
        if (eglDestroyImageKHR) {
            eglDestroyImageKHR(ctx.display, egl_image);
        }
        glDeleteTextures(1, &texture_id);
        return false;
    }
    
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}


bool initDmaInputBuffers(GLContext& ctx, int width, int height) {

    if (ctx.dma_input_y.fd >= 0 && ctx.dma_input_y.ptr != nullptr) {
        return true;
    }
    

    if (!init_dma_heap()) {
        return false;
    }
    

    if (!alloc_dma_buffer(width, height, 1, &ctx.dma_input_y)) {
        return false;
    }
    

    int uv_width = width / 2;
    int uv_height = height / 2;
    if (!alloc_dma_buffer(uv_width, uv_height, 2, &ctx.dma_input_uv)) {
        free_dma_buffer(&ctx.dma_input_y);
        return false;
    }
    
    return true;
}


bool writeNV12ToDmaBuffers(GLContext& ctx, const std::vector<uint8_t>& nv12_data, int width, int height) {
    return writeNV12ToDmaBuffersPtr(ctx, nv12_data.data(), width, height);
}


bool writeNV12ToDmaBuffersPtr(GLContext& ctx, const uint8_t* nv12_data_ptr, int width, int height) {

    if (ctx.dma_input_y.fd < 0 || ctx.dma_input_y.ptr == nullptr) {
        if (!initDmaInputBuffers(ctx, width, height)) {
            return false;
        }
    }
    

    struct dma_buf_sync sync;
    const size_t y_size = width * height;
    const size_t uv_size = width * height / 2;
    

    uint8_t* y_dst = (uint8_t*)ctx.dma_input_y.ptr;
    const uint8_t* y_src = nv12_data_ptr;
    
    if (y_src != y_dst) {


        sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE;
        ioctl(ctx.dma_input_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
        

        memcpy(y_dst, y_src, y_size);
        
        sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
        ioctl(ctx.dma_input_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
    }

    

    if (ctx.dma_input_uv.fd >= 0 && ctx.dma_input_uv.ptr != nullptr) {
        uint8_t* uv_dst = (uint8_t*)ctx.dma_input_uv.ptr;
        const uint8_t* uv_src = nv12_data_ptr + y_size;
        
        if (uv_src != uv_dst) {

            sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE;
            ioctl(ctx.dma_input_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);
            
            memcpy(uv_dst, uv_src, uv_size);
            
            sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
            ioctl(ctx.dma_input_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);
        }

    }
    
    return true;
}


const uint8_t* getDmaInputBufferPtr(GLContext& ctx) {
    if (ctx.dma_input_y.fd >= 0 && ctx.dma_input_y.ptr != nullptr) {
        return static_cast<const uint8_t*>(ctx.dma_input_y.ptr);
    }
    return nullptr;
}


void syncDmaInputBuffers(GLContext& ctx) {
    if (ctx.dma_input_y.fd >= 0 && ctx.dma_input_y.ptr != nullptr) {
        struct dma_buf_sync sync;
        sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE;
        ioctl(ctx.dma_input_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
        sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
        ioctl(ctx.dma_input_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
        
        if (ctx.dma_input_uv.fd >= 0 && ctx.dma_input_uv.ptr != nullptr) {
            sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE;
            ioctl(ctx.dma_input_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);
            sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
            ioctl(ctx.dma_input_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);
        }
    }
}



static void updateDmaBufferData(GLContext& ctx, const uint8_t* nv12_data_ptr, int width, int height) {

    struct dma_buf_sync sync;
    size_t y_size = width * height;
    size_t uv_size = width * height / 2;
    
    uint8_t* y_dst = (uint8_t*)ctx.dma_input_y.ptr;
    const uint8_t* y_src = nv12_data_ptr;
    

    bool y_already_in_dma = (y_src == y_dst);
    
    if (!y_already_in_dma) {

        sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE;
        ioctl(ctx.dma_input_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
        

        memcpy(y_dst, y_src, y_size);
        
        sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
        ioctl(ctx.dma_input_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
    }

    

    if (ctx.dma_input_uv.fd >= 0 && ctx.dma_input_uv.ptr != nullptr) {

        const uint8_t* uv_src = y_already_in_dma ? 
            (const uint8_t*)ctx.dma_input_uv.ptr :
            (nv12_data_ptr + y_size);
        
        uint8_t* uv_dst = (uint8_t*)ctx.dma_input_uv.ptr;
        

        if (uv_src != uv_dst) {

            sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE;
            ioctl(ctx.dma_input_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);
            
            memcpy(uv_dst, uv_src, uv_size);
            
            sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
            ioctl(ctx.dma_input_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);
        }

        


        if (ctx.input_texture_uv != 0 && ctx.egl_image_input_uv == EGL_NO_IMAGE_KHR && uv_dst != nullptr) {

            updateTexture(ctx.input_texture_uv, GL_LUMINANCE_ALPHA, width / 2, height / 2, uv_dst);
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }
}



bool uploadNV12Textures(GLContext& ctx, const vector<uint8_t>& nv12_data, int width, int height, bool create_new, const uint8_t* nv12_data_ptr) {
    auto start = chrono::high_resolution_clock::now();
    


    const uint8_t* data_ptr = (nv12_data_ptr != nullptr) ? nv12_data_ptr : 
                              (nv12_data.empty() ? nullptr : nv12_data.data());
    

    if (!create_new && ctx.dma_input_y.fd >= 0 && ctx.dma_input_y.ptr != nullptr) {


        if (data_ptr != nullptr) {

            updateDmaBufferData(ctx, data_ptr, width, height);
        }


        


        if ((ctx.dma_input_uv.fd < 0 || ctx.dma_input_uv.ptr == nullptr) && 
            ctx.input_texture_uv != 0 && data_ptr != nullptr) {

            const uint8_t* uv_data = data_ptr + width * height;
            updateTexture(ctx.input_texture_uv, GL_LUMINANCE_ALPHA, width / 2, height / 2, uv_data);
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        
        auto end = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
        ctx.texture_upload_time = duration.count() / 1000.0;
        return true;
    }
    

    if (!create_new && ctx.input_texture_y != 0 && ctx.input_texture_uv != 0) {
        const uint8_t* y_data = data_ptr;
        updateTexture(ctx.input_texture_y, GL_LUMINANCE, width, height, y_data);
        
        const uint8_t* uv_data = data_ptr + width * height;
        updateTexture(ctx.input_texture_uv, GL_LUMINANCE_ALPHA, width / 2, height / 2, uv_data);
        

        glBindTexture(GL_TEXTURE_2D, 0);
        
        auto end = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
        ctx.texture_upload_time = duration.count() / 1000.0;
        return true;
    }
    

    bool use_dma_y = false;
    bool use_dma_uv = false;
    

    if (ctx.dma_input_y.fd >= 0 && ctx.dma_input_y.ptr != nullptr) {

        use_dma_y = true;
        cout << "  Using existing DMA buffer for Y plane (zero-copy)" << endl;
    } else if (init_dma_heap()) {

        if (alloc_dma_buffer(width, height, 1, &ctx.dma_input_y)) {

            if (data_ptr != nullptr) {
                const uint8_t* y_data = data_ptr;
                if (y_data != ctx.dma_input_y.ptr) {

                    memcpy(ctx.dma_input_y.ptr, y_data, width * height);
                }
            }
            

            // DRM_FORMAT_R8 = fourcc_code('R', '8', ' ', ' ') = 0x20203852
            if (createDmaTexture(ctx, ctx.dma_input_y, ctx.input_texture_y, 
                                 ctx.egl_image_input_y, 0x20203852)) {
                use_dma_y = true;
            } else {
                free_dma_buffer(&ctx.dma_input_y);
            }
        }
    }
    

    if (use_dma_y) {

        if (ctx.dma_input_uv.fd >= 0 && ctx.dma_input_uv.ptr != nullptr) {

            use_dma_uv = true;
            cout << "  Using existing DMA buffer for UV plane (zero-copy)" << endl;
        } else {
            int uv_width = width / 2;
            int uv_height = height / 2;
            if (alloc_dma_buffer(uv_width, uv_height, 2, &ctx.dma_input_uv)) {

                if (data_ptr != nullptr) {
                    const uint8_t* uv_data = data_ptr + width * height;
                    if (uv_data != ctx.dma_input_uv.ptr) {

                        memcpy(ctx.dma_input_uv.ptr, uv_data, uv_width * uv_height * 2);
                    }
                }
                

                // DRM_FORMAT_GR88 = fourcc_code('G', 'R', '8', '8') = 0x38324752
                // DRM_FORMAT_RG88 = fourcc_code('R', 'G', '8', '8') = 0x38324752


                

                uint32_t gr88 = (('G' << 0) | ('R' << 8) | ('8' << 16) | ('8' << 24));
                uint32_t rg88 = (('R' << 0) | ('G' << 8) | ('8' << 16) | ('8' << 24));
                


                if (createDmaTexture(ctx, ctx.dma_input_uv, ctx.input_texture_uv,
                                    ctx.egl_image_input_uv, gr88)) {
                    use_dma_uv = true;
                    cout << "  UV DMA texture created with DRM_FORMAT_GR88 (0x" << hex << gr88 << dec << ")" << endl;
                } else {


                    if (createDmaTexture(ctx, ctx.dma_input_uv, ctx.input_texture_uv,
                                        ctx.egl_image_input_uv, rg88)) {
                        use_dma_uv = true;
                        ctx.uv_uses_rg88_format = true;
                        cout << "  UV DMA texture created with DRM_FORMAT_RG88 (0x" << hex << rg88 << dec << ")" << endl;
                    } else {


                        cout << "  UV DMA texture creation failed, using DMA buffer for fast updates (fallback to regular texture)" << endl;

                        if (data_ptr != nullptr) {
                            const uint8_t* uv_data = data_ptr + width * height;
                            ctx.input_texture_uv = createTexture(GL_LUMINANCE_ALPHA, uv_width, uv_height, uv_data);
                        } else {

                            ctx.input_texture_uv = createTexture(GL_LUMINANCE_ALPHA, uv_width, uv_height, ctx.dma_input_uv.ptr);
                        }

                    }
                }
            }
        }
        

        if (use_dma_uv && ctx.egl_image_input_uv == EGL_NO_IMAGE_KHR) {
            int uv_width = width / 2;
            int uv_height = height / 2;
            uint32_t gr88 = (('G' << 0) | ('R' << 8) | ('8' << 16) | ('8' << 24));
            uint32_t rg88 = (('R' << 0) | ('G' << 8) | ('8' << 16) | ('8' << 24));
            
            if (createDmaTexture(ctx, ctx.dma_input_uv, ctx.input_texture_uv,
                                ctx.egl_image_input_uv, gr88)) {
                cout << "  UV DMA texture created with DRM_FORMAT_GR88 (0x" << hex << gr88 << dec << ")" << endl;
            } else if (createDmaTexture(ctx, ctx.dma_input_uv, ctx.input_texture_uv,
                                       ctx.egl_image_input_uv, rg88)) {
                ctx.uv_uses_rg88_format = true;
                cout << "  UV DMA texture created with DRM_FORMAT_RG88 (0x" << hex << rg88 << dec << ")" << endl;
            }
        }
        

        if (use_dma_y && ctx.egl_image_input_y == EGL_NO_IMAGE_KHR) {
            if (createDmaTexture(ctx, ctx.dma_input_y, ctx.input_texture_y, 
                                 ctx.egl_image_input_y, 0x20203852)) {
                cout << "  Y DMA texture created" << endl;
            }
        }
    }
    

    if (use_dma_y) {
        cout << "  DMA zero-copy enabled for Y plane" << endl;
        if (use_dma_uv) {
            cout << "  DMA zero-copy enabled for UV plane" << endl;
        } else {

            const uint8_t* uv_data = data_ptr + width * height;
            if (ctx.input_texture_uv == 0) {

                ctx.input_texture_uv = createTexture(GL_LUMINANCE_ALPHA, width / 2, height / 2, uv_data);
            } else if (create_new) {

                glDeleteTextures(1, &ctx.input_texture_uv);
                ctx.input_texture_uv = createTexture(GL_LUMINANCE_ALPHA, width / 2, height / 2, uv_data);
            } else {

                updateTexture(ctx.input_texture_uv, GL_LUMINANCE_ALPHA, width / 2, height / 2, uv_data);
            }
        }
    } else {

        if (ctx.dma_input_y.fd >= 0) {
            free_dma_buffer(&ctx.dma_input_y);
        }
        if (ctx.dma_input_uv.fd >= 0) {
            free_dma_buffer(&ctx.dma_input_uv);
        }
        

        const uint8_t* y_data = data_ptr;
        if (ctx.input_texture_y == 0) {
            ctx.input_texture_y = createTexture(GL_LUMINANCE, width, height, y_data);
        } else if (create_new) {

            glDeleteTextures(1, &ctx.input_texture_y);
            ctx.input_texture_y = createTexture(GL_LUMINANCE, width, height, y_data);
        } else {

            updateTexture(ctx.input_texture_y, GL_LUMINANCE, width, height, y_data);
        }
        

        const uint8_t* uv_data = data_ptr + width * height;
        if (ctx.input_texture_uv == 0) {
            ctx.input_texture_uv = createTexture(GL_LUMINANCE_ALPHA, width / 2, height / 2, uv_data);
        } else if (create_new) {

            glDeleteTextures(1, &ctx.input_texture_uv);
            ctx.input_texture_uv = createTexture(GL_LUMINANCE_ALPHA, width / 2, height / 2, uv_data);
        } else {

            updateTexture(ctx.input_texture_uv, GL_LUMINANCE_ALPHA, width / 2, height / 2, uv_data);
        }
    }
    
    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
    ctx.texture_upload_time = duration.count() / 1000.0;
    

    bool texture_ready = (ctx.input_texture_y != 0 && ctx.input_texture_uv != 0);
    if (!texture_ready) {
        cerr << "Error: Input textures not created (Y=" << ctx.input_texture_y << ", UV=" << ctx.input_texture_uv << ")" << endl;
    }
    return texture_ready;
}

