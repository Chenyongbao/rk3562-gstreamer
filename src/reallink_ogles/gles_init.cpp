#include "gles_context.h"
#include "dma_buffer.h"
#include <iostream>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <iomanip>

using namespace std;


bool initGLContext(GLContext& ctx, int width, int height) {
    auto start = chrono::high_resolution_clock::now();
    

    ctx.drm_fd = -1;
    ctx.gbm_device = nullptr;
    ctx.gbm_surface = nullptr;
    ctx.display = EGL_NO_DISPLAY;
    ctx.context = EGL_NO_CONTEXT;
    ctx.egl_surface = EGL_NO_SURFACE;
    ctx.input_texture_y = 0;
    ctx.input_texture_uv = 0;
    ctx.map_texture_xy = 0;
    ctx.output_texture = 0;
    ctx.framebuffer = 0;
    ctx.egl_image_output = EGL_NO_IMAGE_KHR;
    ctx.egl_image_input_y = EGL_NO_IMAGE_KHR;
    ctx.egl_image_input_uv = EGL_NO_IMAGE_KHR;
    ctx.uv_uses_rg88_format = false;
    ctx.input_uv_prefers_rg88 = false;
    ctx.input_uses_external_dmabuf = false;
    memset(&ctx.dma_output, 0, sizeof(ctx.dma_output));
    ctx.dma_output.fd = -1;

    memset(&ctx.dma_output_y, 0, sizeof(ctx.dma_output_y));
    memset(&ctx.dma_output_uv, 0, sizeof(ctx.dma_output_uv));
    ctx.dma_output_y.fd = -1;
    ctx.dma_output_uv.fd = -1;
    memset(ctx.output_pool, 0, sizeof(ctx.output_pool));
    for (int i = 0; i < BEV_OUTPUT_POOL_SIZE; ++i) {
        ctx.output_pool[i].dma.fd = -1;
        ctx.output_pool[i].egl_image_y = EGL_NO_IMAGE_KHR;
        ctx.output_pool[i].egl_image_uv = EGL_NO_IMAGE_KHR;
    }
    ctx.output_pool_count = 0;
    ctx.output_pool_index = 0;
    memset(&ctx.dma_input_y, 0, sizeof(ctx.dma_input_y));
    ctx.dma_input_y.fd = -1;
    memset(&ctx.dma_input_uv, 0, sizeof(ctx.dma_input_uv));
    ctx.dma_input_uv.fd = -1;
    ctx.shader_program = 0;
    ctx.sync_query_y = 0;
    ctx.sync_query_uv = 0;
    ctx.sync_query_supported = false;
    

    cout << "  Getting EGL display..." << endl;
    

    PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT = nullptr;
    eglGetPlatformDisplayEXT = (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
    
    if (eglGetPlatformDisplayEXT) {

        const char* drm_nodes[] = {
            "/dev/dri/renderD128",
            "/dev/dri/renderD129",
            "/dev/dri/card0",
            "/dev/dri/card1",
        };
        
        for (const char* node : drm_nodes) {
            ctx.drm_fd = open(node, O_RDWR | O_CLOEXEC);
            if (ctx.drm_fd < 0) {
                continue;
            }
            
            cout << "  Opened DRM device: " << node << " (fd=" << ctx.drm_fd << ")" << endl;
            
            ctx.gbm_device = gbm_create_device(ctx.drm_fd);
            if (!ctx.gbm_device) {
                cout << "  Failed to create GBM device for " << node << endl;
                close(ctx.drm_fd);
                ctx.drm_fd = -1;
                continue;
            }
            
            ctx.display = eglGetPlatformDisplayEXT(EGL_PLATFORM_GBM_KHR, ctx.gbm_device, nullptr);
            if (ctx.display != EGL_NO_DISPLAY) {
                EGLint major, minor;
                if (eglInitialize(ctx.display, &major, &minor)) {
                    cout << "  Successfully initialized EGL with GBM platform (" << node << ")" << endl;
                    cout << "  EGL version: " << major << "." << minor << endl;
                    goto egl_initialized;
                } else {
                    cout << "  eglInitialize failed for GBM platform, trying next..." << endl;
                    eglTerminate(ctx.display);
                    ctx.display = EGL_NO_DISPLAY;
                }
            }
            

            if (ctx.gbm_device) {
                gbm_device_destroy(ctx.gbm_device);
                ctx.gbm_device = nullptr;
            }
            close(ctx.drm_fd);
            ctx.drm_fd = -1;
        }
        
        cout << "  All GBM platform attempts failed, trying fallback..." << endl;
    }
    

    ctx.display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (ctx.display == EGL_NO_DISPLAY) {
        cerr << "ERROR: Failed to get EGL display" << endl;
        return false;
    }
    
    cout << "  Initializing EGL..." << endl;
    EGLint major, minor;
    if (!eglInitialize(ctx.display, &major, &minor)) {
        EGLint error = eglGetError();
        cerr << "ERROR: Failed to initialize EGL, error: 0x" << hex << error << dec << endl;
        eglTerminate(ctx.display);
        if (ctx.gbm_device) gbm_device_destroy(ctx.gbm_device);
        if (ctx.drm_fd >= 0) close(ctx.drm_fd);
        return false;
    }
    
egl_initialized:
    cout << "  EGL initialized successfully" << endl;
    

    EGLint config_attrs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 8,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_NONE
    };
    
    EGLint num_configs;
    if (!eglChooseConfig(ctx.display, config_attrs, &ctx.config, 1, &num_configs) || num_configs == 0) {
        cerr << "Failed to choose EGL config" << endl;
        eglTerminate(ctx.display);
        return false;
    }
    cout << "  EGL config chosen" << endl;
    

    bool use_dma = false;
    if (init_dma_heap()) {
        if (alloc_dma_buffer(width, height, 4, &ctx.dma_output)) {

            PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR = 
                (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
            
            if (eglCreateImageKHR) {
                EGLint attribs[] = {
                    EGL_WIDTH, width,
                    EGL_HEIGHT, height,
                    EGL_LINUX_DRM_FOURCC_EXT, 0x34325241, // DRM_FORMAT_ABGR8888
                    EGL_DMA_BUF_PLANE0_FD_EXT, ctx.dma_output.fd,
                    EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
                    EGL_DMA_BUF_PLANE0_PITCH_EXT, ctx.dma_output.stride,
                    EGL_NONE
                };
                
                ctx.egl_image_output = eglCreateImageKHR(
                    ctx.display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, 
                    (EGLClientBuffer)NULL, attribs);
                
                if (ctx.egl_image_output != EGL_NO_IMAGE_KHR) {

                    glGenTextures(1, &ctx.output_texture);
                    glBindTexture(GL_TEXTURE_2D, ctx.output_texture);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    
                    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES =
                        (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
                    
                    if (glEGLImageTargetTexture2DOES) {
                        glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, ctx.egl_image_output);
                        if (glGetError() == GL_NO_ERROR) {

                            glGenFramebuffers(1, &ctx.framebuffer);
                            glBindFramebuffer(GL_FRAMEBUFFER, ctx.framebuffer);
                            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
                                                   GL_TEXTURE_2D, ctx.output_texture, 0);
                            
                            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE) {
                                use_dma = true;
                                cout << "  DMA buffer enabled for zero-copy output" << endl;
                            }
                        }
                    }
                }
            }
        }
    }
    

    if (!use_dma) {
        if (ctx.dma_output.fd >= 0) {
            free_dma_buffer(&ctx.dma_output);
        }
        
        EGLint pbuffer_attrs[] = {
            EGL_WIDTH, width,
            EGL_HEIGHT, height,
            EGL_NONE
        };
        
        ctx.egl_surface = eglCreatePbufferSurface(ctx.display, ctx.config, pbuffer_attrs);
        if (ctx.egl_surface == EGL_NO_SURFACE) {
            EGLint error = eglGetError();
            cerr << "Failed to create EGL PBuffer surface, error: 0x" << hex << error << dec << endl;
            eglTerminate(ctx.display);
            return false;
        }
        cout << "  EGL PBuffer surface created (fallback)" << endl;
    }
    

    EGLint context_attrs[] = {
        EGL_CONTEXT_CLIENT_VERSION, 2,
        EGL_NONE
    };
    ctx.context = eglCreateContext(ctx.display, ctx.config, EGL_NO_CONTEXT, context_attrs);
    if (ctx.context == EGL_NO_CONTEXT) {
        cerr << "Failed to create EGL context" << endl;
        eglDestroySurface(ctx.display, ctx.egl_surface);
        eglTerminate(ctx.display);
        return false;
    }
    

    if (!eglMakeCurrent(ctx.display, ctx.egl_surface, ctx.egl_surface, ctx.context)) {
        cerr << "Failed to make EGL context current" << endl;
        eglDestroyContext(ctx.display, ctx.context);
        eglDestroySurface(ctx.display, ctx.egl_surface);
        eglTerminate(ctx.display);
        return false;
    }
    
    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
    ctx.egl_init_time = duration.count() / 1000.0;
    
    return true;
}


void cleanupGLContext(GLContext& ctx) {

    PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR = 
        (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
    
    if (eglDestroyImageKHR) {
        if (ctx.egl_image_output != EGL_NO_IMAGE_KHR) {
            eglDestroyImageKHR(ctx.display, ctx.egl_image_output);
            ctx.egl_image_output = EGL_NO_IMAGE_KHR;
        }
        if (ctx.egl_image_input_y != EGL_NO_IMAGE_KHR) {
            eglDestroyImageKHR(ctx.display, ctx.egl_image_input_y);
            ctx.egl_image_input_y = EGL_NO_IMAGE_KHR;
        }
        if (ctx.egl_image_input_uv != EGL_NO_IMAGE_KHR) {
            eglDestroyImageKHR(ctx.display, ctx.egl_image_input_uv);
            ctx.egl_image_input_uv = EGL_NO_IMAGE_KHR;
        }
    }
    
    if (ctx.dma_output.fd >= 0) {
        free_dma_buffer(&ctx.dma_output);
    }
    if (ctx.dma_input_y.fd >= 0) {
        free_dma_buffer(&ctx.dma_input_y);
    }
    if (ctx.dma_input_uv.fd >= 0) {
        free_dma_buffer(&ctx.dma_input_uv);
    }

    cleanupOutputPool(ctx);
    
    if (ctx.framebuffer) glDeleteFramebuffers(1, &ctx.framebuffer);
    if (ctx.output_texture) glDeleteTextures(1, &ctx.output_texture);
    if (ctx.input_texture_y) glDeleteTextures(1, &ctx.input_texture_y);
    if (ctx.input_texture_uv) glDeleteTextures(1, &ctx.input_texture_uv);
    if (ctx.map_texture_xy) glDeleteTextures(1, &ctx.map_texture_xy);
    if (ctx.shader_program) glDeleteProgram(ctx.shader_program);
    

    if (ctx.sync_query_y != 0) glDeleteQueries(1, &ctx.sync_query_y);
    if (ctx.sync_query_uv != 0) glDeleteQueries(1, &ctx.sync_query_uv);
    
    eglMakeCurrent(ctx.display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (ctx.context != EGL_NO_CONTEXT) eglDestroyContext(ctx.display, ctx.context);
    if (ctx.egl_surface != EGL_NO_SURFACE) eglDestroySurface(ctx.display, ctx.egl_surface);
    eglTerminate(ctx.display);
    if (ctx.gbm_device) gbm_device_destroy(ctx.gbm_device);
    if (ctx.drm_fd >= 0) close(ctx.drm_fd);
    
    deinit_dma_heap();
}

