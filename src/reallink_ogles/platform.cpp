#include "platform.h"
#include "dma_buffer.h"
#include <iostream>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <GLES3/gl3.h>
#include <gbm.h>
#include <iomanip>
#include <cstring>

using namespace std;
using namespace cv;


typedef EGLDisplay (EGLAPIENTRY *PFNEGLGETPLATFORMDISPLAYEXTPROC)(EGLenum platform, void *native_display, const EGLint *attrib_list);
typedef EGLImageKHR (EGLAPIENTRY *PFNEGLCREATEIMAGEKHRPROC)(EGLDisplay dpy, EGLContext ctx, EGLenum target, EGLClientBuffer buffer, const EGLint *attrib_list);
typedef EGLBoolean (EGLAPIENTRY *PFNEGLDESTROYIMAGEKHRPROC)(EGLDisplay dpy, EGLImageKHR image);
typedef void (GL_APIENTRY *PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)(GLenum target, GLeglImageOES image);


bool platformInit(GLContext& ctx, int output_width, int output_height, 
                  int input_width, int input_height) {
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

    memset(&ctx.dma_output_y, 0, sizeof(ctx.dma_output_y));
    ctx.dma_output_y.fd = -1;
    memset(&ctx.dma_output_uv, 0, sizeof(ctx.dma_output_uv));
    ctx.dma_output_uv.fd = -1;
    ctx.output_texture_y = 0;
    ctx.output_texture_uv = 0;
    ctx.framebuffer_y = 0;
    ctx.framebuffer_uv = 0;
    ctx.egl_image_output_y = EGL_NO_IMAGE_KHR;
    ctx.egl_image_output_uv = EGL_NO_IMAGE_KHR;
    

    ctx.shader_program_y = 0;
    ctx.shader_program_uv = 0;
    ctx.uniform_map_xy_uv = -1;
    

    ctx.pbo_y[0] = 0;
    ctx.pbo_y[1] = 0;
    ctx.pbo_uv[0] = 0;
    ctx.pbo_uv[1] = 0;
    ctx.current_pbo_index = 0;
    ctx.pbo_y_mapped[0] = nullptr;
    ctx.pbo_y_mapped[1] = nullptr;
    ctx.pbo_uv_mapped[0] = nullptr;
    ctx.pbo_uv_mapped[1] = nullptr;
    ctx.pbo_initialized = false;
    

    ctx.vao = 0;
    ctx.vao_initialized = false;
    

    memset(&ctx.dma_output, 0, sizeof(ctx.dma_output));
    ctx.dma_output.fd = -1;
    memset(&ctx.dma_input_y, 0, sizeof(ctx.dma_input_y));
    ctx.dma_input_y.fd = -1;
    memset(&ctx.dma_input_uv, 0, sizeof(ctx.dma_input_uv));
    ctx.dma_input_uv.fd = -1;
    ctx.shader_program = 0;
    

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
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
        EGL_NONE
    };
    

    EGLint num_configs = 0;
    if (!eglChooseConfig(ctx.display, config_attrs, &ctx.config, 1, &num_configs) || num_configs == 0) {
        cout << "  OpenGL ES 3.0+ not available, falling back to ES 2.0" << endl;
        config_attrs[9] = EGL_OPENGL_ES2_BIT;
        if (!eglChooseConfig(ctx.display, config_attrs, &ctx.config, 1, &num_configs) || num_configs == 0) {
            cerr << "Failed to choose EGL config" << endl;
            eglTerminate(ctx.display);
            if (ctx.gbm_device) gbm_device_destroy(ctx.gbm_device);
            if (ctx.drm_fd >= 0) close(ctx.drm_fd);
            return false;
        }
    } else {
        cout << "  OpenGL ES 3.0+ config available (PBO supported)" << endl;
    }
    cout << "  EGL config chosen" << endl;
    

    cout << "  Initializing DMA heap for zero-copy..." << endl;
    if (!init_dma_heap()) {
        cerr << "  Warning: Failed to initialize DMA heap, falling back to regular mode" << endl;
    } else {

        cout << "  Creating input DMA buffer (Y plane) for zero-copy: " 
             << input_width << "x" << input_height << endl;
        if (alloc_dma_buffer(input_width, input_height, 1, &ctx.dma_input_y)) {
            cout << "    ✓ Input DMA buffer (Y) created: " 
                 << ctx.dma_input_y.width << "x" << ctx.dma_input_y.height 
                 << ", stride=" << ctx.dma_input_y.stride << " bytes" << endl;
            cout << "    ✓ Zero-copy enabled: Input data can be written directly to DMA buffer" << endl;
        } else {
            cerr << "    ✗ Failed to create input DMA buffer (Y)" << endl;
        }
        

        int uv_width = input_width / 2;
        int uv_height = input_height / 2;
        cout << "  Creating input DMA buffer (UV plane) for zero-copy: " 
             << uv_width << "x" << uv_height << endl;
        if (alloc_dma_buffer(uv_width, uv_height, 2, &ctx.dma_input_uv)) {
            cout << "    ✓ Input DMA buffer (UV) created: " 
                 << ctx.dma_input_uv.width << "x" << ctx.dma_input_uv.height 
                 << ", stride=" << ctx.dma_input_uv.stride << " bytes" << endl;
        } else {
            cerr << "    ✗ Failed to create input DMA buffer (UV)" << endl;
        }
        

        cout << "  Creating output DMA buffer (Y plane) for zero-copy: " 
             << output_width << "x" << output_height << endl;
        if (alloc_dma_buffer(output_width, output_height, 1, &ctx.dma_output_y)) {
            cout << "    ✓ Output DMA buffer (Y) created: " 
                 << ctx.dma_output_y.width << "x" << ctx.dma_output_y.height 
                 << ", stride=" << ctx.dma_output_y.stride << " bytes" << endl;
        } else {
            cerr << "    ✗ Failed to create output DMA buffer (Y)" << endl;
        }
        

        int uv_output_width = output_width / 2;
        int uv_output_height = output_height / 2;
        cout << "  Creating output DMA buffer (UV plane) for zero-copy: " 
             << uv_output_width << "x" << uv_output_height << endl;
        if (alloc_dma_buffer(uv_output_width, uv_output_height, 2, &ctx.dma_output_uv)) {
            cout << "    ✓ Output DMA buffer (UV) created: " 
                 << ctx.dma_output_uv.width << "x" << ctx.dma_output_uv.height 
                 << ", stride=" << ctx.dma_output_uv.stride << " bytes" << endl;
        } else {
            cerr << "    ✗ Failed to create output DMA buffer (UV)" << endl;
        }
        

        if (alloc_dma_buffer(output_width, output_height, 4, &ctx.dma_output)) {

        }
    }
    

    EGLint context_attrs[] = {
        EGL_CONTEXT_CLIENT_VERSION, 3,
        EGL_NONE
    };
    
    EGLContext test_context = eglCreateContext(ctx.display, ctx.config, EGL_NO_CONTEXT, context_attrs);
    if (test_context == EGL_NO_CONTEXT) {

        cout << "  OpenGL ES 3.0 context creation failed, falling back to ES 2.0" << endl;
        context_attrs[1] = 2;
    } else {
        eglDestroyContext(ctx.display, test_context);
        cout << "  OpenGL ES 3.0 context available (PBO supported)" << endl;
    }
    ctx.context = eglCreateContext(ctx.display, ctx.config, EGL_NO_CONTEXT, context_attrs);
    if (ctx.context == EGL_NO_CONTEXT) {
        cerr << "Failed to create EGL context" << endl;
        eglTerminate(ctx.display);
        return false;
    }
    

    EGLint pbuffer_attrs[] = {
        EGL_WIDTH, output_width,
        EGL_HEIGHT, output_height,
        EGL_NONE
    };
    
    ctx.egl_surface = eglCreatePbufferSurface(ctx.display, ctx.config, pbuffer_attrs);
    if (ctx.egl_surface == EGL_NO_SURFACE) {
        EGLint error = eglGetError();
        cerr << "Failed to create EGL PBuffer surface, error: 0x" << hex << error << dec << endl;
        eglDestroyContext(ctx.display, ctx.context);
        eglTerminate(ctx.display);
        return false;
    }
    cout << "  EGL PBuffer surface created" << endl;
    

    if (!eglMakeCurrent(ctx.display, ctx.egl_surface, ctx.egl_surface, ctx.context)) {
        cerr << "Failed to make EGL context current" << endl;
        eglDestroyContext(ctx.display, ctx.context);
        eglDestroySurface(ctx.display, ctx.egl_surface);
        eglTerminate(ctx.display);
        return false;
    }
    

    PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR = 
        (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES =
        (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
    
    if (eglCreateImageKHR && glEGLImageTargetTexture2DOES) {

        if (ctx.dma_output_y.fd >= 0) {
            EGLint attribs_y[] = {
                EGL_WIDTH, output_width,
                EGL_HEIGHT, output_height,
                EGL_LINUX_DRM_FOURCC_EXT, 0x20203852,
                EGL_DMA_BUF_PLANE0_FD_EXT, ctx.dma_output_y.fd,
                EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
                EGL_DMA_BUF_PLANE0_PITCH_EXT, ctx.dma_output_y.stride,
                EGL_NONE
            };
            
            ctx.egl_image_output_y = eglCreateImageKHR(
                ctx.display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, 
                (EGLClientBuffer)NULL, attribs_y);
            
            if (ctx.egl_image_output_y != EGL_NO_IMAGE_KHR) {
                glGenTextures(1, &ctx.output_texture_y);
                glBindTexture(GL_TEXTURE_2D, ctx.output_texture_y);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                
                glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, ctx.egl_image_output_y);
                if (glGetError() == GL_NO_ERROR) {
                    glGenFramebuffers(1, &ctx.framebuffer_y);
                    glBindFramebuffer(GL_FRAMEBUFFER, ctx.framebuffer_y);
                    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
                                           GL_TEXTURE_2D, ctx.output_texture_y, 0);
                    
                    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE) {
                        cout << "  ✓ Y plane DMA buffer enabled for zero-copy output" << endl;
                    }
                }
            }
        }
        

        int uv_output_width = output_width / 2;
        int uv_output_height = output_height / 2;
        
        if (ctx.dma_output_uv.fd >= 0) {

            uint32_t gr88 = (('G' << 0) | ('R' << 8) | ('8' << 16) | ('8' << 24));
            EGLint attribs_uv[] = {
                EGL_WIDTH, uv_output_width,
                EGL_HEIGHT, uv_output_height,
                EGL_LINUX_DRM_FOURCC_EXT, (EGLint)gr88,
                EGL_DMA_BUF_PLANE0_FD_EXT, ctx.dma_output_uv.fd,
                EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
                EGL_DMA_BUF_PLANE0_PITCH_EXT, ctx.dma_output_uv.stride,
                EGL_NONE
            };
            
            ctx.egl_image_output_uv = eglCreateImageKHR(
                ctx.display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, 
                (EGLClientBuffer)NULL, attribs_uv);
            
            if (ctx.egl_image_output_uv == EGL_NO_IMAGE_KHR) {

                uint32_t rg88 = (('R' << 0) | ('G' << 8) | ('8' << 16) | ('8' << 24));
                attribs_uv[3] = (EGLint)rg88;
                ctx.egl_image_output_uv = eglCreateImageKHR(
                    ctx.display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, 
                    (EGLClientBuffer)NULL, attribs_uv);
                if (ctx.egl_image_output_uv != EGL_NO_IMAGE_KHR) {
                    ctx.uv_uses_rg88_format = true;
                }
            }
            
            if (ctx.egl_image_output_uv != EGL_NO_IMAGE_KHR) {
                glGenTextures(1, &ctx.output_texture_uv);
                glBindTexture(GL_TEXTURE_2D, ctx.output_texture_uv);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                
                glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, ctx.egl_image_output_uv);
                GLenum gl_error = glGetError();
                
                if (gl_error == GL_NO_ERROR) {
                    glGenFramebuffers(1, &ctx.framebuffer_uv);
                    glBindFramebuffer(GL_FRAMEBUFFER, ctx.framebuffer_uv);
                    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
                                           GL_TEXTURE_2D, ctx.output_texture_uv, 0);
                    
                    GLenum fbo_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
                    
                    if (fbo_status == GL_FRAMEBUFFER_COMPLETE) {
                        cout << "  ✓ UV plane DMA buffer enabled for zero-copy output" << endl;
                    } else {

                        glBindFramebuffer(GL_FRAMEBUFFER, 0);
                        glDeleteFramebuffers(1, &ctx.framebuffer_uv);
                        ctx.framebuffer_uv = 0;
                    }
                }
            }
            


        if (ctx.framebuffer_uv == 0) {
            cout << "  Attempting to create regular FBO for UV plane (fallback)..." << endl;
            

            const char* gl_version = (const char*)glGetString(GL_VERSION);
            bool es3_supported = (gl_version && strncmp(gl_version, "OpenGL ES 3", 11) == 0);
            
            glGenTextures(1, &ctx.output_texture_uv);
            glBindTexture(GL_TEXTURE_2D, ctx.output_texture_uv);
            
            if (es3_supported) {

                glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, uv_output_width, uv_output_height, 0, GL_RG, GL_UNSIGNED_BYTE, nullptr);
                cout << "  Using GL_RG8 format for UV plane (ES 3.0+)" << endl;
            } else {

                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, uv_output_width, uv_output_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
                cout << "  Using GL_RGBA format for UV plane (ES 2.0 fallback)" << endl;
            }
            
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            
            glGenFramebuffers(1, &ctx.framebuffer_uv);
            glBindFramebuffer(GL_FRAMEBUFFER, ctx.framebuffer_uv);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
                                  GL_TEXTURE_2D, ctx.output_texture_uv, 0);
            
            GLenum fbo_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
            if (fbo_status == GL_FRAMEBUFFER_COMPLETE) {
                cout << "  ✓ Regular UV FBO created successfully (fallback)" << endl;
            } else {
                cerr << "  ✗ Regular UV FBO also failed: 0x" << hex << fbo_status << dec << endl;
                return false;
            }
        }
        }
    }
    

    if (ctx.dma_output.fd >= 0 && eglCreateImageKHR) {
        EGLint attribs[] = {
            EGL_WIDTH, output_width,
            EGL_HEIGHT, output_height,
            EGL_LINUX_DRM_FOURCC_EXT, 0x34325241, // DRM_FORMAT_ABGR8888
            EGL_DMA_BUF_PLANE0_FD_EXT, ctx.dma_output.fd,
            EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
            EGL_DMA_BUF_PLANE0_PITCH_EXT, ctx.dma_output.stride,
            EGL_NONE
        };
        
        ctx.egl_image_output = eglCreateImageKHR(
            ctx.display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, 
            (EGLClientBuffer)NULL, attribs);
        
        if (ctx.egl_image_output != EGL_NO_IMAGE_KHR && glEGLImageTargetTexture2DOES) {
            glGenTextures(1, &ctx.output_texture);
            glBindTexture(GL_TEXTURE_2D, ctx.output_texture);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            
            glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, ctx.egl_image_output);
            if (glGetError() == GL_NO_ERROR) {
                glGenFramebuffers(1, &ctx.framebuffer);
                glBindFramebuffer(GL_FRAMEBUFFER, ctx.framebuffer);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
                                       GL_TEXTURE_2D, ctx.output_texture, 0);
            }
        }
    }
    
    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
    ctx.egl_init_time = duration.count() / 1000.0;
    
    cout << "  Platform initialization completed (" << ctx.egl_init_time << " ms)" << endl;
    
    return true;
}


void platformCleanup(GLContext& ctx) {

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
    

    if (ctx.shader_program_y) glDeleteProgram(ctx.shader_program_y);
    if (ctx.shader_program_uv) glDeleteProgram(ctx.shader_program_uv);
    

    if (ctx.framebuffer_y) glDeleteFramebuffers(1, &ctx.framebuffer_y);
    if (ctx.framebuffer_uv) glDeleteFramebuffers(1, &ctx.framebuffer_uv);
    if (ctx.output_texture_y) glDeleteTextures(1, &ctx.output_texture_y);
    if (ctx.output_texture_uv) glDeleteTextures(1, &ctx.output_texture_uv);
    

    if (ctx.input_texture_y) glDeleteTextures(1, &ctx.input_texture_y);
    if (ctx.input_texture_uv) glDeleteTextures(1, &ctx.input_texture_uv);
    if (ctx.map_texture_xy) glDeleteTextures(1, &ctx.map_texture_xy);
    

    if (ctx.framebuffer) glDeleteFramebuffers(1, &ctx.framebuffer);
    if (ctx.output_texture) glDeleteTextures(1, &ctx.output_texture);
    if (ctx.shader_program) glDeleteProgram(ctx.shader_program);
    
    eglMakeCurrent(ctx.display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (ctx.context != EGL_NO_CONTEXT) eglDestroyContext(ctx.display, ctx.context);
    if (ctx.egl_surface != EGL_NO_SURFACE) eglDestroySurface(ctx.display, ctx.egl_surface);
    eglTerminate(ctx.display);
    if (ctx.gbm_device) gbm_device_destroy(ctx.gbm_device);
    if (ctx.drm_fd >= 0) close(ctx.drm_fd);
    
    deinit_dma_heap();
    
    cout << "Platform cleanup completed" << endl;
}


const uint8_t* platformGetInputDmaBufferPtr(GLContext& ctx) {
    if (ctx.dma_input_y.fd >= 0 && ctx.dma_input_y.ptr != nullptr) {
        return static_cast<const uint8_t*>(ctx.dma_input_y.ptr);
    }
    return nullptr;
}


size_t platformGetInputDmaStride(GLContext& ctx) {
    if (ctx.dma_input_y.fd >= 0) {
        return ctx.dma_input_y.stride;
    }
    return 0;
}


int platformGetInputDmaWidth(GLContext& ctx) {
    if (ctx.dma_input_y.fd >= 0) {
        return ctx.dma_input_y.width;
    }
    return 0;
}

int platformGetInputDmaHeight(GLContext& ctx) {
    if (ctx.dma_input_y.fd >= 0) {
        return ctx.dma_input_y.height;
    }
    return 0;
}


void platformNotifyDmaReady(GLContext& ctx) {

    if (ctx.dma_input_y.fd >= 0) {
        struct dma_buf_sync sync;
        sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE;
        ioctl(ctx.dma_input_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
    }
    if (ctx.dma_input_uv.fd >= 0) {
        struct dma_buf_sync sync;
        sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE;
        ioctl(ctx.dma_input_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);
    }
}

