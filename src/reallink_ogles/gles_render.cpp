#include "gles_context.h"
#include <iostream>
#include <chrono>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstring>
#include <algorithm>
#include <cstdio>
#include <GLES3/gl3.h>

using namespace std;
using namespace cv;


static bool initVAO(GLContext& ctx) {
    if (ctx.vao_initialized) {
        return true;
    }
    

    const char* gl_version = (const char*)glGetString(GL_VERSION);
    if (!gl_version || strncmp(gl_version, "OpenGL ES 3", 11) != 0) {
        return false;
    }
    

    glGenVertexArrays(1, &ctx.vao);
    if (ctx.vao == 0) {
        return false;
    }
    
    glBindVertexArray(ctx.vao);
    

    static const GLfloat vertexVertices[] = {
        -1.0f, -1.0f,   1.0f, -1.0f,   -1.0f,  1.0f,   1.0f,  1.0f
    };
    static const GLfloat textureVertices[] = {
        0.0f,  0.0f,   1.0f,  0.0f,    0.0f,  1.0f,   1.0f,  1.0f
    };
    

    GLuint vbo[2];
    glGenBuffers(2, vbo);
    

    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertexVertices), vertexVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(ctx.attr_position);
    glVertexAttribPointer(ctx.attr_position, 2, GL_FLOAT, GL_FALSE, 0, 0);
    

    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(textureVertices), textureVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(ctx.attr_texcoord);
    glVertexAttribPointer(ctx.attr_texcoord, 2, GL_FLOAT, GL_FALSE, 0, 0);
    
    glBindVertexArray(0);
    ctx.vao_initialized = true;
    
    return true;
}


static bool initSyncQueries(GLContext& ctx) {
    if (ctx.sync_query_supported) {
        return true;
    }
    

    const char* gl_version = (const char*)glGetString(GL_VERSION);
    if (!gl_version || strncmp(gl_version, "OpenGL ES 3", 11) != 0) {
        ctx.sync_query_supported = false;
        return false;
    }
    

    glGenQueries(1, &ctx.sync_query_y);
    glGenQueries(1, &ctx.sync_query_uv);
    
    if (ctx.sync_query_y == 0 || ctx.sync_query_uv == 0) {
        ctx.sync_query_supported = false;
        return false;
    }
    
    ctx.sync_query_supported = true;
    return true;
}


static bool performRemapInto(GLContext& ctx, GLuint framebuffer_y, GLuint framebuffer_uv,
                             int output_width, int output_height);

bool performRemap(GLContext& ctx, int /*input_width*/, int /*input_height*/, int output_width, int output_height) {

    if (ctx.framebuffer_y == 0 || ctx.framebuffer_uv == 0) {
        cerr << "Error: Framebuffers not initialized" << endl;
        return false;
    }
    if (ctx.shader_program_y == 0 || ctx.shader_program_uv == 0) {
        cerr << "Error: Shader programs not initialized" << endl;
        return false;
    }

    return performRemapInto(ctx, ctx.framebuffer_y, ctx.framebuffer_uv,
                            output_width, output_height);
}

// 渲染核心：把输入纹理重映射进指定的 Y/UV 两个 FBO（零拷贝输出目标）。
static bool performRemapInto(GLContext& ctx, GLuint framebuffer_y, GLuint framebuffer_uv,
                             int output_width, int output_height) {

    if (framebuffer_y == 0 || framebuffer_uv == 0) {
        cerr << "Error: Framebuffers not initialized" << endl;
        return false;
    }
    if (ctx.shader_program_y == 0 || ctx.shader_program_uv == 0) {
        cerr << "Error: Shader programs not initialized" << endl;
        return false;
    }

    bool use_vao = initVAO(ctx);


    int uv_output_width = output_width / 2;
    int uv_output_height = output_height / 2;


    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, ctx.input_texture_y);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, ctx.input_texture_uv);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, ctx.map_texture_xy);


    auto start = chrono::high_resolution_clock::now();


    if (use_vao) {
        glBindVertexArray(ctx.vao);
    }


    if (!use_vao) {
        static const GLfloat vertexVertices[] = {
            -1.0f, -1.0f,   1.0f, -1.0f,   -1.0f,  1.0f,   1.0f,  1.0f
        };
        static const GLfloat textureVertices[] = {
            0.0f,  0.0f,   1.0f,  0.0f,    0.0f,  1.0f,   1.0f,  1.0f
        };
        glEnableVertexAttribArray(ctx.attr_position);
        glVertexAttribPointer(ctx.attr_position, 2, GL_FLOAT, GL_FALSE, 0, vertexVertices);
        glEnableVertexAttribArray(ctx.attr_texcoord);
        glVertexAttribPointer(ctx.attr_texcoord, 2, GL_FLOAT, GL_FALSE, 0, textureVertices);
    }


    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_y);
    glViewport(0, 0, output_width, output_height);


    glUseProgram(ctx.shader_program_y);

    glUniform1i(ctx.uniform_input_y, 0);
    glUniform1i(ctx.uniform_map_xy, 2);

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);



    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_uv);
    glViewport(0, 0, uv_output_width, uv_output_height);


    glUseProgram(ctx.shader_program_uv);

    glUniform1i(ctx.uniform_input_uv, 1);
    glUniform1i(ctx.uniform_map_xy_uv, 2);


    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    if (use_vao) {
        glBindVertexArray(0);
    }


    glFlush();


    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::nanoseconds>(end - start);
    ctx.remap_render_time = duration.count() / 1000000.0;

    return true;
}


bool initFramebuffers(GLContext& ctx, int output_width, int output_height) {

    const char* gl_version = (const char*)glGetString(GL_VERSION);
    bool es3 = (gl_version && strncmp(gl_version, "OpenGL ES 3", 11) == 0);


    {
        PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR =
            (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
        PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES =
            (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");

        if (eglCreateImageKHR && glEGLImageTargetTexture2DOES) {

            if (ctx.dma_output_y.fd < 0) {
                alloc_dma_buffer(output_width, output_height, 1, &ctx.dma_output_y);
            }
            int uv_w_try = output_width / 2;
            int uv_h_try = output_height / 2;
            if (ctx.dma_output_uv.fd < 0) {
                alloc_dma_buffer(uv_w_try, uv_h_try, 2, &ctx.dma_output_uv);
            }

            bool y_ok = false;
            bool uv_ok = false;

            if (ctx.dma_output_y.fd >= 0) {

                const EGLint drm_r8 = 0x20203852;
                EGLint attribs_y[] = {
                    EGL_WIDTH, output_width,
                    EGL_HEIGHT, output_height,
                    EGL_LINUX_DRM_FOURCC_EXT, drm_r8,
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
                            y_ok = true;
                        }
                    }
                }
            }

            if (ctx.dma_output_uv.fd >= 0) {

                uint32_t gr88 = (('G' << 0) | ('R' << 8) | ('8' << 16) | ('8' << 24));
                EGLint attribs_uv[] = {
                    EGL_WIDTH, uv_w_try,
                    EGL_HEIGHT, uv_h_try,
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
                    attribs_uv[5] = (EGLint)rg88;
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
                    if (glGetError() == GL_NO_ERROR) {
                        glGenFramebuffers(1, &ctx.framebuffer_uv);
                        glBindFramebuffer(GL_FRAMEBUFFER, ctx.framebuffer_uv);
                        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                               GL_TEXTURE_2D, ctx.output_texture_uv, 0);
                        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE) {
                            uv_ok = true;
                        }
                    }
                }
            }

            glBindTexture(GL_TEXTURE_2D, 0);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);

            if (y_ok && uv_ok) {
                return true;
            } else {

                if (ctx.framebuffer_y) { glDeleteFramebuffers(1, &ctx.framebuffer_y); ctx.framebuffer_y = 0; }
                if (ctx.output_texture_y) { glDeleteTextures(1, &ctx.output_texture_y); ctx.output_texture_y = 0; }
                if (ctx.egl_image_output_y != EGL_NO_IMAGE_KHR) { ctx.egl_image_output_y = EGL_NO_IMAGE_KHR; }
                if (ctx.framebuffer_uv) { glDeleteFramebuffers(1, &ctx.framebuffer_uv); ctx.framebuffer_uv = 0; }
                if (ctx.output_texture_uv) { glDeleteTextures(1, &ctx.output_texture_uv); ctx.output_texture_uv = 0; }
                if (ctx.egl_image_output_uv != EGL_NO_IMAGE_KHR) { ctx.egl_image_output_uv = EGL_NO_IMAGE_KHR; }
            }
        }
    }


    glGenTextures(1, &ctx.output_texture_y);
    glBindTexture(GL_TEXTURE_2D, ctx.output_texture_y);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    if (es3) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, output_width, output_height, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);
    } else {

        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, output_width, output_height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, nullptr);
    }


    int uv_w = output_width / 2;
    int uv_h = output_height / 2;
    glGenTextures(1, &ctx.output_texture_uv);
    glBindTexture(GL_TEXTURE_2D, ctx.output_texture_uv);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    if (es3) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, uv_w, uv_h, 0, GL_RG, GL_UNSIGNED_BYTE, nullptr);
    } else {

        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA, uv_w, uv_h, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, nullptr);
    }

    glBindTexture(GL_TEXTURE_2D, 0);


    glGenFramebuffers(1, &ctx.framebuffer_y);
    glBindFramebuffer(GL_FRAMEBUFFER, ctx.framebuffer_y);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, ctx.output_texture_y, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        cerr << "Error: Y framebuffer incomplete" << endl;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }


    glGenFramebuffers(1, &ctx.framebuffer_uv);
    glBindFramebuffer(GL_FRAMEBUFFER, ctx.framebuffer_uv);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, ctx.output_texture_uv, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        cerr << "Error: UV framebuffer incomplete" << endl;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return true;
}


// 初始化一个输出池槽位：分配单块 NV12 dmabuf（Y+UV 连续），对同一 fd 的两个 offset
// 各建一个 EGLImage 并绑成 FBO，使 OpenGL 渲染的 Y/UV 落在同一块 dmabuf 里。
static bool initOutputSlot(GLContext& ctx, int idx, int output_width, int output_height) {
    GLOutputSlot& s = ctx.output_pool[idx];
    memset(&s, 0, sizeof(s));
    s.dma.fd = -1;
    s.egl_image_y = EGL_NO_IMAGE_KHR;
    s.egl_image_uv = EGL_NO_IMAGE_KHR;
    s.uv_rg88 = false;

    PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR =
        (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES =
        (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
    if (!eglCreateImageKHR || !glEGLImageTargetTexture2DOES) {
        return false;
    }

    int uv_w = output_width / 2;
    int uv_h = output_height / 2;
    s.stride_y = output_width;                    // R8，1 字节/像素
    s.stride_uv = uv_w * 2;                       // GR88，2 字节/像素
    s.y_size = (size_t)s.stride_y * (size_t)output_height;
    size_t uv_size = (size_t)s.stride_uv * (size_t)uv_h;
    size_t total_size = s.y_size + uv_size;

    // 单块 dmabuf：Y 在前、UV 紧跟（NV12，mpp 单 buffer 布局）。
    s.dma.fd = dmabuf_alloc(total_size);
    if (s.dma.fd < 0) {
        return false;
    }
    s.dma.ptr = dmabuf_mmap(s.dma.fd, total_size);
    if (!s.dma.ptr) {
        dmabuf_free(s.dma.fd);
        s.dma.fd = -1;
        return false;
    }
    s.dma.size = total_size;
    s.dma.width = output_width;
    s.dma.height = output_height;
    s.dma.stride = s.stride_y;

    bool y_ok = false;
    const EGLint drm_r8 = 0x20203852;
    EGLint attribs_y[] = {
        EGL_WIDTH, output_width,
        EGL_HEIGHT, output_height,
        EGL_LINUX_DRM_FOURCC_EXT, drm_r8,
        EGL_DMA_BUF_PLANE0_FD_EXT, s.dma.fd,
        EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
        EGL_DMA_BUF_PLANE0_PITCH_EXT, s.stride_y,
        EGL_NONE
    };
    s.egl_image_y = eglCreateImageKHR(ctx.display, EGL_NO_CONTEXT,
                                      EGL_LINUX_DMA_BUF_EXT, (EGLClientBuffer)NULL, attribs_y);
    if (s.egl_image_y != EGL_NO_IMAGE_KHR) {
        glGenTextures(1, &s.texture_y);
        glBindTexture(GL_TEXTURE_2D, s.texture_y);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, s.egl_image_y);
        if (glGetError() == GL_NO_ERROR) {
            glGenFramebuffers(1, &s.framebuffer_y);
            glBindFramebuffer(GL_FRAMEBUFFER, s.framebuffer_y);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                   GL_TEXTURE_2D, s.texture_y, 0);
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE) {
                y_ok = true;
            }
        }
    }

    bool uv_ok = false;
    uint32_t gr88 = (('G' << 0) | ('R' << 8) | ('8' << 16) | ('8' << 24));
    EGLint attribs_uv[] = {
        EGL_WIDTH, uv_w,
        EGL_HEIGHT, uv_h,
        EGL_LINUX_DRM_FOURCC_EXT, (EGLint)gr88,
        EGL_DMA_BUF_PLANE0_FD_EXT, s.dma.fd,
        EGL_DMA_BUF_PLANE0_OFFSET_EXT, (EGLint)s.y_size,
        EGL_DMA_BUF_PLANE0_PITCH_EXT, s.stride_uv,
        EGL_NONE
    };
    s.egl_image_uv = eglCreateImageKHR(ctx.display, EGL_NO_CONTEXT,
                                       EGL_LINUX_DMA_BUF_EXT, (EGLClientBuffer)NULL, attribs_uv);
    if (s.egl_image_uv == EGL_NO_IMAGE_KHR) {
        uint32_t rg88 = (('R' << 0) | ('G' << 8) | ('8' << 16) | ('8' << 24));
        attribs_uv[5] = (EGLint)rg88;
        s.egl_image_uv = eglCreateImageKHR(ctx.display, EGL_NO_CONTEXT,
                                           EGL_LINUX_DMA_BUF_EXT, (EGLClientBuffer)NULL, attribs_uv);
        if (s.egl_image_uv != EGL_NO_IMAGE_KHR) {
            s.uv_rg88 = true;
        }
    }
    if (s.egl_image_uv != EGL_NO_IMAGE_KHR) {
        glGenTextures(1, &s.texture_uv);
        glBindTexture(GL_TEXTURE_2D, s.texture_uv);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, s.egl_image_uv);
        if (glGetError() == GL_NO_ERROR) {
            glGenFramebuffers(1, &s.framebuffer_uv);
            glBindFramebuffer(GL_FRAMEBUFFER, s.framebuffer_uv);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                   GL_TEXTURE_2D, s.texture_uv, 0);
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE) {
                uv_ok = true;
            }
        }
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    if (y_ok && uv_ok) {
        return true;
    }

    // 部分成功则回收本槽位已创建资源，避免泄漏。
    if (s.framebuffer_y) { glDeleteFramebuffers(1, &s.framebuffer_y); s.framebuffer_y = 0; }
    if (s.framebuffer_uv) { glDeleteFramebuffers(1, &s.framebuffer_uv); s.framebuffer_uv = 0; }
    if (s.texture_y) { glDeleteTextures(1, &s.texture_y); s.texture_y = 0; }
    if (s.texture_uv) { glDeleteTextures(1, &s.texture_uv); s.texture_uv = 0; }
    if (s.egl_image_y != EGL_NO_IMAGE_KHR) { s.egl_image_y = EGL_NO_IMAGE_KHR; }
    if (s.egl_image_uv != EGL_NO_IMAGE_KHR) { s.egl_image_uv = EGL_NO_IMAGE_KHR; }
    free_dma_buffer(&s.dma);
    return false;
}

bool initOutputPool(GLContext& ctx, int output_width, int output_height) {
    ctx.output_pool_count = 0;
    ctx.output_pool_index = 0;
    for (int i = 0; i < BEV_OUTPUT_POOL_SIZE; ++i) {
        if (!initOutputSlot(ctx, i, output_width, output_height)) {
            break;
        }
        ctx.output_pool_count++;
    }
    if (ctx.output_pool_count == 0) {
        cerr << "[BEV] ERROR: Failed to init any output pool slot" << endl;
        return false;
    }
    return true;
}

bool performRemapPooled(GLContext& ctx, int /*input_width*/, int /*input_height*/,
                        int output_width, int output_height) {
    if (ctx.output_pool_count == 0) {
        cerr << "Error: Output pool not initialized" << endl;
        return false;
    }
    int idx = ctx.output_pool_index;
    GLOutputSlot& s = ctx.output_pool[idx];
    if (!performRemapInto(ctx, s.framebuffer_y, s.framebuffer_uv,
                          output_width, output_height)) {
        return false;
    }
    ctx.output_pool_index = (idx + 1) % ctx.output_pool_count;
    return true;
}

void cleanupOutputPool(GLContext& ctx) {
    PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR =
        (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
    for (int i = 0; i < BEV_OUTPUT_POOL_SIZE; ++i) {
        GLOutputSlot& s = ctx.output_pool[i];
        if (eglDestroyImageKHR) {
            if (s.egl_image_y != EGL_NO_IMAGE_KHR) {
                eglDestroyImageKHR(ctx.display, s.egl_image_y);
                s.egl_image_y = EGL_NO_IMAGE_KHR;
            }
            if (s.egl_image_uv != EGL_NO_IMAGE_KHR) {
                eglDestroyImageKHR(ctx.display, s.egl_image_uv);
                s.egl_image_uv = EGL_NO_IMAGE_KHR;
            }
        }
        if (s.framebuffer_y) { glDeleteFramebuffers(1, &s.framebuffer_y); s.framebuffer_y = 0; }
        if (s.framebuffer_uv) { glDeleteFramebuffers(1, &s.framebuffer_uv); s.framebuffer_uv = 0; }
        if (s.texture_y) { glDeleteTextures(1, &s.texture_y); s.texture_y = 0; }
        if (s.texture_uv) { glDeleteTextures(1, &s.texture_uv); s.texture_uv = 0; }
        if (s.dma.fd >= 0) { free_dma_buffer(&s.dma); }
    }
    ctx.output_pool_count = 0;
    ctx.output_pool_index = 0;
}


static bool exportDMABufferFromDMA(GLContext& ctx, int /*width*/, int /*height*/) {


    struct dma_buf_sync sync;
    


    if (ctx.dma_output_y.fd >= 0) {
        sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
        ioctl(ctx.dma_output_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
    }
    
    if (ctx.dma_output_uv.fd >= 0) {
        sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
        ioctl(ctx.dma_output_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);
    }
    



    glFlush();
    glFinish();
    

    if (ctx.dma_output_y.fd >= 0) {
        sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
        ioctl(ctx.dma_output_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
    }
    
    if (ctx.dma_output_uv.fd >= 0) {
        sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
        ioctl(ctx.dma_output_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);
    }
    

    if (ctx.dma_output.fd >= 0) {
        sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
        ioctl(ctx.dma_output.fd, DMA_BUF_IOCTL_SYNC, &sync);
        sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
        ioctl(ctx.dma_output.fd, DMA_BUF_IOCTL_SYNC, &sync);
    }
    
    return true;
}


static bool exportDMABufferFallback(GLContext& /*ctx*/, int /*width*/, int /*height*/) {


    glFinish();
    return true;
}


bool syncGPU(GLContext& ctx, int width, int height) {
    if ((ctx.dma_output_y.fd >= 0 && ctx.dma_output_y.ptr) ||
        (ctx.dma_output_uv.fd >= 0 && ctx.dma_output_uv.ptr)) {

        return exportDMABufferFromDMA(ctx, width, height);
    } else if (ctx.dma_output.fd >= 0 && ctx.dma_output.ptr) {

        return exportDMABufferFromDMA(ctx, width, height);
    } else {

        return exportDMABufferFallback(ctx, width, height);
    }
}


static bool initPBO(GLContext& ctx, int width, int height) {
    if (ctx.pbo_initialized) {
        static std::chrono::steady_clock::time_point last_log = std::chrono::steady_clock::now() - std::chrono::seconds(5);
        auto now = std::chrono::steady_clock::now();
        if (now - last_log >= std::chrono::seconds(5)) {
            fprintf(stderr, "  [DEBUG] PBO already initialized\n");
            fflush(stderr);
            last_log = now;
        }
        return true;
    }
    

    const char* gl_version = (const char*)glGetString(GL_VERSION);
    if (!gl_version || strncmp(gl_version, "OpenGL ES 3", 11) != 0) {
        fprintf(stderr, "  [DEBUG] PBO not supported (OpenGL ES < 3.0)\n");
        fflush(stderr);
        cout << "  PBO not supported (OpenGL ES < 3.0)" << endl;
        return false;
    }
    
    int uv_width = width / 2;
    int uv_height = height / 2;
    size_t y_size = width * height;
    

    size_t uv_size = uv_width * uv_height * 2;
    

    glGenBuffers(2, ctx.pbo_y);
    for (int i = 0; i < 2; i++) {
        glBindBuffer(GL_PIXEL_PACK_BUFFER, ctx.pbo_y[i]);
        glBufferData(GL_PIXEL_PACK_BUFFER, y_size, nullptr, GL_STREAM_READ);
        ctx.pbo_y_mapped[i] = nullptr;
    }
    

    glGenBuffers(2, ctx.pbo_uv);
    for (int i = 0; i < 2; i++) {
        glBindBuffer(GL_PIXEL_PACK_BUFFER, ctx.pbo_uv[i]);
        glBufferData(GL_PIXEL_PACK_BUFFER, uv_size, nullptr, GL_STREAM_READ);
        ctx.pbo_uv_mapped[i] = nullptr;
    }
    
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    ctx.current_pbo_index = 0;
    ctx.pbo_initialized = true;
    ctx.pbo_primed = false;
    
    cout << "  ✓ PBO initialized for zero-copy read (Y: " << y_size << " bytes, UV: " << uv_size << " bytes)" << endl;
    return true;
}




Mat readOutputBuffer(GLContext& ctx, int width, int height) {
    Mat output_nv12;
    auto rb_start = chrono::high_resolution_clock::now();
    

    if ((ctx.dma_output_y.fd >= 0 && ctx.dma_output_y.ptr) &&
        (ctx.dma_output_uv.fd >= 0 && ctx.dma_output_uv.ptr)) {

        glFlush();
        glFinish();
        

        struct dma_buf_sync sync;
        sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
        ioctl(ctx.dma_output_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
        ioctl(ctx.dma_output_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);


        output_nv12 = Mat(height * 3 / 2, width, CV_8UC1);
        const uint8_t* y_data = (const uint8_t*)ctx.dma_output_y.ptr;
        const uint8_t* uv_data = (const uint8_t*)ctx.dma_output_uv.ptr;
        

        for (int y = 0; y < height; y++) {
            const uint8_t* y_row = y_data + y * ctx.dma_output_y.stride;
            uint8_t* nv12_row = output_nv12.ptr(y);
            memcpy(nv12_row, y_row, width);
        }
        

        int uv_width = width / 2;
        int uv_height = height / 2;
        for (int y = 0; y < uv_height; y++) {
            const uint8_t* uv_row = uv_data + y * ctx.dma_output_uv.stride;
            uint8_t* nv12_uv_row = output_nv12.ptr(height + y);
            memcpy(nv12_uv_row, uv_row, uv_width * 2);
        }
        

        sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
        ioctl(ctx.dma_output_y.fd, DMA_BUF_IOCTL_SYNC, &sync);
        ioctl(ctx.dma_output_uv.fd, DMA_BUF_IOCTL_SYNC, &sync);
        
        auto rb_end = chrono::high_resolution_clock::now();
        auto rb_dur = chrono::duration_cast<chrono::microseconds>(rb_end - rb_start);
        ctx.buffer_export_time = rb_dur.count() / 1000.0; // ms
        return output_nv12;
    }


    bool pbo_init = initPBO(ctx, width, height);
    if (pbo_init) {
        int uv_width = width / 2;
        int uv_height = height / 2;
        


        int write_idx = ctx.current_pbo_index;

        int next_idx = 1 - ctx.current_pbo_index;
        

        if (ctx.pbo_y_mapped[write_idx]) {
            glBindBuffer(GL_PIXEL_PACK_BUFFER, ctx.pbo_y[write_idx]);
            glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
            ctx.pbo_y_mapped[write_idx] = nullptr;
        }
        if (ctx.pbo_uv_mapped[write_idx]) {
            glBindBuffer(GL_PIXEL_PACK_BUFFER, ctx.pbo_uv[write_idx]);
            glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
            ctx.pbo_uv_mapped[write_idx] = nullptr;
        }
        

        glPixelStorei(GL_PACK_ALIGNMENT, 1);



        glBindFramebuffer(GL_FRAMEBUFFER, ctx.framebuffer_y);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, ctx.pbo_y[write_idx]);
        glReadPixels(0, 0, width, height, GL_RED, GL_UNSIGNED_BYTE, 0);
        


        glBindFramebuffer(GL_FRAMEBUFFER, ctx.framebuffer_uv);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, ctx.pbo_uv[write_idx]);
        glReadPixels(0, 0, uv_width, uv_height, GL_RG, GL_UNSIGNED_BYTE, 0);
        

        glFlush();
        

        int read_idx = next_idx;
        const uint8_t* y_data = nullptr;
        const uint8_t* uv_data = nullptr;
        size_t uv_pbo_size = uv_width * uv_height * 2;

        if (!ctx.pbo_primed) {

            glFinish();

            glBindBuffer(GL_PIXEL_PACK_BUFFER, ctx.pbo_y[write_idx]);
            void* y_ptr_now = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, width * height, GL_MAP_READ_BIT);
            if (y_ptr_now) ctx.pbo_y_mapped[write_idx] = y_ptr_now;
            glBindBuffer(GL_PIXEL_PACK_BUFFER, ctx.pbo_uv[write_idx]);
            void* uv_ptr_now = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, uv_pbo_size, GL_MAP_READ_BIT);
            if (uv_ptr_now) ctx.pbo_uv_mapped[write_idx] = uv_ptr_now;
            glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
            y_data = static_cast<const uint8_t*>(ctx.pbo_y_mapped[write_idx]);
            uv_data = static_cast<const uint8_t*>(ctx.pbo_uv_mapped[write_idx]);
            ctx.pbo_primed = true;
        } else {

            if (!ctx.pbo_y_mapped[read_idx]) {
                glBindBuffer(GL_PIXEL_PACK_BUFFER, ctx.pbo_y[read_idx]);
                void* y_ptr_prev = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, width * height, GL_MAP_READ_BIT);
                if (y_ptr_prev) ctx.pbo_y_mapped[read_idx] = y_ptr_prev;
            }
            if (!ctx.pbo_uv_mapped[read_idx]) {
                glBindBuffer(GL_PIXEL_PACK_BUFFER, ctx.pbo_uv[read_idx]);
                void* uv_ptr_prev = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, uv_pbo_size, GL_MAP_READ_BIT);
                if (uv_ptr_prev) ctx.pbo_uv_mapped[read_idx] = uv_ptr_prev;
            }
            glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
            y_data = static_cast<const uint8_t*>(ctx.pbo_y_mapped[read_idx]);
            uv_data = static_cast<const uint8_t*>(ctx.pbo_uv_mapped[read_idx]);
        }


        ctx.current_pbo_index = next_idx;
        

        if (y_data && uv_data) {


            output_nv12 = Mat(height * 3 / 2, width, CV_8UC1);
            

            uint8_t* nv12_y = output_nv12.data;
            memcpy(nv12_y, y_data, width * height);
            


            uint8_t* nv12_uv = output_nv12.data + width * height;
            

            for (int y = 0; y < uv_height; y++) {
                memcpy(nv12_uv + y * width, uv_data + y * uv_width * 2, uv_width * 2);
            }
            


            {
                auto rb_end = chrono::high_resolution_clock::now();
                auto rb_dur = chrono::duration_cast<chrono::microseconds>(rb_end - rb_start);
                ctx.buffer_export_time = rb_dur.count() / 1000.0; // ms
                return output_nv12;
            }
        } else {
            fprintf(stderr, "  [ERROR] Failed to map PBO buffers! Y=%p, UV=%p\n",
                    (void*)y_data, (void*)uv_data);
            fflush(stderr);
        }
    }
    

    if (ctx.dma_output.fd >= 0 && ctx.dma_output.ptr) {

        struct dma_buf_sync sync;
        sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
        ioctl(ctx.dma_output.fd, DMA_BUF_IOCTL_SYNC, &sync);
        
        const uint8_t* rgba_data = (const uint8_t*)ctx.dma_output.ptr;
        Mat rgba_mat;
        if (ctx.dma_output.stride == width * 4) {
            rgba_mat = Mat(height, width, CV_8UC4, (void*)rgba_data).clone();
        } else {
            rgba_mat = Mat(height, width, CV_8UC4);
            for (int y = 0; y < height; y++) {
                memcpy(rgba_mat.ptr(y), rgba_data + y * ctx.dma_output.stride, width * 4);
            }
        }
        
        sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
        ioctl(ctx.dma_output.fd, DMA_BUF_IOCTL_SYNC, &sync);
        

        Mat bgr_mat;
        cvtColor(rgba_mat, bgr_mat, COLOR_RGBA2BGR);
        Mat nv12_mat;
        cvtColor(bgr_mat, nv12_mat, COLOR_BGR2YUV_I420);

        output_nv12 = Mat(height * 3 / 2, width, CV_8UC1);
        nv12_mat.rowRange(0, height).copyTo(output_nv12.rowRange(0, height));

        int uv_height = height / 2;
        for (int y = 0; y < uv_height; y++) {
            const uint8_t* u_row = nv12_mat.ptr(height + y);
            const uint8_t* v_row = nv12_mat.ptr(height + uv_height + y);
            uint8_t* uv_row = output_nv12.ptr(height + y);
            for (int x = 0; x < width / 2; x++) {
                uv_row[x * 2] = u_row[x];
                uv_row[x * 2 + 1] = v_row[x];
            }
        }
        return output_nv12;
    } else {

        vector<uint8_t> rgba_data(width * height * 4);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, rgba_data.data());
        Mat rgba_mat = Mat(height, width, CV_8UC4, rgba_data.data()).clone();
        

        Mat flipped;
        flip(rgba_mat, flipped, 0);
        rgba_mat = flipped;
        

        Mat bgr_mat;
        cvtColor(rgba_mat, bgr_mat, COLOR_RGBA2BGR);
        Mat nv12_mat;
        cvtColor(bgr_mat, nv12_mat, COLOR_BGR2YUV_I420);

        output_nv12 = Mat(height * 3 / 2, width, CV_8UC1);
        nv12_mat.rowRange(0, height).copyTo(output_nv12.rowRange(0, height));

        int uv_height = height / 2;
        for (int y = 0; y < uv_height; y++) {
            const uint8_t* u_row = nv12_mat.ptr(height + y);
            const uint8_t* v_row = nv12_mat.ptr(height + uv_height + y);
            uint8_t* uv_row = output_nv12.ptr(height + y);
            for (int x = 0; x < width / 2; x++) {
                uv_row[x * 2] = u_row[x];
                uv_row[x * 2 + 1] = v_row[x];
            }
        }
    }
    
    {
        auto rb_end = chrono::high_resolution_clock::now();
        auto rb_dur = chrono::duration_cast<chrono::microseconds>(rb_end - rb_start);
        ctx.buffer_export_time = rb_dur.count() / 1000.0; // ms
        return output_nv12;
    }
}

