#ifndef GLES_CONTEXT_H
#define GLES_CONTEXT_H

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <GLES3/gl3.h>
#include <gbm.h>
#include "dma_buffer.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdint>

using namespace cv;


typedef EGLDisplay (EGLAPIENTRY *PFNEGLGETPLATFORMDISPLAYEXTPROC)(EGLenum platform, void *native_display, const EGLint *attrib_list);
typedef EGLImageKHR (EGLAPIENTRY *PFNEGLCREATEIMAGEKHRPROC)(EGLDisplay dpy, EGLContext ctx, EGLenum target, EGLClientBuffer buffer, const EGLint *attrib_list);
typedef EGLBoolean (EGLAPIENTRY *PFNEGLDESTROYIMAGEKHRPROC)(EGLDisplay dpy, EGLImageKHR image);
typedef void (GL_APIENTRY *PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)(GLenum target, GLeglImageOES image);


struct GLContext {
    EGLDisplay display;
    EGLContext context;
    EGLConfig config;
    struct gbm_device *gbm_device;
    struct gbm_surface *gbm_surface;
    EGLSurface egl_surface;
    int drm_fd;
    
    GLuint input_texture_y;
    GLuint input_texture_uv;
    GLuint map_texture_xy;
    

    GLuint output_texture_y;
    GLuint output_texture_uv;
    GLuint framebuffer_y;
    GLuint framebuffer_uv;
    dma_buffer_t dma_output_y;
    dma_buffer_t dma_output_uv;
    EGLImageKHR egl_image_output_y;
    EGLImageKHR egl_image_output_uv;
    

    GLuint pbo_y[2];
    GLuint pbo_uv[2];
    int current_pbo_index;
    void* pbo_y_mapped[2];
    void* pbo_uv_mapped[2];
    bool pbo_initialized;
    bool pbo_primed;
    

    GLuint vao;
    bool vao_initialized;
    

    GLuint output_texture;
    GLuint framebuffer;
    dma_buffer_t dma_output;
    EGLImageKHR egl_image_output;
    
    dma_buffer_t dma_input_y;
    dma_buffer_t dma_input_uv;
    EGLImageKHR egl_image_input_y;
    EGLImageKHR egl_image_input_uv;
    bool uv_uses_rg88_format;
    

    GLuint shader_program_y;
    GLuint shader_program_uv;
    GLint attr_position;
    GLint attr_texcoord;
    GLint uniform_input_y;
    GLint uniform_input_uv;
    GLint uniform_map_xy;
    GLint uniform_map_xy_uv;
    

    GLuint shader_program;
    

    GLuint sync_query_y;
    GLuint sync_query_uv;
    bool sync_query_supported;
    

    double egl_init_time = 0;
    double map_load_time = 0;
    double texture_upload_time = 0;
    double remap_render_time = 0;
    double buffer_export_time = 0;
};


bool initGLContext(GLContext& ctx, int width, int height);


bool createShaderProgram(GLContext& ctx);


bool loadMapTextures(GLContext& ctx, const Mat& mapX, const Mat& mapY);



bool initDmaInputBuffers(GLContext& ctx, int width, int height);



bool writeNV12ToDmaBuffers(GLContext& ctx, const std::vector<uint8_t>& nv12_data, int width, int height);



bool writeNV12ToDmaBuffersPtr(GLContext& ctx, const uint8_t* nv12_data_ptr, int width, int height);



const uint8_t* getDmaInputBufferPtr(GLContext& ctx);


void syncDmaInputBuffers(GLContext& ctx);








bool uploadNV12Textures(GLContext& ctx, const std::vector<uint8_t>& nv12_data, int width, int height, bool create_new = true, const uint8_t* nv12_data_ptr = nullptr);


bool performRemap(GLContext& ctx, int input_width, int input_height, int output_width, int output_height);


bool initFramebuffers(GLContext& ctx, int output_width, int output_height);


bool syncGPU(GLContext& ctx, int width, int height);



Mat readOutputBuffer(GLContext& ctx, int width, int height);


void cleanupGLContext(GLContext& ctx);

#endif // GLES_CONTEXT_H

