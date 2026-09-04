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


// BEV 输出多缓冲槽位数：OpenGL 每帧渲染进不同槽位，推流异步消费，
// 避免编码器还在读上一帧时被下一帧渲染覆盖。
#define BEV_OUTPUT_POOL_SIZE 3

// 一个输出槽位：单块 NV12 dmabuf（Y+UV 连续，mpp 单 buffer 布局）。
// 用同一 fd 的两个 offset 各建一个 EGLImage（Y=R8@offset0，UV=GR88@offset y_size），
// OpenGL 分别渲染进这两块 FBO，但数据落在同一块 dmabuf，推流只需一个 fd。
struct GLOutputSlot {
    dma_buffer_t dma;          // 单块 NV12 dmabuf（Y+UV 连续）
    EGLImageKHR egl_image_y;   // 指向 dma offset 0（Y 平面，R8）
    EGLImageKHR egl_image_uv;  // 指向 dma offset y_size（UV 平面，GR88）
    GLuint texture_y;
    GLuint texture_uv;
    GLuint framebuffer_y;
    GLuint framebuffer_uv;
    bool uv_rg88;
    int stride_y;              // Y 平面行宽（字节）
    int stride_uv;             // UV 平面行宽（字节）
    size_t y_size;             // Y 平面总字节数（= UV 在 buffer 内的偏移）
};


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

    // 零拷贝推流输出池：轮转渲染，供 RTSP 异步消费。
    GLOutputSlot output_pool[BEV_OUTPUT_POOL_SIZE];
    int output_pool_count;   // 实际成功初始化的槽位数
    int output_pool_index;   // 下一帧要渲染进的槽位
    

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
    bool input_uv_prefers_rg88;
    bool input_uses_external_dmabuf;
    

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

bool uploadNV12DmabufTextures(GLContext& ctx,
                              int nv12_fd,
                              int width,
                              int height,
                              int stride,
                              size_t size);



const uint8_t* getDmaInputBufferPtr(GLContext& ctx);


void syncDmaInputBuffers(GLContext& ctx);








bool uploadNV12Textures(GLContext& ctx, const std::vector<uint8_t>& nv12_data, int width, int height, bool create_new = true, const uint8_t* nv12_data_ptr = nullptr);


bool performRemap(GLContext& ctx, int input_width, int input_height, int output_width, int output_height);


bool initFramebuffers(GLContext& ctx, int output_width, int output_height);


// 零拷贝输出池：初始化 / 渲染到当前槽位 / 清理。
bool initOutputPool(GLContext& ctx, int output_width, int output_height);
bool performRemapPooled(GLContext& ctx, int input_width, int input_height, int output_width, int output_height);
void cleanupOutputPool(GLContext& ctx);


bool syncGPU(GLContext& ctx, int width, int height);



Mat readOutputBuffer(GLContext& ctx, int width, int height);


void cleanupGLContext(GLContext& ctx);

#endif // GLES_CONTEXT_H

