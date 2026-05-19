#define _POSIX_C_SOURCE 199309L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include "v4l2_capture.h"



static bool v4l2_set_fps(int fd, int fps, bool use_mplane) {
    struct v4l2_streamparm sp;
    memset(&sp, 0, sizeof(sp));
    sp.type = use_mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE 
                         : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if (ioctl(fd, VIDIOC_G_PARM, &sp) < 0) {
        return false;
    }
    
    sp.parm.capture.timeperframe.numerator = 1;
    sp.parm.capture.timeperframe.denominator = fps;
    
    return ioctl(fd, VIDIOC_S_PARM, &sp) == 0;
}

uint32_t v4l2_camera_buffer_type(const V4L2Camera* cam) {
    if (cam && cam->use_mplane) {
        return V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    }
    return V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

bool v4l2_camera_fill_dequeue_result(const V4L2Camera* cam, uint32_t buf_index,
                                    int* out_index, void** out_data,
                                    size_t* out_size, int* out_dmabuf_fd) {
    if (!cam || !out_index || !out_data || !out_size) {
        return false;
    }
    if (buf_index >= (uint32_t)cam->buffer_count) {
        fprintf(stderr, "[V4L2] Invalid buffer index from driver: %u (buffers=%d)\n",
                buf_index, cam->buffer_count);
        return false;
    }

    *out_index = (int)buf_index;
    *out_data = cam->buffers[buf_index];
    *out_size = cam->buffer_lengths[buf_index];
    if (out_dmabuf_fd) {
        *out_dmabuf_fd = cam->dmabuf_fds ? cam->dmabuf_fds[buf_index] : -1;
    }
    return true;
}


bool v4l2_camera_open(V4L2Camera* cam, const char* device, 
                     int width, int height, int buffer_count) {
    if (!cam || !device) {
        fprintf(stderr, "[V4L2] Invalid parameters\n");
        return false;
    }
    
    memset(cam, 0, sizeof(*cam));
    cam->fd = -1;
    strncpy(cam->device_path, device, sizeof(cam->device_path) - 1);
    cam->device_path[sizeof(cam->device_path) - 1] = '\0';
    cam->requested_width = width;
    cam->requested_height = height;
    cam->requested_buffer_count = buffer_count;
    
 
    cam->fd = open(device, O_RDWR | O_NONBLOCK);
    if (cam->fd < 0) {
        fprintf(stderr, "[V4L2] Failed to open %s: %s\n", device, strerror(errno));
        return false;
    }
    
    
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    if (ioctl(cam->fd, VIDIOC_QUERYCAP, &cap) < 0) {
        fprintf(stderr, "[V4L2] VIDIOC_QUERYCAP failed: %s\n", strerror(errno));
        close(cam->fd);
        cam->fd = -1;
        return false;
    }
    
    // 
    bool use_mplane = (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0;
    cam->use_mplane = use_mplane;
    
    // 
    enum v4l2_buf_type type = (enum v4l2_buf_type)v4l2_camera_buffer_type(cam);
    ioctl(cam->fd, VIDIOC_STREAMOFF, &type);
    
    // 
    struct v4l2_requestbuffers req_zero;
    memset(&req_zero, 0, sizeof(req_zero));
    req_zero.type = type;
    req_zero.memory = V4L2_MEMORY_MMAP;
    req_zero.count = 0;
    ioctl(cam->fd, VIDIOC_REQBUFS, &req_zero);
    
    // 
    width = (width + 7) & ~7;
    height = (height + 7) & ~7;
    
    // 
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    
    if (use_mplane) {
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.width = width;
        fmt.fmt.pix_mp.height = height;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 2;
        fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_SMPTE170M;
    } else {
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
    }
    
    if (ioctl(cam->fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "[V4L2] VIDIOC_S_FMT failed: %s\n", strerror(errno));
        fprintf(stderr, "[V4L2] Requested: %dx%d NV12 (%s mode)\n", 
                width, height, use_mplane ? "MPLANE" : "PLANAR");
        fprintf(stderr, "[V4L2] Please check supported formats with:\n");
        fprintf(stderr, "[V4L2]   v4l2-ctl --device=%s --list-formats-ext\n", device);
        close(cam->fd);
        cam->fd = -1;
        return false;
    }
    
    // 
    if (use_mplane) {
        cam->width = fmt.fmt.pix_mp.width;
        cam->height = fmt.fmt.pix_mp.height;
        cam->y_stride = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
    } else {
        cam->width = fmt.fmt.pix.width;
        cam->height = fmt.fmt.pix.height;
        cam->y_stride = fmt.fmt.pix.bytesperline;
    }
    
    // 
    v4l2_set_fps(cam->fd, 30, use_mplane);
    
    // 
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = buffer_count;
    req.type = type;
    req.memory = V4L2_MEMORY_MMAP;
    
    if (ioctl(cam->fd, VIDIOC_REQBUFS, &req) < 0) {
        fprintf(stderr, "[V4L2] VIDIOC_REQBUFS failed: %s\n", strerror(errno));
        close(cam->fd);
        cam->fd = -1;
        return false;
    }
    
    if (req.count < 2) {
        fprintf(stderr, "[V4L2] Not enough buffers (got %d)\n", req.count);
        close(cam->fd);
        cam->fd = -1;
        return false;
    }
    
    cam->buffer_count = req.count;
    cam->buffers = (void**)calloc(req.count, sizeof(void*));
    cam->buffer_lengths = (size_t*)calloc(req.count, sizeof(size_t));
    if (!cam->buffers || !cam->buffer_lengths) {
        fprintf(stderr, "[V4L2] Failed to allocate camera buffers metadata\n");
        v4l2_camera_close(cam);
        return false;
    }
    
    // 
    cam->dmabuf_fds = (int*)calloc(req.count, sizeof(int));
    for (uint32_t i = 0; i < req.count; i++) {
        cam->dmabuf_fds[i] = -1;  // 
    }
    
    // 
    for (int i = 0; i < cam->buffer_count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = (uint32_t)i;
        
        struct v4l2_plane planes[1];
        if (use_mplane) {
            memset(planes, 0, sizeof(planes));
            buf.length = 1;
            buf.m.planes = planes;
        }
        
        if (ioctl(cam->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            fprintf(stderr, "[V4L2] VIDIOC_QUERYBUF[%d] failed: %s\n", i, strerror(errno));
            v4l2_camera_close(cam);
            return false;
        }
        
        size_t length;
        off_t offset;
        
        if (use_mplane) {
            length = buf.m.planes[0].length;
            offset = buf.m.planes[0].m.mem_offset;
        } else {
            length = buf.length;
            offset = buf.m.offset;
        }
        
        cam->buffers[i] = mmap(NULL, length, PROT_READ | PROT_WRITE, 
                              MAP_SHARED, cam->fd, offset);
        
        if (cam->buffers[i] == MAP_FAILED) {
            fprintf(stderr, "[V4L2] mmap[%d] failed: %s\n", i, strerror(errno));
            v4l2_camera_close(cam);
            return false;
        }
        
        cam->buffer_lengths[i] = length;
    }
    
    // 
    fprintf(stderr, "[V4L2] Exporting DMABUF file descriptors...\n");
    bool dmabuf_success = true;
    for (int i = 0; i < cam->buffer_count; i++) {
        struct v4l2_exportbuffer expbuf;
        memset(&expbuf, 0, sizeof(expbuf));
        expbuf.type = type;
        expbuf.index = i;
        expbuf.plane = 0;  
        expbuf.flags = O_RDWR;  
        
        if (ioctl(cam->fd, VIDIOC_EXPBUF, &expbuf) < 0) {
            fprintf(stderr, "[V4L2]   EXPBUF failed for buffer %d: %s\n", 
                    i, strerror(errno));
            dmabuf_success = false;
            break;
        }
        
        cam->dmabuf_fds[i] = expbuf.fd;
    }
    
    if (dmabuf_success) {
        fprintf(stderr, "[V4L2]  All %d DMABUF fds exported successfully (zero-copy enabled)\n", 
                cam->buffer_count);
    } else {
        fprintf(stderr, "[V4L2] DMABUF export failed, will use memcpy fallback\n");
    }
    
    fprintf(stderr, "[V4L2] Opened %s: %dx%d NV12, stride=%d, buffers=%d (%s)\n",
            device, cam->width, cam->height, cam->y_stride, cam->buffer_count,
            use_mplane ? "MPLANE" : "PLANAR");
    
    return true;
}

bool v4l2_camera_start(V4L2Camera* cam) {
    if (!cam || cam->fd < 0) {
        return false;
    }
    
    enum v4l2_buf_type type = (enum v4l2_buf_type)v4l2_camera_buffer_type(cam);
    
    
    for (int i = 0; i < cam->buffer_count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        
        struct v4l2_plane planes[1];
        if (cam->use_mplane) {
            memset(planes, 0, sizeof(planes));
            buf.length = 1;
            buf.m.planes = planes;
        }
        
        if (ioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "[V4L2] VIDIOC_QBUF[%d] failed: %s\n", i, strerror(errno));
            return false;
        }
    }
    
    // 
    if (ioctl(cam->fd, VIDIOC_STREAMON, &type) < 0) {
        fprintf(stderr, "[V4L2] VIDIOC_STREAMON failed: %s\n", strerror(errno));
        return false;
    }
    
    fprintf(stderr, "[V4L2] Stream started\n");
    return true;
}

bool v4l2_camera_stop(V4L2Camera* cam) {
    if (!cam || cam->fd < 0) {
        return false;
    }
    
    enum v4l2_buf_type type = (enum v4l2_buf_type)v4l2_camera_buffer_type(cam);
    
    if (ioctl(cam->fd, VIDIOC_STREAMOFF, &type) < 0) {
        fprintf(stderr, "[V4L2] VIDIOC_STREAMOFF failed: %s\n", strerror(errno));
        return false;
    }
    
    fprintf(stderr, "[V4L2] Stream stopped\n");
    return true;
}

bool v4l2_camera_resume(V4L2Camera* cam) {
    if (!cam || cam->fd < 0) {
        return false;
    }
    
    enum v4l2_buf_type type = (enum v4l2_buf_type)v4l2_camera_buffer_type(cam);
    
    // 重新入队所有缓冲区
    for (int i = 0; i < cam->buffer_count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        
        struct v4l2_plane planes[1];
        if (cam->use_mplane) {
            memset(planes, 0, sizeof(planes));
            buf.length = 1;
            buf.m.planes = planes;
        }
        
        if (ioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "[V4L2] VIDIOC_QBUF[%d] failed: %s\n", i, strerror(errno));
            return false;
        }
    }
    
    if (ioctl(cam->fd, VIDIOC_STREAMON, &type) < 0) {
        fprintf(stderr, "[V4L2] VIDIOC_STREAMON failed: %s\n", strerror(errno));
        return false;
    }
    
    fprintf(stderr, "[V4L2] Stream resumed\n");
    return true;
}

bool v4l2_camera_recover(V4L2Camera* cam) {
    if (!cam) {
        return false;
    }

    char device_path[sizeof(cam->device_path)];
    memset(device_path, 0, sizeof(device_path));
    strncpy(device_path, cam->device_path, sizeof(device_path) - 1);
    const int requested_width = cam->requested_width > 0 ? cam->requested_width : cam->width;
    const int requested_height = cam->requested_height > 0 ? cam->requested_height : cam->height;
    const int requested_buffer_count = cam->requested_buffer_count > 0
        ? cam->requested_buffer_count
        : cam->buffer_count;

    fprintf(stderr, "[V4L2] Recovering stream on %s\n",
            device_path[0] ? device_path : "<unknown>");

    if (cam->fd >= 0) {
        enum v4l2_buf_type type = (enum v4l2_buf_type)v4l2_camera_buffer_type(cam);
        if (ioctl(cam->fd, VIDIOC_STREAMOFF, &type) < 0) {
            fprintf(stderr, "[V4L2] Recovery STREAMOFF warning: %s\n", strerror(errno));
        }

        if (v4l2_camera_resume(cam)) {
            fprintf(stderr, "[V4L2] Recovery succeeded via stream restart\n");
            return true;
        }

        fprintf(stderr, "[V4L2] Stream restart failed, attempting full reopen\n");
    }

    if (!device_path[0] ||
        requested_width <= 0 ||
        requested_height <= 0 ||
        requested_buffer_count <= 0) {
        fprintf(stderr, "[V4L2] Recovery failed: missing reopen configuration\n");
        return false;
    }

    v4l2_camera_close(cam);
    if (!v4l2_camera_open(cam, device_path, requested_width,
                          requested_height, requested_buffer_count)) {
        fprintf(stderr, "[V4L2] Recovery reopen failed\n");
        return false;
    }
    if (!v4l2_camera_start(cam)) {
        fprintf(stderr, "[V4L2] Recovery restart after reopen failed\n");
        v4l2_camera_close(cam);
        return false;
    }

    fprintf(stderr, "[V4L2] Recovery succeeded via reopen\n");
    return true;
}

bool v4l2_camera_dequeue(V4L2Camera* cam, int* out_index, 
                        void** out_data, size_t* out_size, int* out_dmabuf_fd) {
    if (!cam || cam->fd < 0 || !out_index || !out_data || !out_size) {
        return false;
    }
    
    // 
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = v4l2_camera_buffer_type(cam);
    buf.memory = V4L2_MEMORY_MMAP;
    
    struct v4l2_plane planes[1];
    if (cam->use_mplane) {
        memset(planes, 0, sizeof(planes));
        buf.length = 1;
        buf.m.planes = planes;
    }
    
    // 
    fd_set fds;
    struct timeval tv;
    FD_ZERO(&fds);
    FD_SET(cam->fd, &fds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    
    int ret = select(cam->fd + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0) {
        if (ret == 0) {
            fprintf(stderr, "[V4L2] Timeout waiting for frame\n");
        }
        return false;
    }
    

    if (ioctl(cam->fd, VIDIOC_DQBUF, &buf) < 0) {
        fprintf(stderr, "[V4L2] VIDIOC_DQBUF failed: %s\n", strerror(errno));
        return false;
    }
    
    return v4l2_camera_fill_dequeue_result(cam, buf.index,
                                           out_index, out_data,
                                           out_size, out_dmabuf_fd);
}

bool v4l2_camera_queue(V4L2Camera* cam, int index) {
    if (!cam || cam->fd < 0 || index < 0 || index >= cam->buffer_count) {
        return false;
    }
    
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = v4l2_camera_buffer_type(cam);
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    
    struct v4l2_plane planes[1];
    if (cam->use_mplane) {
        memset(planes, 0, sizeof(planes));
        buf.length = 1;
        buf.m.planes = planes;
    }
    
    if (ioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
        fprintf(stderr, "[V4L2] VIDIOC_QBUF[%d] failed: %s\n", index, strerror(errno));
        return false;
    }
    
    return true;
}

void v4l2_camera_close(V4L2Camera* cam) {
    if (!cam) return;

    char device_path[sizeof(cam->device_path)];
    memset(device_path, 0, sizeof(device_path));
    strncpy(device_path, cam->device_path, sizeof(device_path) - 1);
    const int requested_width = cam->requested_width;
    const int requested_height = cam->requested_height;
    const int requested_buffer_count = cam->requested_buffer_count;
    const int fd = cam->fd;
    const bool use_mplane = cam->use_mplane;
    const int buffer_count = cam->buffer_count;
    void** buffers = cam->buffers;
    size_t* buffer_lengths = cam->buffer_lengths;
    int* dmabuf_fds = cam->dmabuf_fds;
    
    if (fd >= 0) {
        enum v4l2_buf_type type = use_mplane
            ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
            : V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd, VIDIOC_STREAMOFF, &type);
        
        // 
        if (buffers) {
            for (int i = 0; i < buffer_count; i++) {
                if (buffers[i] && buffers[i] != MAP_FAILED) {
                    munmap(buffers[i], buffer_lengths[i]);
                }
            }
            free(buffers);
        }
        
        if (buffer_lengths) {
            free(buffer_lengths);
        }
        
        //  
        if (dmabuf_fds) {
            for (int i = 0; i < buffer_count; i++) {
                if (dmabuf_fds[i] >= 0) {
                    close(dmabuf_fds[i]);
                }
            }
            free(dmabuf_fds);
        }
        
        close(fd);
    }
    
    memset(cam, 0, sizeof(*cam));
    cam->fd = -1;
    strncpy(cam->device_path, device_path, sizeof(cam->device_path) - 1);
    cam->device_path[sizeof(cam->device_path) - 1] = '\0';
    cam->requested_width = requested_width;
    cam->requested_height = requested_height;
    cam->requested_buffer_count = requested_buffer_count;
    fprintf(stderr, "[V4L2] Camera closed\n");
}

