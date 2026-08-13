#define _POSIX_C_SOURCE 199309L

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define _POSIX_C_SOURCE 199309L

#include "pipeline/1_source/v4l2_capture.h"
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

// 获取摄像头支持的缓冲区类型（MPLANE 或 普通类型）
// MPLANE (Multi-planar) 允许图像的多个平面（如 Y, U,
// V）存放在不同的内存缓冲区中
uint32_t v4l2_camera_buffer_type(const V4L2Camera *cam) {
  if (cam && cam->use_mplane) {
    return V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  }
  return V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

bool v4l2_camera_fill_dequeue_result(const V4L2Camera *cam, uint32_t buf_index,
                                     int *out_index, void **out_data,
                                     size_t *out_size, int *out_dmabuf_fd) {
  // 将驱动返回的 buffer index 映射成上层统一使用的出队结果结构。
  if (!cam || !out_index || !out_data || !out_size) {
    return false;
  }
  if (buf_index >= (uint32_t)cam->buffer_count) {
    fprintf(stderr,
            "[V4L2] Invalid buffer index from driver: %u (buffers=%d)\n",
            buf_index, cam->buffer_count);
    return false;
  }

  *out_index = (int)buf_index;                // 告诉外面：是第 2 号盘子
  *out_data = cam->buffers[buf_index];        // 告诉外面：到这里来取数据
  *out_size = cam->buffer_lengths[buf_index]; // 告诉外面：一共有多大
  if (out_dmabuf_fd) {
    *out_dmabuf_fd = cam->dmabuf_fds ? cam->dmabuf_fds[buf_index] : -1;
  }
  return true;
}

// 【第二步：实现设备初始化】
// 打开并初始化 V4L2 摄像头设备
// width, height: 请求的分辨率
// buffer_count: 请求的内存映射缓冲区数量
bool v4l2_camera_open(V4L2Camera *cam, const char *device, int width,
                      int height, int buffer_count) {
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

  // 【2.1 open()】：以非阻塞模式打开视频设备节点 (如 /dev/video0)
  cam->fd = open(device, O_RDWR | O_NONBLOCK);
  if (cam->fd < 0) {
    fprintf(stderr, "[V4L2] Failed to open %s: %s\n", device, strerror(errno));
    return false;
  }

  struct v4l2_capability cap;
  memset(&cap, 0, sizeof(cap));
  // 【2.2 VIDIOC_QUERYCAP】：查询设备能力，确认它真的是个视频捕获设备
  if (ioctl(cam->fd, VIDIOC_QUERYCAP, &cap) < 0) {
    fprintf(stderr, "[V4L2] VIDIOC_QUERYCAP failed: %s\n", strerror(errno));
    close(cam->fd);
    cam->fd = -1;
    return false;
  }

  //
  bool use_mplane = (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0;
  cam->use_mplane = use_mplane;

  // 确保在重新配置前，先关闭之前的视频流 (防御性编程)
  enum v4l2_buf_type type = (enum v4l2_buf_type)v4l2_camera_buffer_type(cam);
  ioctl(cam->fd, VIDIOC_STREAMOFF, &type);

  // 请求 0 个缓冲区，强制驱动释放之前可能残留的缓冲区资源 (清理状态)
  struct v4l2_requestbuffers req_zero;
  memset(&req_zero, 0, sizeof(req_zero));
  req_zero.type = type;
  req_zero.memory = V4L2_MEMORY_MMAP;
  req_zero.count = 0;
  ioctl(cam->fd, VIDIOC_REQBUFS, &req_zero);

  // 某些驱动要求宽高按 8 对齐，这里提前做一次保守修正。
  width = (width + 7) & ~7;
  height = (height + 7) & ~7;

  // 设置视频格式，请求 NV12 像素格式 (YUV 4:2:0)
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

  // 【2.3 VIDIOC_S_FMT】：向驱动下发并锁定视频流的宽高和像素格式
  if (ioctl(cam->fd, VIDIOC_S_FMT, &fmt) < 0) {
    fprintf(stderr, "[V4L2] VIDIOC_S_FMT failed: %s\n", strerror(errno));
    fprintf(stderr, "[V4L2] Requested: %dx%d NV12 (%s mode)\n", width, height,
            use_mplane ? "MPLANE" : "PLANAR");
    fprintf(stderr, "[V4L2] Please check supported formats with:\n");
    fprintf(stderr, "[V4L2]   v4l2-ctl --device=%s --list-formats-ext\n",
            device);
    close(cam->fd);
    cam->fd = -1;
    return false;
  }

  // 记录驱动最终接受的宽高和 stride，后续帧描述都以此为准。
  if (use_mplane) {
    cam->width = fmt.fmt.pix_mp.width;
    cam->height = fmt.fmt.pix_mp.height;
    cam->y_stride = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
  } else {
    cam->width = fmt.fmt.pix.width;
    cam->height = fmt.fmt.pix.height;
    cam->y_stride = fmt.fmt.pix.bytesperline;
  }

  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = buffer_count;
  req.type = type;
  req.memory = V4L2_MEMORY_MMAP;

  // 【2.4 VIDIOC_REQBUFS】：向内核申请指定数量的视频缓冲区
  // 申请缓冲区
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
  cam->buffers = (void **)calloc(req.count, sizeof(void *));
  cam->buffer_lengths = (size_t *)calloc(req.count, sizeof(size_t));
  if (!cam->buffers || !cam->buffer_lengths) {
    fprintf(stderr, "[V4L2] Failed to allocate camera buffers metadata\n");
    v4l2_camera_close(cam);
    return false;
  }

  // 为每个驱动缓冲建立 mmap 映射，供 CPU 读取和导出 DMABUF 使用。
  cam->dmabuf_fds = (int *)calloc(req.count, sizeof(int));
  for (uint32_t i = 0; i < req.count; i++) {
    cam->dmabuf_fds[i] = -1; // 默认标记为尚未成功导出。
  }

  // 查询并映射每个 V4L2 缓冲区。
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

    // 缓冲区查询
    if (ioctl(cam->fd, VIDIOC_QUERYBUF, &buf) < 0) {
      fprintf(stderr, "[V4L2] VIDIOC_QUERYBUF[%d] failed: %s\n", i,
              strerror(errno));
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

    // 【2.5 mmap()】：将内核分配的缓冲区内存映射到用户态，方便 CPU 直接访问数据
    cam->buffers[i] =
        mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, cam->fd, offset);

    if (cam->buffers[i] == MAP_FAILED) {
      fprintf(stderr, "[V4L2] mmap[%d] failed: %s\n", i, strerror(errno));
      v4l2_camera_close(cam);
      return false;
    }

    cam->buffer_lengths[i] = length;
  }

  // 导出 DMABUF fd，成功后下游可走零拷贝或 RGA 直接处理路径。
  fprintf(stderr, "[V4L2] Exporting DMABUF file descriptors...\n");
  bool dmabuf_success = true;
  for (int i = 0; i < cam->buffer_count; i++) {
    struct v4l2_exportbuffer expbuf;
    memset(&expbuf, 0, sizeof(expbuf));
    expbuf.type = type;
    expbuf.index = i;
    expbuf.plane = 0;
    expbuf.flags = O_RDWR;

    // 【2.6 VIDIOC_EXPBUF】(进阶)：将 V4L2 缓冲区导出为一个 DMABUF
    // 句柄，可供零拷贝传输
    if (ioctl(cam->fd, VIDIOC_EXPBUF, &expbuf) < 0) {
      fprintf(stderr, "[V4L2]   EXPBUF failed for buffer %d: %s\n", i,
              strerror(errno));
      dmabuf_success = false;
      break;
    }

    cam->dmabuf_fds[i] = expbuf.fd;
  }

  if (dmabuf_success) {
    fprintf(
        stderr,
        "[V4L2]  All %d DMABUF fds exported successfully (zero-copy enabled)\n",
        cam->buffer_count);
  } else {
    fprintf(stderr, "[V4L2] DMABUF export failed, will use memcpy fallback\n");
  }

  fprintf(stderr, "[V4L2] Opened %s: %dx%d NV12, stride=%d, buffers=%d (%s)\n",
          device, cam->width, cam->height, cam->y_stride, cam->buffer_count,
          use_mplane ? "MPLANE" : "PLANAR");

  return true;
}

// =============================================================================
// 【内部辅助函数】将所有缓冲区重新入驱动队列
// 消除 start 和 resume 中完全相同的入队逻辑，遵循 DRY 原则。
// 无需对外暴露，故用 static 限定为文件内私有。
// =============================================================================
static bool enqueue_all_buffers(V4L2Camera *cam) {
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
      fprintf(stderr, "[V4L2] VIDIOC_QBUF[%d] failed: %s\n", i,
              strerror(errno));
      return false;
    }
  }
  return true;
}

// 【第三步：实现启动与停止 (Start部分)】
// 启动摄像头视频流采集
bool v4l2_camera_start(V4L2Camera *cam) {
  if (!cam || cam->fd < 0) {
    return false;
  }

  enum v4l2_buf_type type = (enum v4l2_buf_type)v4l2_camera_buffer_type(cam);

  // 【3.1 VIDIOC_QBUF】：把所有申请好的缓冲区放入驱动队列 (入队)
  if (!enqueue_all_buffers(cam)) {
    return false;
  }

  // 先把所有缓冲重新入队，再统一 STREAMON 开始采集。
  // 【3.2 VIDIOC_STREAMON】：告诉驱动开始干活，把硬件数据写入队列里的缓冲区
  if (ioctl(cam->fd, VIDIOC_STREAMON, &type) < 0) {
    fprintf(stderr, "[V4L2] VIDIOC_STREAMON failed: %s\n", strerror(errno));
    return false;
  }

  fprintf(stderr, "[V4L2] Stream started\n");
  return true;
}

// 【第三步：实现启动与停止 (Stop部分)】
// 停止摄像头视频流采集
bool v4l2_camera_stop(V4L2Camera *cam) {
  if (!cam || cam->fd < 0) {
    return false;
  }

  enum v4l2_buf_type type = (enum v4l2_buf_type)v4l2_camera_buffer_type(cam);

  // 【3.3 VIDIOC_STREAMOFF】：告诉驱动停止写入数据
  if (ioctl(cam->fd, VIDIOC_STREAMOFF, &type) < 0) {
    fprintf(stderr, "[V4L2] VIDIOC_STREAMOFF failed: %s\n", strerror(errno));
    return false;
  }

  fprintf(stderr, "[V4L2] Stream stopped\n");
  return true;
}
// 【第六步 (进阶)：容错与恢复】
// 一级恢复：先强制 STREAMOFF 确保驱动状态干净，
// 再把所有缓冲重新入队，然后重新 STREAMON。
bool v4l2_camera_resume(V4L2Camera *cam) {
  if (!cam || cam->fd < 0) {
    return false;
  }

  enum v4l2_buf_type type = (enum v4l2_buf_type)v4l2_camera_buffer_type(cam);

  // 【修复】先强行停流，确保驱动状态干净，防止部分 buffer
  // 仍被应用层持有时直接 STREAMON 导致死锁或丢帧。
  // 忽略返回值：即使流本来就是停止状态，这里也是安全的。
  ioctl(cam->fd, VIDIOC_STREAMOFF, &type);

  // 重新把所有缓冲区入队（调用公共辅助函数，不再重复代码）
  if (!enqueue_all_buffers(cam)) {
    return false;
  }

  // 【3.2 VIDIOC_STREAMON】重新启动
  if (ioctl(cam->fd, VIDIOC_STREAMON, &type) < 0) {
    fprintf(stderr, "[V4L2] VIDIOC_STREAMON failed: %s\n", strerror(errno));
    return false;
  }

  fprintf(stderr, "[V4L2] Stream resumed\n");
  return true;
}
// 二级恢复：在一级恢复失败后，执行完整的 close -> open -> start 流程。
bool v4l2_camera_recover(V4L2Camera *cam) {
  if (!cam) {
    return false;
  }

  // 【简化】提前备份恢复所需的参数。
  // 使用固定大小的栈数组，避免堆分配；用 memcpy 代替 memset+strncpy 两步。
  char device_path[sizeof(cam->device_path)];
  memcpy(device_path, cam->device_path, sizeof(device_path));
  // 优先使用 requested_xxx（用户请求值），如果没有则回退到当前实际值。
  const int w = cam->requested_width > 0 ? cam->requested_width : cam->width;
  const int h = cam->requested_height > 0 ? cam->requested_height : cam->height;
  const int n = cam->requested_buffer_count > 0 ? cam->requested_buffer_count
                                                : cam->buffer_count;

  fprintf(stderr, "[V4L2] Recovering stream on %s\n",
          device_path[0] ? device_path : "<unknown>");

  // ── 一级抢救：重新入队 + STREAMON ────────────────────────────────────────
  // resume 内部已经包含了 STREAMOFF -> enqueue_all_buffers -> STREAMON，
  // 无需在这里再单独调 STREAMOFF，避免重复停流。
  if (cam->fd >= 0) {
    if (v4l2_camera_resume(cam)) {
      fprintf(stderr, "[V4L2] Recovery succeeded via stream restart\n");
      return true;
    }
    fprintf(stderr, "[V4L2] Stream restart failed, attempting full reopen\n");
  }

  // ── 二级抢救：完整 close -> open -> start ────────────────────────────────
  // 检查备份的参数是否齐全，防止用无效参数去重新打开设备。
  if (!device_path[0] || w <= 0 || h <= 0 || n <= 0) {
    fprintf(stderr, "[V4L2] Recovery failed: missing reopen configuration\n");
    return false;
  }

  // 先彻底释放所有旧资源（fd、mmap 内存、DMABUF 句柄），再用备份参数重建。
  v4l2_camera_close(cam);
  if (!v4l2_camera_open(cam, device_path, w, h, n)) {
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
// 【第四步：实现数据流转 (取帧)】
// 借 (出队一帧画面)
bool v4l2_camera_dequeue(V4L2Camera *cam, int *out_index, void **out_data,
                         size_t *out_size, int *out_dmabuf_fd) {
  if (!cam || cam->fd < 0 || !out_index || !out_data || !out_size) {
    return false;
  }

  // 准备出队操作的 v4l2_buffer 结构体
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

  // 使用 select() 实现非阻塞/超时等待，避免 dequeue 死锁
  // 这里设置了 2 秒超时时间
  fd_set fds;            // 定义一个"监视名单"集合
  struct timeval tv;     // 定义一个"倒计时器"
  FD_ZERO(&fds);         // 把白纸上的所有标记都擦干净
  FD_SET(cam->fd, &fds); // 把摄像头的 FD（文件描述符）画在这张白纸上
  tv.tv_sec = 2;         // 设置等待时间：2秒
  tv.tv_usec = 0;        // 设置等待时间：0微妙

  // 【4.1 监听 fd】：等待驱动把图像数据填满缓冲区
  int ret = select(cam->fd + 1, &fds, NULL, NULL, &tv);
  if (ret <= 0) {
    if (ret == 0) {
      fprintf(stderr, "[V4L2] Timeout waiting for frame\n");
    }
    return false;
  }

  // 【4.2 VIDIOC_DQBUF】：将驱动写好数据的缓冲区从队列中取出
  if (ioctl(cam->fd, VIDIOC_DQBUF, &buf) < 0) {
    fprintf(stderr, "[V4L2] VIDIOC_DQBUF failed: %s\n", strerror(errno));
    return false;
  }

  return v4l2_camera_fill_dequeue_result(cam, buf.index, out_index, out_data,
                                         out_size, out_dmabuf_fd);
}
// 【第四步：实现数据流转 (还帧)】
// 还 (将处理完的缓冲区还给驱动)
bool v4l2_camera_queue(V4L2Camera *cam, int index) {
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

  // 【4.3
  // VIDIOC_QBUF】：用完这块内存后，必须还给内核队列，内核才能继续往里写下一帧
  if (ioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
    fprintf(stderr, "[V4L2] VIDIOC_QBUF[%d] failed: %s\n", index,
            strerror(errno));
    return false;
  }

  return true;
}

// 【第五步：实现资源清理】
// 关闭摄像头，释放所有相关的内存映射和文件描述符
void v4l2_camera_close(V4L2Camera *cam) {
  if (!cam)
    return;

  char device_path[sizeof(cam->device_path)];
  memset(device_path, 0, sizeof(device_path));
  strncpy(device_path, cam->device_path, sizeof(device_path) - 1);
  const int requested_width = cam->requested_width;
  const int requested_height = cam->requested_height;
  const int requested_buffer_count = cam->requested_buffer_count;
  const int fd = cam->fd;
  const bool use_mplane = cam->use_mplane;
  const int buffer_count = cam->buffer_count;
  void **buffers = cam->buffers;
  size_t *buffer_lengths = cam->buffer_lengths;
  int *dmabuf_fds = cam->dmabuf_fds;

  if (fd >= 0) {
    enum v4l2_buf_type type = use_mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                                         : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    // 解除 mmap 映射的内存
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

    // 关闭导出的 DMABUF 文件描述符
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
