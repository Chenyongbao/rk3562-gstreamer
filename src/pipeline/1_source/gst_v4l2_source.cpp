#include "pipeline/1_source/gst_v4l2_source.h"

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <gst/allocators/gstdmabuf.h>

#include <cstdio>

#include "config.h"
#include "core/logging.h"

// 本文件实现采集源：用 GStreamer 的 v4l2src 从摄像头抓帧，再经 tee 分流成三路，
// 供下游（原画 / BEV / 快照）各自独立拉取。核心设计目标：
//   - 全程 DMABUF 零拷贝：io-mode=dmabuf 让 v4l2src 直接产出 dmabuf 缓冲，
//     下游拿 fd 做 RGA/BEV 处理，不把整帧数据拷进用户态。
//   - 三路独立背压：每个分支各配一个 leaky=downstream 的 queue，只留最新帧、
//     满了丢旧帧，保证直播低延迟、三路互不拖累。

namespace {

constexpr int kAppSinkMaxBuffers = 2;                       // 每路 appsink 最多缓存 2 帧
constexpr GstClockTime kPullTimeoutNs = 200 * GST_MSECOND;  // 拉帧超时 200ms

const char* kBranchNames[] = {"orig", "bev", "snap"};

// 原画分支的 RGA 缩放元素名（Rockchip gst 插件的 rgaconvert）。
const char* kScaleElementName = "rgaconvert";

// 配置分支 queue：leaky=2（GST_QUEUE_LEAK_DOWNSTREAM）表示队列满时丢弃最旧
// 的帧腾位置给新帧，max-size-buffers=2 限制只缓存 2 帧。
// 这样每条分支都只保留最新画面，消费者忙不过来就自动丢帧而非越积越滞后。
void configure_branch_queue(GstElement* queue) {
  g_object_set(G_OBJECT(queue),
               "leaky", 2,               // GST_QUEUE_LEAK_DOWNSTREAM
               "max-size-buffers", (guint)kAppSinkMaxBuffers,
               "max-size-time", (guint64)0,
               "max-size-bytes", (guint64)0,
               nullptr);
}

} // namespace

GstV4L2Source::GstV4L2Source() = default;

// 析构：直接 close() 回收管线，避免资源泄漏。
GstV4L2Source::~GstV4L2Source() { close(); }

// 打开 v4l2 设备并构建采集分流管线：
//   v4l2src(io-mode=dmabuf) ! capsfilter(NV12) ! tee
//     tee. ! queue ! appsink (orig)
//     tee. ! queue ! appsink (bev)
//     tee. ! queue ! appsink (snap)
bool GstV4L2Source::open(const char *device, int width, int height) {
  if (!device || width <= 0 || height <= 0) {
    return false;
  }
  close();  // 幂等：先关掉可能存在的旧管线

  // 创建 v4l2src 源元素。
  GstElement *src = gst_element_factory_make("v4l2src", "src");
  if (!src) {
    spdlog::error("[GstSource] v4l2src element not found");
    return false;
  }
  // 指定设备节点，并设置 io-mode=dmabuf：
  // 让 v4l2src 用 DMA-BUF 方式与驱动交互，下游拿到的就是 dmabuf 缓冲，
  // 后续 RGA/BEV 可直接用 fd 零拷贝处理，避免 CPU memcpy 整帧。
  g_object_set(G_OBJECT(src), "device", device, nullptr);
  gst_util_set_object_arg(G_OBJECT(src), "io-mode", "dmabuf");

  // capsfilter 固定输出格式为 NV12 + 指定分辨率，作为协商的约束。
  GstCaps *caps = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, "NV12", "width", G_TYPE_INT, width,
      "height", G_TYPE_INT, height, nullptr);
  GstElement *filter = gst_element_factory_make("capsfilter", "filter");
  if (!filter) {
    gst_caps_unref(caps);
    gst_object_unref(src);
    return false;
  }
  g_object_set(G_OBJECT(filter), "caps", caps, nullptr);
  gst_caps_unref(caps);  // caps 已被 filter 引用，这里释放本地引用

  // tee 把同一路视频流复制分发到多个分支（本实现 3 路）。
  GstElement *tee = gst_element_factory_make("tee", "tee");
  if (!tee) {
    gst_object_unref(src);
    gst_object_unref(filter);
    return false;
  }

  // 组装主干：src -> filter -> tee。
  pipeline_ = gst_pipeline_new("capture");
  gst_bin_add_many(GST_BIN(pipeline_), src, filter, tee, nullptr);
  if (!gst_element_link_many(src, filter, tee, nullptr)) {
    spdlog::error("[GstSource] Failed to link v4l2src -> tee");
    close();
    return false;
  }

  // 为三个分支各建 queue + appsink 并接到 tee 上；原画分支额外插入 RGA 缩放，
  // 把 4224x3136 缩到 1280x960，让 appsink 直接产出目标分辨率，原画工作线程
  // 只需转发、无需再手动 RGA + 私有池。
  for (int i = 0; i < 3; ++i) {
    // queue 与 appsink 需各自唯一命名，否则 GStreamer 元素重名会失败。
    char queue_name[32];
    char sink_name[32];
    snprintf(queue_name, sizeof(queue_name), "%s-queue", kBranchNames[i]);
    snprintf(sink_name, sizeof(sink_name), "%s-sink", kBranchNames[i]);

    GstElement *queue = gst_element_factory_make("queue", queue_name);
    GstElement *sink = gst_element_factory_make("appsink", sink_name);
    if (!queue || !sink) {
      spdlog::error("[GstSource] Failed to create branch {}", i);
      close();
      return false;
    }

    // queue 配置为只留最新帧（丢旧帧），保证直播低延迟。
    configure_branch_queue(queue);
    // appsink：sync=FALSE 不等时钟（尽快吐帧），drop=TRUE 拉取不及时就丢帧。
    g_object_set(G_OBJECT(sink), "sync", FALSE, "max-buffers",
                 (guint)kAppSinkMaxBuffers, "drop", TRUE, nullptr);

    gst_bin_add_many(GST_BIN(pipeline_), queue, sink, nullptr);
    if (!gst_element_link(tee, queue)) {
      spdlog::error("[GstSource] Failed to link tee -> branch {}", i);
      close();
      return false;
    }

    if (i == static_cast<int>(GstSourceBranch::kOriginal)) {
      // 原画分支：插入 RGA 硬件缩放 + capsfilter 约束目标分辨率。
      GstElement *scale =
          gst_element_factory_make(kScaleElementName, "orig-scale");
      GstElement *scale_filter =
          gst_element_factory_make("capsfilter", "orig-scale-filter");
      if (!scale || !scale_filter) {
        spdlog::error("[GstSource] Failed to create RGA scale element for orig "
                      "branch (element '{}')",
                      kScaleElementName);
        if (scale) gst_object_unref(scale);
        if (scale_filter) gst_object_unref(scale_filter);
        close();
        return false;
      }
      GstCaps *scale_caps = gst_caps_new_simple(
          "video/x-raw", "format", G_TYPE_STRING, "NV12", "width", G_TYPE_INT,
          ORIGINAL_WIDTH, "height", G_TYPE_INT, ORIGINAL_HEIGHT, nullptr);
      g_object_set(G_OBJECT(scale_filter), "caps", scale_caps, nullptr);
      gst_caps_unref(scale_caps);

      gst_bin_add_many(GST_BIN(pipeline_), scale, scale_filter, nullptr);
      if (!gst_element_link_many(queue, scale, scale_filter, sink, nullptr)) {
        spdlog::error("[GstSource] Failed to link orig scale chain");
        close();
        return false;
      }
    } else {
      if (!gst_element_link(queue, sink)) {
        spdlog::error("[GstSource] Failed to link branch {} -> appsink", i);
        close();
        return false;
      }
    }
    appsinks_[i] = sink;  // 记录该分支的 appsink，供 dequeue 使用
  }

  // 启动整条管线到 PLAYING 状态，采集正式开始。
  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) ==
      GST_STATE_CHANGE_FAILURE) {
    spdlog::error("[GstSource] Failed to start pipeline");
    close();
    return false;
  }

  spdlog::info("[GstSource] Opened {}: {}x{} NV12 (tee -> 3 appsinks)", device,
               width, height);
  return true;
}

// 关闭并销毁 GStreamer 管线及所有资源。
void GstV4L2Source::close() {
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);  // 先停管线
    gst_object_unref(pipeline_);                       // 再释放引用
    pipeline_ = nullptr;
  }
  for (int i = 0; i < 3; ++i) {
    appsinks_[i] = nullptr;  // appsink 已被 pipeline unref 连带释放，这里仅清指针
  }
}

// 检查设备是否成功打开。
bool GstV4L2Source::is_open() const { return pipeline_ != nullptr; }

// 从指定分支阻塞拉取一帧（200ms 超时）。用完必须 release() 归还。
//
// 返回的 GstFrame 同时携带：
//   buffer   —— 借用了一份引用的 GstBuffer（DMABUF 零拷贝）
//   dmabuf_fd —— 从 buffer 第 0 块 memory 提取的裸 fd（供 RGA/BEV 零拷贝用）
//   stride   —— Y 平面真实行宽（bytesperline），可能与 width 不同
bool GstV4L2Source::dequeue(GstSourceBranch branch, GstFrame *frame) {
  const int idx = static_cast<int>(branch);
  if (!frame || idx < 0 || idx >= 3 || !appsinks_[idx]) {
    return false;
  }

  // 第 1 层：appsink 阻塞拉取返回 GstSample（GStreamer 的"样本"容器，
  //          内含 GstBuffer + caps + 时间戳等元信息）。
  GstSample *sample =
      gst_app_sink_try_pull_sample(GST_APP_SINK(appsinks_[idx]), kPullTimeoutNs);
  if (!sample) {
    return false; // 超时或 EOS
  }

  // 第 2 层：从 sample 里拆出真正承载像素数据的 GstBuffer。
  GstBuffer *buffer = gst_sample_get_buffer(sample);
  if (!buffer) {
    gst_sample_unref(sample);
    return false;
  }

  // 借用语义：多持一份引用（gst_buffer_ref），调用方 release() 时 unref 归还。
  // 这样 sample 释放后 buffer 仍有效，v4l2src 缓冲池在 unref 后才会回收复用。
  frame->buffer = gst_buffer_ref(buffer);

  // 第 3 层：从 buffer 最内层取出 DMABUF fd。io-mode=dmabuf 下第 0 块 memory
  // 就是 GstDmabufMemory，gst_dmabuf_memory_get_fd 直接拿到裸 fd。
  // RGA/BEV 拿到这个 fd 即可零拷贝处理，无需把整帧数据拷进用户态。
  GstMemory *mem = gst_buffer_peek_memory(buffer, 0);
  frame->dmabuf_fd = mem ? gst_dmabuf_memory_get_fd(mem) : -1;

  // 读 GstVideoMeta：v4l2src 会按驱动实际分配的 bytesperline 填 stride。
  // 后续 RGA/BEV 必须用真实 stride 寻址，不能用 width 代替（硬件缓冲常按
  // 对齐补行宽，width != stride）。
  GstVideoMeta *vmeta = gst_buffer_get_video_meta(buffer);
  if (vmeta) {
    frame->width = vmeta->width;
    frame->height = vmeta->height;
    frame->stride = vmeta->stride[0];
  }
  frame->size = gst_buffer_get_size(buffer);  // NV12 总字节数

  gst_sample_unref(sample);  // sample 用完释放，buffer 仍被 frame 持有一份引用
  return true;
}

// 归还帧：unref GstBuffer，引用计数归零后把内存还给 v4l2src 缓冲池。
// 归还前必须确保没有其他消费者（如 RGA）还在读这块内存。
void GstV4L2Source::release(GstFrame *frame) {
  if (!frame || !frame->buffer) {
    return;
  }
  gst_buffer_unref(frame->buffer);
  frame->buffer = nullptr;
}
