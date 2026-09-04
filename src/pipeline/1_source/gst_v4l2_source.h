#ifndef GST_V4L2_SOURCE_H
#define GST_V4L2_SOURCE_H

#include <stdbool.h>
#include <stddef.h>

// 仅前置声明 GStreamer 类型，头文件不依赖 gst.h，由 .cpp 负责引入。
struct _GstBuffer;
typedef struct _GstBuffer GstBuffer;
struct _GstElement;
typedef struct _GstElement GstElement;

// tee 的三个下游分支。
enum class GstSourceBranch { kOriginal = 0, kBev, kSnapshot, kCount };

// 从 appsink 拉出的一帧。buffer 持有一份引用，用完必须 release()。
struct GstFrame {
    GstBuffer* buffer = nullptr;   // 借用的 GstBuffer（DMABUF 零拷贝）
    int dmabuf_fd = -1;            // 从 buffer 提取的 DMABUF fd，供 RGA/BEV 使用
    int width = 0;
    int height = 0;
    int stride = 0;                // Y 平面 bytesperline
    size_t size = 0;               // NV12 总字节数
};

// 基于 GStreamer v4l2src 的采集源，用 tee 分流为多路。
//
// 管线：
//   v4l2src io-mode=dmabuf
//     ! video/x-raw,format=NV12,width=W,height=H
//     ! tee name=t
//       t. ! queue(leaky=downstream) ! rgaconvert ! capsfilter(1280x960) ! appsink (original)
//       t. ! queue(leaky=downstream) ! appsink   (bev)
//       t. ! queue(leaky=downstream) ! appsink   (snapshot)
//
// 每路独立拉取、独立背压（各自 drop 旧帧），采集与算力层之间用 DMABUF 零拷贝。
class GstV4L2Source {
public:
    GstV4L2Source();
    ~GstV4L2Source();

    // 持有 GStreamer 裸指针，禁止拷贝，避免 double-unref。
    GstV4L2Source(const GstV4L2Source&) = delete;
    GstV4L2Source& operator=(const GstV4L2Source&) = delete;

    bool open(const char* device, int width, int height);
    void close();
    bool is_open() const;

    // 从指定分支拉取一帧（200ms 超时）。成功返回 true，用完后必须 release()。
    bool dequeue(GstSourceBranch branch, GstFrame* frame);

    // 归还帧：unref GstBuffer，把内存还给 v4l2src 缓冲池。
    static void release(GstFrame* frame);

private:
    GstElement* pipeline_ = nullptr;
    GstElement* appsinks_[3] = {nullptr, nullptr, nullptr};
};

#endif // GST_V4L2_SOURCE_H
