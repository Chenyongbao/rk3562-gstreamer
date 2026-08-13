#include "pipeline/3_consumers/common/LatestNv12FrameBuffer.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include "app/capture_state.h"

namespace {

constexpr int kMainCameraCopyRetryDelayMs = 40;

// NV12 大小固定为 Y 平面 + 交错 UV 平面，即 width * height * 3 / 2。
size_t nv12_size_of(int width, int height) {
    if (width <= 0 || height <= 0) {
        return 0;
    }
    return static_cast<size_t>(width) * static_cast<size_t>(height) * 3 / 2;
}

LatestNv12FrameBuffer g_bev_frame_buffer;
LatestNv12FrameBuffer g_main_camera_frame_buffer;

}  // namespace

bool LatestNv12FrameBuffer::init(int width, int height)
{
    // 初始化时一次性分配完整帧缓存，后续 update/copy 只做 memcpy。
    const size_t frame_size = nv12_size_of(width, height);
    if (frame_size == 0) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    frame_buffer_.assign(frame_size, 0);
    frame_size_ = frame_size;
    width_ = width;
    height_ = height;
    frame_id_ = 0;
    has_frame_ = false;
    return true;
}

void LatestNv12FrameBuffer::cleanup()
{
    std::lock_guard<std::mutex> lock(mutex_);
    frame_buffer_.clear();
    frame_buffer_.shrink_to_fit();
    frame_size_ = 0;
    width_ = 0;
    height_ = 0;
    frame_id_ = 0;
    has_frame_ = false;
}

bool LatestNv12FrameBuffer::update(const uint8_t* nv12_data, size_t size, uint64_t frame_id)
{
    if (!nv12_data || size == 0) {
        return false;
    }

    // 仅接受与初始化尺寸完全一致的帧，避免缓冲越界。
    std::lock_guard<std::mutex> lock(mutex_);
    if (frame_buffer_.empty() || size < frame_size_) {
        return false;
    }

    std::memcpy(frame_buffer_.data(), nv12_data, frame_size_);
    frame_id_ = frame_id;
    has_frame_ = true;
    return true;
}

bool LatestNv12FrameBuffer::copy(uint8_t* dst, size_t dst_size, size_t* out_size, uint64_t* out_frame_id) const
{
    if (!dst || dst_size == 0) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (frame_buffer_.empty() || !has_frame_ || dst_size < frame_size_) {
        return false;
    }

    std::memcpy(dst, frame_buffer_.data(), frame_size_);
    if (out_size) {
        *out_size = frame_size_;
    }
    if (out_frame_id) {
        *out_frame_id = frame_id_;
    }
    return true;
}

bool LatestNv12FrameBuffer::copy_newer_than(uint8_t* dst,
                                            size_t dst_size,
                                            size_t* out_size,
                                            uint64_t* out_frame_id,
                                            uint64_t min_frame_id,
                                            int timeout_ms,
                                            int poll_interval_ms) const
{
    if (!dst || dst_size == 0) {
        return false;
    }

    if (poll_interval_ms <= 0) {
        poll_interval_ms = 10;
    }
    if (timeout_ms < 0) {
        timeout_ms = 0;
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    // 通过简单轮询等待更新后的帧，适合跨线程请求“新鲜一帧”的场景。
    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!frame_buffer_.empty() &&
                has_frame_ &&
                dst_size >= frame_size_ &&
                frame_id_ > min_frame_id) {
                std::memcpy(dst, frame_buffer_.data(), frame_size_);
                if (out_size) {
                    *out_size = frame_size_;
                }
                if (out_frame_id) {
                    *out_frame_id = frame_id_;
                }
                return true;
            }
        }

        if (std::chrono::steady_clock::now() >= deadline) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
    }
}

int LatestNv12FrameBuffer::get_width() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return width_;
}

int LatestNv12FrameBuffer::get_height() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return height_;
}

size_t LatestNv12FrameBuffer::get_frame_size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_size_;
}

bool LatestNv12FrameBuffer::has_frame() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return has_frame_;
}

bool bev_frame_buffer_init(int width, int height)
{
    const bool ok = g_bev_frame_buffer.init(width, height);
    if (ok) {
        std::fprintf(stderr, "[BEVBuffer] Initialized (size=%zu, %dx%d)\n",
                     g_bev_frame_buffer.get_frame_size(), width, height);
    }
    return ok;
}

void bev_frame_buffer_cleanup()
{
    g_bev_frame_buffer.cleanup();
}

bool bev_frame_buffer_update(const uint8_t* nv12_data, size_t size, uint64_t frame_id)
{
    return g_bev_frame_buffer.update(nv12_data, size, frame_id);
}

bool bev_frame_buffer_copy(uint8_t* dst, size_t dst_size, size_t* out_size, uint64_t* out_frame_id)
{
    return g_bev_frame_buffer.copy(dst, dst_size, out_size, out_frame_id);
}

bool bev_frame_buffer_copy_newer_than(uint8_t* dst,
                                      size_t dst_size,
                                      size_t* out_size,
                                      uint64_t* out_frame_id,
                                      uint64_t min_frame_id,
                                      int timeout_ms,
                                      int poll_interval_ms)
{
    return g_bev_frame_buffer.copy_newer_than(dst,
                                              dst_size,
                                              out_size,
                                              out_frame_id,
                                              min_frame_id,
                                              timeout_ms,
                                              poll_interval_ms);
}

int bev_frame_buffer_get_width()
{
    return g_bev_frame_buffer.get_width();
}

int bev_frame_buffer_get_height()
{
    return g_bev_frame_buffer.get_height();
}

size_t bev_frame_buffer_get_frame_size()
{
    return g_bev_frame_buffer.get_frame_size();
}

bool bev_frame_buffer_has_frame()
{
    return g_bev_frame_buffer.has_frame();
}

bool main_camera_frame_buffer_init(int width, int height)
{
    const bool ok = g_main_camera_frame_buffer.init(width, height);
    if (ok) {
        std::fprintf(stderr, "[MainCameraBuffer] Initialized (size=%zu, %dx%d)\n",
                     g_main_camera_frame_buffer.get_frame_size(), width, height);
    }
    return ok;
}

void main_camera_frame_buffer_cleanup()
{
    g_main_camera_frame_buffer.cleanup();
}

bool main_camera_frame_buffer_update(const uint8_t* nv12_data, size_t size, uint64_t frame_id)
{
    return g_main_camera_frame_buffer.update(nv12_data, size, frame_id);
}

bool main_camera_frame_buffer_copy(uint8_t* dst, size_t dst_size, size_t* out_size, uint64_t* out_frame_id)
{
    // 主相机首次启动时可能还没来得及 publish，允许一次短暂重试。
    if (g_main_camera_frame_buffer.copy(dst, dst_size, out_size, out_frame_id)) {
        return true;
    }
    if (g_main_camera_frame_buffer.has_frame()) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kMainCameraCopyRetryDelayMs));
    return g_main_camera_frame_buffer.copy(dst, dst_size, out_size, out_frame_id);
}

bool main_camera_frame_buffer_copy_newer_than(uint8_t* dst,
                                              size_t dst_size,
                                              size_t* out_size,
                                              uint64_t* out_frame_id,
                                              uint64_t min_frame_id,
                                              int timeout_ms,
                                              int poll_interval_ms)
{
    return g_main_camera_frame_buffer.copy_newer_than(dst,
                                                      dst_size,
                                                      out_size,
                                                      out_frame_id,
                                                      min_frame_id,
                                                      timeout_ms,
                                                      poll_interval_ms);
}

bool main_camera_frame_buffer_request_fresh_copy(CaptureLoopState* state,
                                                 uint8_t* dst,
                                                 size_t dst_size,
                                                 size_t* out_size,
                                                 uint64_t* out_frame_id,
                                                 int timeout_ms,
                                                 int poll_interval_ms)
{
    if (!state || !dst || dst_size == 0) {
        return false;
    }

    // 先记住当前最新帧号，再触发刷新请求并等待更大的 frame_id 出现。
    const uint64_t min_frame_id = getLatestMainCameraFrameId(state);
    requestMainCameraRefresh(state);
    return main_camera_frame_buffer_copy_newer_than(dst,
                                                    dst_size,
                                                    out_size,
                                                    out_frame_id,
                                                    min_frame_id,
                                                    timeout_ms,
                                                    poll_interval_ms);
}

int main_camera_frame_buffer_get_width()
{
    return g_main_camera_frame_buffer.get_width();
}

int main_camera_frame_buffer_get_height()
{
    return g_main_camera_frame_buffer.get_height();
}

size_t main_camera_frame_buffer_get_frame_size()
{
    return g_main_camera_frame_buffer.get_frame_size();
}

bool main_camera_frame_buffer_has_frame()
{
    return g_main_camera_frame_buffer.has_frame();
}
