#ifndef BEV_FRAME_BUFFER_H
#define BEV_FRAME_BUFFER_H

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

struct CaptureLoopState;

class LatestNv12FrameBuffer {
public:
    bool init(int width, int height);
    void cleanup();

    bool update(const uint8_t* nv12_data, size_t size, uint64_t frame_id);
    bool copy(uint8_t* dst, size_t dst_size, size_t* out_size, uint64_t* out_frame_id) const;
    bool copy_newer_than(uint8_t* dst,
                         size_t dst_size,
                         size_t* out_size,
                         uint64_t* out_frame_id,
                         uint64_t min_frame_id,
                         int timeout_ms,
                         int poll_interval_ms = 10) const;

    int get_width() const;
    int get_height() const;
    size_t get_frame_size() const;
    bool has_frame() const;

private:
    std::vector<uint8_t> frame_buffer_;
    size_t frame_size_ = 0;
    int width_ = 0;
    int height_ = 0;
    uint64_t frame_id_ = 0;
    bool has_frame_ = false;
    mutable std::mutex mutex_;
};

//=========主摄俯视图图片采集=======
bool bev_frame_buffer_init(int width, int height);
void bev_frame_buffer_cleanup();
bool bev_frame_buffer_update(const uint8_t* nv12_data, size_t size, uint64_t frame_id);
bool bev_frame_buffer_copy(uint8_t* dst, size_t dst_size, size_t* out_size, uint64_t* out_frame_id);
bool bev_frame_buffer_copy_newer_than(uint8_t* dst,
                                      size_t dst_size,
                                      size_t* out_size,
                                      uint64_t* out_frame_id,
                                      uint64_t min_frame_id,
                                      int timeout_ms,
                                      int poll_interval_ms = 10);
int bev_frame_buffer_get_width();
int bev_frame_buffer_get_height();
size_t bev_frame_buffer_get_frame_size();
bool bev_frame_buffer_has_frame();


//========主摄原图片采集=========
bool main_camera_frame_buffer_init(int width, int height);
void main_camera_frame_buffer_cleanup();
bool main_camera_frame_buffer_update(const uint8_t* nv12_data, size_t size, uint64_t frame_id);
bool main_camera_frame_buffer_copy(uint8_t* dst, size_t dst_size, size_t* out_size, uint64_t* out_frame_id);
bool main_camera_frame_buffer_copy_newer_than(uint8_t* dst,
                                              size_t dst_size,
                                              size_t* out_size,
                                              uint64_t* out_frame_id,
                                              uint64_t min_frame_id,
                                              int timeout_ms,
                                              int poll_interval_ms = 10);
bool main_camera_frame_buffer_request_fresh_copy(CaptureLoopState* state,
                                                 uint8_t* dst,
                                                 size_t dst_size,
                                                 size_t* out_size,
                                                 uint64_t* out_frame_id,
                                                 int timeout_ms,
                                                 int poll_interval_ms = 10);
int main_camera_frame_buffer_get_width();
int main_camera_frame_buffer_get_height();
size_t main_camera_frame_buffer_get_frame_size();
bool main_camera_frame_buffer_has_frame();

#endif // BEV_FRAME_BUFFER_H
