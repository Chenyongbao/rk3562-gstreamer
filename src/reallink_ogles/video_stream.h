#ifndef VIDEO_STREAM_H
#define VIDEO_STREAM_H

#include <string>
#include <vector>
#include <cstdint>


class VideoStream {
public:
    VideoStream();
    ~VideoStream();
    

    bool open(const std::string& device, int width, int height, const std::string& format = "NV12");
    

    void close();
    



    bool readFrame(std::vector<uint8_t>& data);
    

    const uint8_t* readFramePtr();
    


    bool releaseFrame();
    



    int getCurrentFrameFd() const;
    

    bool isOpen() const { return fd_ >= 0; }
    

    int width() const { return width_; }
    

    int height() const { return height_; }

private:
    int fd_;
    int width_;
    int height_;
    std::vector<void*> buffers_;
    std::vector<size_t> buffer_lengths_;
    std::vector<int> buffer_fds_;
    bool streaming_;
    int current_buffer_index_;
    

    bool setFormat(int width, int height, uint32_t pixel_format);
    bool requestBuffers();
    bool startStreaming();
    void stopStreaming();
    bool unmapBuffers();
    bool exportBufferFd(int buffer_index, int& dma_fd);
};

#endif // VIDEO_STREAM_H

