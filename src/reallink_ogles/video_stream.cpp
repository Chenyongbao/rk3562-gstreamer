#include "video_stream.h"
#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>
#include <errno.h>

VideoStream::VideoStream()
    : fd_(-1)
    , width_(0)
    , height_(0)
    , streaming_(false)
    , current_buffer_index_(-1)
{
}

VideoStream::~VideoStream() {
    close();
}

bool VideoStream::open(const std::string& device, int width, int height, const std::string& format) {

    if (isOpen()) {
        close();
    }
    

    fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "Failed to open device: " << device << std::endl;
        return false;
    }
    
    width_ = width;
    height_ = height;
    

    uint32_t pixel_format = 0;
    if (format == "NV12") {
        pixel_format = V4L2_PIX_FMT_NV12;
    } else {
        std::cerr << "Unsupported format: " << format << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    

    if (!setFormat(width, height, pixel_format)) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    

    if (!requestBuffers()) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    

    if (!startStreaming()) {
        unmapBuffers();
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    std::cout << "Video stream opened: " << device 
              << " (" << width << "x" << height << ", " << format << ")" << std::endl;
    
    return true;
}

void VideoStream::close() {
    if (streaming_) {
        stopStreaming();
    }
    
    unmapBuffers();
    
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    
    width_ = 0;
    height_ = 0;
    streaming_ = false;
}

bool VideoStream::setFormat(int width, int height, uint32_t pixel_format) {

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_G_FMT, &fmt) < 0) {
        std::cerr << "Failed to get current format: " << strerror(errno) << std::endl;
        return false;
    }
    

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pixel_format;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    
    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        std::cerr << "Failed to set format: " << strerror(errno) << std::endl;
        std::cerr << "  Requested: " << width << "x" << height 
                  << ", format: " << std::hex << pixel_format << std::dec << std::endl;
        return false;
    }
    

    if (fmt.fmt.pix.pixelformat != pixel_format) {
        std::cerr << "Format not set as expected. Got format: " 
                  << std::hex << fmt.fmt.pix.pixelformat << std::dec << std::endl;
        return false;
    }
    

    if (fmt.fmt.pix.width != static_cast<uint32_t>(width) ||
        fmt.fmt.pix.height != static_cast<uint32_t>(height)) {
        std::cerr << "Warning: Resolution adjusted by driver: " 
                  << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height 
                  << " (requested: " << width << "x" << height << ")" << std::endl;

        width_ = fmt.fmt.pix.width;
        height_ = fmt.fmt.pix.height;
    }
    
    return true;
}

bool VideoStream::requestBuffers() {
    struct v4l2_requestbuffers req = {};
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    req.count = 4;
    
    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        std::cerr << "Failed to request buffers" << std::endl;
        return false;
    }
    
    if (req.count < 2) {
        std::cerr << "Insufficient buffers allocated: " << req.count << std::endl;
        return false;
    }
    

    buffers_.clear();
    buffer_lengths_.clear();
    buffer_fds_.clear();
    
    for (uint32_t i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        
        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            std::cerr << "Failed to query buffer " << i << std::endl;
            unmapBuffers();
            return false;
        }
        
        void* ptr = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
        if (ptr == MAP_FAILED) {
            std::cerr << "Failed to mmap buffer " << i << std::endl;
            unmapBuffers();
            return false;
        }
        
        buffers_.push_back(ptr);
        buffer_lengths_.push_back(buf.length);
        

        int dma_fd = -1;
        if (exportBufferFd(i, dma_fd)) {
            buffer_fds_.push_back(dma_fd);
        } else {
            buffer_fds_.push_back(-1);
        }
    }
    
    return true;
}

bool VideoStream::startStreaming() {

    for (size_t i = 0; i < buffers_.size(); ++i) {
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        
        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            std::cerr << "Failed to queue buffer " << i << std::endl;
            return false;
        }
    }
    

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        std::cerr << "Failed to start streaming" << std::endl;
        return false;
    }
    
    streaming_ = true;
    return true;
}

void VideoStream::stopStreaming() {
    if (!streaming_) {
        return;
    }
    
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd_, VIDIOC_STREAMOFF, &type);
    streaming_ = false;
}

bool VideoStream::unmapBuffers() {

    if (current_buffer_index_ >= 0) {
        releaseFrame();
    }
    

    for (int fd : buffer_fds_) {
        if (fd >= 0) {
            ::close(fd);
        }
    }
    

    bool success = true;
    for (size_t i = 0; i < buffers_.size(); ++i) {
        if (buffers_[i] != nullptr) {
            if (munmap(buffers_[i], buffer_lengths_[i]) < 0) {
                std::cerr << "Failed to unmap buffer " << i << std::endl;
                success = false;
            }
        }
    }
    
    buffers_.clear();
    buffer_lengths_.clear();
    buffer_fds_.clear();
    current_buffer_index_ = -1;
    return success;
}

bool VideoStream::readFrame(std::vector<uint8_t>& data) {

    const uint8_t* ptr = readFramePtr();
    if (ptr == nullptr) {
        return false;
    }
    
    size_t frame_size = width_ * height_ * 3 / 2; // NV12
    data.resize(frame_size);
    memcpy(data.data(), ptr, frame_size);
    
    return releaseFrame();
}

const uint8_t* VideoStream::readFramePtr() {
    if (!streaming_) {
        return nullptr;
    }
    

    if (current_buffer_index_ >= 0) {
        releaseFrame();
    }
    

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    

    fd_set fds;
    struct timeval tv;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    
    int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ret == 0) {
        std::cerr << "Timeout waiting for frame" << std::endl;
        return nullptr;
    } else if (ret < 0) {
        std::cerr << "Error waiting for frame" << std::endl;
        return nullptr;
    }
    
    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        std::cerr << "Failed to dequeue buffer" << std::endl;
        return nullptr;
    }
    

    current_buffer_index_ = buf.index;
    

    return static_cast<const uint8_t*>(buffers_[buf.index]);
}

bool VideoStream::releaseFrame() {
    if (current_buffer_index_ < 0) {
        return true;
    }
    

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = current_buffer_index_;
    
    if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        std::cerr << "Failed to requeue buffer" << std::endl;
        current_buffer_index_ = -1;
        return false;
    }
    
    current_buffer_index_ = -1;
    return true;
}

int VideoStream::getCurrentFrameFd() const {
    if (current_buffer_index_ < 0 || current_buffer_index_ >= static_cast<int>(buffer_fds_.size())) {
        return -1;
    }
    return buffer_fds_[current_buffer_index_];
}

bool VideoStream::exportBufferFd(int buffer_index, int& dma_fd) {


    struct v4l2_exportbuffer expbuf = {};
    expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    expbuf.index = buffer_index;
    expbuf.flags = O_RDWR;
    
    if (ioctl(fd_, VIDIOC_EXPBUF, &expbuf) < 0) {

        dma_fd = -1;
        return false;
    }
    
    dma_fd = expbuf.fd;
    return true;
}

