#ifndef DMA_BUFFER_H
#define DMA_BUFFER_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <sys/mman.h>

#include "core/dmabuf_utils.h"


struct dma_buffer_t {
    int fd;
    void* ptr;
    size_t size;
    int width;
    int height;
    int stride;
};


bool init_dma_heap();
void deinit_dma_heap();


bool alloc_dma_buffer(int width, int height, int bytes_per_pixel, dma_buffer_t* buf);
void free_dma_buffer(dma_buffer_t* buf);

inline bool init_dma_heap()
{
    return dmabuf_heap_init();
}

inline void deinit_dma_heap()
{
    dmabuf_heap_deinit();
}

inline bool alloc_dma_buffer(int width, int height, int bytes_per_pixel, dma_buffer_t* buf)
{
    if (!buf || width <= 0 || height <= 0 || bytes_per_pixel <= 0) {
        return false;
    }
    if (!init_dma_heap()) {
        return false;
    }

    std::memset(buf, 0, sizeof(dma_buffer_t));
    buf->fd = -1;
    buf->width = width;
    buf->height = height;
    buf->stride = width * bytes_per_pixel;
    buf->size = static_cast<size_t>(buf->stride) * static_cast<size_t>(height);

    buf->fd = dmabuf_alloc(buf->size);
    if (buf->fd < 0) {
        return false;
    }

    buf->ptr = dmabuf_mmap(buf->fd, buf->size);
    if (!buf->ptr) {
        dmabuf_free(buf->fd);
        buf->fd = -1;
        return false;
    }

    return true;
}

inline void free_dma_buffer(dma_buffer_t* buf)
{
    if (!buf) {
        return;
    }

    const int fd = buf->fd;
    const void* ptr = buf->ptr;
    const size_t size = buf->size;

    buf->fd = -1;
    buf->ptr = nullptr;
    buf->size = 0;
    buf->width = 0;
    buf->height = 0;
    buf->stride = 0;

    if (ptr && ptr != MAP_FAILED) {
        dmabuf_munmap(const_cast<void*>(ptr), size);
    }
    if (fd >= 0) {
        dmabuf_free(fd);
    }
}

#endif // DMA_BUFFER_H




