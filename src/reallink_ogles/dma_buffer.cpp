#include "dma_buffer.h"
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstdio>


static int dma_heap_fd = -1;


#ifndef DMA_HEAP_IOCTL_ALLOC
struct dma_heap_allocation_data {
    __u64 len;
    __u32 fd;
    __u32 fd_flags;
    __u64 heap_flags;
};
#define DMA_HEAP_IOCTL_ALLOC _IOWR('H', 0x0, struct dma_heap_allocation_data)
#endif

bool init_dma_heap() {
    if (dma_heap_fd >= 0) return true;
    
    dma_heap_fd = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
    if (dma_heap_fd < 0) {
        dma_heap_fd = open("/dev/dma_heap/system-uncached", O_RDWR | O_CLOEXEC);
        if (dma_heap_fd >= 0) {
            fprintf(stderr, "  DMA heap: using system-uncached\n");
        }
    } else {
        fprintf(stderr, "  DMA heap: using system\n");
    }
    if (dma_heap_fd < 0) {
        fprintf(stderr, "  Error: Failed to open DMA heap device: %s\n", strerror(errno));
        return false;
    }
    return true;
}

void deinit_dma_heap() {
    if (dma_heap_fd >= 0) {
        close(dma_heap_fd);
        dma_heap_fd = -1;
    }
}

bool alloc_dma_buffer(int width, int height, int bytes_per_pixel, dma_buffer_t* buf) {
    if (!init_dma_heap()) {
        fprintf(stderr, "  Error: DMA heap not initialized\n");
        return false;
    }
    
    memset(buf, 0, sizeof(dma_buffer_t));
    buf->fd = -1;
    buf->width = width;
    buf->height = height;
    buf->stride = width * bytes_per_pixel;
    buf->size = buf->stride * height;
    

    struct dma_heap_allocation_data alloc_data;
    memset(&alloc_data, 0, sizeof(alloc_data));
    alloc_data.len = buf->size;
    alloc_data.fd_flags = O_RDWR | O_CLOEXEC;
    alloc_data.heap_flags = 0;
    
    if (ioctl(dma_heap_fd, DMA_HEAP_IOCTL_ALLOC, &alloc_data) < 0) {
        fprintf(stderr, "  Error: DMA heap ioctl failed (size=%zu): %s\n", buf->size, strerror(errno));
        return false;
    }
    
    if (alloc_data.fd < 0) {
        fprintf(stderr, "  Error: DMA heap ioctl returned invalid fd\n");
        return false;
    }
    
    buf->fd = alloc_data.fd;
    

    buf->ptr = mmap(NULL, buf->size, PROT_READ | PROT_WRITE, MAP_SHARED, buf->fd, 0);
    if (buf->ptr == MAP_FAILED) {
        fprintf(stderr, "  Error: mmap DMA buffer failed: %s\n", strerror(errno));
        close(buf->fd);
        return false;
    }
    
    return true;
}

void free_dma_buffer(dma_buffer_t* buf) {
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
        munmap(const_cast<void*>(ptr), size);
    }
    if (fd >= 0) {
        close(fd);
    }
}




