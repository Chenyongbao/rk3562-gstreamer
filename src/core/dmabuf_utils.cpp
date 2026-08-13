#define _POSIX_C_SOURCE 199309L

#include "dmabuf_utils.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <pthread.h>

#ifndef DMA_HEAP_IOCTL_ALLOC
// Local copy of DMA-HEAP alloc struct/ioctl, in case headers are missing
struct dma_heap_allocation_data {
    unsigned long long len;
    unsigned int fd;
    unsigned int fd_flags;
    unsigned long long heap_flags;
};
#define DMA_HEAP_IOCTL_ALLOC _IOWR('H', 0x0, struct dma_heap_allocation_data)
#endif

static int g_dma_heap_fd = -1;
static pthread_mutex_t g_dma_mutex = PTHREAD_MUTEX_INITIALIZER;

bool dmabuf_heap_init(void) {
    pthread_mutex_lock(&g_dma_mutex);
    if (g_dma_heap_fd >= 0) {
        pthread_mutex_unlock(&g_dma_mutex);
        return true;
    }

    int fd = open("/dev/dma_heap/system", O_RDWR
#ifdef O_CLOEXEC
                  | O_CLOEXEC
#endif
    );
    if (fd >= 0) {
        fprintf(stderr, "[DMABUF] Using dma_heap: system\n");
    } else {
        fd = open("/dev/dma_heap/reserved", O_RDWR
#ifdef O_CLOEXEC
                  | O_CLOEXEC
#endif
        );
        if (fd >= 0) {
            fprintf(stderr, "[DMABUF] Using dma_heap: reserved\n");
        }
    }
    if (fd < 0) {
        fd = open("/dev/dma_heap/system-uncached", O_RDWR
#ifdef O_CLOEXEC
                  | O_CLOEXEC
#endif
        );
        if (fd >= 0) {
            fprintf(stderr, "[DMABUF] Using dma_heap: system-uncached\n");
        }
    }
    if (fd < 0) {
        fprintf(stderr, "[DMABUF] Failed to open dma_heap device: %s\n", strerror(errno));
        pthread_mutex_unlock(&g_dma_mutex);
        return false;
    }
    g_dma_heap_fd = fd;
    // Ensure close-on-exec if not set via O_CLOEXEC
    int flags = fcntl(g_dma_heap_fd, F_GETFD);
    if (flags >= 0) {
        fcntl(g_dma_heap_fd, F_SETFD, flags | FD_CLOEXEC);
    }
    pthread_mutex_unlock(&g_dma_mutex);
    return true;
}

void dmabuf_heap_deinit(void) {
    pthread_mutex_lock(&g_dma_mutex);
    if (g_dma_heap_fd >= 0) {
        close(g_dma_heap_fd);
        g_dma_heap_fd = -1;
    }
    pthread_mutex_unlock(&g_dma_mutex);
}

int dmabuf_alloc(size_t size) {
    if (!dmabuf_heap_init()) return -1;
    pthread_mutex_lock(&g_dma_mutex);
    if (g_dma_heap_fd < 0) {
        pthread_mutex_unlock(&g_dma_mutex);
        return -1;
    }
    struct dma_heap_allocation_data alloc;
    memset(&alloc, 0, sizeof(alloc));
    alloc.len = size;
    alloc.fd_flags = O_RDWR;
#ifdef O_CLOEXEC
    alloc.fd_flags |= O_CLOEXEC;
#endif
    alloc.heap_flags = 0;
    if (ioctl(g_dma_heap_fd, DMA_HEAP_IOCTL_ALLOC, &alloc) < 0) {
        fprintf(stderr, "[DMABUF] dma_heap alloc failed (size=%zu): %s\n", size, strerror(errno));
        pthread_mutex_unlock(&g_dma_mutex);
        return -1;
    }
    pthread_mutex_unlock(&g_dma_mutex);
    if ((int)alloc.fd < 0) {
        fprintf(stderr, "[DMABUF] dma_heap returned invalid fd\n");
        return -1;
    }
    // Set close-on-exec on returned fd as well
    int out_fd = (int)alloc.fd;
    int fdflags = fcntl(out_fd, F_GETFD);
    if (fdflags >= 0) {
        fcntl(out_fd, F_SETFD, fdflags | FD_CLOEXEC);
    }
    return out_fd;
}

void dmabuf_free(int fd) {
    if (fd >= 0) close(fd);
}

void* dmabuf_mmap(int fd, size_t length) {
    if (fd < 0 || length == 0) {
        return NULL;
    }
    void* addr = mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        fprintf(stderr, "[DMABUF] mmap failed (fd=%d, len=%zu): %s\n", fd, length, strerror(errno));
        return NULL;
    }
    return addr;
}

void dmabuf_munmap(void* addr, size_t length) {
    if (!addr || length == 0) {
        return;
    }
    if (munmap(addr, length) != 0) {
        fprintf(stderr, "[DMABUF] munmap failed (len=%zu): %s\n", length, strerror(errno));
    }
}
