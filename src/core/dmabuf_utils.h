#ifndef DMABUF_UTILS_H
#define DMABUF_UTILS_H

#include <stddef.h>
#include <stdbool.h>


// Initialize DMA-HEAP device (/dev/dma_heap/system or system-uncached)
bool dmabuf_heap_init(void);
// Deinitialize DMA-HEAP device
void dmabuf_heap_deinit(void);
// Allocate a DMA-BUF of given total size (in bytes). Returns fd >= 0 on success, -1 on failure.
int dmabuf_alloc(size_t size);
// Free a DMA-BUF file descriptor
void dmabuf_free(int fd);

// Map a DMA-BUF into CPU address space. Returns NULL on failure.
void* dmabuf_mmap(int fd, size_t length);
// Unmap a previously mapped DMA-BUF region.
void dmabuf_munmap(void* addr, size_t length);


#endif // DMABUF_UTILS_H
