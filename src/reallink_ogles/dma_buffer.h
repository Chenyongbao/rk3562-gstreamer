#ifndef DMA_BUFFER_H
#define DMA_BUFFER_H

#include <cstddef>
#include <cstdint>


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

#endif // DMA_BUFFER_H




