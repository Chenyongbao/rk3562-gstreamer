#ifndef FRAME_QUEUE_H
#define FRAME_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>


#define FRAME_QUEUE_SIZE 32


typedef struct {

    int dmabuf_fd;
    int v4l2_index;
    int width;
    int height;
    int stride;
    

    uint8_t* buffer;
    size_t size;
    

    uint64_t frame_idx;
    bool valid;
    bool is_dmabuf_mode;
    bool force_process;
} FrameSlot;


typedef struct {
    FrameSlot slots[FRAME_QUEUE_SIZE];
    int write_idx;
    int read_idx;
    int count;
    size_t buffer_size;
    
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
    pthread_cond_t not_full;
    
    bool shutdown;
    

    uint64_t total_pushed;
    uint64_t total_popped;
    uint64_t dropped_frames;
} FrameQueue;


bool frame_queue_init(FrameQueue* queue, size_t buffer_size);


bool frame_queue_push_dmabuf(FrameQueue* queue, 
                             int dmabuf_fd, int v4l2_index,
                             int width, int height, int stride,
                             uint64_t frame_idx,
                             void* v4l2_cam);


bool frame_queue_push(FrameQueue* queue,
                      const uint8_t* data,
                      size_t size,
                      uint64_t frame_idx,
                      bool force_process = false);


FrameSlot* frame_queue_pop_dmabuf(FrameQueue* queue, uint64_t* frame_idx);


int frame_queue_count(FrameQueue* queue);


void frame_queue_shutdown(FrameQueue* queue);


void frame_queue_cleanup(FrameQueue* queue);

#endif // FRAME_QUEUE_H

