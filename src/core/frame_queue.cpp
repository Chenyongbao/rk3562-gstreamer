#include "frame_queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void frame_slot_release(FrameSlot* slot)
{
    if (!slot || !slot->is_dmabuf_mode) {
        return;
    }

    if (slot->release_cb) {
        slot->release_cb(slot->release_user_data, slot->dmabuf_fd);
    }

    slot->release_cb = NULL;
    slot->release_user_data = NULL;
}

bool frame_queue_init(FrameQueue* queue, size_t buffer_size) {
    if (!queue || buffer_size == 0) {
        return false;
    }
    
    memset(queue, 0, sizeof(FrameQueue));
    queue->buffer_size = buffer_size;
    

    for (int i = 0; i < FRAME_QUEUE_SIZE; i++) {
        queue->slots[i].buffer = (uint8_t*)malloc(buffer_size);
        if (!queue->slots[i].buffer) {

            for (int j = 0; j < i; j++) {
                free(queue->slots[j].buffer);
            }
            return false;
        }
        queue->slots[i].dmabuf_fd = -1;
        queue->slots[i].width = 0;
        queue->slots[i].height = 0;
        queue->slots[i].stride = 0;
        queue->slots[i].valid = false;
        queue->slots[i].size = 0;
        queue->slots[i].frame_idx = 0;
        queue->slots[i].is_dmabuf_mode = false;
        queue->slots[i].release_cb = NULL;
        queue->slots[i].release_user_data = NULL;
    }
    
    queue->write_idx = 0;
    queue->read_idx = 0;
    queue->count = 0;
    queue->shutdown = false;
    

    pthread_mutex_init(&queue->mutex, NULL);
    pthread_cond_init(&queue->not_empty, NULL);
    pthread_cond_init(&queue->not_full, NULL);
    
    fprintf(stderr, "[FrameQueue] Initialized: buffer_size=%zu, slots=%d\n", 
            buffer_size, FRAME_QUEUE_SIZE);
    
    return true;
}


bool frame_queue_push_dmabuf(FrameQueue* queue, 
                             int dmabuf_fd,
                             int width, int height, int stride,
                             size_t size,
                             uint64_t frame_idx) {
    return frame_queue_push_dmabuf_ex(queue,
                                      dmabuf_fd,
                                      width,
                                      height,
                                      stride,
                                      size,
                                      frame_idx,
                                      NULL,
                                      NULL);
}

bool frame_queue_push_dmabuf_ex(FrameQueue* queue,
                                int dmabuf_fd,
                                int width, int height, int stride,
                                size_t size,
                                uint64_t frame_idx,
                                FrameSlotReleaseCallback release_cb,
                                void* release_user_data) {
    if (!queue || dmabuf_fd < 0 || size == 0) {
        return false;
    }
    
    pthread_mutex_lock(&queue->mutex);
    

    if (queue->count >= FRAME_QUEUE_SIZE) {
        FrameSlot* old_slot = &queue->slots[queue->read_idx];
        frame_slot_release(old_slot);
        
        queue->read_idx = (queue->read_idx + 1) % FRAME_QUEUE_SIZE;
        queue->count--;
        queue->dropped_frames++;
        

        if (queue->dropped_frames % 100 == 0) {
            fprintf(stderr, "[FrameQueue] WARNING: Dropped %llu frames (queue full)\n",
                    (unsigned long long)queue->dropped_frames);
        }
    }
    

    FrameSlot* slot = &queue->slots[queue->write_idx];
    slot->dmabuf_fd = dmabuf_fd;
    slot->width = width;
    slot->height = height;
    slot->stride = stride;
    slot->size = size;
    slot->frame_idx = frame_idx;
    slot->valid = true;
    slot->is_dmabuf_mode = true;
    slot->release_cb = release_cb;
    slot->release_user_data = release_user_data;
    
    queue->write_idx = (queue->write_idx + 1) % FRAME_QUEUE_SIZE;
    queue->count++;
    queue->total_pushed++;
    

    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);
    
    return true;
}


bool frame_queue_push(FrameQueue* queue,
                      const uint8_t* data,
                      size_t size,
                      uint64_t frame_idx) {
    if (!queue || !data || size == 0) {
        return false;
    }
    
    if (size > queue->buffer_size) {
        fprintf(stderr, "[FrameQueue] ERROR: Frame size %zu > buffer size %zu\n", 
                size, queue->buffer_size);
        return false;
    }
    
    pthread_mutex_lock(&queue->mutex);
    

    if (queue->count >= FRAME_QUEUE_SIZE) {
        FrameSlot* old_slot = &queue->slots[queue->read_idx];
        frame_slot_release(old_slot);
        queue->read_idx = (queue->read_idx + 1) % FRAME_QUEUE_SIZE;
        queue->count--;
        queue->dropped_frames++;
        

        if (queue->dropped_frames % 100 == 0) {
            fprintf(stderr, "[FrameQueue] WARNING: Dropped %llu frames (queue full)\n",
                    (unsigned long long)queue->dropped_frames);
        }
    }
    

    FrameSlot* slot = &queue->slots[queue->write_idx];
    memcpy(slot->buffer, data, size);
    slot->dmabuf_fd = -1;
    slot->width = 0;
    slot->height = 0;
    slot->stride = 0;
    slot->size = size;
    slot->frame_idx = frame_idx;
    slot->valid = true;
    slot->is_dmabuf_mode = false;
    slot->release_cb = NULL;
    slot->release_user_data = NULL;
    
    queue->write_idx = (queue->write_idx + 1) % FRAME_QUEUE_SIZE;
    queue->count++;
    queue->total_pushed++;
    

    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);
    
    return true;
}


FrameSlot* frame_queue_pop_dmabuf(FrameQueue* queue, uint64_t* frame_idx) {
    if (!queue) {
        return NULL;
    }
    
    pthread_mutex_lock(&queue->mutex);
    

    while (queue->count == 0 && !queue->shutdown) {
        pthread_cond_wait(&queue->not_empty, &queue->mutex);
    }
    

    if (queue->shutdown && queue->count == 0) {
        pthread_mutex_unlock(&queue->mutex);
        return NULL;
    }
    

    FrameSlot* slot = &queue->slots[queue->read_idx];
    if (frame_idx) *frame_idx = slot->frame_idx;
    
    queue->read_idx = (queue->read_idx + 1) % FRAME_QUEUE_SIZE;
    queue->count--;
    queue->total_popped++;
    
    pthread_cond_signal(&queue->not_full);
    pthread_mutex_unlock(&queue->mutex);
    
    return slot;
}

bool frame_queue_pop_copy(FrameQueue* queue, FrameSlot* out_slot, uint64_t* frame_idx) {
    if (!queue || !out_slot) {
        return false;
    }

    pthread_mutex_lock(&queue->mutex);

    while (queue->count == 0 && !queue->shutdown) {
        pthread_cond_wait(&queue->not_empty, &queue->mutex);
    }

    if (queue->shutdown && queue->count == 0) {
        pthread_mutex_unlock(&queue->mutex);
        return false;
    }

    FrameSlot* slot = &queue->slots[queue->read_idx];
    *out_slot = *slot;
    if (frame_idx) {
        *frame_idx = slot->frame_idx;
    }
    slot->dmabuf_fd = -1;
    slot->width = 0;
    slot->height = 0;
    slot->stride = 0;
    slot->size = 0;
    slot->frame_idx = 0;
    slot->valid = false;
    slot->is_dmabuf_mode = false;
    slot->release_cb = NULL;
    slot->release_user_data = NULL;

    queue->read_idx = (queue->read_idx + 1) % FRAME_QUEUE_SIZE;
    queue->count--;
    queue->total_popped++;

    pthread_cond_signal(&queue->not_full);
    pthread_mutex_unlock(&queue->mutex);
    return true;
}

int frame_queue_count(FrameQueue* queue) {
    if (!queue) {
        return 0;
    }
    
    pthread_mutex_lock(&queue->mutex);
    int count = queue->count;
    pthread_mutex_unlock(&queue->mutex);
    
    return count;
}

void frame_queue_shutdown(FrameQueue* queue) {
    if (!queue) {
        return;
    }
    
    pthread_mutex_lock(&queue->mutex);
    queue->shutdown = true;
    

    pthread_cond_broadcast(&queue->not_empty);
    pthread_cond_broadcast(&queue->not_full);
    pthread_mutex_unlock(&queue->mutex);
    
    fprintf(stderr, "[FrameQueue] Shutdown: pushed=%llu, popped=%llu, dropped=%llu\n",
            (unsigned long long)queue->total_pushed,
            (unsigned long long)queue->total_popped,
            (unsigned long long)queue->dropped_frames);
}

void frame_queue_cleanup(FrameQueue* queue) {
    if (!queue) {
        return;
    }
    

    frame_queue_shutdown(queue);

    for (int i = 0; i < FRAME_QUEUE_SIZE; i++) {
        frame_slot_release(&queue->slots[i]);
    }
    

    for (int i = 0; i < FRAME_QUEUE_SIZE; i++) {
        if (queue->slots[i].buffer) {
            free(queue->slots[i].buffer);
            queue->slots[i].buffer = NULL;
        }
        queue->slots[i].valid = false;
        queue->slots[i].dmabuf_fd = -1;
    }
    

    pthread_mutex_destroy(&queue->mutex);
    pthread_cond_destroy(&queue->not_empty);
    pthread_cond_destroy(&queue->not_full);
    
    fprintf(stderr, "[FrameQueue] Cleaned up\n");
}

