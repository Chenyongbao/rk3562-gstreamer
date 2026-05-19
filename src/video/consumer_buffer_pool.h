#ifndef CONSUMER_BUFFER_POOL_H
#define CONSUMER_BUFFER_POOL_H

#include <stddef.h>
#include <vector>

#include "video_frame_types.h"

class ConsumerBufferPool {
public:
    bool add_buffer(const ConsumerBuffer& buffer);
    bool acquire_buffer(ConsumerBuffer* out);
    void release_buffer(int dmabuf_fd);
    const std::vector<ConsumerBuffer>& buffers() const;
    void clear();

    size_t total_count() const;
    size_t free_count() const;

private:
    struct PoolEntry {
        ConsumerBuffer buffer;
        bool in_use = false;
    };

    std::vector<PoolEntry> entries_;
    std::vector<ConsumerBuffer> snapshot_;
};

#endif // CONSUMER_BUFFER_POOL_H
