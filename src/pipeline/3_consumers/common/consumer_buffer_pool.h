#ifndef CONSUMER_BUFFER_POOL_H
#define CONSUMER_BUFFER_POOL_H

#include <stddef.h>
#include <vector>

#include "core/video_frame_types.h"

// 维护一组可重复借还的消费者私有缓冲，用于 DMABUF 拷贝场景。
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
    // `entries_` 持有真实占用状态，`snapshot_` 仅供只读枚举和释放时遍历。
    struct PoolEntry {
        ConsumerBuffer buffer;
        bool in_use = false;
    };

    std::vector<PoolEntry> entries_;
    std::vector<ConsumerBuffer> snapshot_;
};

#endif // CONSUMER_BUFFER_POOL_H
