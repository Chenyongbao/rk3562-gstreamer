#include "pipeline/3_consumers/common/consumer_buffer_pool.h"

bool ConsumerBufferPool::add_buffer(const ConsumerBuffer& buffer)
{
    // 仅接受有效的 DMABUF 句柄，避免把空缓冲放进池中。
    if (buffer.dmabuf_fd < 0) {
        return false;
    }

    PoolEntry entry{};
    entry.buffer = buffer;
    entries_.push_back(entry);
    snapshot_.push_back(buffer);
    return true;
}

bool ConsumerBufferPool::acquire_buffer(ConsumerBuffer* out)
{
    if (!out) {
        return false;
    }

    // 顺序查找一个空闲槽位，命中后标记占用并返回其元数据。
    for (PoolEntry& entry : entries_) {
        if (entry.in_use) {
            continue;
        }
        entry.in_use = true;
        *out = entry.buffer;
        return true;
    }
    return false;
}

void ConsumerBufferPool::release_buffer(int dmabuf_fd)
{
    // 通过 DMABUF fd 归还对应槽位，便于回调释放时无须保存索引。
    for (PoolEntry& entry : entries_) {
        if (entry.buffer.dmabuf_fd != dmabuf_fd) {
            continue;
        }
        entry.in_use = false;
        return;
    }
}

const std::vector<ConsumerBuffer>& ConsumerBufferPool::buffers() const
{
    return snapshot_;
}

void ConsumerBufferPool::clear()
{
    entries_.clear();
    snapshot_.clear();
}

size_t ConsumerBufferPool::total_count() const
{
    return entries_.size();
}

size_t ConsumerBufferPool::free_count() const
{
    size_t count = 0;
    // 仅统计当前未借出的缓冲数量，供上层观察池水位。
    for (const PoolEntry& entry : entries_) {
        if (!entry.in_use) {
            ++count;
        }
    }
    return count;
}
