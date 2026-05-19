#include "consumer_buffer_pool.h"

bool ConsumerBufferPool::add_buffer(const ConsumerBuffer& buffer)
{
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
    for (const PoolEntry& entry : entries_) {
        if (!entry.in_use) {
            ++count;
        }
    }
    return count;
}
