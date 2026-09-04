#include "pipeline/common/frame_provider.h"

#include <chrono>

// 生产者：发布一帧（内部拷贝，返回后调用方可复用入参内存）。
// 锁外把帧数据构造进一个不可变块，锁内仅 O(1) 换指针 + 更新元数据，
// 从而不把"拷贝一整帧"放进持锁窗口。代价：每帧一次堆分配。
void FrameProvider::publish(const uint8_t *nv12, size_t size, uint64_t frame_id,
                            uint32_t width, uint32_t height, uint32_t stride) {
  if (!nv12 || size == 0) {
    return;
  }
  // 锁外构造新一帧：深拷贝进独立不可变块（此刻不持锁，不阻塞消费者）。
  // 注：closed_ 在锁内才检查，极端情况下（生产者与 clear 并发）会白做这一次
  // 拷贝后丢弃——可接受，避免为省一次关闭期拷贝而引入双重加锁。
  auto data = std::make_shared<const std::vector<uint8_t>>(nv12, nv12 + size);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) { // 检查在锁内
      return;
    }
    frame_ = std::move(data); // 原子换指针，旧帧由仍持有旧 shared_ptr 者释放
    frame_id_ = frame_id;
    width_ = width;
    height_ = height;
    stride_ = stride;
    has_frame_ = true;
  }
  // 解锁后再通知，避免被唤醒者立刻阻塞在锁上（惊群）。
  cv_.notify_all();
}

// 生产者查询：是否有消费者阻塞等待新帧。
bool FrameProvider::hasWaiter() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return waiter_count_ > 0;
}

// 消费者：取最新帧。已有帧立即返回（不等待、不触发新采集）；
// 无帧则登记为等待者并阻塞，直到生产者 publish 一帧或超时。
bool FrameProvider::grab(Snapshot &out, const GrabParams *params,
                         int timeout_ms) {
  (void)params; // 预留参数，当前忽略
  if (timeout_ms < 0) {
    timeout_ms = 0;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  // 1. 系统关闭检查：若系统正在退出（如 clear() 被调用），直接退出，绝不傻等
  if (closed_) {
    return false;
  }
  FrameRef ref;
  if (has_frame_) {
    // 2. 快路径（Cache Hit）：已有最新帧，锁内 O(1) 捕获引用即可
    ref = captureLocked();
  } else {
    // 3. 慢路径（Waiting）：当前没有缓存帧，登记为等待者并阻塞，直到生产者
    // publish 一帧或超时
    ++waiter_count_;
    // 4. 阻塞等待：使用条件变量挂起当前线程，等待生产者 publish 唤醒或超时
    const bool ready = cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                    [&] { return closed_ || has_frame_; });
    // 唤醒后立刻归还计数
    --waiter_count_;
    // 5. 校验结果：超时、退出或仍无帧则失败
    if (!ready || closed_ || !has_frame_) {
      return false;
    }
    ref = captureLocked();
  }
  // 6. 先解锁再做像素深拷贝：拷贝窗口（memcpy 量级）不再串行化生产者 /
  // 其他消费者；ref 持有 shared_ptr，块在拷贝期间不会被回收。
  lock.unlock();
  return materialize(ref, out);
}

// 消费者：阻塞等待"比 min_frame_id 更新"的帧。
// 带谓词的 wait_for 同时解决假唤醒与"通知丢失"竞态（进入 wait
// 前先检查一次谓词）。 谓词加入 closed_：关闭后等待者立即被唤醒并返回
// false，而非傻等超时。
bool FrameProvider::grabNewerThan(uint64_t min_frame_id, Snapshot &out,
                                  const GrabParams *params, int timeout_ms) {
  (void)params;
  if (timeout_ms < 0) {
    timeout_ms = 0;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  if (closed_) {
    return false;
  }
  ++waiter_count_;
  const bool ready =
      cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] {
        return closed_ || (has_frame_ && frame_id_ > min_frame_id);
      });
  --waiter_count_;
  if (!ready || closed_ || !has_frame_ || frame_id_ <= min_frame_id) {
    return false;
  }
  FrameRef ref = captureLocked();
  lock.unlock();
  return materialize(ref, out);
}

// 获取最新一帧图像的 ID（线程安全）
uint64_t FrameProvider::latestFrameId() const {
  std::lock_guard<std::mutex> lock(
      mutex_); // 加锁，防止读取时被其他写入线程打断
  return frame_id_;
}

// 检查当前缓存中是否已有可用的图像帧（线程安全）
bool FrameProvider::hasFrame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_frame_;
}

// 关闭并清空：置关闭标志、释放内存、唤醒所有等待线程，让它们立即返回 false
// 后退出。
void FrameProvider::clear() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_.reset(); // 释放当前帧（消费者若仍持有旧 shared_ptr，其块由其释放）
    frame_id_ = 0;
    width_ = 0;
    height_ = 0;
    stride_ = 0;
    has_frame_ = false;
    closed_ = true;
  }
  cv_.notify_all();
}

// 前置条件：调用方已持有 mutex_。
// O(1) 捕获当前帧的共享指针与元数据（不拷贝像素），供锁外 materialize 使用。
FrameProvider::FrameRef FrameProvider::captureLocked() {
  FrameRef ref;
  ref.data = frame_;
  ref.id = frame_id_;
  ref.width = width_;
  ref.height = height_;
  ref.stride = stride_;
  return ref;
}

// 锁外执行：把引用里的像素深拷贝进 out（值语义，用完即弃）。
// ref.data 的 shared_ptr 保证块在拷贝期间不会被生产者 / clear 回收。
bool FrameProvider::materialize(const FrameRef &ref, Snapshot &out) {
  if (!ref.data) {
    return false;
  }
  out.nv12.assign(ref.data->begin(), ref.data->end());
  out.frame_id = ref.id;
  out.width = ref.width;
  out.height = ref.height;
  out.stride = ref.stride;
  return true;
}
