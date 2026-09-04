#ifndef FRAME_PROVIDER_H
#define FRAME_PROVIDER_H

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

// 一帧快照：值语义，出作用域自动释放内存（RAII 实现"拿到内存、用完即弃"）。
// 相比旧设计（LatestNv12FrameBuffer 的 copy(dst, dst_size, out_size,
// out_frame_id)） 不再要求调用方手动 malloc 一块 dst 缓冲区并传四个指针。
struct Snapshot {
  std::vector<uint8_t> nv12; // NV12 帧数据（Y 平面 + 交错 UV）
  uint64_t frame_id = 0;     // 单调递增帧序号，用于判断"新旧"
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t stride = 0; // Y 平面 bytesperline（可能 > width）
};

// 抓帧参数（预留能力：指定分辨率 / ROI 区域）。
// 字段全 0 表示"原始整帧"；缩放/裁剪待接入 RGA 后实现。
struct GrabParams {
  uint32_t width = 0;  // 目标宽度，0 = 原始
  uint32_t height = 0; // 目标高度，0 = 原始
  uint32_t roi_x = 0;  // ROI 左上角 x
  uint32_t roi_y = 0;  // ROI 左上角 y
  uint32_t roi_w = 0;  // ROI 宽，0 = 整帧
  uint32_t roi_h = 0;  // ROI 高，0 = 整帧
};

// 取帧服务：跨线程的"按需取帧"数据源。
//
// 职责单一化：它只负责"保存最新一帧 + 让消费者拿到一帧拥有内存的快照"，
// 并把"刷新请求/是否有消费者在等"这个状态封装在对象内部（旧设计散落在
// capture_state.h 的原子标志 + runSnapshotLoop + LatestNv12FrameBuffer 三处）。
//
// 协作模型（按需拷贝）：
//   生产者（快照循环）每帧先调 hasWaiter() 判断是否有消费者在等；
//   只有有人等时才做昂贵的 map + memcpy（避免每帧无谓拷贝 19MB 原始帧）。
//   消费者 grab()/grabNewerThan() 阻塞等待，被 publish 唤醒。
//
// 线程安全：内部 mutex 保护帧数据与等待计数，condition_variable 用于阻塞等待。
class FrameProvider {
public:
  FrameProvider() = default;
  FrameProvider(const FrameProvider &) = delete;
  FrameProvider &operator=(const FrameProvider &) = delete;

  // 生产者：发布一帧（内部拷贝，返回后调用方可复用入参内存）。
  // 锁外构造新一帧、锁内仅 O(1) 换指针 + 更新元数据，避免持锁做 memcpy。
  // 代价：每帧一次堆分配（shared_ptr 换指针模型）。
  void publish(const uint8_t *nv12, size_t size, uint64_t frame_id,
               uint32_t width = 0, uint32_t height = 0, uint32_t stride = 0);

  // 生产者查询：是否有消费者阻塞等待新帧。
  // 供生产者决定是否值得做昂贵的 map+memcpy（按需拷贝的核心）。
  bool hasWaiter() const;

  // 消费者：取最新帧。若已有帧立即返回；无帧则请求新帧并等待（带超时）。
  bool grab(Snapshot &out, const GrabParams *params = nullptr,
            int timeout_ms = 500);

  // 消费者：阻塞等待 frame_id > min_frame_id 的新帧，等到后拷贝返回。
  // timeout_ms < 0 视为 0（不等待）。
  bool grabNewerThan(uint64_t min_frame_id, Snapshot &out,
                     const GrabParams *params = nullptr, int timeout_ms = 500);

  // 状态查询（均加锁，线程安全）。
  uint64_t latestFrameId() const;
  bool hasFrame() const;

  // 关闭并清空：置关闭标志、释放内存、唤醒所有等待线程。
  // 被唤醒的 grab/grabNewerThan 会立即返回
  // false（而非傻等超时）。关闭后不可复用。
  void clear();

private:
  // 锁内 O(1) 捕获的帧引用：只持有共享指针 + 元数据，不含像素拷贝。
  // 真正的深拷贝（memcpy 量级）在锁外执行，避免拷贝窗口串行化生产者/其他消费者。
  struct FrameRef {
    std::shared_ptr<const std::vector<uint8_t>> data;
    uint64_t id = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
  };

  // 调用方须持有 mutex_：O(1) 捕获当前帧引用与元数据（不拷贝像素）。
  FrameRef captureLocked();

  // 锁外执行：把引用里的像素深拷贝进 out（值语义，用完即弃）。
  static bool materialize(const FrameRef &ref, Snapshot &out);

  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  // 最新一帧（不可变，shared_ptr 换指针）：生产者锁外构造新块、锁内原子换指针；
  // 消费者锁内只拷走指针（ns 级），深拷贝在锁外。代价：生产者每帧一次堆分配。
  std::shared_ptr<const std::vector<uint8_t>> frame_;
  uint64_t frame_id_ = 0;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t stride_ = 0;
  bool has_frame_ = false; // 是否有帧
  bool closed_ = false;    // 关闭标志：置位后所有等待立即返回 false
  int waiter_count_ = 0;   // 阻塞等待的消费者数量（生产者据此决定是否拷贝）
};

#endif // FRAME_PROVIDER_H
