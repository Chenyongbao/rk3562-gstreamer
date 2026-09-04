#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "pipeline/common/frame_provider.h"

namespace {

std::vector<uint8_t> MakeFrame(uint8_t value, size_t size) {
    return std::vector<uint8_t>(size, value);
}

// 轮询等待条件成立（最多 500ms），避免线程调度抖动导致偶发失败。
void WaitUntil(const std::function<bool()>& cond, int timeout_ms = 500) {
    for (int elapsed = 0; elapsed < timeout_ms && !cond(); elapsed += 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

}  // namespace

// publish 后 grab 能取回同一份数据（值拷贝 round-trip）。
TEST(FrameProviderTest, PublishThenGrabReturnsFrame) {
    FrameProvider provider;
    const size_t size = 4096;
    auto data = MakeFrame(0xAB, size);

    provider.publish(data.data(), size, 1, 1280, 960, 1280);

    Snapshot snap;
    ASSERT_TRUE(provider.grab(snap, nullptr, 100));
    EXPECT_EQ(snap.frame_id, 1u);
    EXPECT_EQ(snap.width, 1280u);
    EXPECT_EQ(snap.height, 960u);
    EXPECT_EQ(snap.stride, 1280u);
    ASSERT_EQ(snap.nv12.size(), size);
    EXPECT_EQ(snap.nv12, data);
}

// 没有帧时 grab 阻塞，直到生产者 publish 唤醒。
TEST(FrameProviderTest, GrabBlocksUntilPublished) {
    FrameProvider provider;
    std::atomic<bool> grabbed{false};
    Snapshot snap;

    std::thread consumer([&] {
        grabbed = provider.grab(snap, nullptr, 1000);
    });

    WaitUntil([&] { return provider.hasWaiter(); });
    auto data = MakeFrame(0x11, 1024);
    provider.publish(data.data(), data.size(), 7, 64, 64, 64);

    consumer.join();
    EXPECT_TRUE(grabbed);
    EXPECT_EQ(snap.frame_id, 7u);
    EXPECT_EQ(snap.nv12, data);
}

// grab 超时且无帧时返回 false。
TEST(FrameProviderTest, GrabTimesOutWithoutFrame) {
    FrameProvider provider;
    Snapshot snap;
    EXPECT_FALSE(provider.grab(snap, nullptr, 50));
}

// hasWaiter 反映是否有消费者在阻塞等待。
TEST(FrameProviderTest, HasWaiterReflectsBlockedConsumers) {
    FrameProvider provider;
    EXPECT_FALSE(provider.hasWaiter());

    std::thread consumer([&] {
        Snapshot snap;
        provider.grab(snap, nullptr, 500);
    });

    WaitUntil([&] { return provider.hasWaiter(); });
    EXPECT_TRUE(provider.hasWaiter());

    auto data = MakeFrame(0x22, 512);
    provider.publish(data.data(), data.size(), 1, 1, 1, 1);
    consumer.join();
}

// grabNewerThan 要求 frame_id 严格更大，旧帧不满足。
TEST(FrameProviderTest, GrabNewerThanRequiresNewerFrameId) {
    FrameProvider provider;
    auto d1 = MakeFrame(0x01, 256);
    provider.publish(d1.data(), d1.size(), 100, 1, 1, 1);

    Snapshot snap;
    EXPECT_FALSE(provider.grabNewerThan(100, snap, nullptr, 50));

    std::thread producer([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        auto d2 = MakeFrame(0x02, 256);
        provider.publish(d2.data(), d2.size(), 101, 1, 1, 1);
    });

    EXPECT_TRUE(provider.grabNewerThan(100, snap, nullptr, 1000));
    producer.join();
    EXPECT_EQ(snap.frame_id, 101u);
}

// 值拷贝隔离：publish 新帧后，之前 grab 到的 Snapshot 不被覆盖。
TEST(FrameProviderTest, SnapshotIsolatedFromLaterPublish) {
    FrameProvider provider;
    auto d1 = MakeFrame(0xAA, 2048);
    provider.publish(d1.data(), d1.size(), 1, 1, 1, 1);

    Snapshot first;
    ASSERT_TRUE(provider.grab(first, nullptr, 100));

    auto d2 = MakeFrame(0xBB, 2048);
    provider.publish(d2.data(), d2.size(), 2, 1, 1, 1);

    EXPECT_EQ(first.nv12, d1);
    EXPECT_EQ(first.frame_id, 1u);
}

// clear 唤醒所有阻塞的 grab 并让其返回 false。
TEST(FrameProviderTest, ClearWakesBlockedGrabbers) {
    FrameProvider provider;
    std::atomic<bool> grabbed{false};

    std::thread consumer([&] {
        Snapshot snap;
        grabbed = provider.grab(snap, nullptr, 5000);
    });

    WaitUntil([&] { return provider.hasWaiter(); });
    provider.clear();
    consumer.join();

    EXPECT_FALSE(grabbed);
}

// clear 之后 publish 被拒绝、grab 直接返回 false。
TEST(FrameProviderTest, ClosedProviderRejectsPublishAndGrab) {
    FrameProvider provider;
    provider.clear();

    auto data = MakeFrame(0x33, 128);
    provider.publish(data.data(), data.size(), 1, 1, 1, 1);

    Snapshot snap;
    EXPECT_FALSE(provider.hasFrame());
    EXPECT_FALSE(provider.grab(snap, nullptr, 50));
}

// 连续 publish 后 grab 拿到的是最新帧。
TEST(FrameProviderTest, LatestPublishWins) {
    FrameProvider provider;
    auto d1 = MakeFrame(0x10, 1024);
    auto d2 = MakeFrame(0x20, 1024);
    provider.publish(d1.data(), d1.size(), 1, 1, 1, 1);
    provider.publish(d2.data(), d2.size(), 2, 1, 1, 1);

    Snapshot snap;
    ASSERT_TRUE(provider.grab(snap, nullptr, 100));
    EXPECT_EQ(snap.nv12, d2);
    EXPECT_EQ(snap.frame_id, 2u);
    EXPECT_EQ(provider.latestFrameId(), 2u);
}
