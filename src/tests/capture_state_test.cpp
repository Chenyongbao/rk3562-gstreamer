#include <gtest/gtest.h>

#include "app/capture_state.h"

TEST(CaptureLoopStateTest, InitZeroesCount) {
    CaptureLoopState s;
    s.bev_refresh_request_count = 42;
    initCaptureLoopState(&s);
    EXPECT_EQ(getPendingBevRefreshRequestCount(&s), 0);
}

TEST(CaptureLoopStateTest, RequestIncrements) {
    CaptureLoopState s;
    initCaptureLoopState(&s);
    requestBevRefresh(&s);
    requestBevRefresh(&s);
    EXPECT_EQ(getPendingBevRefreshRequestCount(&s), 2);
}

TEST(CaptureLoopStateTest, ConsumeDecrementsWithoutGoingNegative) {
    CaptureLoopState s;
    initCaptureLoopState(&s);

    consumeBevRefreshRequest(&s);  // 0 -> 保持 0
    EXPECT_EQ(getPendingBevRefreshRequestCount(&s), 0);

    requestBevRefresh(&s);
    requestBevRefresh(&s);
    consumeBevRefreshRequest(&s);
    EXPECT_EQ(getPendingBevRefreshRequestCount(&s), 1);
}

TEST(CaptureLoopStateTest, NullSafety) {
    requestBevRefresh(nullptr);
    consumeBevRefreshRequest(nullptr);
    EXPECT_EQ(getPendingBevRefreshRequestCount(nullptr), 0);
}
