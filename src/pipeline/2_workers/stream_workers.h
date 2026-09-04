#ifndef STREAM_WORKERS_H
#define STREAM_WORKERS_H

#include "app/app_context.h"

// 启动原画与 BEV 两个后台工作线程。
bool startStreamWorkers(AppContext& app);
// 通知线程退出并等待回收。
void stopStreamWorkers(AppContext& app);

// 主线程运行的快照循环：从 snapshot 分支拉帧，按需刷新主相机最新帧缓存。
void runSnapshotLoop(AppContext& app, volatile sig_atomic_t* running);

#endif // STREAM_WORKERS_H
