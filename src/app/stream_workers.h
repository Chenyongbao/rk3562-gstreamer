#ifndef STREAM_WORKERS_H
#define STREAM_WORKERS_H

#include "app_context.h"

// 启动原图与 BEV 两个工作线程。
bool startStreamWorkers(AppContext& app);
// 通知线程退出并等待回收。
void stopStreamWorkers(AppContext& app);

#endif // STREAM_WORKERS_H
