#ifndef CAPTURE_LOOP_H
#define CAPTURE_LOOP_H

#include "app_context.h"

// 主采集循环：从 V4L2 取帧、分发到队列并周期输出统计信息。
void runCaptureLoop(AppContext& app, 
    volatile sig_atomic_t* running);

#endif // CAPTURE_LOOP_H
