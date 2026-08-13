#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include <signal.h>

#include "app/app_context.h"

// 应用运行时封装：负责初始化、运行主循环和统一清理。
class AppRuntime {
public:
    // running_flag 由外部维护（通常是全局信号标记）。
    explicit AppRuntime(volatile sig_atomic_t* running_flag);

    // 启动完整业务流程，返回进程退出码。
    int run();

private:
    // 初始化基础服务：curl、Klipper、RGA 等。
    bool initCoreServices();
    // 初始化媒体管线：RTSP/V4L2/队列/线程上下文。
    bool initMediaPipeline();
    // 初始化命令服务：YOLO + 统一 Socket 指令路由。
    bool initCommandServer();
    // 按依赖逆序释放资源。
    void shutdownApp();

private:
    volatile sig_atomic_t* running_flag_;
    AppContext app_;
};

#endif // APP_RUNTIME_H
