#define _POSIX_C_SOURCE 199309L

#include <signal.h>
#include <unistd.h>

#include "config.h"
#include "app/app_runtime.h"
#include "core/logging.h"

static volatile sig_atomic_t g_running = 1;

static void signal_handler(int sig)
{
    // 在 signal handler 中避免使用非 async-signal-safe 的函数（例如 spdlog/fprintf）。
    // 直接退出进程，确保 Ctrl+C 能立即终止所有标定/子线程阻塞流程。
    const char msg[] = "\n[MAIN] Caught signal, exiting now...\n";
    (void)write(STDERR_FILENO, msg, sizeof(msg) - 1);
    _exit(128 + sig);
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    logging::initLogging();
    spdlog::info("reallinkCV version: {}", REALLINKCV_VERSION);

    // 注册退出信号，统一走快速终止路径。
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    g_running = 1;

    // 运行应用主流程（初始化 -> 工作循环 -> 清理）。
    AppRuntime app(&g_running);
    return app.run();
}
