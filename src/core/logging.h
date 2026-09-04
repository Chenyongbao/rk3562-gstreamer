#ifndef CORE_LOGGING_H
#define CORE_LOGGING_H

#include <memory>

#include <spdlog/sinks/stderr_color_sinks.h>
#include <spdlog/spdlog.h>

// 统一日志：进程启动时调用一次 logging::initLogging()，之后全局使用
// spdlog::info / spdlog::warn / spdlog::error 输出。
//
// 日志走 stderr（与原 fprintf(stderr, ...) 行为一致），彩色分级、带时间戳。
// 旧代码的 "[TAG]" 前缀保留在消息文本内，便于按模块检索，不改变既有可读性。
namespace logging {

inline void initLogging() {
    auto sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    auto logger = std::make_shared<spdlog::logger>("reallink", std::move(sink));
    logger->set_level(spdlog::level::info);
    spdlog::set_default_logger(std::move(logger));
}

}  // namespace logging

#endif  // CORE_LOGGING_H
