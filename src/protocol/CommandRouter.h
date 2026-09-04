#ifndef COMMAND_ROUTER_H
#define COMMAND_ROUTER_H

#include "CommandHandler.h"

#include <map>

// 命令路由器：维护"命令名 → 处理器"的映射，并负责按命令名分发。
// 线程模型：注册发生在 server 启动前（单线程），分发只在 server 线程串行执行，
// 无并发访问，因此不需要加锁。
class CommandRouter {
public:
    void registerHandler(CommandHandlerPtr handler);
    CommandResult dispatch(CommandContext& ctx);
    size_t getCommandCount() const;

private:
    std::map<std::string, CommandHandlerPtr> handlers_;
};

#endif // COMMAND_ROUTER_H
