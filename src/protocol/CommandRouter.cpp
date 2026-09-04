#include "CommandRouter.h"

#include <cstdio>

void CommandRouter::registerHandler(CommandHandlerPtr handler) {
    const std::string name = handler->getName();
    const std::string description = handler->getDescription();
    handlers_.emplace(name, std::move(handler));
    fprintf(stderr, "[CommandRouter] Registered: %s - %s\n",
            name.c_str(), description.c_str());
}

CommandResult CommandRouter::dispatch(CommandContext& ctx) {
    auto it = handlers_.find(ctx.command);
    if (it != handlers_.end()) {
        fprintf(stderr, "[CommandRouter] Dispatching: %s from %s\n",
                ctx.command.c_str(), ctx.client_ip.c_str());
        //执行命令处理器的execute方法
        //迭代器找value(智能指针)
        return it->second->execute(ctx);
    }

    //接受指令的二次兜底
    // CALIB 系列子命令（CALIB-FOCALS / CALIB-1 等）统一转发到 CALIB 处理器。
    if (ctx.command.rfind("CALIB", 0) == 0) {
        auto calib_it = handlers_.find("CALIB");
        if (calib_it != handlers_.end()) {
            fprintf(stderr, "[CommandRouter] Dispatching: %s from %s (CALIB alias)\n",
                    ctx.command.c_str(), ctx.client_ip.c_str());
            return calib_it->second->execute(ctx);
        }
    }

    fprintf(stderr, "[CommandRouter] Unknown command: %s\n", ctx.command.c_str());

    char error_msg[256];
    snprintf(error_msg, sizeof(error_msg),
            "Unknown command '%s' (supported: PING, DETECT, CALIB, CALIB-FOCALS, CALIB-1, METRICS)",
            ctx.command.c_str());
    ctx.sendErrorResponse("UNKNOWN_COMMAND", error_msg);

    return CommandResult::ERROR_CONTINUE;  // 未知命令不断开连接
}

size_t CommandRouter::getCommandCount() const {
    return handlers_.size();
}
