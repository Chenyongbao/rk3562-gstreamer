#ifndef COMMAND_ROUTER_H
#define COMMAND_ROUTER_H

#include "CommandHandler.h"
#include <map>
#include <mutex>
#include <cstdio>

class CommandRouter {
private:
    std::map<std::string, CommandHandlerPtr> handlers_;
    mutable std::mutex mutex_;
    
public:
    // 注册命令处理器
    void registerHandler(CommandHandlerPtr handler) {
        std::lock_guard<std::mutex> lock(mutex_);
        handlers_[handler->getName()] = handler;
        fprintf(stderr, "[CommandRouter] Registered: %s - %s\n",
                handler->getName().c_str(), 
                handler->getDescription().c_str());
    }
    
    // 分发命令
    CommandResult dispatch(CommandContext& ctx) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto it = handlers_.find(ctx.command);
        if (it != handlers_.end()) {
            fprintf(stderr, "[CommandRouter] Dispatching: %s from %s\n",
                    ctx.command.c_str(), ctx.client_ip.c_str());
            return it->second->execute(ctx);
        }

        if (ctx.command.rfind("CALIB", 0) == 0) {
            auto calib_it = handlers_.find("CALIB");
            if (calib_it != handlers_.end()) {
                fprintf(stderr, "[CommandRouter] Dispatching: %s from %s (CALIB alias)\n",
                        ctx.command.c_str(), ctx.client_ip.c_str());
                return calib_it->second->execute(ctx);
            }
        }
        
        // 未找到命令
        fprintf(stderr, "[CommandRouter] Unknown command: %s\n", ctx.command.c_str());
        
        char error_msg[256];
        snprintf(error_msg, sizeof(error_msg), 
                "Unknown command '%s' (supported: PING, DETECT, CALIB, CALIB-0, CALIB-1, QUIT)",
                ctx.command.c_str());
        ctx.sendErrorResponse("UNKNOWN_COMMAND", error_msg);
        
        return CommandResult::ERROR_CONTINUE;  // 未知命令不断开连接
    }
    
    // 获取命令数量
    size_t getCommandCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return handlers_.size();
    }
};

#endif // COMMAND_ROUTER_H
