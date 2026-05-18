#ifndef PING_COMMAND_HANDLER_H
#define PING_COMMAND_HANDLER_H

#include "../protocol/CommandHandler.h"
#include <cstdio>

class PingCommandHandler : public ICommandHandler {
public:
    std::string getName() const override { return "PING"; }
    
    std::string getDescription() const override { 
        return "Heartbeat test - responds with PONG"; 
    }
    
    CommandResult execute(CommandContext& ctx) override {
        fprintf(stderr, "[Unified Server] PING received from %s\n", ctx.client_ip.c_str());
        
        const char* pong_json = "{\"status\":\"PONG\"}\n";
        
        if (!ctx.sendBinaryResponse(pong_json)) {
            fprintf(stderr, "[Unified Server] Failed to send PONG\n");
            return CommandResult::ERROR_DISCONNECT;
        }
        
        fprintf(stderr, "[Unified Server] ✅ PONG sent\n");
        return CommandResult::SUCCESS;
    }
};

#endif // PING_COMMAND_HANDLER_H
