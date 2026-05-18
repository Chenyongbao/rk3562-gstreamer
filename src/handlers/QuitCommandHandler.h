#ifndef QUIT_COMMAND_HANDLER_H
#define QUIT_COMMAND_HANDLER_H

#include "../protocol/CommandHandler.h"

#include <cstdio>

class QuitCommandHandler : public ICommandHandler {
public:
    std::string getName() const override { return "QUIT"; }

    std::string getDescription() const override
    {
        return "Close the current client connection";
    }

    bool requiresExclusiveExecution() const override { return false; }

    CommandResult execute(CommandContext& ctx) override
    {
        fprintf(stderr, "[Unified Server] QUIT received from %s\n", ctx.client_ip.c_str());
        const char* json = "{\"status\":\"SUCCESS\",\"code\":\"OK\",\"command\":\"QUIT\"}\n";
        if (!ctx.sendBinaryResponse(json)) {
            return CommandResult::ERROR_DISCONNECT;
        }
        return CommandResult::DISCONNECT;
    }
};

#endif // QUIT_COMMAND_HANDLER_H
