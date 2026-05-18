#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

#include "CommandRegistry.h"
#include "CommandScheduler.h"

#include <cstdio>

class CommandDispatcher {
public:
    explicit CommandDispatcher(CommandRegistry& registry,
                               CommandScheduler& scheduler)
        : registry_(registry), scheduler_(scheduler)
    {
    }

    CommandResult dispatch(CommandContext& ctx)
    {
        CommandHandlerPtr handler = registry_.findHandler(ctx.command);
        if (!handler) {
            fprintf(stderr, "[CommandDispatcher] Unknown command: %s\n", ctx.command.c_str());

            char error_msg[256];
            snprintf(error_msg, sizeof(error_msg),
                     "Unknown command '%s' (supported: PING, DETECT, CALIB, CALIB-0, CALIB-1, QUIT)",
                     ctx.command.c_str());
            ctx.sendErrorResponse("UNKNOWN_COMMAND", error_msg);
            return CommandResult::ERROR_CONTINUE;
        }

        fprintf(stderr, "[CommandDispatcher] Dispatching: %s from %s\n",
                ctx.command.c_str(), ctx.client_ip.c_str());

        auto acquire = scheduler_.tryAcquire(ctx.command,
                                             ctx.client_ip,
                                             handler->requiresExclusiveExecution());
        if (!acquire.acquired) {
            fprintf(stderr,
                    "[CommandDispatcher] Command rejected because device is busy: %s from %s\n",
                    ctx.command.c_str(),
                    ctx.client_ip.c_str());
            ctx.sendErrorResponse(acquire.error);
            return CommandResult::ERROR_CONTINUE;
        }

        return handler->execute(ctx);
    }

private:
    CommandRegistry& registry_;
    CommandScheduler& scheduler_;
};

#endif // COMMAND_DISPATCHER_H
