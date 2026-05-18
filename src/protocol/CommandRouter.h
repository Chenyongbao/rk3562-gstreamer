#ifndef COMMAND_ROUTER_H
#define COMMAND_ROUTER_H

#include "../command/CommandDispatcher.h"
#include "../command/CommandRegistry.h"
#include "../command/CommandScheduler.h"

class CommandRouter {
public:
    CommandRouter()
        : dispatcher_(registry_, scheduler_)
    {
    }

    void registerHandler(CommandHandlerPtr handler)
    {
        registry_.registerHandler(handler);
    }

    CommandResult dispatch(CommandContext& ctx)
    {
        return dispatcher_.dispatch(ctx);
    }

    size_t getCommandCount() const
    {
        return registry_.getCommandCount();
    }

    bool isExecutingCommand() const
    {
        return scheduler_.isBusy();
    }

private:
    CommandRegistry registry_;
    CommandScheduler scheduler_;
    CommandDispatcher dispatcher_;
};

#endif // COMMAND_ROUTER_H
