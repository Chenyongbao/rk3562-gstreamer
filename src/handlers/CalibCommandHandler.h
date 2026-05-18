#ifndef CALIB_COMMAND_HANDLER_H
#define CALIB_COMMAND_HANDLER_H

#include "../protocol/CommandHandler.h"

class CalibCommandHandler : public ICommandHandler {
public:
    std::string getName() const override;
    std::string getDescription() const override;
    bool isLongRunning() const override;
    CommandResult execute(CommandContext& ctx) override;
};

#endif // CALIB_COMMAND_HANDLER_H
