#ifndef COMMAND_REGISTRY_H
#define COMMAND_REGISTRY_H

#include "../protocol/CommandHandler.h"

#include <cstdio>
#include <map>
#include <mutex>
#include <string>

class CommandRegistry {
public:
    void registerHandler(CommandHandlerPtr handler)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        handlers_[handler->getName()] = handler;
        fprintf(stderr, "[CommandRegistry] Registered: %s - %s\n",
                handler->getName().c_str(),
                handler->getDescription().c_str());
    }

    CommandHandlerPtr findHandler(const std::string& command) const
    {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = handlers_.find(command);
        if (it != handlers_.end()) {
            return it->second;
        }

        if (command.rfind("CALIB", 0) == 0) {
            auto calib_it = handlers_.find("CALIB");
            if (calib_it != handlers_.end()) {
                return calib_it->second;
            }
        }

        return nullptr;
    }

    size_t getCommandCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return handlers_.size();
    }

private:
    std::map<std::string, CommandHandlerPtr> handlers_;
    mutable std::mutex mutex_;
};

#endif // COMMAND_REGISTRY_H
