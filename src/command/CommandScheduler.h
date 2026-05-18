#ifndef COMMAND_SCHEDULER_H
#define COMMAND_SCHEDULER_H

#include "../errors/ErrorStatus.h"

#include <memory>
#include <mutex>
#include <sstream>
#include <string>

class CommandScheduler {
public:
    class Lease {
    public:
        Lease() = default;

        Lease(CommandScheduler* owner, std::shared_ptr<int> token)
            : owner_(owner), token_(std::move(token))
        {
        }

        Lease(const Lease&) = delete;
        Lease& operator=(const Lease&) = delete;

        Lease(Lease&& other) noexcept
            : owner_(other.owner_), token_(std::move(other.token_))
        {
            other.owner_ = nullptr;
        }

        Lease& operator=(Lease&& other) noexcept
        {
            if (this != &other) {
                release();
                owner_ = other.owner_;
                token_ = std::move(other.token_);
                other.owner_ = nullptr;
            }
            return *this;
        }

        ~Lease()
        {
            release();
        }

        explicit operator bool() const
        {
            return owner_ != nullptr && token_ != nullptr;
        }

        void release()
        {
            if (owner_ && token_) {
                owner_->release(token_);
                owner_ = nullptr;
                token_.reset();
            }
        }

    private:
        CommandScheduler* owner_ = nullptr;
        std::shared_ptr<int> token_;
    };

    struct AcquireResult {
        bool acquired = false;
        Lease lease;
        ErrorStatus error = ErrorStatus::ok();
    };

    AcquireResult tryAcquire(const std::string& command,
                             const std::string& client_id,
                             bool requires_exclusive)
    {
        if (!requires_exclusive) {
            return {true, Lease(), ErrorStatus::ok()};
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (busy_) {
            std::ostringstream message;
            message << "device is executing " << current_command_;
            if (!current_client_.empty()) {
                message << " from " << current_client_;
            }
            return {
                false,
                Lease(),
                ErrorStatus::failure(ErrorCode::Busy,
                                     ErrorDomain::Scheduler,
                                     message.str(),
                                     true,
                                     false)
            };
        }

        busy_ = true;
        current_command_ = command;
        current_client_ = client_id;
        current_token_ = std::make_shared<int>(1);

        return {true, Lease(this, current_token_), ErrorStatus::ok()};
    }

    bool isBusy() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return busy_;
    }

    std::string currentCommand() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_command_;
    }

    std::string currentClient() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_client_;
    }

private:
    void release(const std::shared_ptr<int>& token)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (token != current_token_) {
            return;
        }

        busy_ = false;
        current_command_.clear();
        current_client_.clear();
        current_token_.reset();
    }

    mutable std::mutex mutex_;
    bool busy_ = false;
    std::string current_command_;
    std::string current_client_;
    std::shared_ptr<int> current_token_;
};

#endif // COMMAND_SCHEDULER_H
