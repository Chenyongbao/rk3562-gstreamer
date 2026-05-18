#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "../errors/ErrorResponse.h"

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <BaseTsd.h>
#include <winsock2.h>
using ssize_t = SSIZE_T;
#ifdef ERROR_CONTINUE
#undef ERROR_CONTINUE
#endif
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#endif

struct AppContext;

enum class CommandResult {
    SUCCESS,
    ERROR_CONTINUE,
    ERROR_DISCONNECT,
    DISCONNECT
};

struct CommandContext {
    int client_fd;
    std::string client_ip;
    std::string command;
    bool is_binary_protocol;
    AppContext* app = nullptr;

    bool sendRawData(const void* data, size_t size);
    bool sendBinaryResponse(const std::string& json_str,
                            const std::vector<uint8_t>& jpeg_data = std::vector<uint8_t>());
    bool sendErrorResponse(const std::string& error_code, const std::string& message);
    bool sendErrorResponse(const ErrorStatus& status);
};

class ICommandHandler {
public:
    virtual ~ICommandHandler() = default;

    virtual std::string getName() const = 0;
    virtual std::string getDescription() const = 0;
    virtual CommandResult execute(CommandContext& ctx) = 0;

    // Commands that touch hardware must run one at a time across all clients.
    virtual bool requiresExclusiveExecution() const { return isLongRunning(); }
    virtual bool isLongRunning() const { return false; }
};

using CommandHandlerPtr = std::shared_ptr<ICommandHandler>;

inline bool CommandContext::sendRawData(const void* data, size_t size)
{
    const uint8_t* ptr = static_cast<const uint8_t*>(data);
    size_t sent_total = 0;
    while (sent_total < size) {
#ifdef _WIN32
        const int chunk_size = static_cast<int>(size - sent_total);
        ssize_t sent = send(client_fd,
                            reinterpret_cast<const char*>(ptr + sent_total),
                            chunk_size,
                            0);
#else
        ssize_t sent = send(client_fd, ptr + sent_total, size - sent_total, 0);
#endif
        if (sent <= 0) {
            const int saved_errno = errno;
            fprintf(stderr,
                    "[CommandContext] sendRawData failed: fd=%d, sent=%zd, sent_total=%zu, target=%zu, errno=%d (%s)\n",
                    client_fd,
                    sent,
                    sent_total,
                    size,
                    saved_errno,
                    strerror(saved_errno));
            return false;
        }
        sent_total += static_cast<size_t>(sent);
    }
    return true;
}

inline bool CommandContext::sendBinaryResponse(const std::string& json_str,
                                               const std::vector<uint8_t>& jpeg_data)
{
    const uint32_t json_len = static_cast<uint32_t>(json_str.size());
    const uint32_t jpeg_len = static_cast<uint32_t>(jpeg_data.size());
    const uint32_t total_len = 4 + json_len + jpeg_len;

    uint32_t net_total = htonl(total_len);
    if (!sendRawData(&net_total, 4)) {
        return false;
    }

    uint32_t net_json = htonl(json_len);
    if (!sendRawData(&net_json, 4)) {
        return false;
    }

    if (!sendRawData(json_str.data(), json_str.size())) {
        return false;
    }

    if (!jpeg_data.empty()) {
        if (!sendRawData(jpeg_data.data(), jpeg_data.size())) {
            return false;
        }
    }

    return true;
}

inline bool CommandContext::sendErrorResponse(const std::string& error_code,
                                              const std::string& message)
{
    return sendErrorResponse(mapLegacyError(error_code, message));
}

inline bool CommandContext::sendErrorResponse(const ErrorStatus& status)
{
    return sendBinaryResponse(buildErrorResponseJson(status, command));
}

#endif // COMMAND_HANDLER_H
