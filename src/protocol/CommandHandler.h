#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct AppContext;

// 命令执行结果
enum class CommandResult {
    SUCCESS,              // 命令执行成功，继续保持连接
    ERROR_CONTINUE,       // 命令失败但可继续（业务错误）
    ERROR_DISCONNECT      // 命令失败需断开（协议错误）
};

// 命令上下文：承载一次命令的 socket 与协议信息，并提供响应发送能力。
struct CommandContext {
    int client_fd;
    std::string client_ip;
    std::string command;
    bool is_binary_protocol;
    AppContext* app = nullptr;

    // 发送原始数据（循环 send 直到发完或失败）。
    bool sendRawData(const void* data, size_t size);

    // 发送二进制协议响应：4字节 total_len + 4字节 json_len + JSON + 可选 JPEG。
    bool sendBinaryResponse(const std::string& json_str,
                            const std::vector<uint8_t>& jpeg_data = std::vector<uint8_t>());

    // 发送 JSON 格式的错误响应（含 error_code / error_message）。
    bool sendErrorResponse(const std::string& error_code, const std::string& message);
};

// 命令处理器抽象基类。
class ICommandHandler {
public:
    virtual ~ICommandHandler() = default;

    virtual std::string getName() const = 0;
    virtual std::string getDescription() const = 0;

    // 执行命令（纯虚函数）。
    virtual CommandResult execute(CommandContext& ctx) = 0;
};

// 命令处理器智能指针类型：独占所有权（router 的 map 是唯一持有者）。
using CommandHandlerPtr = std::unique_ptr<ICommandHandler>;

#endif // COMMAND_HANDLER_H
