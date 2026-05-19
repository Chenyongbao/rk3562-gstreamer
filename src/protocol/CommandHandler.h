#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <string>
#include <memory>
#include <vector>
#include <cstdint>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <arpa/inet.h>

struct AppContext;

// 命令执行结果
enum class CommandResult {
    SUCCESS,              // 命令执行成功，继续保持连接
    ERROR_CONTINUE,       // 命令失败但可继续（业务错误）
    ERROR_DISCONNECT,     // 命令失败需断开（协议错误）
    DISCONNECT            // 正常断开连接（QUIT命令）
};

// 命令上下文
struct CommandContext {
    int client_fd;
    std::string client_ip;
    std::string command;
    bool is_binary_protocol;
    AppContext* app = nullptr;
    
    // 辅助方法：发送原始数据
    bool sendRawData(const void* data, size_t size);
    
    // 发送二进制协议响应 (4字节total_len + 4字节json_len + JSON数据 + 可选JPEG数据)
    bool sendBinaryResponse(const std::string& json_str, const std::vector<uint8_t>& jpeg_data = std::vector<uint8_t>());
    
    // 发送JSON格式的错误响应
    bool sendErrorResponse(const std::string& error_code, const std::string& message);
};

// 命令处理器抽象基类
class ICommandHandler {
public:
    virtual ~ICommandHandler() = default;
    
    // 获取命令名称
    virtual std::string getName() const = 0;
    
    // 获取命令描述
    virtual std::string getDescription() const = 0;
    
    // 执行命令（纯虚函数）
    virtual CommandResult execute(CommandContext& ctx) = 0;
    
    // 是否需要长时间执行（影响超时设置）
    virtual bool isLongRunning() const { return false; }
};

// 命令处理器智能指针类型
using CommandHandlerPtr = std::shared_ptr<ICommandHandler>;

// ==================== CommandContext 实现 ====================

inline bool CommandContext::sendRawData(const void* data, size_t size) {
    const uint8_t* ptr = (const uint8_t*)data;
    size_t sent_total = 0;
    while (sent_total < size) {
        ssize_t sent = send(client_fd, ptr + sent_total, size - sent_total, 0);
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
        sent_total += (size_t)sent;
    }
    return true;
}

inline bool CommandContext::sendBinaryResponse(const std::string& json_str, const std::vector<uint8_t>& jpeg_data) {
    // 计算总长度：4字节json_len + JSON数据 + JPEG数据
    uint32_t json_len = json_str.size();
    uint32_t jpeg_len = jpeg_data.size();
    uint32_t total_len = 4 + json_len + jpeg_len;
    
    // 发送 total_len (网络字节序)
    uint32_t net_total = htonl(total_len);
    if (!sendRawData(&net_total, 4)) {
        return false;
    }
    
    // 发送 json_len (网络字节序)
    uint32_t net_json = htonl(json_len);
    if (!sendRawData(&net_json, 4)) {
        return false;
    }
    
    // 发送 JSON 数据
    if (!sendRawData(json_str.data(), json_str.size())) {
        return false;
    }
    
    // 发送 JPEG 数据（如果有）
    if (!jpeg_data.empty()) {
        if (!sendRawData(jpeg_data.data(), jpeg_data.size())) {
            return false;
        }
    }
    
    return true;
}

inline bool CommandContext::sendErrorResponse(const std::string& error_code, const std::string& message) {
    // 简化：只发送 FAILED 状态，不包含详细错误信息
    const char* json_buffer = "{\"status\":\"FAILED\"}\n";
    return sendBinaryResponse(json_buffer);
}

#endif // COMMAND_HANDLER_H
