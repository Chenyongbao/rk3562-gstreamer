#include "CommandHandler.h"

#include <arpa/inet.h>   // htonl
#include <cerrno>        // errno
#include <cstdio>        // fprintf
#include <cstring>       // strerror
#include <sys/socket.h>  // send
#include <sys/types.h>   // ssize_t

bool CommandContext::sendRawData(const void* data, size_t size) {
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

bool CommandContext::sendBinaryResponse(const std::string& json_str, const std::vector<uint8_t>& jpeg_data) {
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

bool CommandContext::sendErrorResponse(const std::string& error_code, const std::string& message) {
    // 组装带错误码与错误描述的 JSON 返回（转义特殊字符，避免畸形 JSON）。
    auto escape = [](const std::string& s) {
        std::string out;
        for (char c : s) {
            switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:   out += c;      break;
            }
        }
        return out;
    };
    const std::string json = "{\"status\":\"FAILED\",\"error_code\":\"" + escape(error_code) +
                             "\",\"error_message\":\"" + escape(message) + "\"}\n";
    return sendBinaryResponse(json);
}
