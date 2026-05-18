#include "UnifiedSocketServer.h"
#include "../config.h"
#include "../app/app_context.h"
#include <cstdio>

UnifiedSocketServer::UnifiedSocketServer(int port, AppContext* app)
    : port_(port), server_fd_(-1), running_(false),
      app_(app),
      router_(std::make_unique<CommandRouter>()) {
}

UnifiedSocketServer::~UnifiedSocketServer() {
    stop();
}

bool UnifiedSocketServer::start() {
    // 创建 socket
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        fprintf(stderr, "[UnifiedServer] socket() failed: %s\n", strerror(errno));
        return false;
    }
    
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(SERVER_BIND_ADDRESS);  // 使用配置的绑定地址
    server_addr.sin_port = htons(port_);
    
    if (bind(server_fd_, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        fprintf(stderr, "[UnifiedServer] bind() failed: %s\n", strerror(errno));
        close(server_fd_);
        return false;
    }
    
    if (listen(server_fd_, 10) < 0) {
        fprintf(stderr, "[UnifiedServer] listen() failed: %s\n", strerror(errno));
        close(server_fd_);
        return false;
    }
    
    running_ = true;
    server_thread_ = std::thread(&UnifiedSocketServer::serverLoop, this);
    
    fprintf(stderr, "[UnifiedServer]  Server started on %s:%d\n", SERVER_BIND_ADDRESS, port_);
    fprintf(stderr, "[UnifiedServer] Registered %zu commands\n", router_->getCommandCount());
    return true;
}

void UnifiedSocketServer::stop() {
    if (running_) {
        running_ = false;
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

    {
        std::lock_guard<std::mutex> lock(client_threads_mutex_);
        for (auto& thread : client_threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        client_threads_.clear();
    }
    
    if (server_fd_ >= 0) {
        close(server_fd_);
        server_fd_ = -1;
    }
    
    fprintf(stderr, "[UnifiedServer] Server stopped\n");
}

void UnifiedSocketServer::serverLoop() {
    fprintf(stderr, "[UnifiedServer] Server loop started\n");
    
    while (running_) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(server_fd_, &readfds);
        
        timeval tv{1, 0};  // 1秒超时
        int ret = select(server_fd_ + 1, &readfds, nullptr, nullptr, &tv);
        
        if (ret < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "[UnifiedServer] select() error: %s\n", strerror(errno));
            break;
        }
        
        if (ret == 0) continue;
        
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        int client_fd = accept(server_fd_, (sockaddr*)&client_addr, &addr_len);
        
        if (client_fd < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "[UnifiedServer] accept() error: %s\n", strerror(errno));
            continue;
        }
        
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
        
        // 处理客户端连接（同步模式）
        {
            std::lock_guard<std::mutex> lock(client_threads_mutex_);
            client_threads_.emplace_back(&UnifiedSocketServer::handleClientConnection,
                                         this,
                                         client_fd,
                                         std::string(ip_str));
        }
    }
    
    fprintf(stderr, "[UnifiedServer] Server loop stopped\n");
}
//打印客户端的连接日志，并进入命令处理的循环，直到连接断开
void UnifiedSocketServer::handleClientConnection(int client_fd, const std::string& client_ip) {
    fprintf(stderr, "[UnifiedServer] Client connected from %s\n", client_ip.c_str());
    
    // 配置超时
    timeval timeout{120, 0};  // 120秒发送/接收超时
    setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    
    // 长连接循环
    while (running_) {
        CommandContext ctx;
        ctx.client_fd = client_fd;
        ctx.client_ip = client_ip;
        ctx.app = app_;
        
        if (!receiveCommand(client_fd, ctx)) {
            break;  // 接收失败或超时
        }
        
        fprintf(stderr, "[UnifiedServer] Command: '%s' from %s (%s protocol)\n",
                ctx.command.c_str(), ctx.client_ip.c_str(),
                ctx.is_binary_protocol ? "binary" : "text");
        
        // 分发命令
        CommandResult result = router_->dispatch(ctx);
        
        // 根据结果决定是否断开
        if (result == CommandResult::DISCONNECT || 
            result == CommandResult::ERROR_DISCONNECT) {
            fprintf(stderr, "[UnifiedServer] Connection closing (result: %d)\n", static_cast<int>(result));
            break;
        }
    }
    
    close(client_fd);
    fprintf(stderr, "[UnifiedServer] Connection closed from %s\n", client_ip.c_str());
}

bool UnifiedSocketServer::receiveCommand(int client_fd, CommandContext& ctx) {
    // select 超时检测
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(client_fd, &readfds);
    
    timeval timeout{60, 0};  // 60秒空闲超时
    int ret = select(client_fd + 1, &readfds, nullptr, nullptr, &timeout);
    
    if (ret <= 0) {
        if (ret == 0) {
            fprintf(stderr, "[UnifiedServer] ⏱️ Connection idle timeout (60s)\n");
        }
        return false;  // 超时或错误
    }
    
    // 协议检测
    if (!detectProtocol(client_fd, ctx.is_binary_protocol)) {
        return false;
    }
    
    // 接收命令
    if (ctx.is_binary_protocol) {
        return receiveBinaryCommand(client_fd, ctx.command);
    } else {
        return receiveTextCommand(client_fd, ctx.command);
    }
}

bool UnifiedSocketServer::detectProtocol(int client_fd, bool& is_binary) {
    // 先尝试读取前4个字节，检测协议类型
    uint8_t header[4];
    ssize_t n = recv(client_fd, header, 4, MSG_PEEK);  // MSG_PEEK 不移除数据
    
    if (n <= 0) {
        return false;
    }
    
    if (n >= 4) {
        // 检测是否为二进制协议（前2个字节通常是0x00）
        if (header[0] == 0x00 && header[1] == 0x00) {
            is_binary = true;
            fprintf(stderr, "[UnifiedServer] Detected binary protocol\n");
        } else {
            is_binary = false;
            fprintf(stderr, "[UnifiedServer] Detected text protocol\n");
        }
        return true;
    }
    
    return false;
}

bool UnifiedSocketServer::receiveBinaryCommand(int client_fd, std::string& cmd) {
    // 读取4字节长度头
    uint32_t msg_len_net;
    ssize_t n = recv(client_fd, &msg_len_net, 4, 0);
    if (n != 4) {
        fprintf(stderr, "[UnifiedServer] Failed to read length header\n");
        return false;
    }
    
    uint32_t msg_len = ntohl(msg_len_net);
    fprintf(stderr, "[UnifiedServer] Binary protocol: message length = %u bytes\n", msg_len);
    
    // 验证长度合理性
    if (msg_len < 1 || msg_len > 1024) {
        fprintf(stderr, "[UnifiedServer] Invalid message length: %u\n", msg_len);
        return false;
    }
    
    // 读取命令数据
    std::vector<char> buffer(msg_len + 1);
    size_t total_received = 0;
    while (total_received < msg_len) {
        n = recv(client_fd, buffer.data() + total_received, msg_len - total_received, 0);
        if (n <= 0) {
            fprintf(stderr, "[UnifiedServer] Failed to read command data\n");
            return false;
        }
        total_received += n;
    }
    buffer[total_received] = '\0';
    
    cmd = std::string(buffer.data(), total_received);
    return true;
}

bool UnifiedSocketServer::receiveTextCommand(int client_fd, std::string& cmd) {
    std::vector<char> buffer(1024);
    size_t total_received = 0;
    
    while (total_received < buffer.size() - 1) {
        ssize_t n = recv(client_fd, buffer.data() + total_received, buffer.size() - 1 - total_received, 0);
        if (n <= 0) {
            return false;
        }
        total_received += n;
        
        // 检查是否收到换行符（命令结束）
        if (buffer[total_received - 1] == '\n' || buffer[total_received - 1] == '\r') {
            break;
        }
    }
    
    buffer[total_received] = '\0';
    
    // 去除末尾的换行符和空格
    while (total_received > 0 && (buffer[total_received - 1] == '\n' || 
                                  buffer[total_received - 1] == '\r' || 
                                  buffer[total_received - 1] == ' ' || 
                                  buffer[total_received - 1] == '\t')) {
        buffer[total_received - 1] = '\0';
        total_received--;
    }
    
    cmd = std::string(buffer.data(), total_received);
    return true;
}
