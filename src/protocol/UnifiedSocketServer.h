#ifndef UNIFIED_SOCKET_SERVER_H
#define UNIFIED_SOCKET_SERVER_H

#include "CommandRouter.h"
#include <thread>
#include <atomic>
#include <memory>
#include <vector>
#include <mutex>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

struct AppContext;

class UnifiedSocketServer {
private:
    int port_;
    int server_fd_;
    std::thread server_thread_;
    std::vector<std::thread> client_threads_;
    std::mutex client_threads_mutex_;
    std::atomic<bool> running_;
    AppContext* app_;
    
    std::unique_ptr<CommandRouter> router_;
    
    // 协议解析
    bool receiveCommand(int client_fd, CommandContext& ctx);
    bool detectProtocol(int client_fd, bool& is_binary);
    bool receiveBinaryCommand(int client_fd, std::string& cmd);
    bool receiveTextCommand(int client_fd, std::string& cmd);
    
    // 连接处理
    void handleClientConnection(int client_fd, const std::string& client_ip);
    void serverLoop();
    
public:
    explicit UnifiedSocketServer(int port, AppContext* app = nullptr);
    ~UnifiedSocketServer();
    
    // 禁止拷贝
    UnifiedSocketServer(const UnifiedSocketServer&) = delete;
    UnifiedSocketServer& operator=(const UnifiedSocketServer&) = delete;
    
    // 启动和停止
    bool start();
    void stop();
    
    // 获取路由器（用于注册命令）
    CommandRouter& getRouter() { return *router_; }
    
    // 检查是否运行中
    bool isRunning() const { return running_; }
};

#endif // UNIFIED_SOCKET_SERVER_H
