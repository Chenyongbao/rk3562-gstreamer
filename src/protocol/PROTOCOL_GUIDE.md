# Protocol 模块指南：19 个知识点 + 3 大核心要点

> 目录：`src/protocol/`（CommandHandler、CommandRouter、UnifiedSocketServer）
> 配套：`src/ARCHITECTURE.md`、`src/yolo/REFACTOR.md`

本文整理 protocol 模块涉及的 C++ / 网络编程知识点，按文件分组。标 ★ 为三大核心要点。

---

## 一、CommandHandler.h/.cpp —— 协议与上下文

### 1. TCP 部分发送问题（`sendRawData`，CommandHandler.cpp:10-30）★

```cpp
while (sent_total < size) { ... send(...) }   // 循环直到发完
```

TCP 的 `send` 一次可能只发出去一部分（发送缓冲区满、对端处理慢），**必须循环发送直到全部发完**。这是 TCP 和 UDP 的本质区别：

- **UDP**：一次整包（datagram），有消息边界。
- **TCP**：字节流（stream），**无包边界**，可能拆包、粘包。

### 2. length-prefixed framing —— TCP 粘包/拆包解法（CommandHandler.cpp:32-63）★

二进制协议帧格式：

```
4字节 total_len (htonl) | 4字节 json_len (htonl) | JSON 数据 | JPEG 数据
```

接收方先读 4 字节长度头，再按长度读正文，即可从字节流中切出"一条完整消息"。这是解决 TCP 粘包/拆包的标准手法。

### 3. 网络字节序 `htonl` / `ntohl`（CommandHandler.cpp:39、UnifiedSocketServer.cpp:206）

整数直接 `send` 会因大小端不一致而错乱（x86 小端 vs 网络大端）。发送端 `htonl` 转网络字节序，接收端 `ntohl` 还原。两端必须配对使用。

### 4. `CommandResult` 三态（CommandHandler.h:13-17）

```cpp
enum class CommandResult {
    SUCCESS,          // 命令成功，保持连接
    ERROR_CONTINUE,   // 命令失败但可继续（业务错误）
    ERROR_DISCONNECT  // 命令失败需断开（协议错误）
};
```

**区分"业务错误"和"协议错误"**：业务失败连接还能用（ERROR_CONTINUE），协议级错误（如发送彻底失败）才断开（ERROR_DISCONNECT）。server 据此决定是否 `break`（UnifiedSocketServer.cpp:134）。

### 5. Context Object 模式 + 依赖注入（CommandHandler.h:20-36、app_context.h:50）

`CommandContext` 把"fd + 客户端信息 + app 全局句柄 + 发送能力"打包成一个对象传下去：

- handler 不需要知道网络细节，也不需要知道 server 是谁。
- `ctx.app->klipper` 是依赖注入，取代 `KlipperManager::instance()` 全局单例。

### 6. 手写 JSON 转义（CommandHandler.cpp:65-83）

`sendErrorResponse` 内手写 `escape` lambda，处理 `" \ \n \r \t`。任何拼 JSON 的地方都必须转义，否则客户端传入特殊字符会生成畸形 JSON。

---

## 二、CommandRouter.h/.cpp —— 分发与所有权

### 7. 命令模式 + 注册表

```cpp
std::map<std::string, CommandHandlerPtr> handlers_;
// 注册 = 往里存（registerHandler）
// 分发 = 查表执行（dispatch）
```

`CommandResult` 从 `execute` 一路透传给 server。

### 8. 前缀 fallback（CommandRouter.cpp:23-31）

先精确查找，再 `rfind("CALIB", 0) == 0` 前缀兜底，统一转发给 `"CALIB"` 处理器。

- 让 `CALIB-1` / `CALIB-FOCALS` 不加注册就能路由。
- 严格讲破坏了"map 完全泛化"的纯粹性，但换来便利，是**务实的 tradeoff**。

> 补充：`rfind("CALIB", 0)` 的 `pos=0` 把搜索范围锁死在开头，等价于 C++20 的 `starts_with("CALIB")`。C++17 下这是"前缀判断"的标准惯用法（`find` 需配 `== 0` 才能用，语义不自锁）。

### 9. `snprintf` 安全格式化（CommandRouter.cpp:34-38）

`char error_msg[256]` + `snprintf` 带长度上限，不会缓冲区溢出。`char[]` 是 C 风格残留，但 `sizeof` 保护使其安全。

### 10. 单线程无锁设计（CommandRouter.h、CommandRouter.cpp）

- `registerHandler` 只在 server 启动前调用（app_runtime.cpp:96-107），单线程。
- `dispatch` / `getCommandCount` 只在 server 线程串行执行。
- **没有任何并发访问者**，锁是纯样板 → 已删除。
- 代价：若未来改多客户端，需按"运行时只读 map"重新设计（见第五节）。

---

## 三、UnifiedSocketServer.h/.cpp —— 网络服务核心

### 11. `MSG_PEEK` 协议探测（UnifiedSocketServer.cpp:173-195）★

```cpp
recv(client_fd, header, 4, MSG_PEEK);   // 偷看但不消费数据
```

`MSG_PEEK` 把数据读出来看，但**数据仍留在内核缓冲区**，不影响后续正式读取。这里用它先看前 4 字节判断协议类型（`0x00 0x00` 开头 = 二进制），是最冷门但极实用的 socket 技巧。

### 12. 双层超时（UnifiedSocketServer.cpp:111-151）

- socket 级：`SO_RCVTIMEO` / `SO_SNDTIMEO` 120s 收发超时。
- 命令级：`select` 60s 空闲超时。
- `select` 同时用于 accept 循环（1s 超时轮询 `running_`）。

防止死等、防恶意连接占用。

### 13. 串行单连接模型（UnifiedSocketServer.cpp:100-101）

```cpp
handleClientConnection(client_fd, ip_str);   // 同步阻塞，处理完才 accept 下一个
```

一次只处理一个客户端 → **整条链路天然单线程、无并发**。这是 CommandRouter 不需要锁的根本原因，也是将来支持多客户端时的最大改造点。

### 14. 长度合法性校验（UnifiedSocketServer.cpp:210-213）

```cpp
if (msg_len < 1 || msg_len > 1024) return false;   // 防恶意报文
```

读取前先校验长度上限，避免恶意客户端声称"长度 4GB"导致 `vector<char>` 爆内存。**安全编程必备**。

### 15. 优雅关闭（UnifiedSocketServer.cpp:52-66）

`running_` 是 `std::atomic<bool>`：`stop()` 置 false → select 1s 超时醒来看到 → 退出循环 → `join()` 等线程收尾。atomic 保证跨线程可见性（非原子 bool 会有数据竞争）。

### 16. 文本协议去空白收包（UnifiedSocketServer.cpp:244-258）

按换行符切包 + 循环剥掉尾部 `\n \r 空格 \t`。客户端发 `"DETECT\n"` 和 `"DETECT \r\n"` 都能正确解析成 `"DETECT"`。

### 17. `SO_REUSEADDR`（UnifiedSocketServer.cpp:25）

允许 server 重启后立即复用同一端口（否则 TIME_WAIT 状态会 bind 失败）。开发期反复重启服务必加。

---

## 四、跨文件架构知识点

### 18. 组合根 + 依赖注入

`AppContext`（app_context.h）持有所有单例（klipper、yolo_model、server），handler 通过 `ctx.app` 拿依赖，杜绝全局单例，比 `KlipperManager::instance()` 更可测。

### 19. 三层分层

```
解析层（UnifiedSocketServer）→ 路由层（CommandRouter）→ 业务层（handlers）
```

各层职责单一、向下调用。这是这套代码最干净的架构骨架。

---

## 五、延伸：多客户端支持改造方案

### 现状盘点（底子已半准备好）

| 组件 | 现状 | 多客户端影响 |
|---|---|---|
| `CommandRouter` | 已去锁，`dispatch` 只读 map | 需重新确认只读安全性 |
| `handleClientConnection` | 同步串行（一次一个客户端） | 核心改造点：accept 后开线程 |
| Klipper 设备客户端（klipper_manager.h） | 普通客户端，无仲裁（硬件所有者模式已移除） | 多线程化需先补并发仲裁 |
| `CaptureLoopState`（capture_state.h） | `bev_refresh_request_count` 已用 `std::atomic` | 线程安全已就绪 |
| `FrameProvider`（frame_provider.h:83-94） | 内部 `mutex_` + `cv_` 保护帧缓冲与等待计数 | 线程安全已就绪 |
| `CommandContext` | 每次连接栈上构造，fd 独立 | 无共享，安全 |

### 第一步：server 端——串行 → 每连接一线程

现状（UnifiedSocketServer.cpp:101）：
```cpp
handleClientConnection(client_fd, ip_str);   // 同步阻塞
```

改为：
```cpp
std::thread(&UnifiedSocketServer::handleClientConnection,
            this, client_fd, ip_str).detach();
```

**注意**：
- 并发客户端多时线程数失控 → 需加连接上限（信号量或计数）。
- `stop()` 时 `detach` 的线程无法 join，需靠 `running_` + 每连接超时自然退出。

### 第二步：router 端——运行期只读，零锁

注册只发生在启动前（app_runtime.cpp:96-107），`start()` 之后 map **永不改变**：

- **方案 A（推荐）**：`handlers_` 视为"启动后只读"，多线程并发 `find` 是安全的（读读不冲突）。**不要把锁加在 `execute` 外面**。注意：硬件所有者仲裁模式已移除（`KlipperManager` 现为普通客户端），若改多线程，需先补一套 Klipper 并发仲裁机制（参考 DESIGN_KlipperService.md 的历史方案）。
- **方案 B**：`dispatch` 内加锁但只锁 `find`、不锁 `execute`。但运行期 map 只读，锁依然多余。

### 第三步：处理器内部——确认无共享写

`DetectCommandHandler::execute` 内所有状态都在栈上（payload、first_results 等）。共享状态已由 `std::atomic`、FrameProvider 内部锁分别保护。**无需改动处理器**（但需为 Klipper 并发访问补仲裁，见上文）。

### 待确认问题

1. **目标并发数**：将来最多几个客户端同时连？若只是"一个上位机 + 一个调试终端"，保持现状即可。
2. **连接上限策略**：若改多线程，需要定义最大连接数与拒绝策略。
3. **`stop()` 的线程收尾**：`detach` 后线程如何保证在 `stop()` 后尽快退出（依赖超时 vs 主动关闭 socket）。

---

## 三大核心要点速查 ★

| 知识点 | 位置 | 一句话 |
|---|---|---|
| TCP 部分发送循环 | CommandHandler.cpp:10-30 | `send` 可能只发一部分，必须循环发完 |
| length-prefixed framing | CommandHandler.cpp:32-63 | 长度头 + 正文，解决粘包/拆包 |
| `MSG_PEEK` 协议探测 | UnifiedSocketServer.cpp:173-195 | 偷看但不消费数据，用于协议识别 |
