#ifndef KLIPPER_MANAGER_H
#define KLIPPER_MANAGER_H

#include <string>
#include <mutex>

// Klipper管理器的配置结构体
struct KlipperManagerConfig {
    std::string host = "192.168.50.1";  // Klipper主机地址
    int port = 7125;   // Klipper的端口号
    int homing_timeout_ms = 300000;  // 归位超时时间（毫秒）
};

// ============================================================================
// Z 轴下探 + 激光测距的契约类型（原定义于已删除的 iklipper_device.h，迁入本头）。
// ============================================================================

// 停止规则：决定"下探到什么时候算成功"。
enum class ZProbeStopRule {
    FirstValid,          // 拿到首个有效读数即停止
    InRangeOrBelowMin,   // 距离进入 [stop_min_mm, stop_max_mm] 或低于下限，且连续命中达到次数
    AtMost,              // 距离不大于 stop_max_mm 即停止
};

// 结束原因：明确"为什么这个流程结束了"（成功或失败）。
enum class ZProbeEndReason {
    StopMet,               // 满足停止规则，成功
    ZExceededMax,          // Z 目标超过上限（enforce_z_max）
    MoveFailed,            // 移动失败
    TriggerFailed,         // 触发激光失败（trigger_failure_is_fatal 时）
    QueryFailed,           // 查询失败（query_failure_is_fatal 时）
    NoValidAfterMax,       // 尝试满仍未拿到有效读数
    RuleNotMetAfterMax,    // 尝试满仍未满足停止规则
};

// 下探配置：把"怎么下探、什么时候停"参数化。
struct ZProbeConfig {
    int max_attempts = 1;          // 最大尝试轮数
    int query_retries = 1;         // 单轮内查询重试次数
    int trigger_settle_ms = 0;     // 触发激光后的稳定等待毫秒
    int consecutive_hits = 1;      // 需要连续命中的次数（InRangeOrBelowMin 用）
    double step_mm = 1.0;          // 每轮下探的绝对 Z 步进（第 index 轮移到 index*step_mm）
    long move_timeout_sec = 40L;   // 移动 gcode 超时（秒）
    long trigger_timeout_sec = 20L;// 触发激光 gcode 超时（秒）
    bool enforce_z_max = false;    // 是否启用 Z 上限保护
    double z_max_mm = 0.0;         // Z 上限（enforce_z_max 时生效）
    std::string log_prefix;        // 日志前缀（为空则不打印）
    double feedrate = 1500.0;      // 移动速度 (mm/min)
    ZProbeStopRule stop_rule = ZProbeStopRule::FirstValid;
    double stop_min_mm = 0.0;      // 目标区间下限
    double stop_max_mm = 0.0;      // 目标区间上限（AtMost 的阈值）
    bool trigger_failure_is_fatal = false;  // 触发失败是否直接失败
    bool query_failure_is_fatal = false;    // 查询失败是否直接失败
};

// 下探结果：流程结束后的产出。
struct ZProbeResult {
    ZProbeEndReason end_reason = ZProbeEndReason::StopMet;
    int attempts = 0;              // 实际尝试轮数
    double final_z_mm = 0.0;       // 最终的绝对 Z 位置
    double final_distance_mm = 0.0;// 最后的激光距离读数
    bool final_valid = false;      // 最后的读数是否有效
    int consecutive_hits = 0;      // 结束时累计连续命中数
    bool had_any_valid = false;    // 过程中是否拿到过有效读数
};

// ============================================================================
// 【Klipper 设备客户端】KlipperManager
// ============================================================================
//
// 定位：管理与 Klipper 通信的唯一 HTTP 入口（libcurl → Moonraker → Klipper）。
// 单例，由组合根（app_runtime.cpp）创建并持有，业务服务经 ctx.app->klipper 获取。
//
// 说明：早期版本曾引入"硬件所有者"模式（IKlipperDevice 接口 + 独占租约 +
// 状态机 + 任务队列 + RAII 守卫）做资源仲裁。该模式下头文件被删除后，本类
// 退化为普通设备客户端：只负责通信原语与归位/下探等业务流程，不再承担仲裁。
// 命令层当前为单线程串行，无并发抢占风险，需要仲裁时再引入独立机制。
// ============================================================================

class KlipperManager {
public:
    // 获取Klipper管理器的单例对象
    static KlipperManager& instance();

    // 初始化Klipper管理器
    bool initialize(const KlipperManagerConfig& config, std::string* err = nullptr);
    // 关闭Klipper管理器
    void shutdown();

    // 检查Klipper管理器是否已初始化
    bool isInitialized() const;
    // 重置归位状态
    void resetHomedState();

    // 确保Klipper已归位
    bool ensureHomed(std::string* err = nullptr);
    // 强制归位
    bool forceHome(std::string* err = nullptr);

    // 发送Gcode指令
    bool sendGcode(const std::string& script,
                   std::string* out_body = nullptr,
                   long timeout_sec = 40L,
                   std::string* err = nullptr);

    // 查询激光距离
    bool queryLaserDistance(double& out_distance,
                            bool& out_valid,
                            std::string* err = nullptr,
                            int max_attempts = 3);

    // 通用 Z 轴下探 + 激光测距循环
    bool descendZAndProbe(const ZProbeConfig& cfg,
                          ZProbeResult& out,
                          std::string* err = nullptr);

    // 查询焦距
    bool queryFocal(double& focal_long,
                    double& focal_short,
                    std::string* err = nullptr,
                    int timeout_ms = 1500);

    // 设置填充灯亮度
    bool setFillLight(int brightness, std::string* err = nullptr);
    // 打开激光
    bool laserOn(std::string* err = nullptr);
    // 关闭激光
    bool laserOff(std::string* err = nullptr);
    // 蜂鸣器
    void deep(std::string* err = nullptr);

private:
    // 构造函数，私有化以实现单例模式
    KlipperManager() = default;

    // 发送JSON请求
    bool postJson(const std::string& endpoint,
                  const std::string& payload,
                  long timeout_sec,
                  std::string* out_body,
                  long* out_status,
                  std::string* err);

    // 发送GET请求
    bool get(const std::string& endpoint,
             long timeout_sec,
             std::string* out_body,
             long* out_status,
             std::string* err);

    // 通用 HTTP 请求（内部复用，post/get 仅负责传入各自参数）
    bool performRequest(const std::string& endpoint,
                        const std::string* payload,
                        bool use_post,
                        bool add_json_content_type,
                        long timeout_sec,
                        std::string* out_body,
                        long* out_status,
                        std::string* err);

    // 等待归位轴
    bool waitHomedAxes(int timeout_ms, std::string* err);

    // descendZAndProbe 的单次下探循环体（"拆大函数"：把循环体抽成命名函数，见 .cpp）。
    // 一轮下探 = 移动 → 触发激光 → 查询距离 → 判断停止规则。
    enum class ProbeStepResult {
        Continue,  // 继续下一轮下探
        StopMet,   // 满足停止规则（成功）
        Failed,    // 致命失败（end_reason 已在 out 中设置）
    };
    ProbeStepResult probeOnce(const ZProbeConfig& cfg,
                              int index,
                              int max_attempts,
                              ZProbeResult& out,
                              int& consecutive_hits,
                              std::string* err);

private:
    std::mutex homing_mutex_;  // 归位互斥锁
    mutable std::mutex mutex_;  // 互斥锁
    KlipperManagerConfig config_;  // Klipper管理器配置
    bool initialized_ = false;  // Klipper管理器是否已初始化
    bool homed_ = false;  // Klipper是否已归位
};

#endif // KLIPPER_MANAGER_H