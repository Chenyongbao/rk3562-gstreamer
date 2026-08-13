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

// Z 轴下探停止策略
enum class ZProbeStopRule {
    FirstValid,           // 只要拿到 valid=true 即停止
    InRangeOrBelowMin,    // valid 且 (inRange || belowMin) 停止
    AtMost                // valid 且 distance <= stop_max_mm 停止
};

// Z 轴下探结束原因
enum class ZProbeEndReason {
    StopMet,
    ZExceededMax,
    NoValidAfterMax,
    RuleNotMetAfterMax,
    MoveFailed,
    TriggerFailed,
    QueryFailed
};

// Z 轴下探配置
struct ZProbeConfig {
    double feedrate = 2000.0;
    double step_mm = 1.0;
    int max_attempts = 1;
    int query_retries = 1;
    int trigger_settle_ms = 150;
    long move_timeout_sec = 40L;
    long trigger_timeout_sec = 20L;

    bool enforce_z_max = false;
    double z_max_mm = 0.0;

    double stop_min_mm = 0.0;
    double stop_max_mm = 0.0;
    ZProbeStopRule stop_rule = ZProbeStopRule::FirstValid;
    int consecutive_hits = 1;

    bool trigger_failure_is_fatal = true;
    bool query_failure_is_fatal = false;

    std::string log_prefix;  // 例如 "[XYoffset]"
};

// Z 轴下探结果
struct ZProbeResult {
    ZProbeEndReason end_reason = ZProbeEndReason::RuleNotMetAfterMax;
    int attempts = 0;
    double final_z_mm = 0.0;
    double final_distance_mm = 0.0;
    bool final_valid = false;
    bool had_any_valid = false;
    int consecutive_hits = 0;
};

// Klipper管理器类，用于管理与Klipper通信的相关操作
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
private:
    std::mutex homing_mutex_;  // 归位互斥锁
    mutable std::mutex mutex_;  // 互斥锁
    KlipperManagerConfig config_;  // Klipper管理器配置
    bool initialized_ = false;  // Klipper管理器是否已初始化
    bool homed_ = false;  // Klipper是否已归位
};

#endif // KLIPPER_MANAGER_H
