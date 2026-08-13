#include "klipper_manager.h"

#include "../config.h"

#include <curl/curl.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>
#include "../tools/json_utils.h"

namespace {

struct HttpResponse {
    std::string data;
    long status = 0;
    CURLcode code = CURLE_OK;
};

size_t writeCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    const size_t total = size * nmemb;
    auto* resp = static_cast<HttpResponse*>(userp);
    resp->data.append(static_cast<char*>(contents), total);
    return total;
}

void setError(std::string* err, const std::string& msg) {
    if (err) {
        *err = msg; 
    }
}

bool extractHomedAxes(const std::string& body, std::string& axes_out) {
    const std::string key = "\"homed_axes\"";
    const size_t key_pos = body.find(key);
    if (key_pos == std::string::npos) {
        return false;
    }
    const size_t colon_pos = body.find(':', key_pos + key.size());
    if (colon_pos == std::string::npos) {
        return false;
    }
    const size_t quote_begin = body.find('"', colon_pos + 1);
    if (quote_begin == std::string::npos) {
        return false;
    }
    const size_t quote_end = body.find('"', quote_begin + 1);
    if (quote_end == std::string::npos) {
        return false;
    }

    axes_out = body.substr(quote_begin + 1, quote_end - quote_begin - 1);
    return true;
}
//查找xyz
bool hasHomedXYZ(const std::string& body) {
    std::string axes;
    if (!extractHomedAxes(body, axes)) {
        return false;
    }
    return axes.find('x') != std::string::npos
        && axes.find('y') != std::string::npos
        && axes.find('z') != std::string::npos;
}
} // namespace

KlipperManager& KlipperManager::instance() {
    static KlipperManager manager;
    return manager;
}
//初始化
bool KlipperManager::initialize(const KlipperManagerConfig& config, std::string* err) {
    // Precondition: curl_global_init/cleanup is owned by AppRuntime.
    (void)err;

    std::lock_guard<std::mutex> lock(mutex_);

    KlipperManagerConfig normalized;
    normalized.host = config.host.empty() ? KLIPPER_MOONRAKER_DEFAULT_HOST : config.host;
    normalized.port = config.port > 0 ? config.port : KLIPPER_MOONRAKER_PORT;
    normalized.homing_timeout_ms = config.homing_timeout_ms > 0
        ? config.homing_timeout_ms
        : KLIPPER_HOMING_TIMEOUT_MS;

    const bool changed = (!initialized_)
        || normalized.host != config_.host
        || normalized.port != config_.port
        || normalized.homing_timeout_ms != config_.homing_timeout_ms;

    config_ = std::move(normalized);
    initialized_ = true;

    if (changed) {
        homed_ = false;
    }

    return true;
}
//关闭
void KlipperManager::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    initialized_ = false;
    homed_ = false;
}
//是否已经初始化
bool KlipperManager::isInitialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_;
}
//重置归位状态
void KlipperManager::resetHomedState() {
    std::lock_guard<std::mutex> lock(mutex_);
    homed_ = false;
}

//通信
bool KlipperManager::performRequest(const std::string& endpoint,
                                    const std::string* payload,
                                    bool use_post,
                                    bool add_json_content_type,
                                    long timeout_sec,
                                    std::string* out_body,
                                    long* out_status,
                                    std::string* err) {
    KlipperManagerConfig cfg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) {
            setError(err, "KlipperManager is not initialized");
            return false;
        }
        cfg = config_;
    }

    CURL* curl = curl_easy_init();
    if (!curl) {
        setError(err, "curl_easy_init failed");
        return false;
    }

    std::ostringstream url;
    url << "http://" << cfg.host << ":" << cfg.port << endpoint;

    HttpResponse resp;
    std::string request_body;
    curl_easy_setopt(curl, CURLOPT_URL, url.str().c_str());
    if (use_post) {
        request_body = payload ? *payload : std::string();
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(request_body.size()));
    } else {
        curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
    }
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 7L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_sec > 0 ? timeout_sec : 5L);

    struct curl_slist* headers = nullptr;
    if (add_json_content_type) {
        headers = curl_slist_append(headers, "Content-Type: application/json");
    }
    if (headers) {
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    }

    resp.code = curl_easy_perform(curl);
    if (headers) {
        curl_slist_free_all(headers);
    }

    if (resp.code != CURLE_OK) {
        setError(err, std::string("CURL error: ") + curl_easy_strerror(resp.code));
        curl_easy_cleanup(curl);
        return false;
    }

    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &resp.status);
    if (out_body) {
        *out_body = resp.data;
    }
    if (out_status) {
        *out_status = resp.status;
    }

    if (resp.status < 200 || resp.status >= 300) {
        std::ostringstream oss;
        oss << "Moonraker returned status " << resp.status;
        if (!resp.data.empty()) {
            oss << ": " << resp.data;
        }
        setError(err, oss.str());
        curl_easy_cleanup(curl);
        return false;
    }

    curl_easy_cleanup(curl);
    return true;
}

bool KlipperManager::postJson(const std::string& endpoint,
                              const std::string& payload,
                              long timeout_sec,
                              std::string* out_body,
                              long* out_status,
                              std::string* err) {
    return performRequest(endpoint, &payload, true, true, timeout_sec, out_body, out_status, err);
}

bool KlipperManager::get(const std::string& endpoint,
                         long timeout_sec,
                         std::string* out_body,
                         long* out_status,
                         std::string* err) {
    return performRequest(endpoint, nullptr, false, false, timeout_sec, out_body, out_status, err);
}

bool KlipperManager::sendGcode(const std::string& script,
                               std::string* out_body,
                               long timeout_sec,
                               std::string* err) {
    //执行gcode的字段
    const std::string payload = std::string("{\"script\":\"") + JsonUtils::escape(script) + "\"}";
    return postJson("/printer/gcode/script", payload, timeout_sec, out_body, nullptr, err);
}


/**
 * 等待归位轴。
 *
 * @param timeout_ms 等待的毫秒数。
 * @param err 如果归位失败，将存储错误信息。
 * @return 如果归位成功，返回true；否则，返回false。
 */
bool KlipperManager::waitHomedAxes(int timeout_ms, std::string* err) {
    constexpr int kPollMs = 250;
    constexpr int kProgressLogMs = 5000;
    constexpr long kMinReqTimeoutSec = 1;
    constexpr long kMaxReqTimeoutSec = 3;

    if (timeout_ms <= 0) {
        timeout_ms = 1;
    }

    const auto begin = std::chrono::steady_clock::now();
    const auto deadline = begin + std::chrono::milliseconds(timeout_ms);
    auto last_log_time = begin;
    std::string last_axes = "unknown";
    std::string last_req_error;

    while (std::chrono::steady_clock::now() < deadline) {
        const auto now = std::chrono::steady_clock::now();
        const auto remaining_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
        long req_timeout_sec = (remaining_ms + 999) / 1000;
        req_timeout_sec = std::max(kMinReqTimeoutSec, std::min(kMaxReqTimeoutSec, req_timeout_sec));

        std::string body;
        std::string req_error;
        if (get("/printer/objects/query?toolhead", req_timeout_sec, &body, nullptr, &req_error)) {
            std::string axes;
            last_axes = extractHomedAxes(body, axes) ? (axes.empty() ? "empty" : axes) : "missing";
            if (hasHomedXYZ(body)) {
                return true;
            }
        } else if (!req_error.empty()) {
            last_req_error = req_error;
        }

        const auto after_query = std::chrono::steady_clock::now();
        const auto remain_after_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - after_query).count();

        // 长时间等待时偶尔打印一次进度，避免误以为程序卡死。
        if (std::chrono::duration_cast<std::chrono::milliseconds>(after_query - last_log_time).count()
            >= kProgressLogMs) {
            std::cerr << "[KlipperManager] waitHomedAxes pending: elapsed="
                      << std::chrono::duration_cast<std::chrono::milliseconds>(after_query - begin).count()
                      << "ms, remaining=" << std::max<long long>(0LL, remain_after_ms)
                      << "ms, homed_axes=" << last_axes;
            if (!last_req_error.empty()) {
                std::cerr << ", last_error=" << last_req_error;
            }
            std::cerr << std::endl;
            last_log_time = after_query;
        }

        if (remain_after_ms <= 0) {
            break;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(std::min<long long>(kPollMs, remain_after_ms)));
    }

    std::ostringstream oss;
    oss << "在" << timeout_ms << "毫秒内未确认归位"
        << " (homed_axes=" << last_axes;
    if (!last_req_error.empty()) {
        oss << ", last_error=" << last_req_error;
    }
    oss << ")";
    setError(err, oss.str());
    return false;
}

//测总高调用，待优化
/**
 * 确保已经归位。
 * 
 * @param err 如果归位失败，将存储错误信息。
 * @return 如果归位成功，返回true；否则，返回false。
 */
bool KlipperManager::ensureHomed(std::string* err) {
    std::lock_guard<std::mutex> homing_lock(homing_mutex_);  //

    int timeout_ms = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);  // 
        if (!initialized_) {
            setError(err, "KlipperManager未初始化");
            return false;
        }
        timeout_ms = config_.homing_timeout_ms;
        homed_ = false;
    }

    std::string local_error;
    std::cerr << "[KlipperManager] ensureHomed: forcing explicit G28" << std::endl;
    if (!sendGcode("G90\nG28\n", nullptr, 30L, &local_error)) {
        setError(err, local_error);
        std::lock_guard<std::mutex> lock(mutex_);  // 
        homed_ = false;
        return false;
    }

    if (!waitHomedAxes(timeout_ms, &local_error)) {
        setError(err, local_error);
        std::lock_guard<std::mutex> lock(mutex_);  // 
        homed_ = false;
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);  // 
    homed_ = true;
    return true;
}
//强制归位
bool KlipperManager::forceHome(std::string* err) {
    std::lock_guard<std::mutex> homing_lock(homing_mutex_);

    int timeout_ms = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) {
            setError(err, "KlipperManager未初始化");
            return false;
        }
        homed_ = false;
        timeout_ms = config_.homing_timeout_ms;
    }

    std::string local_error;
    std::cerr << "[KlipperManager] forceHome: sending explicit G28 + M400" << std::endl;
    if (!sendGcode("G90\nG28\nM400\n", nullptr, 60L, &local_error)) {
        setError(err, local_error);
        std::lock_guard<std::mutex> lock(mutex_);
        homed_ = false;
        return false;
    }

    if (!waitHomedAxes(timeout_ms, &local_error)) {
        setError(err, local_error);
        std::lock_guard<std::mutex> lock(mutex_);
        homed_ = false;
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    homed_ = true;
    return true;
}
//高度查询
bool KlipperManager::queryLaserDistance(double& out_distance,
                                        bool& out_valid,
                                        std::string* err,
                                        int max_attempts) {
    if (max_attempts <= 0) {
        max_attempts = 1;
    }

    //查询 Moonraker 对象
    const std::string sensor_obj = "laser_range_sensor my_range_sensor";
    //构造json
    std::ostringstream payload;
    payload << "{\"objects\":{\"" << sensor_obj << "\":[\"distance\",\"distance_valid\"]}}";
    auto retryOrFail = [&](int attempt, const std::string& message) -> bool {
        if (attempt + 1 >= max_attempts) {
            setError(err, message);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        return true;
    };

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        std::string body;
        std::string local_error;
        if (!postJson("/printer/objects/query", payload.str(), 10L, &body, nullptr, &local_error)) {
            if (!retryOrFail(attempt, local_error)) {
                return false;
            }
            continue;
        }

        const size_t anchor = body.find(sensor_obj);
        if (anchor == std::string::npos) {
            if (!retryOrFail(attempt, "queryLaserDistance missing sensor object")) {
                return false;
            }
            continue;
        }

        out_valid = false;
        const size_t valid_pos = body.find("\"distance_valid\"", anchor);
        if (valid_pos != std::string::npos) {
            const size_t colon = body.find(':', valid_pos);
            if (colon != std::string::npos) {
                const size_t tpos = body.find("true", colon);
                const size_t fpos = body.find("false", colon);
                if (tpos != std::string::npos && (fpos == std::string::npos || tpos < fpos)) {
                    out_valid = true;
                } else if (fpos != std::string::npos) {
                    out_valid = false;
                }
            }
        }

        const size_t dist_pos = body.find("\"distance\"", anchor);
        if (dist_pos == std::string::npos) {
            if (!retryOrFail(attempt, "queryLaserDistance missing distance")) {
                return false;
            }
            continue;
        }

        const size_t colon = body.find(':', dist_pos);
        if (colon == std::string::npos) {
            if (!retryOrFail(attempt, "queryLaserDistance parse distance failed")) {
                return false;
            }
            continue;
        }

        const char* start = body.c_str() + colon + 1;
        char* endptr = nullptr;
        const double value = std::strtod(start, &endptr);
        if (endptr == start || !std::isfinite(value)) {
            if (!retryOrFail(attempt, "queryLaserDistance strtod failed")) {
                return false;
            }
            continue;
        }

        out_distance = value;
        return true;
    }

    setError(err, "queryLaserDistance failed");
    return false;
}

bool KlipperManager::descendZAndProbe(const ZProbeConfig& cfg,
                                      ZProbeResult& out,
                                      std::string* err) {
    out = ZProbeResult{};

    // 将外部配置规整为可执行参数，避免 0/负值导致流程失真。
    const int max_attempts = cfg.max_attempts > 0 ? cfg.max_attempts : 1;
    const int query_retries = cfg.query_retries > 0 ? cfg.query_retries : 1;
    const int trigger_settle_ms = cfg.trigger_settle_ms >= 0 ? cfg.trigger_settle_ms : 0;
    const int required_hits = cfg.consecutive_hits > 0 ? cfg.consecutive_hits : 1;
    const double step_mm = cfg.step_mm > 0.0 ? cfg.step_mm : 1.0;
    const long move_timeout_sec = cfg.move_timeout_sec > 0 ? cfg.move_timeout_sec : 40L;
    const long trigger_timeout_sec = cfg.trigger_timeout_sec > 0 ? cfg.trigger_timeout_sec : 20L;

    int consecutive_hits = 0;

    for (int i = 0; i < max_attempts; ++i) {
        // 以固定步长逐次下探：第 i 次移动到 i * step_mm 的绝对 Z 位置。
        const double z_target = i* step_mm;
        out.attempts = i + 1;
        out.final_z_mm = z_target;

        // 若启用了 Z 上限保护，超过允许范围就立即停止，避免机构继续下探。
        if (cfg.enforce_z_max && z_target > cfg.z_max_mm) {
            out.end_reason = ZProbeEndReason::ZExceededMax;
            setError(err, "Z target exceeded max limit");
            return false;
        }

        if (!cfg.log_prefix.empty()) {
            std::cout << cfg.log_prefix << " AutoZ attempt " << (i + 1)
                      << " moving to Z=" << z_target << std::endl;
        }

        std::ostringstream zscript;
        zscript.setf(std::ios::fixed);
        zscript.precision(3);
        zscript << "G90\n";
        zscript << "G1 Z" << z_target << " F" << cfg.feedrate << "\n";
        zscript << "M400\n";

        std::string move_error;
        if (!sendGcode(zscript.str(), nullptr, move_timeout_sec, &move_error)) {
            out.end_reason = ZProbeEndReason::MoveFailed;
            setError(err, move_error);
            return false;
        }

        // 运动完成后主动触发一次激光测距；是否因触发失败而终止由配置决定。
        std::string trigger_error;
        if (!sendGcode("LASER_RANGE_SENSOR SENSOR=my_range_sensor", nullptr, trigger_timeout_sec, &trigger_error)) {
            if (cfg.trigger_failure_is_fatal) {
                out.end_reason = ZProbeEndReason::TriggerFailed;
                setError(err, trigger_error);
                return false;
            }
            if (!cfg.log_prefix.empty()) {
                std::cout << cfg.log_prefix
                          << " WARNING: Failed to trigger laser range sensor: "
                          << trigger_error << std::endl;
            }
        }

        if (trigger_settle_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(trigger_settle_ms));
        }

        double distance = 0.0;
        bool valid = false;
        bool got_valid_reading = false;
        bool got_query_response = false;
        std::string last_query_error;

        // 单次触发后允许做多次查询重试，直到拿到 valid=true 的稳定读数。
        for (int attempt = 0; attempt < query_retries; ++attempt) {
            std::string query_error;
            if (queryLaserDistance(distance, valid, &query_error, 1)) {
                got_query_response = true;
                if (valid) {
                    got_valid_reading = true;
                    break;
                }
            } else {
                last_query_error = query_error;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
        }

        out.final_distance_mm = distance;
        out.final_valid = valid;

        // 这一轮没有拿到有效读数：根据配置决定是否失败，否则继续下一步下探。
        if (!got_valid_reading) {
            consecutive_hits = 0;
            out.consecutive_hits = 0;

            if (!got_query_response && cfg.query_failure_is_fatal) {
                out.end_reason = ZProbeEndReason::QueryFailed;
                setError(err, last_query_error.empty() ? "queryLaserDistance failed" : last_query_error);
                return false;
            }

            if (!cfg.log_prefix.empty()) {
                std::cout << cfg.log_prefix << " AutoZ attempt " << (i + 1)
                          << " no valid laser reading after " << query_retries
                          << " retries, continue" << std::endl;
            }

            if (i == max_attempts - 1) {
                out.end_reason = ZProbeEndReason::NoValidAfterMax;
                setError(err, "no valid laser reading after max attempts");
                return false;
            }
            continue;
        }

        out.had_any_valid = true;

        if (!cfg.log_prefix.empty()) {
            std::cout << cfg.log_prefix << " AutoZ attempt " << (i + 1)
                      << " distance=" << distance
                      << " valid=" << (valid ? "true" : "false")
                      << std::endl;
        }

        bool stop_met = false;
        switch (cfg.stop_rule) {
        case ZProbeStopRule::FirstValid:
            // 只要拿到首个有效读数就停止。
            stop_met = valid;
            if (stop_met) {
                consecutive_hits = std::max(consecutive_hits, 1);
            }
            break;

        case ZProbeStopRule::InRangeOrBelowMin: {
            // 距离进入目标区间，或已经低于下限，都记为一次命中。
            const bool in_range = (distance >= cfg.stop_min_mm && distance <= cfg.stop_max_mm);
            const bool below_min = (distance < cfg.stop_min_mm);
            if (valid && (in_range || below_min)) {
                ++consecutive_hits;
            } else {
                consecutive_hits = 0;
            }

            if (!cfg.log_prefix.empty()) {
                std::cout << cfg.log_prefix << " AutoZ check: distance=" << distance
                          << " range=[" << cfg.stop_min_mm << "," << cfg.stop_max_mm << "]"
                          << " inRange=" << (in_range ? "true" : "false")
                          << " belowMin=" << (below_min ? "true" : "false")
                          << " consecutiveHits=" << consecutive_hits
                          << std::endl;
            }

            stop_met = valid && consecutive_hits >= required_hits;
            break;
        }

        case ZProbeStopRule::AtMost:
            // 只要距离不大于 stop_max_mm 就停止。
            stop_met = valid && distance <= cfg.stop_max_mm;
            if (stop_met) {
                consecutive_hits = std::max(consecutive_hits, 1);
            } else {
                consecutive_hits = 0;
            }
            break;
        }

        out.consecutive_hits = consecutive_hits;

        if (stop_met) {
            out.end_reason = ZProbeEndReason::StopMet;
            return true;
        }

        // 已经到达最后一次尝试仍未满足停止规则，则按规则未命中失败返回。
        if (i == max_attempts - 1) {
            out.end_reason = ZProbeEndReason::RuleNotMetAfterMax;
            setError(err, "stop rule not met after max attempts");
            return false;
        }
    }

    out.end_reason = ZProbeEndReason::RuleNotMetAfterMax;
    setError(err, "stop rule not met after max attempts");
    return false;
}
//焦距查询
bool KlipperManager::queryFocal(double& focal_long,
                                double& focal_short,
                                std::string* err,
                                int timeout_ms) {
    std::string body;
    if (!get("/printer/objects/query?laser_board",
             std::max(1, timeout_ms / 1000),
             &body,
             nullptr,
             err)) {
        return false;
    }

    if (!JsonUtils::parseNumber(body, "focal_long", focal_long)
        || !JsonUtils::parseNumber(body, "focal_short", focal_short)) {
        setError(err, "Failed to parse focal from Moonraker response");
        return false;
    }
    return true;
}
//开灯
bool KlipperManager::setFillLight(int brightness, std::string* err) {
    std::ostringstream script;
    script << "SET_FILL_LIGHT FILL_LIGHT=chamber_lights BRIGHTNESS=" << brightness;
    return sendGcode(script.str(), nullptr, 20L, err);
}
//开启激光
bool KlipperManager::laserOn(std::string* err) {
    return sendGcode("M3 S1", nullptr, 20L, err);
}
//关闭激光
bool KlipperManager::laserOff(std::string* err) {
    return sendGcode("M5", nullptr, 20L, err);
}
//蜂鸣器
void KlipperManager::deep(std::string* err) {
    std::string local_error;
    if (!sendGcode("BEEP_ONCE", nullptr, 5L, &local_error)) {
        if (err) {
            *err = local_error;
        }
    }
}