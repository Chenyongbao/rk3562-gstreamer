#ifndef METRICS_COMMAND_HANDLER_H
#define METRICS_COMMAND_HANDLER_H

#include "config.h"
#include "protocol/CommandHandler.h"
#include "reallink_ogles/file_utils.h"
#include "camera_calibration/totalHigh.h"

#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

// METRICS 命令：返回焦距与总高。
//
// 这是"组件化"的样板：一个命令处理器 = 一个组件，它——
//   1. 拥有自己的状态（构造时注入的配置路径）；
//   2. execute 只做"调度"（读数据 → 建 JSON → 发送），具体逻辑拆到命名函数；
//   3. 不再在 execute 里硬读全局宏（依赖注入）。
//
// 对照旧写法（execute 里 inline 读全局宏 + 60 行挤在一起），
// 组件化之后：加依赖改构造参数，不用动 execute 骨架；每个函数单一职责、可单独测试。
class MetricsCommandHandler : public ICommandHandler {
public:
    // 依赖注入：构造时传入配置路径，组件持有它们作为自己的状态。
    explicit MetricsCommandHandler(std::string conf_path, std::string bin_path)
        : conf_path_(std::move(conf_path)), bin_path_(std::move(bin_path)) {}

    std::string getName() const override { return "METRICS"; }

    std::string getDescription() const override {
        return "Returns focal_long, focal_short, and TotalHigh";
    }

    CommandResult execute(CommandContext& ctx) override {
        fprintf(stderr, "[Unified Server] METRICS received from %s\n", ctx.client_ip.c_str());

        // 调度：读数据 → 建响应 → 发送。
        const MetricsData data = loadMetrics();
        if (!ctx.sendBinaryResponse(buildJson(data))) {
            fprintf(stderr, "[Unified Server] Failed to send METRICS response\n");
            return CommandResult::ERROR_DISCONNECT;
        }

        fprintf(stderr,
                "[Unified Server] METRICS response sent: focal_long=%.3f, focal_short=%.3f, TotalHigh=%.3f\n",
                data.focal_long, data.focal_short, data.total_high);
        return CommandResult::SUCCESS;
    }

private:
    // 组件自己的状态：配置路径（依赖注入进来）。
    std::string conf_path_;
    std::string bin_path_;

    // 组件返回的数据（属于组件的"状态"的一部分）。
    struct MetricsData {
        double focal_long = DEFAULT_FOCAL_LONG;   // 默认值兜底（标定失败/未标定时）
        double focal_short = DEFAULT_FOCAL_SHORT;
        double total_high = 0.0;
        bool focal_ok = false;
        bool total_high_ok = false;
    };

    // 读数据：从配置文件 + bin 文件读出焦距与总高。
    MetricsData loadMetrics() const {
        MetricsData data;

        ReallinkCVConfig config;
        if (!readReallinkCVConf(conf_path_, config)) {
            fprintf(stderr,
                    "[Unified Server] WARNING: Failed to read %s, using default focal values\n",
                    conf_path_.c_str());
        } else {
            data.focal_long = config.focal_long;
            data.focal_short = config.focal_short;
            data.focal_ok = true;
        }

        std::string total_high_error;
        if (readPersistedTotalHigh(conf_path_, bin_path_, data.total_high, &total_high_error)) {
            data.total_high_ok = true;
        } else {
            fprintf(stderr, "[Unified Server] WARNING: Failed to resolve totalHigh: %s\n",
                    total_high_error.c_str());
        }

        return data;
    }

    // 建 JSON 响应（单一职责）。
    std::string buildJson(const MetricsData& data) const {
        std::ostringstream json_stream;
        json_stream << std::fixed << std::setprecision(3);
        json_stream << "{";

        if (data.focal_ok) {
            json_stream << "\"focal_long\":" << data.focal_long << ","
                        << "\"focal_short\":" << data.focal_short << ",";
        } else {
            json_stream << "\"focal_long\":" << DEFAULT_FOCAL_LONG << ","
                        << "\"focal_short\":" << DEFAULT_FOCAL_SHORT << ",";
        }

        if (data.total_high_ok) {
            json_stream << "\"TotalHigh\":" << data.total_high;
        } else {
            json_stream << "\"TotalHigh\":0";
        }

        json_stream << "}\n";
        return json_stream.str();
    }
};

#endif // METRICS_COMMAND_HANDLER_H
