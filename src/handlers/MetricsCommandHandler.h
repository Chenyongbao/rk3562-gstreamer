#ifndef METRICS_COMMAND_HANDLER_H
#define METRICS_COMMAND_HANDLER_H

#include "../config.h"
#include "../protocol/CommandHandler.h"
#include "../reallink_ogles/file_utils.h"
#include "../camera_calibreation/totalHigh.h"

#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>

class MetricsCommandHandler : public ICommandHandler {
public:
    std::string getName() const override { return "METRICS"; }

    std::string getDescription() const override {
        return "Returns focal_long, focal_short, and TotalHigh";
    }

    CommandResult execute(CommandContext& ctx) override {
        fprintf(stderr, "[Unified Server] METRICS received from %s\n", ctx.client_ip.c_str());

        // 默认值
        double focal_long = 12.052;
        double focal_short = 8.154;
        double total_high = 0.0;
        bool focal_ok = false;
        bool total_high_ok = false;

        const std::string conf_path = std::string(REALLINK_CV_CONF_PATH);
        const std::string bin_path =
            std::string(CALIB_RESULT_DIR) + "/" + std::string(CALIB_BIN_NAME);

        // 读取焦距
        ReallinkCVConfig config;
        if (!readReallinkCVConf(conf_path, config)) {
            fprintf(stderr, "[Unified Server] WARNING: Failed to read %s, using default focal values\n", conf_path.c_str());
        } else {
            focal_long = config.focal_long;
            focal_short = config.focal_short;
            focal_ok = true;
        }

        // 读取总高 (从 bin 文件)
        std::string total_high_error;
        if (readPersistedTotalHigh(conf_path, bin_path, total_high, &total_high_error)) {
            total_high_ok = true;
        } else {
            fprintf(stderr, "[Unified Server] WARNING: Failed to resolve totalHigh: %s\n",
                    total_high_error.c_str());
        }

        // 构建 JSON 响应
        std::ostringstream json_stream;
        json_stream << std::fixed << std::setprecision(3);
        json_stream << "{";

        if (focal_ok) {
            json_stream << "\"focal_long\":" << focal_long << ","
                        << "\"focal_short\":" << focal_short << ",";
        } else {
            json_stream << "\"focal_long\":12.052,"
                        << "\"focal_short\":8.154,";
        }

        if (total_high_ok) {
            json_stream << "\"TotalHigh\":" << total_high;
        } else {
            json_stream << "\"TotalHigh\":0";
        }

        json_stream << "}\n";

        if (!ctx.sendBinaryResponse(json_stream.str())) {
            fprintf(stderr, "[Unified Server] Failed to send METRICS response\n");
            return CommandResult::ERROR_DISCONNECT;
        }

        fprintf(stderr, "[Unified Server] METRICS response sent: focal_long=%.3f, focal_short=%.3f, TotalHigh=%.3f\n",
                focal_long, focal_short, total_high);
        return CommandResult::SUCCESS;
    }
};

#endif // METRICS_COMMAND_HANDLER_H
